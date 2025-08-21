import torch as th
import torch.nn as nn
import torch.nn.functional as F
from stable_baselines3.common.torch_layers import BaseFeaturesExtractor
from stable_baselines3.common.policies import ActorCriticPolicy

class Swish(nn.Module):
    def forward(self, x):
        return x * th.sigmoid(x)

import torch as th
import torch.nn as nn
import torch.nn.functional as F
from torch_geometric.nn import GATv2Conv, GATConv, global_mean_pool

class HumanSTGraph(nn.Module):
    def __init__(self, human_size=5, hidden_dim=32, number_max_human=5, history=4):
        super().__init__()
        self.number_max_human = number_max_human
        self.history = history
        self.input_dim = human_size

        # Temporal MLP + Conv
        self.mlp_in = nn.Sequential(
            nn.Linear(self.input_dim, hidden_dim),
            nn.SiLU()
        )
        self.temp_conv = nn.Conv1d(in_channels=hidden_dim, out_channels=hidden_dim, kernel_size=history)

        # Spatial GNN avec edge features (distances)
        self.gat1 = GATv2Conv(hidden_dim, hidden_dim, heads=4, concat=False, edge_dim=1)
        self.gat2 = GATv2Conv(hidden_dim, hidden_dim, heads=4, concat=False, edge_dim=1)

        # --- Attention pooling over humans ---
        self.attn_pool = nn.Sequential(
            nn.Linear(hidden_dim + 1, 64),  # node features + distance to robot
            nn.SiLU(),
            nn.Linear(64, 1)
        )

    
    def forward(self, humans):
        """
        humans: (batch, N, history, human_size)
        """
        batch_size, N, H, F_h = humans.shape

        # --- Temporal conv ---
        h = humans.reshape(batch_size*N, H, F_h)  # (B*N, history, 5)
        h = h.permute(0, 2, 1)                   # (B*N, 5, history)
        h = self.mlp_in(h.permute(0, 2, 1))      # Linear 5->32
        h = h.permute(0, 2, 1)                   # (B*N, hidden, history)
        h = self.temp_conv(h)                     # (B*N, hidden, 1)
        h = h.squeeze(-1)                         # (B*N, hidden)

        # --- Positions for edges ---
        positions = humans[:, :, -1, :2]  # dx, dy relative to robot

        # --- Fully connected edges ---
        edge_index = self._fully_connected_edges(N, batch_size, h.device)
        edge_attr = self._edge_distances(positions, edge_index)

        # --- GAT layers ---
        if edge_index.numel() != 0:
            h = self.gat1(h, edge_index, edge_attr)
            h = F.silu(h)
            h = self.gat2(h, edge_index, edge_attr)
            h = F.silu(h)

        # --- Attention pooling over humans ---
        # Node feature: distance to robot (last element of humans vector)
        dist_robot = humans[:, :, -1, -1].reshape(batch_size*N, 1)
        attn_input = th.cat([h, dist_robot], dim=1)
        attn_weights = th.sigmoid(self.attn_pool(attn_input))
        h = (h * attn_weights).reshape(batch_size, N, -1)
        h = h.sum(dim=1)  # weighted sum pooling

        return h  # (batch, hidden_dim)

    def _fully_connected_edges(self, N, batch_size, device):
        # Connect each human to each human within the same batch
        edges = []
        for b in range(batch_size):
            for i in range(N):
                for j in range(N):
                    if i != j:
                        edges.append([b*N + i, b*N + j])
        edge_index = th.tensor(edges, dtype=th.long).t().contiguous().to(device)
        return edge_index

    def _edge_distances(self, positions, edge_index):
        """
        positions: (batch, N, 2)
        edge_index: (2, E)
        """
        batch_size, N, _ = positions.shape
        pos_flat = positions.reshape(batch_size*N, 2)
        src, dst = edge_index
        delta = pos_flat[src] - pos_flat[dst]
        dist = delta.norm(dim=1, keepdim=True)  # (E, 1)
        return dist


class MyGraphFeatureExtractor(BaseFeaturesExtractor):
    def __init__(self, observation_space, features_dim=512, config=None):
        super().__init__(observation_space, features_dim)
        self.config = config

        # Dimensions robot / goal
        self.goal_size = 1 + int(self.config.env.obs.goal_dist)
        self.vel_size = 2 if self.config.env.obs.robot_velocity else 0
        self.robot_obs_size = self.goal_size + self.vel_size

        # Humans
        self.human_size = 5
        self.human_history = self.config.env.obs.human_history if self.config.env.obs.human else 0
        self.number_max_human = self.config.env.obs.human_number if self.config.env.obs.human else 0

        # Scan
        self.scan_size = self.config.env.obs.scan_dim
        self.nb_slice = self.config.env.obs.scan_slice
        self.scan_history = self.config.env.obs.scan_history
        self.scan_tile = self.config.env.obs.scan_tile
        self.scan_obs_size = int(self.scan_size * self.scan_history * self.scan_tile)
        size = int(self.config.env.obs.scan_avg_pool) + int(self.config.env.obs.scan_min_pool)
        if size > 0:
            self.scan_obs_size = int((self.scan_size / self.nb_slice) * self.scan_history * self.scan_tile * size)

        # ---------------- Scan CNN ----------------
        self.scan_net = nn.Sequential(
            nn.Conv1d(1, 16, kernel_size=5, stride=2, padding=2),
            Swish(),
            nn.Conv1d(16, 32, kernel_size=5, stride=2, padding=2),
            Swish(),
        )

        with th.no_grad():
            dummy_input = th.zeros(1, 1, self.scan_obs_size)
            dummy_out = self.scan_net(dummy_input)
            conv_output_size = dummy_out.numel()

        self.scan_fc = nn.Sequential(
            nn.Linear(conv_output_size, 256),
            Swish()
        )

        # ---------------- Humans STGraph ----------------
        self.human_branch = HumanSTGraph(human_size=5, hidden_dim=32, number_max_human=self.number_max_human, history=self.human_history)


        # ---------------- Robot MLP ----------------
        self.robot_fc = nn.Sequential(
            nn.Linear(self.robot_obs_size, 32),
            Swish(),
            nn.Linear(32, 32),
            Swish()
        )

        # ---------------- Final FC ----------------
        self.final_fc = nn.Sequential(
            nn.Linear(256 + 32 + 32, features_dim),
            Swish()
        )

    def forward(self, observations):
        # découpage des observations
        scan = observations[:, :self.scan_obs_size]
        robot = observations[:, self.scan_obs_size:self.scan_obs_size + self.robot_obs_size]
        humans = observations[:, self.scan_obs_size + self.robot_obs_size:]

        # ---- Scan branch ----
        x_scan = scan.unsqueeze(1)  # (batch, 1, scan_size)
        x_scan = self.scan_net(x_scan)
        x_scan = x_scan.flatten(start_dim=1)
        x_scan = self.scan_fc(x_scan)

        # ---- Humans branch ----
        humans = humans.reshape(-1, self.number_max_human, self.human_history, self.human_size)
        x_humans = self.human_branch(humans)  # (batch, 32)

        # ---- Robot branch ----
        x_robot = self.robot_fc(robot)  # (batch, 32)

        # ---- Fusion ----
        x = th.cat([x_scan, x_humans, x_robot], dim=1)
        x = self.final_fc(x)

        return x


class MyGraphPolicy(ActorCriticPolicy):
    def __init__(self, observation_space, action_space, lr_schedule, **kwargs):
        super().__init__(
            observation_space,
            action_space,
            lr_schedule,
            **kwargs,
        )

        latent_dim = self.mlp_extractor.latent_dim_pi  # ← Taille des features policy
        
        self.mu = nn.Linear(latent_dim, action_space.shape[0])
        self.log_std = nn.Parameter(th.zeros(action_space.shape[0]))

    def forward_actor(self, features):
        mu = self.mu(features)
        std = th.exp(self.log_std)
        return mu, std

    def _get_action_dist_from_latent(self, latent_pi, latent_vf=None):
        mean_actions, std = self.forward_actor(latent_pi)
        return self.action_dist.proba_distribution(mean_actions, std)

    def forward_critic(self, features):
        return self.value_net(features)
