import torch as th
import torch.nn as nn
import torch.nn.functional as F
from stable_baselines3.common.torch_layers import BaseFeaturesExtractor
from stable_baselines3.common.policies import ActorCriticPolicy

class Swish(nn.Module):
    def forward(self, x):
        return x * th.sigmoid(x)

class MyFeatureExtractor(BaseFeaturesExtractor):
    def __init__(self, observation_space, features_dim=512, config=None):
        super().__init__(observation_space, features_dim)
        self.config = config

        # Dimensions robot / goal
        self.goal_size = 1 + int(self.config.env.obs.goal_dist)
        self.vel_size = 2 if self.config.env.obs.robot_velocity else 0
        self.robot_obs_size = self.goal_size + self.vel_size

        # Humans
        self.human_size = 5
        self.N = self.config.env.obs.human_number if self.config.env.obs.human else 0

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

        # ---------------- Humans MLP + Transformer ----------------
        self.human_mlp = nn.Sequential(
            nn.Linear(self.human_size, 32),
            Swish(),
            nn.Linear(32, 32),
            Swish()
        )

        encoder_layer = nn.TransformerEncoderLayer(d_model=32, nhead=4, batch_first=True)
        self.human_transformer = nn.TransformerEncoder(encoder_layer, num_layers=2)
        self.human_attention = nn.Linear(32, 1)

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
        humans = humans.view(-1, self.N, self.human_size)  # (batch, N, 5)
        h = self.human_mlp(humans)  # (batch, N, 32)
        h = self.human_transformer(h)  # (batch, N, 32)
        # Attention pooling
        attn_weights = F.softmax(self.human_attention(h), dim=1)  # (batch, N, 1)
        x_humans = (h * attn_weights).sum(dim=1)  # (batch, 32)

        # ---- Robot branch ----
        x_robot = self.robot_fc(robot)  # (batch, 32)

        # ---- Fusion ----
        x = th.cat([x_scan, x_humans, x_robot], dim=1)
        x = self.final_fc(x)

        return x


class MyPolicy(ActorCriticPolicy):
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
