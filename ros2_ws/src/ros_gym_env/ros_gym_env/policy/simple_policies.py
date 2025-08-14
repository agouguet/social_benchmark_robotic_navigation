import torch as th
import torch.nn as nn
import torch.nn.functional as F
from stable_baselines3.common.torch_layers import BaseFeaturesExtractor
from stable_baselines3.common.policies import ActorCriticPolicy

class Swish(nn.Module):
    def forward(self, x):
        return x * th.sigmoid(x)

class MultiModalFeatureExtractor(BaseFeaturesExtractor):
    def __init__(self, observation_space, features_dim=512, config=None):
        super().__init__(observation_space, features_dim)
        self.config = config

        self.goal_size = 1 + int(self.config.env.obs.goal_dist)
        self.vel_size = 2 if self.config.env.obs.robot_velocity else 0
        self.human_size = 5 * self.config.env.obs.human_number if self.config.env.obs.human else 0 # si human_number agents

        # Dimensions supposées (à adapter selon ton env)
        self.scan_size = self.config.env.obs.scan_dim
        self.nb_slice = self.config.env.obs.scan_slice
        self.scan_history = self.config.env.obs.scan_history
        self.scan_tile = self.config.env.obs.scan_tile
        # taille du scan brute en entrée
        self.scan_obs_size = int(self.scan_size * self.scan_history * self.scan_tile)
        size = int(self.config.env.obs.scan_avg_pool) + int(self.config.env.obs.scan_min_pool)
        if size > 0:
            self.scan_obs_size = int((self.scan_size / self.nb_slice) * self.scan_history * self.scan_tile * size)
        

        # CNN pour le scan
        self.scan_net = nn.Sequential(
            nn.Conv1d(1, 16, kernel_size=5, stride=2, padding=2),
            Swish(),
            nn.Conv1d(16, 32, kernel_size=5, stride=2, padding=2),
            Swish(),
        )

        # Calcul dynamique de la taille après conv
        with th.no_grad():
            dummy_input = th.zeros(1, 1, self.scan_obs_size)
            dummy_out = self.scan_net(dummy_input)
            conv_output_size = dummy_out.numel()  # total features après flatten

        self.scan_fc = nn.Sequential(
            nn.Linear(conv_output_size, 256),
            Swish()
        )

        # MLP pour les autres features concaténées
        self.other_fc = nn.Sequential(
            nn.Linear(self.goal_size + self.vel_size + self.human_size, 256),
            Swish(),
            nn.Linear(256, 256),
            Swish(),
        )

        # FC final
        self.final_fc = nn.Sequential(
            nn.Linear(512, features_dim),
            Swish(),
        )

    def forward(self, observations):
        # découpage des observations
        scan = observations[:, :self.scan_obs_size]
        other = observations[:, self.scan_obs_size:]

        # CNN scan
        x_scan = scan.unsqueeze(1)  # (batch, channel=1, scan_size)
        x_scan = self.scan_net(x_scan)
        # print("Shape after conv:", x_scan.shape)
        x_scan = x_scan.flatten(start_dim=1)
        # print("Shape after flatten:", x_scan.shape)
        x_scan = self.scan_fc(x_scan)

        # MLP autres
        x_other = self.other_fc(other)

        # concat final
        x = th.cat([x_scan, x_other], dim=1)
        x = self.final_fc(x)
        return x


class SimplePolicy(ActorCriticPolicy):
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
