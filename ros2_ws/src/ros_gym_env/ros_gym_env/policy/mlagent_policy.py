import torch as th
import torch.nn as nn
import torch.nn.functional as F
from stable_baselines3.common.torch_layers import BaseFeaturesExtractor
from stable_baselines3.common.policies import ActorCriticPolicy

class Swish(nn.Module):
    def forward(self, x):
        return x * th.sigmoid(x)

class MLAgentFeatureExtractor(BaseFeaturesExtractor):
    def __init__(self, observation_space, features_dim=512, config = None):
        super().__init__(observation_space, features_dim)
        self.config = config
        self.obs_size = 90 #85 #645 #325 #6405
        self.norm_mean = nn.Parameter(th.zeros(self.obs_size), requires_grad=False)
        self.norm_std = nn.Parameter(th.ones(self.obs_size), requires_grad=False)

        self.net = nn.Sequential(
            nn.Linear(self.obs_size, 512),
            Swish(),
            nn.Linear(512, 512),
            Swish(),
            nn.Linear(512, 512),
            Swish(),
        )

    def forward(self, observations):
        x = (observations - self.norm_mean) / self.norm_std
        return self.net(x)

class MLAgentPolicy(ActorCriticPolicy):
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
