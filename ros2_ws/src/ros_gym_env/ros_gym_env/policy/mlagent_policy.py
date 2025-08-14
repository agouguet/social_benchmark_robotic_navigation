import torch as th
import torch.nn as nn
import torch.nn.functional as F
from stable_baselines3.common.torch_layers import BaseFeaturesExtractor
from stable_baselines3.common.policies import ActorCriticPolicy

class Swish(nn.Module):
    def forward(self, x):
        return x * th.sigmoid(x)

class MLAgentFeatureExtractor(BaseFeaturesExtractor):
    def __init__(self, observation_space, features_dim=256, config=None):
        super().__init__(observation_space, features_dim)
        self.config = config
        self.obs_size = 34  # adapté à ton observation

        # Track mean/std en ligne pour normaliser
        self.register_buffer("running_mean", th.zeros(self.obs_size))
        self.register_buffer("running_var", th.ones(self.obs_size))
        self.momentum = 0.01
        self.count = 0

        self.net = nn.Sequential(
            nn.Linear(self.obs_size, 256),
            Swish(),
            nn.Linear(256, 256),
            Swish(),
            nn.Linear(256, features_dim),
            Swish(),
        )

    def forward(self, observations: th.Tensor) -> th.Tensor:
        # Update running mean/var (train only)
        if self.training:
            batch_mean = observations.mean(dim=0)
            batch_var = observations.var(dim=0, unbiased=False)

            self.running_mean = (1 - self.momentum) * self.running_mean + self.momentum * batch_mean
            self.running_var = (1 - self.momentum) * self.running_var + self.momentum * batch_var
            self.count += 1

        # Normalisation
        std = th.sqrt(self.running_var + 1e-8)
        x = (observations - self.running_mean) / std

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


class MLAgentPolicyDiscrete(ActorCriticPolicy):
    def __init__(self, observation_space, action_space, lr_schedule, **kwargs):
        super().__init__(observation_space, action_space, lr_schedule, **kwargs)

        latent_dim = self.mlp_extractor.latent_dim_pi

        # Pour du discret : juste un layer qui donne les logits
        self.action_net = nn.Linear(latent_dim, action_space.n)

    def _get_action_dist_from_latent(self, latent_pi, latent_vf=None):
        action_logits = self.action_net(latent_pi)
        return self.action_dist.proba_distribution(action_logits=action_logits)

    def forward_actor(self, features):
        return self.action_net(features)

    def forward_critic(self, features):
        return self.value_net(features)