# #!/usr/bin/env python

from typing import Optional
import torch
import torch.nn as nn
import torch.nn.functional as F
import gym
from stable_baselines3.common.torch_layers import BaseFeaturesExtractor

class Scan1DCNN(BaseFeaturesExtractor):
    def __init__(self, observation_space: gym.spaces.Box, features_dim: int = 256, config: Optional[dict] = None):
        super().__init__(observation_space, features_dim)

        self.config = config
        self.scan_dim = self.config.env.obs.scan_dim
        goal_dim = observation_space.shape[0] - self.scan_dim 

        self.cnn1d = nn.Sequential(
            nn.Conv1d(1, 32, kernel_size=8, stride=4),  # output: (32, ~1598)
            nn.ReLU(),
            nn.Conv1d(32, 64, kernel_size=4, stride=2),  # output: (64, ~798)
            nn.ReLU(),
            nn.Flatten()
        )

        # Calculate output shape dynamically
        with torch.no_grad():
            dummy_input = torch.zeros((1, 1, self.scan_dim))
            conv_out_size = self.cnn1d(dummy_input).shape[1]

        # Final linear layer with goal concat
        self.linear = nn.Sequential(
            nn.Linear(conv_out_size + goal_dim, features_dim),
            nn.ReLU()
        )

    def forward(self, observations: torch.Tensor) -> torch.Tensor:
        scan = observations[:, :self.scan_dim].unsqueeze(1)  # (batch, 1, 6400)
        goal = observations[:, self.scan_dim:]               # (batch, 2)

        x = self.cnn1d(scan)                        # (batch, features)
        x = torch.cat((x, goal), dim=1)             # concat with goal
        x = self.linear(x)
        return x
    

class SCAN2DCNN(BaseFeaturesExtractor):
    def __init__(self, observation_space: gym.spaces.Box, features_dim: int = 256):
        super(SCAN2DCNN, self).__init__(observation_space, features_dim)

        # Taille d'entrée attendue (80x80), 1 canal
        self.cnn = nn.Sequential(
            nn.Conv2d(1, 32, kernel_size=5, stride=2, padding=2),  # -> 40x40
            nn.ReLU(),
            nn.Conv2d(32, 64, kernel_size=3, stride=2, padding=1),  # -> 20x20
            nn.ReLU(),
            nn.Conv2d(64, 128, kernel_size=3, stride=2, padding=1),  # -> 10x10
            nn.ReLU(),
            nn.AdaptiveAvgPool2d((1, 1))  # -> (128, 1, 1)
        )

        self.linear = nn.Sequential(
            nn.Linear(128 + 2, features_dim),
            nn.ReLU()
        )

    def forward(self, observations: torch.Tensor) -> torch.Tensor:
        # Split observation: [0:6400] = 80x80 scan, [6400:] = 2D goal
        scan = observations[:, :6400].reshape(-1, 1, 80, 80)
        goal = observations[:, 6400:]

        # Downsample to 40x40 for speed
        scan = F.interpolate(scan, size=(40, 40), mode='bilinear', align_corners=False)

        # CNN branch
        x = self.cnn(scan)
        x = x.view(x.size(0), -1)  # Flatten

        # Concatenate goal info
        x = torch.cat((x, goal), dim=1)

        # Final fully connected layer
        x = self.linear(x)
        return x