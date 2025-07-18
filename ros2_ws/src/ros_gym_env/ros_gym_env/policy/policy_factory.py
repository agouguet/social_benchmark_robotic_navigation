from ros_gym_env.policy.drl_vo_cnn import DRL_VO_CNN
# from ros_gym_env.policy.srnn_old import SRNN_OLD
# from ros_gym_env.policy.srnn import SRNN
from ros_gym_env.policy.simple_policies import Scan1DCNN, SCAN2DCNN
from ros_gym_env.policy.mlagent_policy import MLAgentPolicy, MLAgentFeatureExtractor
import torch.nn as nn

def get_activation_fn(name):
    name = name.lower()
    activations = {
        "tanh": nn.Tanh,
        "relu": nn.ReLU,
        "elu": nn.ELU,
        "leakyrelu": nn.LeakyReLU,
        "sigmoid": nn.Sigmoid,
        "gelu": nn.GELU,
        "selu": nn.SELU
    }
    if name not in activations:
        raise ValueError(f"Activation '{name}' non reconnue. Choisis parmi : {list(activations.keys())}")
    return activations[name]

class Method():
    def __init__(self, policy, features_extractor_class):
        self.policy = policy
        self.features_extractor_class = features_extractor_class

    def get_policy(self, config):
        # PPO Policy
        policy_kwargs = dict(
            features_extractor_class=self.features_extractor_class,
            features_extractor_kwargs=dict(features_dim=config.policy.features_dim, config=config),
            net_arch=[dict(pi=config.policy.pi_layers, vf=config.policy.vf_layers)],
            activation_fn=get_activation_fn(config.policy.activation_fn)
        )

        return self.policy, policy_kwargs


method_factory = dict()
def none_policy():
    return None

method_factory['none'] = Method("MlpPolicy", None)
method_factory['scan'] = Method("CnnPolicy", Scan1DCNN)
method_factory['scan2d'] = Method("CnnPolicy", SCAN2DCNN)
method_factory['drl-vo'] = Method("CnnPolicy", DRL_VO_CNN)
method_factory['mlagent'] = Method(MLAgentPolicy, MLAgentFeatureExtractor)
# method_factory['srnn'] = SRNN_OLD