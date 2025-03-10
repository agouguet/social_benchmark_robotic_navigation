from abc import ABC, abstractmethod

class QFunction:

    """ Update the Q-value of (state, action) by delta """
    @abstractmethod
    def update(self, state, action, delta):
        pass

    """ Get a Q value for a given state-action pair """
    @abstractmethod
    def get_q_value(self, state, action):
        pass

    """ Save a policy to a specified filename """
    @abstractmethod
    def save_policy(self, filename):
        pass

    """ Load a policy from a specified filename """
    @abstractmethod
    def load_policy(self, filename):
        pass

    """ Return a pair containing the action and Q-value, where the
        action has the maximum Q-value in state
    """
    def get_max_q(self, state, actions):
        arg_max_q = None
        max_q = float("-inf")
        for action in actions:
            value = self.get_q_value(state, action)
            if max_q < value:
                arg_max_q = action
                max_q = value
        return (arg_max_q, max_q)