import numpy as np
import matplotlib.pyplot as plt


class ValueIteration:
    def __init__(self, mdp, gamma):
        self.mdp = mdp
        self.gamma = gamma
        self.values = {}
        self.full_values = {}
        self.policy = None

    def one_iteration(self):
        delta = 0
        nb_state = 0
        for state in self.mdp.get_states():
            # s = GraphState(i[0], self.mdp.goal_node, i[1])
            temp = self.values[state] if state in self.values else 0
            v_list = dict.fromkeys(self.mdp.get_actions(state), 0)
            for action in self.mdp.get_actions(state):
                for (new_state, probability) in self.mdp.get_transitions(state, action):
                    if new_state in self.values:
                        v_list[action] += self.mdp.get_reward(state, action, new_state) + self.gamma * np.sum(probability * self.values[new_state])
                    else:
                        v_list[action] += self.mdp.get_reward(state, action, new_state) 

            self.values[state] = max(v_list.values())
            self.full_values[state] = v_list
            delta = max(delta, abs(temp - self.values[state]))
            nb_state += 1
        
        return delta    

    def get_policy(self):
        pi = {}
        for state in self.mdp.get_states():
            v_list = dict.fromkeys(self.mdp.get_actions(state), 0)
            for action in self.mdp.get_actions(state):
                for (new_state, probability) in self.mdp.get_transitions(state, action):
                    if new_state in self.values:
                        v_list[action] += self.mdp.get_reward(state, action, new_state) + self.gamma * np.sum(probability * self.values[new_state])
                    else:
                        v_list[action] += self.mdp.get_reward(state, action, new_state) 
            max_index = []
            max_val = max(v_list.values())
            for action in self.mdp.get_actions(state):
                if v_list[action] == max_val:
                    max_index.append(action)
            pi[state] = np.random.choice(max_index)
        return pi

    def train(self, tol=1e-3, plot=False):
        epoch = 0
        delta = self.one_iteration()
        delta_history = [delta]
        while delta > tol:
            epoch += 1
            delta = self.one_iteration()
            delta_history.append(delta)
            print("\rDelta: {:.4f}".format(delta), end='', flush=True)
            if delta < tol:
                break
        print(" Done.")
        self.policy = self.get_policy()

        if plot is True:
            fig, ax = plt.subplots(1, 1, figsize=(3, 2), dpi=200)
            ax.plot(np.arange(len(delta_history)) + 1, delta_history, marker='o', markersize=4,
                    alpha=0.7, color='#2ca02c', label=r'$\gamma= $' + f'{self.gamma}')
            ax.set_xlabel('Iteration')
            ax.set_ylabel('Delta')
            ax.legend()
            plt.tight_layout()
            plt.show()

