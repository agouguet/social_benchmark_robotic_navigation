import json
from collections import defaultdict
from hbsn.solver.qfunction import QFunction


class QTable(QFunction):
    def __init__(self, default=0.0):
        self.qtable = defaultdict(lambda: default)

    def update(self, state, action, delta):
        self.qtable[(state, action)] = self.qtable[(state, action)] + delta

    def get_q_value(self, state, action):
        return self.qtable[(state, action)]

    def save(self, filename):
        with open(filename, "w") as file:
            serialised = {str(key): value for key, value in self.qtable.items()}
            json.dump(serialised, file)

    def load(self, filename, default=0.0):
        with open(filename, "r") as file:
            serialised = json.load(file)
            self.qtable = defaultdict(
                lambda: default,
                {tuple(eval(key)): value for key, value in serialised.items()},
            )
