import json


class State():
    def __init__(self, robot_state, humans):
        self.robot = robot_state
        self.humans = humans

    @property
    def robot(self):
        return self._robot
    
    @robot.setter
    def robot(self, robot_state):
        self._robot = robot_state

    def __str__(self):
        return "s" + "(" + str(self.robot) + ", " + str(self.humans) + ")"
    
    def __repr__(self) -> str:
        return self.__str__()

    def __eq__(self, other):
        return other != None and self.robot == other.robot and self.humans == other.humans

    def __hash__(self):
        return hash((self.robot, tuple(self.humans)))

