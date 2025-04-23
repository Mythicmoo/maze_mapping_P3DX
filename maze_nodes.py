from collections import deque

class Node:
    def __init__(self, position, path, walls=None):
        
        self.x = position[0]
        self.y = position[1]
        self.path = path # "NNSSENWW -> algo assim"
        self.walls = walls
        self.neighbours = set()
        self.name = f"Node({self.x},{self.y})"
    

    def __repr__(self):
        return f"Node[({self.x},{self.y}) ; {self.path} ; {self.neighbours} ; {self.walls}]"

    def __str__(self):
        return f"Node[({self.x}, {self.y}) ; {self.path} ; {self.neighbours} ; {self.walls}]"
    

class Dfso:
    def __init__(self):
        self.visitedStates = []
        self.visibleStates = deque()
        self._mappedStates = None
        self.actualState = None

    def set_mappedStates(self):
        self._mappedStates = set(self.visitedStates) | set(self.visibleStates)

    def get_mappedStates(self):
        return self._mappedStates
    
