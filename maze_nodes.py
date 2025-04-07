class Node:
    def __init__(self, position, name, path, walls):
        
        self.x = position[0]
        self.y = position[1]
        self.path = path # "NNSSENWW -> algo assim"
        self.walls = walls
        self.neighbours = []
        self.name = name


    def __repr__(self):
        return f"Node[({self.x},{self.y}) ; {self.path} ; {self.neighbours} ; {self.walls}]"

    def __str__(self):
        return f"Node[({self.x}, {self.y}) ; {self.path} ; {self.neighbours} ; {self.walls}]"
    
