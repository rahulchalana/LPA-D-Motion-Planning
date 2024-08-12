import math
import numpy as np

class PriorityQueue(object):
    def __init__(self):
        self.queue = []
 
    def __str__(self):
        return ' '.join([str(i) for i in self.queue])
 
    # for checking if the queue is empty
    def isEmpty(self):
        return len(self.queue) == 0
 
    # for inserting an element in the queue
    def append(self, data):
        self.queue.append(data)
 
    # for popping an element based on Priority
    def pop(self):
        try:
            min_val = 0
            for i in range(len(self.queue)):
                if self.queue[i] < self.queue[min_val]:
                    min_val = i
            item = self.queue[min_val]
            del self.queue[min_val]
            return item
        except IndexError:
            print()
            exit()



class DStarLite:

    def __init__(self,  map):
        self.map = map
        self.start = map.start
        self.path = []
        self.frindge = PriorityQueue()
        self.parent = {}
        self.closed = []
        self.moves = [[-1,0],  [0,1], [0,-1],  [1,0]]

    def heuristic(self, cur,goal):
        x = abs(goal[0]-cur[0])+ abs(goal[1]-cur[1])
        return x/3
        ## found this heuristic to be the best consistent for this problem.

    def DStarSearch(self, robot_state, bills, first_path):
        print("D* Search")
        state = np.rint(robot_state[0:2])
        print(state)
        
        goal = map.goal
        if not first_path :
            self.parent.clear()
            self.closed.clear()
            self.fringe = PriorityQueue()

            ## find the index i of state in the path
            i = 0
            for i in range(len(self.path)):
                if self.path[i] == state:
                    break
            n = len(self.path)
            if i + 4 >= n :
                i = n - 1
            else : 
                i = i + 4

            self.path = self.path[i:n]
            goal = self.path[0]
    
        self.parent[hash(tuple(goal))] = [[-1,-1], 0]
        self.fringe.append([0,goal])
        

        while not self.fringe.isEmpty():
            current = self.fringe.pop()
            if hash(tuple(current[1])) in self.closed:
                continue

            self.closed.append(hash(tuple(current[1])))

            
            for move in self.moves:
                new = [current[1][0]+move[0], current[1][1]+move[1]]

                busy = False
                if not first_path :
                    for i in range(len(bills)):
                        bill =np.array( bills[i])
                        if np.linalg.norm(new-bill) <= 1 : 
                            busy = True
                if busy and not first_path :
                    continue

                if new[0] >= 0 and new[0] < len(map.map_data) and new[1] >= 0 and new[1] < len(map.map_data[0]) and map.map_data[new[0]][new[1]] != 3:
                    new_cost = self.parent[hash(tuple(current[1]))][1] + 1
                    if hash(tuple(new)) not in self.closed or new_cost < self.parent[hash(tuple(new))][1]:
                        self.parent[hash(tuple(new))] = [current[1], new_cost]
                        self.fringe.append([new_cost + self.heuristic(new, state), new])

        new_path = []
        while True:
            new_path.append(state)
            if hash(tuple(state)) == hash(tuple(map.goal)):
                break
            state = self.parent[hash(tuple(state))][0]
            

        self.path = new_path + self.path
        return self.path