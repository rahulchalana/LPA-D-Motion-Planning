import math
import numpy as np

path = []
moves = [[-1,0],  [0,1], [0,-1],  [1,0] ]
# moves = [[1,1], [-1,-1],  [-1,1], [1,-1], [-1,0],  [0,1], [0,-1],  [1,0]]

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

def heuristic(cur,goal):
    x = abs(goal[0]-cur[0])+ abs(goal[1]-cur[1])
    return x/3
    ## found this heuristic to be the best consistent for this problem.
    
def AStarSearch(robot_state, map, bills):
    print("A* Search")
    state = np.rint(robot_state[0:2])
    print(state)
    goal = map.goal
    closed = []
    fringe = PriorityQueue()
      
    fringe.append((0+heuristic(state,goal), state, []))
    
    while not fringe.isEmpty() :
        (pc, par, direc) = fringe.pop()
        pc -= heuristic(par,goal) 

        if par[0] == map.goal[0] and par[1] == map.goal[1] :
            return direc
    
        if par in closed :
          continue

        closed.append(hash(str(par)))
    
        for move in moves :
          nd = direc.copy()
          child  = [int(move[0]+ par[0]), int(move[1]+ par[1])]
          if child[0] < 0 or child[0] >= len(map.map_data[0]) or child[1] < 0 or child[1] >= len(map.map_data) :
            continue
          if map.map_data[child[0]][child[1]] == map.obstacle_id :
            continue
          
          busy = False
          for i in range(len(bills)):
            bill =np.array( bills[i])
            if np.linalg.norm(child-bill) <= 1 :
               busy = True

          if busy :
             continue
            
          nd.append(child)
          cost = math.sqrt(move[0]**2 + move[1]**2)
    
          if not hash(str(child)) in closed :
            fringe.append((cost + pc + heuristic(child,goal) ,child, nd))
    return []

def get_path(robot_state, map, bill_state) :
    path = AStarSearch(robot_state, map, bill_state)
    return path