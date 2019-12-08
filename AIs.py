import heapq, time
from Cube import *
from Heuristic import *

class BFS:
    def __init__(self, cube):
        self.cube = cube

    def solve(self, timeout=float('inf')):
        start_time = time.time()
        goal_state = Cube(self.cube.size).__hash__()
        depth = 0
        if self.cube.__hash__() == goal_state:
            print('Found goal at depth ' + str(depth))
            return [(None, self.cube)]

        seen = {}
        seen[self.cube.__hash__()] = (self.cube, None, None) #Current cube, parent cube, move from parent to current
        fringe = {}
        fringe[self.cube.__hash__()] = (self.cube, None, None) 

        while True:
            if time.time() - start_time >= timeout:
                print('time: ' + str(time.time()))
                raise Exception('Code timed out')

            depth += 1
            print('Depth: ' + str(depth) + ', length of fringe: ' + str(len(fringe)) + '; len seen: ' + str(len(seen)))
            print('time: ' + str(time.time()) + '; overlaped time: ' + str(time.time()-start_time))

            new_fringe = {}
            for i in fringe:
                if time.time() - start_time >= timeout:
                    print('time: ' + str(time.time()))
                    raise Exception('Code timed out')
                
                for j in fringe[i][0].children('all'):
                    if j[1].__hash__() == goal_state:
                        print('Found goal at depth ' + str(depth))
                        return self.find_path(seen, (j[1], fringe[i][0], j[0], -1))
                    if j[1].__hash__() not in fringe and j[1].__hash__() not in seen:
                        new_fringe[j[1].__hash__()] = (j[1], fringe[i][0], j[0])
                        seen[j[1].__hash__()] = (j[1], fringe[i][0], j[0])
            fringe = new_fringe

    def find_path(self, seen, goal_state):
        last_state = goal_state
        path = [ (last_state[2], last_state[0]) ]
        last_state = seen[last_state[1].__hash__()]

        while last_state != None:
            path = [ (last_state[2], last_state[0]) ] + path
            if last_state[1] == None:
                return path
            last_state = seen[last_state[1].__hash__()]

        return path

class Better_BFS:
    def __init__(self, cube):
        self.cube = cube

    def solve(self, timeout=float('inf')):
        start_time = time.time()
        goal_state = Cube(self.cube.size).__hash__()
        depth = 0
        if self.cube.__hash__() == goal_state:
            print('Found goal at depth ' + str(depth))
            return [(None, self.cube)]

        seen = {}
        seen[self.cube.__hash__()] = (self.cube, None, None, -1) #Current cube, parent cube, move from parent to current, move from parent of parent to parent
        fringe = {}
        fringe[self.cube.__hash__()] = (self.cube, None, None, -1) 

        while True:
            if time.time() - start_time >= timeout:
                print('time: ' + str(time.time()))
                raise Exception('Code timed out')

            depth += 1
            print('Depth: ' + str(depth) + ', length of fringe: ' + str(len(fringe)) + '; len seen: ' + str(len(seen)))
            print('time: ' + str(time.time()) + '; overlaped time: ' + str(time.time()-start_time))

            new_fringe = {}
            for i in fringe:
                if time.time() - start_time >= timeout:
                    print('time: ' + str(time.time()))
                    raise Exception('Code timed out')

                for j in fringe[i][0].children('2x'):
                    if j[1].__hash__() == goal_state:
                        print('Found goal at depth ' + str(depth))
                        return self.find_path(seen, (j[1], fringe[i][0], j[0], -1))
                    if j[0][0] == fringe[i][3]:
                        continue
                    if j[1].__hash__() not in fringe and j[1].__hash__() not in seen:
                        new_fringe[j[1].__hash__()] = (j[1], fringe[i][0], j[0], j[0][0])
                        seen[j[1].__hash__()] = (j[1], fringe[i][0], j[0], j[0][0])
            fringe = new_fringe

    def find_path(self, seen, goal_state):
        last_state = goal_state
        path = [ (last_state[2], last_state[0]) ]
        last_state = seen[last_state[1].__hash__()]

        while last_state != None:
            path = [ (last_state[2], last_state[0]) ] + path
            if last_state[1] == None:
                return path
            last_state = seen[last_state[1].__hash__()]

        return path

class A_Star:
    def __init__(self, cube, heuristic=Heuristic.manhattanDistance):
        self.cube = cube
        self.heuristic = heuristic

    def solve(self, timeout=float('inf')):
        start_time = time.time()
        start_state = State(self.cube, None, 0, 0, None)
        goal_state = State(Cube(self.cube.size), None, 0, 0, None)
        explored = set()
        fringe = [start_state]
        heapq.heapify(fringe)

        print("starting solve")
        while len(fringe) > 0:
            if time.time() - start_time >= timeout:
                print('time: ' + str(time.time()))
                raise Exception('Code timed out')

            current_state = heapq.heappop(fringe)
            #print(current_state)
            if current_state.current_state.isSolved():
                return self.find_path(start_state, current_state)
            if current_state.__hash__() in explored:
                continue
            for i in current_state.current_state.children('2x'):
                if i.__hash__() not in explored:
                    new_addition = State(i[1], current_state, current_state.depth+1+self.heuristic(i[1]), current_state.depth+1, i[0])
                    heapq.heappush(fringe, new_addition)
                    explored.add(current_state.__hash__())

    def find_path(self, start_state, end_state):
        last_state = end_state
        path = [ [last_state.move, last_state.current_state] ]
        last_state = last_state.parent_state

        while last_state != None and start_state.current_state.__hash__() != path[0][1].__hash__():
            path = [ [last_state.move, last_state.current_state] ] + path
            last_state = last_state.parent_state

        return path

class IDA_Star:
    def __init__(self, cube, heuristic=Heuristic.manhattanDistance):
        self.cube = cube
        self.heuristic = heuristic

    def solve(self, timeout=float('inf')):
        start_time = time.time()
        bound = self.heuristic(self.cube)
        path = [(None, self.cube)] #(move, cube)
        while True:
            if time.time() - start_time >= timeout:
                print('time: ' + str(time.time()))
                raise Exception('Code timed out')

            print('Path len: ' + str(len(path)) + '; bound: ' + str(bound) + '; path head: (' + str(path[-1][0]) + ', ' + str(path[-1][1].state) + ')')

            t = self.search(path, 0, bound)
            if t[0]:
                return path
            if t[2] == float('inf'):
                return []

            bound = t[2]

    def search(self, path, g, bound, times=(float('inf'),0)):
        node = path[-1][1]
        f = g + self.heuristic(node)
        #print("ida* f :", f)
        #print("ida* bound : ", bound)
        if f > bound:
            return False, path, f
        if node.isSolved():
            return True, path, f
        min_val = False, path, float('inf')
        for succ in node.children('2x'):
            if time.time() - times[1] >= times[0]:
                print('time: ' + str(time.time()))
                raise Exception('Code timed out')

            if succ[1] not in path:
                path.append(succ)
                t = self.search(path, g+1, bound)
                if t[0]:
                    return t
                if t[2] < min_val[2]:
                    min_val = t[0], path, t[2]
                del path[-1]
        return min_val

class State:
    def __init__(self, current_state, parent_state, fValue, depth, move):
        self.current_state = current_state
        self.parent_state = parent_state
        self.fValue = fValue
        self.depth = depth
        self.move = move

    def __eq__(self, other):
        if self.current_state == other:
            return True
        return False

    def __lt__(self, other):
        return self.fValue < other.fValue

    def __bool__(self):
        return True

    def __hash__(self):
        return self.current_state.__hash__()

    def __str__(self):
        return "depth:" + str(self.depth) + "; fValue:" + str(self.fValue) + "; current_state:" + str(self.current_state.__hash__()) + '; move:' + str(self.move) + '; solved:' + str(self.current_state.isSolved())