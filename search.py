
"""
In search.py, you will implement generic search algorithms which are called by
Pacman nodes (in searchnodes.py).
"""
#I'm sorry I coppied "PetropoulakisPanagiotis" solution as I couldn't for the life of me figure out how to use any of the classes
#In less than 2 weeks, I'll make this repository private when I am done with the project
import util

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem):
    from util import Stack

    nodes = Stack()

    visited = []

    path = []

    if problem.isGoalState(problem.getStartState()):
        return []
    
    nodes.push((problem.getStartState(),[]))

    while(True):
        if nodes.isEmpty():
            return []
        
        coordinates,path = nodes.pop() 
        visited.append(coordinates)

        if problem.isGoalState(coordinates):
            return path
        
        successors = problem.getSuccessors(coordinates)

        if successors:
            for successor in successors:
                if successor[0] not in visited and successor[0] not in (state[0] for state in nodes.list):
                  if problem.isGoalState(successor[0]):
                      return path + [successor[1]]

                newPath = path + [successor[1]]
                
                nodes.push((successor[0],newPath))

def breadthFirstSearch(problem):
    from util import Queue

    nodes = Queue()

    visited = [] 
    path = []

    if problem.isGoalState(problem.getStartState()):
        return []
    
    nodes.push((problem.getStartState(),[]))

    while(True):
        
        if nodes.isEmpty():
            return []
        
        coordinates,path = nodes.pop()

        visited.append(coordinates)
        if problem.isGoalState(coordinates):
            return path
        successors = problem.getSuccessors(coordinates)

        # Add new states in queue and fix their path #
        if successors:
            for successor in successors:
                if successor[0] not in visited and successor[0] not in (state[0] for state in nodes.list):
                    if problem.isGoalState(successor[0]):
                      return path + [successor[1]]

                    newPath = path + [successor[1]]

                    nodes.push((successor[0],newPath))

def uniformCostSearch(problem):
    from util import PriorityQueue

    nodes = PriorityQueue()

    visited = []
    path = []

    if problem.isGoalState(problem.getStartState()):
        return []
    
    nodes.push((problem.getStartState(),[]),0)

    while(True):
        if nodes.isEmpty():
            return []

        coordinates,path = nodes.pop()

        visited.append(coordinates)

        if problem.isGoalState(coordinates):
            return path

        successors = problem.getSuccessors(coordinates)

        if successors:
            for successor in successors:
                if successor[0] not in visited and (successor[0] not in (state[2][0] for state in nodes.heap)):

                    newPath = path + [successor[1]]
                    price = problem.getCostOfActions(newPath)

                    nodes.push((successor[0],newPath),price)

                elif successor[0] not in visited and (successor[0] in (state[2][0] for state in nodes.heap)):
                    for state in nodes.heap:
                        if state[2][0] == successor[0]:
                            oldPri = problem.getCostOfActions(state[2][1])

                    newPrice = problem.getCostOfActions(path + [successor[1]])

                    if oldPri > newPrice:
                        newPath = path + [successor[1]]
                        nodes.update((successor[0],newPath),newPrice)

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

from util import PriorityQueue #I did not think about this
class MyPriorityQueueWithFunction(PriorityQueue):
    def  __init__(self, problem, priorityFunction):
        self.priorityFunction = priorityFunction      
        PriorityQueue.__init__(self)        
        self.problem = problem
    def push(self, successor, heuristic):
        PriorityQueue.push(self, successor, self.priorityFunction(self.problem,successor,heuristic))
def f(problem,state,heuristic):
    return problem.getCostOfActions(state[1]) + heuristic(state[0],problem)

def aStarSearch(problem, heuristic=nullHeuristic):
    nodes = MyPriorityQueueWithFunction(problem,f)

    path = [] 
    visited = []

    if problem.isGoalState(problem.getStartState()):
        return []

    element = (problem.getStartState(),[])

    nodes.push(element,heuristic)

    while(True):
        if nodes.isEmpty():
            return []
        
        coordinates,path = nodes.pop()

        if coordinates in visited:
            continue

        visited.append(coordinates)

        if problem.isGoalState(coordinates):
            return path
        
        successors = problem.getSuccessors(coordinates)

        if successors:
            for successor in successors:
                if successor[0] not in visited:

                    newPath = path + [successor[1]]
                    element = (successor[0],newPath)
                    nodes.push(element,heuristic)    

bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch

# Editor:
# Sdi1500129
# Petropoulakis Panagiotis