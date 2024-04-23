
"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""
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
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


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
    def DFS(root, branchActions):
        branchActions.add(root.action)

        if(root.isGoalState(root)):
            return branchActions
             
        leaves = root.getSuccessors(root)

        if(leaves is None):
            return None        
        for leaf in leaves:
            branchActions = DFS(leaf, branchActions)
            if(branchActions is not None):
                return branchActions
    
    return DFS(problem.getStartState(), ["""empty"""])

def breadthFirstSearch(problem):
    def BFS(root, leafActions):
        leafActions.add(root.action)
        leaves = root.getSuccessors(root)

        if(leaves is None):
            return None
        
        for leaf in leaves:
            if(leaf.isGoalState(leaf)):
                return leafActions
            
        for leaf in leaves:
            leafActions = BFS(leaf, leafActions)
            if(leafActions is not None):
                return leafActions
    
    return BFS(problem.getStartState, ["""empty"""])

def uniformCostSearch(problem):
    tuple = ["""goalPathNumbers"""]["""0 for action""""""1 for cost"""]
    goalPathNumbers = 0
    tuple[goalPathNumbers][0] = ["""illegal"""]
    tuple[goalPathNumbers][1] = ["""infinity"""]

    def USC(root, branchActions):
        if(branchActions.getCostOfActions(branchActions) > tuple[goalPathNumbers][1]):
            return None
            
        branchActions.add(root.action)

        if(root.isGoalState(root)):
            tuple[goalPathNumbers][0] = branchActions
            tuple[goalPathNumbers][1] = branchActions.getCostOfActions(branchActions)
            goalPathNumbers += 1
            return
             
        leaves = root.getSuccessors(root)
        for leaf in leaves:
            branchActions = USC(leaf, branchActions)
            
    USC(problem.getStartState(), ["""empty"""])

    for number in range(1,goalPathNumbers):
        if(tuple[0][1] > tuple[number][1]):
            tuple[0] = tuple[number]
        
    return tuple[0][0]

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def myAStarHeuristic(state, problem):
    return problem.goal - state

"""---------------------------------------------------------------------------------------------"""

def aStarSearch(problem, heuristic=nullHeuristic):        
    tuple = ["""goalPathNumbers"""]["""0 for action""""""1 for cost"""]
    goalPathNumbers = 0
    tuple[goalPathNumbers][0] = ["""illegal"""]
    tuple[goalPathNumbers][1] = ["""infinity"""]

    def AStar(root, branchActions):
        if(branchActions.getCostOfActions(branchActions) > tuple[goalPathNumbers][1]):
            return None
            
        branchActions.add(root.action)

        if(root.isGoalState(root)):
            tuple[goalPathNumbers][0] = branchActions
            tuple[goalPathNumbers][1] = branchActions.getCostOfActions(branchActions)
            goalPathNumbers += 1
            return
             
        leaves = root.getSuccessors(root)

        for leaf in leaves:
            branchActions = AStar(leaf, branchActions)
            
    AStar(problem.getStartState(), ["""empty"""])

    for number in range(1,goalPathNumbers):
        if(tuple[0][1] > tuple[number][1]):
            tuple[0] = tuple[number]
        
    return tuple[0][0]


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
