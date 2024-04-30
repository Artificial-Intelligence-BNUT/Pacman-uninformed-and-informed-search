
"""
In search.py, you will implement generic search algorithms which are called by
Pacman nodes (in searchnodes.py).
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
    from util import Stack
    from util import Queue
    from search import SearchProblem
    from pacman import GameState

    nodes = Stack

    actions = Stack

    nodes.push(SearchProblem.getStartState(GameState))
    
    while(not nodes.isEmpty()):

        node = nodes.pop()

        GameState.getLegalActions(node)

        actions.push(node.action)

        if(SearchProblem.isGoalState(GameState,node)):
            return actions
        
        successors = Queue
        
        successors.push(SearchProblem.getSuccessors(nodes.pop()))

        while(successors.isEmpty()):
            actions.pop()
            tempQueue = Queue(SearchProblem.getSuccessors(nodes.pop()))
            for temp in tempQueue:
                if(temp not in GameState.explored):
                    successors.push(temp)

        while(not successors.isEmpty()):
            nodes.push(successors.pop())
            
    return False

def breadthFirstSearch(problem):
    from util import Stack
    from util import Queue
    from search import SearchProblem
    from pacman import GameState

    nodes = Queue    

    actions = Stack

    def bfs(node,actions):

        actions = Stack(actions)

        if(node in GameState.explored):
            return None
        
        GameState.getLegalActions(node)

        actions.push(node.action)

        if(SearchProblem.isGoalState(GameState,node)):
            return actions
        
        frontier = Queue
        
        frontier.push(SearchProblem.getSuccessors(nodes.pop()))

        while(not frontier.isEmpty()):
            actionsResult = Stack(bfs(frontier.pop(),actions))
            if(not actions.isEmpty()):
                return actionsResult
            
        return None
            
    return bfs(SearchProblem.getStartState(GameState),actions)

def uniformCostSearch(problem):
    from util import PriorityQueue
    from util import Queue
    from util import Stack
    from search import SearchProblem
    from pacman import GameState

    nodesQueue = PriorityQueue

    actionsQueue = PriorityQueue

    actions = Stack

    nodes = Queue

    def ucs(node,nodes,actions):

        nodes = Queue(nodes)
        
        GameState.getLegalActions(node)

        actions = Stack(actions)

        if(node in GameState.explored):
            return None

        actions.push(node.action)

        nodes.push(node)

        actionsQueue.update(actions, SearchProblem.getCostOfActions(actions)*-1)

        nodesQueue.update(nodes, SearchProblem.getCostOfActions(actions)*-1)

        tempActions = Stack(actionsQueue.pop())

        actionsQueue.push(tempActions, SearchProblem.getCostOfActions(tempActions))

        tempNodes = Stack(nodesQueue.pop())
        
        nodesQueue.push(tempNodes, SearchProblem.getCostOfActions(tempActions)*-1)

        tempNode = tempNodes.pop()

        tempNodes.push(tempNode)

        if(SearchProblem.isGoalState(GameState,tempNode)):
            return tempNodes
        
        frontier = PriorityQueue

        tempQueue = Queue

        tempQueue.push(SearchProblem.getSuccessors(nodes.pop()))

        for temp in tempQueue:
            frontier.push(temp,temp.stepCost*-1)
        
        while(not frontier.isEmpty()):
            actionsResult = Stack(ucs(frontier.pop(),actions,GameState))
            if(not actionsResult.isEmpty()):
                return actionsResult
            
        return None
            
    return ucs(SearchProblem.getStartState(GameState),nodes,actions)

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def myHeuristic(state, problem):
    
    return ((state[x] - problem[x])**2 + (state[y] - problem[y])**2)**0.5

"""---------------------------------------------------------------------------------------------"""


def aStarSearch(problem, heuristic=nullHeuristic):
    from util import PriorityQueue
    from util import Queue
    from util import Stack
    from search import SearchProblem
    from pacman import GameState

    nodesQueue = PriorityQueue

    actionsQueue = PriorityQueue

    actions = Stack

    nodes = Queue

    def astar(node,nodes,actions):

        nodes = Queue(nodes)

        actions = Stack(actions)

        if(node in GameState.explored):
            return None
        
        GameState.getLegalActions(node)

        nodeHeuristic = myHeuristic(node, GameState)

        actions.push(node.action)

        nodes.push(node)

        actionsQueue.update(actions, SearchProblem.getCostOfActions(actions)*-1 - nodeHeuristic)

        nodesQueue.update(nodes, SearchProblem.getCostOfActions(actions)*-1 - nodeHeuristic)

        tempNodes = Stack(nodesQueue.pop())

        tempNode = tempNodes.pop()

        tempNodes.push(tempNode)

        tempHeuristic = myHeuristic(tempNode, GameState)

        tempActions = Stack(actionsQueue.pop())

        actionsQueue.push(tempActions, SearchProblem.getCostOfActions(tempActions)*-1 - tempHeuristic)
        
        nodesQueue.push(tempNodes, SearchProblem.getCostOfActions(tempActions)*-1 - tempHeuristic)

        if(SearchProblem.isGoalState(GameState,tempNode)):
            return tempActions
        
        frontier = PriorityQueue

        tempQueue = Queue

        tempQueue.push(SearchProblem.getSuccessors(nodes.pop()))

        for temp in tempQueue:
            frontier.push(temp,temp.stepCost*-1 - myHeuristic(temp, GameState))
        
        while(not frontier.isEmpty()):
            actionsResult = Stack(astar(frontier.pop(),actions,GameState))
            if(not actionsResult.isEmpty()):
                return actionsResult
            
        return None
            
    return astar(SearchProblem.getStartState(GameState),nodes,actions)

# util | pacman | game
# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch