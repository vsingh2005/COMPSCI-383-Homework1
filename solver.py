import sys
import puzz
import pdqpq

GOAL_STATE = puzz.EightPuzzleBoard("012345678")


def solve_puzzle(start_state, flavor):
    """Perform a search to find a solution to a puzzle.
    
    Args:
        start_state (EightPuzzleBoard): the start state for the search
        flavor (str): tag that indicate which type of search to run.  Can be one of the following:
            'bfs' - breadth-first search
            'ucost' - uniform-cost search
            'greedy-h1' - Greedy best-first search using a misplaced tile count heuristic
            'greedy-h2' - Greedy best-first search using a Manhattan distance heuristic
            'greedy-h3' - Greedy best-first search using a weighted Manhattan distance heuristic
            'astar-h1' - A* search using a misplaced tile count heuristic
            'astar-h2' - A* search using a Manhattan distance heuristic
            'astar-h3' - A* search using a weighted Manhattan distance heuristic
    
    Returns: 
        A dictionary containing describing the search performed, containing the following entries:
        'path' - list of 2-tuples representing the path from the start to the goal state (both 
            included).  Each entry is a (str, EightPuzzleBoard) pair indicating the move and 
            resulting successor state for each action.  Omitted if the search fails.
        'path_cost' - the total cost of the path, taking into account the costs associated with 
            each state transition.  Omitted if the search fails.
        'frontier_count' - the number of unique states added to the search frontier at any point 
            during the search.
        'expanded_count' - the number of unique states removed from the frontier and expanded 
            (successors generated)

    """
    if flavor.find('-') > -1:
        strat, heur = flavor.split('-')
    else:
        strat, heur = flavor, None

    if strat == 'bfs':
        return BreadthFirstSolver().solve(start_state)
    elif strat == 'ucost':
        return UniformCostSolver().solve(start_state)
    elif strat == 'greedy':
        return BestFirstSolver(heuristic=heur).solve(start_state)
    elif strat == 'astar':
        return AStarSolver(heuristic=heur).solve(start_state)
    else:
        raise ValueError("Unknown search flavor '{}'".format(flavor))


class BreadthFirstSolver:
    """Implementation of Breadth-First Search based puzzle solver"""

    def __init__(self):
        self.goal = GOAL_STATE
        self.parents = {}  # state -> parent_state
        self.frontier = pdqpq.FifoQueue()
        self.explored = set()
        self.frontier_count = 0  # increment when we add something to frontier
        self.expanded_count = 0  # increment when we pull something off frontier and expand
    
    def solve(self, start_state):
        """Carry out the search for a solution path to the goal state.
        
        Args:
            start_state (EightPuzzleBoard): start state for the search 
        
        Returns:
            A dictionary describing the search from the start state to the goal state.

        """
        self.parents[start_state] = None
        self.frontier.add(start_state)
        self.frontier_count += 1

        if start_state == self.goal:
            return self.get_results_dict(start_state)

        while not self.frontier.is_empty():
            node = self.frontier.pop()
            self.explored.add(node)
            self.expanded_count += 1
            for move, succ in node.successors().items():
                if succ not in self.explored and succ not in self.frontier:
                    self.parents[succ] = node

                    # BFS checks for goal state _before_ adding to frontier
                    if succ == self.goal:
                        return self.get_results_dict(succ)
                    self.frontier.add(succ)
                    self.frontier_count += 1

        return self.get_results_dict(None)

    def get_results_dict(self, state):
        results = {
            'frontier_count': self.frontier_count,
            'expanded_count': self.expanded_count
        }
        if state:
            path = self.get_path(state)
            moves = ['start'] + [path[i - 1].get_move(path[i]) for i in range(1, len(path))]
            results['path'] = list(zip(moves, path))
            results['path_cost'] = self.get_cost(path)
        return results

    def get_path(self, state):
        """Return the solution path from the start state of the search to a target.
        
        Results are obtained by retracing the path backwards through the parent tree to the start
        state for the serach at the root.
        
        Args:
            state (EightPuzzleBoard): target state in the search tree
        
        Returns:
            A list of EightPuzzleBoard objects representing the path from the start state to the
            target state

        """
        path = []
        while state is not None:
            path.append(state)
            state = self.parents[state]
        path.reverse()
        return path

    def get_cost(self, path):
        """Calculate the path cost from start state to a target state.
        
        Transition costs between states are equal to the square of the number on the tile that 
        was moved. 

        Args:
            state (EightPuzzleBoard): target state in the search tree
        
        Returns:
            Integer indicating the cost of the solution path

        """
        cost = 0
        for i in range(1, len(path)):
            x, y = path[i - 1].find(None)
            tile = path[i].get_tile(x, y)
            cost += int(tile) ** 2
        return cost

def print_table(flav__results, include_path=False):
    """Print out a comparison of search strategy results.

    Args:
        flav__results (dictionary): a dictionary mapping search flavor tags result statistics. See
            solve_puzzle() for detail.
        include_path (bool): indicates whether to include the actual solution paths in the table

    """
    result_tups = sorted(flav__results.items())
    c = len(result_tups)
    na = "{:>12}".format("n/a")
    rows = [  # abandon all hope ye who try to modify the table formatting code...
        "flavor  " + "".join([ "{:>12}".format(tag) for tag, _ in result_tups]),
        "--------" + ("  " + "-"*10)*c,
        "length  " + "".join([ "{:>12}".format(len(res['path'])) if 'path' in res else na 
                                for _, res in result_tups ]),
        "cost    " + "".join([ "{:>12,}".format(res['path_cost']) if 'path_cost' in res else na 
                                for _, res in result_tups ]),
        "frontier" + ("{:>12,}" * c).format(*[res['frontier_count'] for _, res in result_tups]),
        "expanded" + ("{:>12,}" * c).format(*[res['expanded_count'] for _, res in result_tups])
    ]
    if include_path:
        rows.append("path")
        longest_path = max([ len(res['path']) for _, res in result_tups if 'path' in res ] + [0])
        print("longest", longest_path)
        for i in range(longest_path):
            row = "        "
            for _, res in result_tups:
                if len(res.get('path', [])) > i:
                    move, state = res['path'][i]
                    row += " " + move[0] + " " + str(state)
                else:
                    row += " "*12
            rows.append(row)
    print("\n" + "\n".join(rows), "\n")

class SearchSolver:
    def __init__(self, heuristic=None):
        self.goal = GOAL_STATE
        self.parents = {}
        self.g_scores = {}
        self.heuristic = heuristic
        self.frontier = pdqpq.PriorityQueue()
        self.explored = set()
        self.frontier_count = 0
        self.expanded_count = 0

    def solve(self, start_state):
        self.parents[start_state] = None
        self.g_scores[start_state] = 0
        self.frontier.add(start_state, self.f(start_state))
        self.frontier_count += 1

        while not self.frontier.is_empty():
            node = self.frontier.pop()

            if node == self.goal:
                return self.get_results_dict(node)

            self.explored.add(node)
            self.expanded_count += 1

            for move, succ in node.successors().items():
                cost = int(node.get_tile(*succ.find(None))) ** 2
                new_g = self.g_scores[node] + cost
                if succ not in self.g_scores or new_g < self.g_scores[succ]:
                    self.parents[succ] = node
                    self.g_scores[succ] = new_g
                    if succ not in self.explored:
                        self.frontier.add(succ, self.f(succ))
                        self.frontier_count += 1

        return self.get_results_dict(None)

    def h(self, state):
        if self.heuristic == 'h1':
            return sum(1 for i in range(1, 9) if state.find(str(i)) != GOAL_STATE.find(str(i)))
        elif self.heuristic == 'h2':
            return sum(abs(state.find(str(i))[0] - GOAL_STATE.find(str(i))[0]) +
                       abs(state.find(str(i))[1] - GOAL_STATE.find(str(i))[1])
                       for i in range(1, 9))
        elif self.heuristic == 'h3':
            return sum((abs(state.find(str(i))[0] - GOAL_STATE.find(str(i))[0]) +
                        abs(state.find(str(i))[1] - GOAL_STATE.find(str(i))[1])) * int(i) ** 2
                       for i in range(1, 9))
        return 0

    def f(self, state):
        raise NotImplementedError

    def get_results_dict(self, state):
        results = {
            'frontier_count': self.frontier_count,
            'expanded_count': self.expanded_count
        }
        if state:
            path = self.get_path(state)
            moves = ['start'] + [path[i - 1].get_move(path[i]) for i in range(1, len(path))]
            results['path'] = list(zip(moves, path))
            results['path_cost'] = self.g_scores[state]
        return results

    def get_path(self, state):
        path = []
        while state is not None:
            path.append(state)
            state = self.parents[state]
        path.reverse()
        return path


class UniformCostSolver(SearchSolver):
    def f(self, state):
        return self.g_scores[state]


class BestFirstSolver(SearchSolver):
    def f(self, state):
        return self.h(state)


class AStarSolver(SearchSolver):
    def f(self, state):
        return self.g_scores[state] + self.h(state)


def get_test_puzzles():
    test1 = puzz.EightPuzzleBoard("125340678")  # 3-5 moves
    test2 = puzz.EightPuzzleBoard("724506831")  # 10-15 moves
    test3 = puzz.EightPuzzleBoard("802356174")  # >=25 moves
    return (test1, test2, test3)


if __name__ == '__main__':

    # parse the command line args
    start = puzz.EightPuzzleBoard(sys.argv[1])
    if sys.argv[2] == 'all':
        flavors = ['bfs', 'ucost', 'greedy-h1', 'greedy-h2', 
                   'greedy-h3', 'astar-h1', 'astar-h2', 'astar-h3']
    else:
        flavors = sys.argv[2:]

    # run the search(es)
    results = {}
    for flav in flavors:
        print("solving puzzle {} with {}".format(start, flav))
        results[flav] = solve_puzzle(start, flav)

    print_table(results, include_path=False)  # change to True to see the paths!