Name: Vansh Singh


References:
    1. Python heapq & priority queues (used for pdqpq.PriorityQueue):  
       https://docs.python.org/3/library/heapq.html
    
    2. Stuart Russell and Peter Norvig, “Artificial Intelligence: A Modern Approach,” 4th ed., Pearson, 2020.  
       – Chapters on uninformed and informed search (Uniform‐Cost, A*, heuristics).  
       https://aima.cs.berkeley.edu/  


Notes/Warnings:
    1. Greedy best-first with h3 (weighted Manhattan) sometimes expands very few nodes, but can return highly sub-optimal solutions; 
    careful if cost-optimality is needed.
    2. Getting the print_table function to output neatly was probably one of the harder aspects to be honest.
    3. Error handling for impossible 8-puzzles like 123456870. The program will return n/a when no path is found. You can also double check this by
        check 'expanded nodes' since it'll show 181440 nodes expanded, which is 9! (aka every single path checked).


Reflection Questions (using the ≥25‐move test, start=802356174):
    1. Optimality
        a. UCS, astar[h1, h2, h3] all retyrned the same cost of 692 for the >25 move puzzle. BFS, Greedy h1, h2, and h3 all gave higher costs. 
        This confirms that UCS and astar are the most cost optimal. On the other hand, BFS, which optimizes move count, don't guarantee minimal path cost.

    2. Workload
        a. MOST WORK: astar-h1 expanded 176723 states; ucs expanded 177783 states; bfs expanded 140495.
        b. LEAST WORK: greedy-h2 expanded only 338 states; greedy-h1 expanded 975 states, and greedy-h3 expanded 626 states.


    3. Heuristic Impact on A*
        a. When we move up in heuristic value for astar, the number of expanded nodes drop by a large amount:
                ar-h1 -> 176723
                astar-h2 -> 175478
                astar-h3 -> 3590
            This is because a stronger heuristic value prunes more of the search tree by raising f = g+ h, causing fewer states to look promising and get enqueued.