import numpy as np
from queue import Queue
from copy import deepcopy

def centrisity(A, dists=None, paths=None):
    '''Find centrisity measure.  

    Returns np.array of size "len(A)" where centrisity measure for each node  
    is stored.

    Arguments:
    A     -- adjacency matrix of count, can be either list(list(int)) or  
    np.array(np.array(integer))  
    dists -- can be None or list(list(int)) or np.array(np.array(int)).  
    Needs to be same size as A. Used to store distances of shortest  
    paths. If None, distances are not saved. Default to None.  
    paths -- can be None or list(list). Needs to be same size as len(A). 
    Used to store shortest paths. If None, paths are not saved.  
    Default to None.  

    '''

    def _dijkstra(start_node):
        def _relax_node(node):
            for i in range(n):
                if i == node:
                    continue
                if A[relax_node][i] != -1:
                    new_dist = dists[node] + A[node][i]
                    if new_dist < dists[i]:
                        parents[i] = node                   # Save information to restore paths.
                        dists[i] = new_dist
            # Once relaxed nodes are never meant to be used again.  
            used[node] = True
        
        def _min_distanced_node():
            idx = -1
            for i in range(n):
                if used[i] or dist[i] == float('inf'):
                    continue
                elif idx == -1 or dist[i] < dist[idx]:
                    idx = i
            return idx
        
        used = [False for _ in range(n)]
        dists = [float('inf') for _ in range(n)]
        dists[start_node] = 0
        parents = [-1 for _ in range(n)]
        paths = [[] for _ in range(n)]
        
        # Relax all nodes until we process them all.  
        _relax_node(start_node)
        for _ in range(n-1):
            node_i = _min_distanced_node()
            if node_i == -1:
                break
            _relax_node(node_i)
            
        # Restoring paths.  
        for i in range(n):
            last_node = parents[i]
            if last_node == -1:
                continue

            while last_node != start_node:
                paths[i].append(last_node)
                last_node = parents[last_node]
            paths[i].append(start_node)
        # Restored paths are in backward order. One needs to reverse them.
        paths = [[x for x in reversed(path)] for path in paths]
        
        return np.array(dists),np.array(paths)
    
    n = len(A)
    result = np.array([0 for _ in range(n)],dtype=np.float64)
    save_dists = type(dists) == type(np.array(0)) or \
                   type(dists) == type(list())
    save_paths = type(paths) == type(np.array(0)) or \
                   type(paths) == type(list())
    paths_no = 0
    
    # Idea: process Dijkstra for each node and find amount of   
    # shortest paths, then count every node in every path.  
    for i in range(n):
        dists_i,paths_i = _dijksta(i)
        # One don't need to count way from the node to itself.  
        # So we subtract that path from total count.  
        paths_no += len(np.where(dists_i < float('inf'))[0]) - 1

        for path in paths:
            for node in path:
                result[node] += 1

        # Save distances and/or paths, if needed.  
        if save_dists:
            dists[i][j] = deepcopy(dists_i)
        if save_paths:
            for j in range(n):
                paths[i][j] = deepcopy(paths_i[j])
                
    # Normalizing results.  
    result /= paths_no

    return res
