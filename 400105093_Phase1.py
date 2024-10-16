from haversine import haversine


#find closest point to the given location and return id
def closest_ID(loc,nodes,graph):
  min_distance = 1000000
  ret_val = None
  for node in nodes:
    tmploc = graph[graph['u'] == node ]
    #if given point is not a start point for street, it should be end point of a street. 
    if tmploc.shape[0]==0: 
      tmploc = graph[graph['v'] == node ]
      tmploc = tmploc.iloc[0]
      tmploc = [tmploc['v_lat'],tmploc['v_lon']]
    else:
      tmploc = tmploc.iloc[0]
      tmploc = [tmploc['u_lat'],tmploc['u_lon']]  
      
    #using haversine to compute distance 
    tmp = haversine(loc,tmploc)
    if tmp < min_distance:
      min_distance = tmp
      ret_val = node
  return ret_val




# TODO: You should implement this from scratch and you cannot use any library (such as networkx) for finding the shortest path.
def find_shotest_path(start_node, end_node, graph):

    """
    Find the shortest path between two nodes in a graph.
    :param start_node: The start node
    :param end_node: The end node
    :param graph: The graph
    :return: The shortest path. It is a list of node_ids from start_node to end_node.
    Note that you use all of the data in "pasdaran_streets" dataset appropriately such as "street_length" and "one_way".
    """


    #init nodes and adjacency lists for the graph
    nodes = set(list(list(graph.u) + list(graph.v)))
    edges = {}
    for node in nodes:
      edges[node] = {}
    for e in graph.iloc:
      edges[e.u][e.v] = e.length
      edges[e.v][e.u] = e.length

    # translate start point and end point to Id
    start_node = closest_ID(start_node,nodes,graph)
    end_node  = closest_ID(end_node,nodes,graph)
    if (start_node == end_node): return [start_node]#,[0]

    # every node is unvisited
    unvisited_nodes = nodes

    # keep visiting cost in this dict
    shortest_path = {}
    previous_nodes = {}
    # every node is inf in start except start node
    inf = 2**62
    for node in unvisited_nodes:
        shortest_path[node] = inf
    shortest_path[start_node] = 0

     
    while unvisited_nodes:
        # find node with minimum cost
        current_min_node = None
        for node in unvisited_nodes: 
            if current_min_node == None:
                current_min_node = node
            elif shortest_path[node] < shortest_path[current_min_node]:
                current_min_node = node
        # updates distance
        for e in edges[current_min_node]:
          nv = shortest_path[current_min_node] + edges[current_min_node][e]
          if nv < shortest_path[e]:
                shortest_path[e] = nv
                previous_nodes[e] = current_min_node
        # remove node from list
        unvisited_nodes.remove(current_min_node)
    # best path to end node
    path = []
    # distance of each node in the path from start node // optional
    dist = []
    # backtrack:
    st = end_node
    try:
      while st != start_node:
        path.append(st)
        dist.append(shortest_path[st])
        st = previous_nodes[st]
      path.append(int(st))
      dist.append(0)
      path.reverse()
      dist.reverse()

      return path#,dist
    except:
      print(start_node, end_node)
      return None#,None

