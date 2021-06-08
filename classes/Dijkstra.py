# Adapted from https://gist.github.com/betandr/541a1f6466b6855471de5ca30b74cb31
from decimal import Decimal
import json

class Edge:
    def __init__(self, to_node, length):
        self.to_node = to_node
        self.length = length
    
    # def to_json(self):
    #     return json.dumps(self, default=lambda o: o.__dict__)
    
    def __repr__(self):
        return str(self.to_node) + '/' + str(self.length)


class Graph:
    def __init__(self):
        self.nodes = set()
        self.edges = dict()

    def add_node(self, node):
        self.nodes.add(node)

    def add_edge(self, from_node, to_node, length):
        edge = Edge(to_node, length)
        if from_node in self.edges:
            from_node_edges = self.edges[from_node]
        else:
            self.edges[from_node] = dict()
            from_node_edges = self.edges[from_node]
        from_node_edges[to_node] = edge
    
    def export_csv(self,output_filename):
        csv_file = open(output_filename,'w')
        for i in range(len(self.nodes)):
            node = self.nodes[i]
            csv_file.write(str(node))
            if node not in self.edges:
                print("Error exporting... graph incomplete")
                csv_file.close()
                return
            else:
                node_edges = self.edges[node]
                for j in range(len(node_edges)):
                    csv_file.write(',' + str(node_edges[i]))
                csv_file.write('\n')
        csv_file.close()

    def import_csv(self,input_filename):
        csv_file = open(input_filename,'r')
        data = csv_file.readlines()
        for i in range(len(data)):
            node_data = data[i].split(',')
            self.add_node(node_data[0])
            for j in range(1,len(node_data)):
                edge = node_data[j].split('/')
                self.add_edge(node_data[0],node_data[i],edge[0],int(edge[1]))
        csv_file.close()

def min_dist(q, dist):
    """
    Returns the node with the smallest distance in q.
    Implemented to keep the main algorithm clean.
    """
    min_node = None
    for node in q:
        if min_node == None:
            min_node = node
        elif dist[node] < dist[min_node]:
            min_node = node

    return min_node


INFINITY = float('Infinity')


def dijkstra(graph, source):
    q = set()
    dist = {}
    prev = {}

    for v in graph.nodes:       # initialization
        dist[v] = INFINITY      # unknown distance from source to v
        prev[v] = INFINITY      # previous node in optimal path from source
        q.add(v)                # all nodes initially in q (unvisited nodes)

    # distance from source to source
    dist[source] = 0

    while q:
        # node with the least distance selected first
        u = min_dist(q, dist)

        q.remove(u)

        try:
            if u in graph.edges:
                for _, v in graph.edges[u].items():
                    alt = dist[u] + v.length
                    if alt < dist[v.to_node]:
                        # a shorter path to v has been found
                        dist[v.to_node] = alt
                        prev[v.to_node] = u
        except:
            pass

    return dist, prev


def to_array(prev, from_node):
    """Creates an ordered list of labels as a route."""
    previous_node = prev[from_node]
    route = [from_node]

    while previous_node != INFINITY:
        route.append(previous_node)
        temp = previous_node
        previous_node = prev[temp]

    route.reverse()
    return route
