from collections import defaultdict
import sys
import math
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.patches import Rectangle
from PIL import Image
import numpy as np
from sklearn.neighbors import NearestNeighbors
import shapely.geometry
import argparse
import pickle

from .Dijkstra import Graph, dijkstra, to_array
from .Utils import Utils


class PRMController:
    def __init__(self, numOfRandomCoordinates, collisionMap, current, destination):
        self.numOfCoords = numOfRandomCoordinates
        self.coordsList = np.array([])
        self.collisionMap = collisionMap
        self.current = np.array(current)
        self.destination = np.array(destination)
        self.graph = Graph()
        self.utils = Utils()
        self.solutionFound = False

    def runPRM(self, initialRandomSeed, saveImage=True):
        seed = initialRandomSeed
        # Keep resampling if no solution found
        while(not self.solutionFound):
            print("Trying with random seed {}".format(seed))
            np.random.seed(seed)

            # Generate n random samples called milestones
            self.genCoords()

            # Check if milestones are collision free
            self.checkIfCollisonFree()

            # Link each milestone to k nearest neighbours.
            # Retain collision free links as local paths.
            self.findNearestNeighbour()

            # Search for shortest path from start to end node - Using Dijksta's shortest path alg
            self.shortestPath()

            seed = np.random.randint(1, 100000)
            # self.coordsList = np.array([])
            # self.graph = Graph()

        if(saveImage):
            plt.savefig("{}_samples.png".format(self.numOfCoords))
        plt.show()

    def genCoords(self, sizeOfX=700,sizeOfY=800):
        x = np.random.randint(1,sizeOfX,size=self.numOfCoords)
        y = np.random.randint(1,sizeOfY, size=self.numOfCoords)
        self.coordsList = np.column_stack((x,y))
        
        # Adding begin and end points
        self.current = self.current.reshape(1, 2)
        self.destination = self.destination.reshape(1, 2)
        self.coordsList = np.concatenate(
            (self.coordsList, self.current, self.destination), axis=0)

    def checkIfCollisonFree(self):
        collision = False
        self.collisionFreePoints = np.array([])
        for point in self.coordsList:
            collision = self.checkPointCollision(point)
            if(not collision):
                if(self.collisionFreePoints.size == 0):
                    self.collisionFreePoints = point
                else:
                    self.collisionFreePoints = np.vstack(
                        [self.collisionFreePoints, point])
        self.plotPoints(self.collisionFreePoints)

    def findNearestNeighbour(self, k=25):
        X = self.collisionFreePoints
        knn = NearestNeighbors(n_neighbors=k)
        knn.fit(X)
        distances, indices = knn.kneighbors(X)
        self.collisionFreePaths = np.empty((1, 2), int)

        for i, p in enumerate(X):
            # Ignoring nearest neighbour - nearest neighbour is the point itself
            for j, neighbour in enumerate(X[indices[i][1:]]):
                start_line = p
                end_line = neighbour
                if(not self.checkPointCollision(start_line) and not self.checkPointCollision(end_line)):
                    if(not self.checkLineCollision(start_line, end_line)):
                        self.collisionFreePaths = np.concatenate(
                            (self.collisionFreePaths, p.reshape(1, 2), neighbour.reshape(1, 2)), axis=0)

                        a = str(self.findNodeIndex(p))
                        b = str(self.findNodeIndex(neighbour))
                        self.graph.add_node(a)
                        self.graph.add_edge(a, b, distances[i, j+1])
                        x = [p[0], neighbour[0]]
                        y = [p[1], neighbour[1]]
                        plt.plot(x, y)

    def shortestPath(self):
        print("in shortestPath")
        self.startNode = str(self.findNodeIndex(self.current))
        self.endNode = str(self.findNodeIndex(self.destination))

        dist, prev = dijkstra(self.graph, self.startNode)

        pathToEnd = to_array(prev, self.endNode)

        if(len(pathToEnd) > 1):
            self.solutionFound = True
        else:
            print("path not found")
            self.shortestPath()
            return

        # Plotting shorest path
        pointsToDisplay = [(self.findPointsFromNode(path))
                           for path in pathToEnd]

        x = [int(item[0]) for item in pointsToDisplay]
        y = [int(item[1]) for item in pointsToDisplay]

        plt.plot(x, y, c="white", linewidth=3.5,)
        plt.imshow(self.collisionMap,origin="lower")

        pointsToEnd = [str(self.findPointsFromNode(path))
                       for path in pathToEnd]
        print("****Output****")

        print("The quickest path from {} to {} is: \n {} \n with a distance of {}".format(
            self.collisionFreePoints[int(self.startNode)],
            self.collisionFreePoints[int(self.endNode)],
            " \n ".join(pointsToEnd),
            str(dist[self.endNode])
        )
        )

        plt.show()

    def checkLineCollision(self, start_line, end_line):
        dist = abs(end_line - start_line)
        number_of_points = max(dist)
        x = np.linspace(start_line[0],end_line[0],number_of_points,dtype=int)
        y = np.linspace(start_line[1],end_line[1],number_of_points,dtype=int)
        for y,x in np.column_stack((x,y)):
            if(self.collisionMap[x][y]):
                return True
        return False
        # line = shapely.geometry.LineString([start_line, end_line])
        # for obs in self.allObs:
        #     if(self.utils.isWall(obs)):
        #         uniqueCords = np.unique(obs.allCords, axis=0)
        #         wall = shapely.geometry.LineString(
        #             uniqueCords)
        #         if(line.intersection(wall)):
        #             collision = True
        #     else:
        #         obstacleShape = shapely.geometry.Polygon(
        #             obs.allCords)
        #         collision = line.intersects(obstacleShape)
        #     if(collision):
        #         return True
        # return False

    def findNodeIndex(self, p):
        #print('p=' + str(np.where((self.collisionFreePoints == p).all(axis=1))[0][0]))
        return np.where((self.collisionFreePoints == p).all(axis=1))[0][0]

    def findPointsFromNode(self, n):
        return self.collisionFreePoints[int(n)]

    def plotPoints(self, points):
        x = [item[0] for item in points]
        y = [item[1] for item in points]
        plt.scatter(x, y, c="black", s=1)

    def checkCollision(self, obs, point):
        p_x = point[0]
        p_y = point[1]
        if(obs.bottomLeft[0] <= p_x <= obs.bottomRight[0] and obs.bottomLeft[1] <= p_y <= obs.topLeft[1]):
            return True
        else:
            return False

    def checkPointCollision(self, point):
        p_x = point[1]
        p_y = point[0]
        return self.collisionMap[p_x,p_y]
        # for obs in self.allObs:
        #     collision = self.checkCollision(obs, point)
        #     if(collision):
        #         return True
        # return False

    def newPath(self,current,destination):
        self.current = np.array(current)
        self.destination = np.array(destination)

        if(not self.checkPointCollision(current) and not self.checkPointCollision(destination)):
            if( np.where((self.collisionFreePoints == current).all(axis=1))[0].size == 0):
                current = np.reshape(current,(-1,2))
                current_distances = np.sum(np.square(self.collisionFreePoints - current))
                current_nearest = self.collisionFreePoints[np.argmin(current_distances,axis=1)]
                current_nearest_length = np.min(current_distances,axis=1)
                self.collisionFreePoints = np.vstack([self.collisionFreePoints,current])
                print("Added successfully")
                self.graph.add_node(str(self.findNodeIndex(current)))
                self.graph.add_edge(str(self.findNodeIndex(current)),str(self.findNodeIndex(current_nearest)),current_nearest_length)
            else:
                print("Current point exists")
                print(self.findNodeIndex(current))
            if ( np.where((self.collisionFreePoints == destination).all(axis=1))[0].size == 0):
                destination = np.reshape(destination,(-1,2))
                destination_distances = np.sum(np.square(self.collisionFreePoints - destination),axis=1)
                destination_nearest =  self.collisionFreePoints[np.argmin(destination_distances)]
                destination_nearest_length = np.min(destination_distances)
                self.collisionFreePoints = np.vstack([self.collisionFreePoints,destination])
                self.graph.add_node(str(self.findNodeIndex(destination)))
                self.graph.add_edge(str(self.findNodeIndex(destination_nearest)),str(self.findNodeIndex(destination)),destination_nearest_length)
            else:
                print("Destination point exists")
                print(self.findNodeIndex(destination))
        else:
            print("Error point collision is true")
    
    def exportData(self,filename):
        graph_file = open(filename + ".plk","wb")
        pickle.dump(self.graph,graph_file,pickle.HIGHEST_PROTOCOL)
        np.savez(filename + "_collision_data",points=self.collisionFreePoints,paths=self.collisionFreePaths)
    
    def importData(self,filename):
        graph_file = open(filename + ".plk", "rb")
        self.graph = pickle.load(graph_file)
        collision_data = np.load(filename + "_collision_data.npz")
        self.collisionFreePoints = collision_data["points"]
        self.collisionFreePaths = collision_data["paths"]