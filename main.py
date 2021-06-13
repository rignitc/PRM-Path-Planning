
import sys
import time
from PIL import Image
import numpy as np
import argparse
from classes import PRMController, Obstacle, Utils


def main(args):

    parser = argparse.ArgumentParser(description='PRM Path Planning Algorithm')
    parser.add_argument('--numSamples', type=int, default=100, metavar='N',
                        help='Number of sampled points')
    args = parser.parse_args()

    numSamples = args.numSamples

    env = open("environment.txt", "r")
    l1 = env.readline().split(";")

    current = list(map(int, l1[0].split(",")))
    destination = list(map(int, l1[1].split(",")))

    print("Current: {} Destination: {}".format(current, destination))

    print("****Obstacles****")
    # allObs = []
    # for l in env:
    #     if(";" in l):
    #         line = l.strip().split(";")
    #         topLeft = list(map(int, line[0].split(",")))
    #         bottomRight = list(map(int, line[1].split(",")))
    #         obs = Obstacle(topLeft, bottomRight)
    #         obs.printFullCords()
    #         allObs.append(obs)

    # utils = Utils()
    # utils.drawMap(allObs, current, destination)
    collisionMap = np.array(Image.open('map.bmp'))
    collisionMap = np.sum(collisionMap,axis=2)
    collisionMap = np.where(collisionMap==255,True,False)

    prm = PRMController(numSamples, collisionMap, current, destination)
    # Initial random seed to try
    generate = True
    if(generate):        
        initialRandomSeed = np.random.randint(1, 100000)
        initialRandomSeed = 0
        prm.runPRM(initialRandomSeed)
        prm.exportData('sample')
    else:
        prm.importData("sample")
        t1 = time.time()
        prm.newPath(current,destination)
        #print(prm.graph.edges[str(prm.findNodeIndex(destination))])
        #print(prm.graph.edges['739'])
        prm.shortestPath()
        print(len(prm.collisionFreePoints))
        print(time.time()-t1)


if __name__ == '__main__':
    main(sys.argv)
