import matplotlib.pyplot as plt
import numpy as np
import json

with open("params/RRT.json") as f:
    paramsRRT = json.load(f)

with open("params/PathPlanner.json") as f:
    paramsPP = json.load(f)

def DrawSample(obstacleList, nodes, delaunayEdges, rnd): 
    plt.clf()
    # fig, axes = plt.subplots(figsize=(9, 9))
    plt.scatter(obstacleList[:,0], obstacleList[:,1], color='black', marker='o')
    plt.plot(rnd[0], rnd[1], "^k")  

    for node in nodes:
        parent = np.array(node[2:])
        plt.plot([node[0], parent[0]], [node[1], parent[1]], "-r")

    for edge in delaunayEdges:
        pointA = edge[:2]
        pointB = edge[2:]
        plt.plot([pointA[0], pointB[0]], [pointA[1], pointB[1]], "black", linewidth=0.5)

    axes = plt.gca()
    
    # for obs in obstacleList:
        # circle = plt.Circle((obs[0],obs[1]), radius=paramsRRT["cone_inner_size"], fill=False, edgecolor='blue')
        # maxCircle = plt.Circle((obs[0],obs[1]), radius=paramsRRT["cone_outer_size"], fill=False, edgecolor='blue')
    # innerCarCircle = plt.Circle((nodes[0][0],nodes[0][1]), radius=paramsPP["targetConesShortDist"], fill=False, edgecolor='green')
    # outerCarCircle = plt.Circle((nodes[0][0],nodes[0][1]), radius=paramsPP["targetConesBigDist"], fill=False, edgecolor='green')
        # axes.add_patch(circle)
        # axes.add_patch(maxCircle)
    # axes.add_patch(innerCarCircle)
    # axes.add_patch(outerCarCircle)

    plt.plot(nodes[0][0], nodes[0][1], "xr")

    axes = plt.gca()
    plt.grid(True)
    plt.pause(0.1)

def DrawGraph(obstacleList, nodeList, delaunayEdges, bestLeaf, bestBranch, midpoints):
    ax = plt.gca()
    plt.scatter(obstacleList[:,0], obstacleList[:,1], color='black', marker='o')
    plt.scatter(bestLeaf[0], bestLeaf[1], color='red', marker='o')
    plt.scatter(midpoints[:,0], midpoints[:,1], color="red", marker="o")

    # for obs in obstacleList:
        # circle = plt.Circle((obs[0],obs[1]), radius=paramsRRT["cone_inner_size"], fill=False, edgecolor='blue')
        # maxCircle = plt.Circle((obs[0],obs[1]), radius=paramsRRT["cone_outer_size"], fill=False, edgecolor='blue')
    # innerCarCircle = plt.Circle((nodeList[0][0],nodeList[0][1]), radius=paramsPP["targetConesShortDist"], fill=False, edgecolor='green')
    # outerCarCircle = plt.Circle((nodeList[0][0],nodeList[0][1]), radius=paramsPP["targetConesBigDist"], fill=False, edgecolor='green')
        # ax.add_patch(circle)
        # ax.add_patch(maxCircle)
    # ax.add_patch(innerCarCircle)
    # ax.add_patch(outerCarCircle)
    
    for node in nodeList:
        parent = np.array(node[2:])
        plt.plot([node[0], parent[0]], [node[1], parent[1]], "-r")

    for edge in delaunayEdges:
        pointA = edge[:2]
        pointB = edge[2:]
        plt.plot([pointA[0], pointB[0]], [pointA[1], pointB[1]], "black", linewidth=0.5)

    for branch in bestBranch:
        pointA = branch[:2]
        pointB = branch[2:]
        plt.plot([pointA[0], pointB[0]], [pointA[1], pointB[1]], "blue", linewidth=1)
        
    plt.axis("equal")
    plt.grid(True)
    plt.show()

def main():
    obstacleList = np.loadtxt("/home/andreas/Documents/PromRacing/Path_planning_code/src_cpp_v2/RRT/plotTxt/obstacleList.txt")
    nodeList = np.loadtxt("/home/andreas/Documents/PromRacing/Path_planning_code/src_cpp_v2/RRT/plotTxt/nodeList.txt")
    rndList = np.loadtxt("/home/andreas/Documents/PromRacing/Path_planning_code/src_cpp_v2/RRT/plotTxt/rndList.txt")
    bestLeaf = np.loadtxt("/home/andreas/Documents/PromRacing/Path_planning_code/src_cpp_v2/RRT/plotTxt/bestLeaf.txt")
    delaunayEdges = np.loadtxt("/home/andreas/Documents/PromRacing/Path_planning_code/src_cpp_v2/RRT/plotTxt/delaunayEdges.txt")
    bestBranch = np.loadtxt("/home/andreas/Documents/PromRacing/Path_planning_code/src_cpp_v2/RRT/plotTxt/bestBranch.txt")
    midpoints = np.loadtxt("/home/andreas/Documents/PromRacing/Path_planning_code/src_cpp_v2/RRT/plotTxt/midpoints.txt")

    nodes = np.array([nodeList[0]])

    for i in range(len(rndList)):
        rnd = rndList[i]
        nodes = np.append(nodes, [nodeList[i+1]], axis=0)
        DrawSample(obstacleList, nodes, delaunayEdges, rnd)
    
    DrawGraph(obstacleList, nodeList, delaunayEdges, bestLeaf, bestBranch, midpoints)
    print(f"Iterations: {len(rndList)}")

if __name__ == '__main__':
    main()