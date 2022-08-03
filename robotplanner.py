import numpy as np
import math
import time
from pqdict import pqdict


def heuristic_l2(node_1,node_2):
    # euclidean distance as heuristic

    x1,y1 = node_1
    x2,y2 = node_2
    return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)

def heuristic_l1(node_1,node_2):
    # manhattan distance as heuristic

    x1,y1 = node_1
    x2,y2 = node_2
    return abs((x1 - x2) + abs(y1 - y2))

def heuristic_inf(node_1,node_2):
    
    x1,y1 = node_1
    x2,y2 = node_2

    dx = abs(x1 - x2)
    dy = abs(y1 - y2)
    return (dx + dy) - min(dx, dy)

def getChildren(Node,envmap):

    #generates a list containing coordinates of all possible, valid children states/nodes

    numofdirs = 8
    dX = [-1, -1, -1, 0, 0, 1, 1, 1]
    dY = [-1,  0,  1, -1, 1, -1, 0, 1]

    pos_children_list = []
    
    for dd in range(numofdirs):
      newx = Node[0] + dX[dd]
      newy = Node[1] + dY[dd]

      if (newx >= 0 and newx < envmap.shape[0] and newy >= 0 and newy < envmap.shape[1]):
        if(envmap[newx, newy] == 0):
            pos_children_list.append((newx, newy))

    return pos_children_list

def robotplanner(envmap, robotpos, targetpos):

    Parent = {}
    closed = []
    eps = 5

    # initialize g score of start to be zero
    g_value = {tuple(robotpos):0}
    
    # open list: priority queue
    open = pqdict()
    open[tuple(robotpos)] = eps*heuristic_l1(robotpos, targetpos)
    
    targetpos = tuple(targetpos)

    while targetpos not in closed:

        # remove node with smallest f score
        currNode = open.pop()

        # insert into closed list
        closed.append(currNode)
        children_list = getChildren(currNode,envmap)

        for j in range(len(children_list)):

            child = tuple(children_list[j])
            if child not in closed:
                if g_value.get(child,np.inf) > g_value[currNode] + 1:
                    # update g values of child
                    g_value[child] = g_value[currNode] + 1 
                    # store parent node
                    Parent[child] = currNode
                    # update the f values of children
                    open[child] = g_value[child] + eps*heuristic_l1(child,targetpos)

    # calculating path sequence
    curr = targetpos
    while Parent[curr] != tuple(robotpos):
        curr = Parent[curr]


    #childlist = getChildren(tuple(robotpos),envmap)
    #for child in childlist:
        #print(child, g_value[child] + eps*heuristic_l2(child,targetpos))

    #print(Parent)
    print(curr)
    return curr