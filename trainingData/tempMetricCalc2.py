import matplotlib.pyplot as plt
import numpy as np
import os

def dist(x1,y1,x2,y2):
    if x1==None:
        return 0
    return ((x2-x1)**2 + (y2-y1)**2)**0.5

def getPathLength(x,y):
    distSum=0
    for index in range(1,len(x)-1):
        distSum += dist(x[index],y[index],x[index+1],y[index+1])
    return distSum

def open_file2(fileName):

    data = np.genfromtxt(fileName,delimiter=",", names=["initialPosX","initialPosZ","finalPosX","finalPosZ","RobotPoseX","RobotPoseZ","RobotPoseRot","MinObstacleAngle","MinObstacleDistance"])

    print(fileName,getPathLength(data['RobotPoseX'], data['RobotPoseZ']))


open_file2('demo.csv')

print("over")
