import cv2 as cv
import struct
import serial
import math
import time
import sys
import numpy as np
from array import *

ser = serial.Serial('COM6')

#cont = input("Continue: ")
cont = "1"


while cont == "1":

    cap = cv.VideoCapture(0, cv.CAP_DSHOW)
    ret, im = cap.read()
    while True:
        ret, im = cap.read()

        if not ret:
            print("Cant receive frame. Exiting...")
            break
        cv.imshow('frame', im)
        if cv.waitKey(1) == ord('q'):
            break

    #cap.release()
    cv.destroyAllWindows()

    #im = cv.imread("mazeTest.png")

    hsv = cv.cvtColor(im, cv.COLOR_BGR2HSV)

    # Identify Car
    lower_green = np.array([40,50,50])
    upper_green = np.array([80,255,255])
    greenMask = cv.inRange(hsv, lower_green, upper_green)
    carM = cv.moments(greenMask)
    #greenRes = cv.bitwise_and(im,im, mask= greenMask)
    #cv.imshow('Green Mask', greenMask)
    #cv.imshow('Green Res', greenRes)
    #k = cv.waitKey(0)
    if carM["m00"] == 0.0:
        sys.exit("Could not identify robot")
    cX = int(carM["m10"] / carM["m00"])
    cY = int(carM["m01"] / carM["m00"])
    carCentroid = [cX,cY]
    cv.circle(im, (cX, cY), 5, (0,0,0), -1)

    # Identify Columns
    lower_blue = np.array([100,75,75])
    upper_blue = np.array([140,255,255])
    blueMask = cv.inRange(hsv, lower_blue, upper_blue)

    kernel = np.ones((5,5),np.uint8)
    #blueErode = cv.erode(blueMask,kernel,iterations = 1)
    blueDilate = cv.dilate(blueMask,kernel,iterations = 2)

    blueRes = cv.bitwise_and(blueDilate,blueDilate, mask= blueMask)
    #cv.imshow('Blue Mask', blueMask)
    #cv.imshow('erode', blueErode)
    #cv.imshow('dilate', blueDilate)
    #cv.imshow('Blue Res', blueRes)
    #cv.imshow('frame', im)
    #k = cv.waitKey(0)
    cv.destroyAllWindows()
    colContours, colHierarchy = cv.findContours(blueDilate, cv.RETR_TREE,cv.CHAIN_APPROX_SIMPLE)
    colCentroids = [[0]*2]*4
    i = 0
    for c in colContours:
        colM = cv.moments(c)
        cX = int(colM["m10"] / colM["m00"])
        cY = int(colM["m01"] / colM["m00"])
        if cX in range(100,600,1) and cY in range(50,440,1):
            colCentroids[i] = [cX,cY]
            #print(f"cX: {cX} cY: {cY}")
            i = i+1
            #cv.circle(im, (cX, cY), 5, (0,0,0), -1)
    if len(colCentroids) != 4:
        sys.exit("Did not identify exactly 4 columns")


    # Identify Walls
    lower_red = np.array([160,70,70])
    upper_red = np.array([180,255,255])
    redMask = cv.inRange(hsv, lower_red, upper_red)

    kernel = np.ones((3,3),np.uint8)
    #redErode = cv.erode(redMask,kernel,iterations = 1)
    redDilate = cv.dilate(redMask,kernel,iterations = 3)

    redRes = cv.bitwise_and(redDilate,redDilate, mask= redMask)
    #cv.imshow('Red Mask', redMask)
    #cv.imshow('erode', redErode)
    #cv.imshow('dilate', redDilate)
    #cv.imshow('Red Res', redRes)
    #cv.imshow('frame', im)
    k = cv.waitKey(0)
    cv.destroyAllWindows()
    wallContours, wallHierarchy = cv.findContours(redDilate, cv.RETR_TREE,cv.CHAIN_APPROX_SIMPLE)
    wallCentroids = [[0]*2]*len(wallContours)
    i = 0
    for c in wallContours:
        wallM = cv.moments(c)
        cX = int(wallM["m10"] / wallM["m00"])
        cY = int(wallM["m01"] / wallM["m00"])
        dist = math.sqrt(((cX-carCentroid[0])**2) + ((cY-carCentroid[1])**2))
        if dist < 30:
            continue
        wallCentroids[i] = [cX,cY]
        i = i+1
        #cv.circle(im, (cX, cY), 5, (255,0,0), -1)


    # Identify Cheese
    lower_yellow = np.array([15,90,90])
    upper_yellow = np.array([45,255,255])

    yellowMask = cv.inRange(hsv, lower_yellow, upper_yellow)
    kernel = np.ones((5,5),np.uint8)
    yellowErode = cv.erode(yellowMask,kernel,iterations = 1)
    yellowDilate = cv.dilate(yellowErode,kernel,iterations = 3)
    chzM = cv.moments(yellowDilate)
    if chzM["m00"] == 0.0:
        sys.exit("Could not find chese")
    cX = int(chzM["m10"] / chzM["m00"])
    cY = int(chzM["m01"] / chzM["m00"])
    chzCentroid = [cX,cY]
    #cv.circle(im, (cX, cY), 5, (0,255,0), -1)
    #yellowRes = cv.bitwise_and(im,im, mask= yellowMask)
    #cv.imshow('Yellow Mask', yellowMask)
    #cv.imshow('Yellow Dilate', yellowDilate)
    #cv.imshow('Yellow Res', yellowRes)
    #k = cv.waitKey(0)




    #cv.imshow('maze', im)
    #k = cv.waitKey(0)


    # Determine which walls are placed
    imHeight, imWidth, imChannels = im.shape
    colCentroidsSorted = [[0]*2]*4
    a12 = False
    a23 = False
    ab1 = False
    ab2 = False
    ab3 = False
    b12 = False
    b23 = False
    bc1 = False
    bc2 = False
    bc3 = False
    c12 = False
    c23 = False
    for c in colCentroids:
        if c[1] < imHeight/2 and c[0] < imWidth/2:
            colCentroidsSorted[0] = c
            for w in wallCentroids:
                if c[1] in range(w[1]-20, w[1]+20,1) and c[0] < w[0] and w[0] < imWidth*0.66:
                    ab2 = True
                if c[0] in range(w[0]-20, w[0]+20,1) and c[1] < w[1] and w[1] < imHeight*0.66:
                    b12 = True
                if c[1] in range(w[1]-20, w[1]+20,1) and c[0] > w[0]:
                    ab1 = True
                if c[0] in range(w[0]-20, w[0]+20,1) and c[1] > w[1]:
                    a12 = True
        if c[1] < imHeight/2 and c[0] > imWidth/2:
            colCentroidsSorted[1] = c
            for w in wallCentroids:
                if c[1] in range(w[1]-20, w[1]+20,1) and c[0] < w[0]:
                    ab3 = True
                if c[0] in range(w[0]-20, w[0]+20,1) and c[1] < w[1] and w[1] < imHeight*0.66:
                    b23 = True
                if c[0] in range(w[0]-20, w[0]+20,1) and c[1] > w[1]:
                    a23 = True
        if c[1] > imHeight/2 and c[0] < imWidth/2:
            colCentroidsSorted[2] = c
            for w in wallCentroids:
                if c[1] in range(w[1]-20, w[1]+20,1) and c[0] < w[0] and w[0] < imWidth*0.66:
                    bc2 = True
                if c[0] in range(w[0]-20, w[0]+20,1) and c[1] < w[1]:
                    c12 = True
                if c[1] in range(w[1]-20, w[1]+20,1) and c[0] > w[0]:
                    bc1 = True
        if c[1] > imHeight/2 and c[0] > imWidth/2:
            colCentroidsSorted[3] = c
            for w in wallCentroids:
                if c[1] in range(w[1]-20, w[1]+20,1) and c[0] < w[0]:
                    bc3 = True
                if c[0] in range(w[0]-20, w[0]+20,1) and c[1] < w[1]:
                    c23 = True

    # Calculate centers of quadrants using columns
    quadCentroids = [[0]*2]*9
    quadCentroids[0] = [colCentroidsSorted[0][0]-68, colCentroidsSorted[0][1]-68]
    quadCentroids[1] = [colCentroidsSorted[0][0]+68, colCentroidsSorted[0][1]-68]
    quadCentroids[2] = [colCentroidsSorted[1][0]+68, colCentroidsSorted[1][1]-68]
    quadCentroids[3] = [colCentroidsSorted[0][0]-68, colCentroidsSorted[0][1]+68]
    quadCentroids[4] = [colCentroidsSorted[0][0]+68, colCentroidsSorted[0][1]+68]
    quadCentroids[5] = [colCentroidsSorted[1][0]+68, colCentroidsSorted[1][1]+68]
    quadCentroids[6] = [colCentroidsSorted[2][0]-68, colCentroidsSorted[2][1]+68]
    quadCentroids[7] = [colCentroidsSorted[2][0]+68, colCentroidsSorted[2][1]+68]
    quadCentroids[8] = [colCentroidsSorted[3][0]+68, colCentroidsSorted[3][1]+68]

    #for index, quad in enumerate(quadCentroids):
    #    cv.circle(im, (quadCentroids[index][0], quadCentroids[index][1]), 5, (0,255,0), -1)

    quad = ["path"]*9

    # Deterine Quadrant of Robot
    if carCentroid[0] in range(quadCentroids[0][0]-75, quadCentroids[0][0]+75,1) and carCentroid[1] in range(quadCentroids[0][1]-75, quadCentroids[0][1]+75,1):
            quad[0] = "car"
            carQuad = 0
    if carCentroid[0] in range(quadCentroids[1][0]-75, quadCentroids[1][0]+75,1) and carCentroid[1] in range(quadCentroids[1][1]-75, quadCentroids[1][1]+75,1):
            quad[1] = "car"
            carQuad = 1
    if carCentroid[0] in range(quadCentroids[2][0]-75, quadCentroids[2][0]+75,1) and carCentroid[1] in range(quadCentroids[2][1]-75, quadCentroids[2][1]+75,1):
            quad[2] = "car"
            carQuad = 2
    if carCentroid[0] in range(quadCentroids[3][0]-75, quadCentroids[3][0]+75,1) and carCentroid[1] in range(quadCentroids[3][1]-75, quadCentroids[3][1]+75,1):
            quad[3] = "car"
            carQuad = 3
    if carCentroid[0] in range(quadCentroids[4][0]-75, quadCentroids[4][0]+75,1) and carCentroid[1] in range(quadCentroids[4][1]-75, quadCentroids[4][1]+75,1):
            quad[4] = "car"
            carQuad = 4
    if carCentroid[0] in range(quadCentroids[5][0]-75, quadCentroids[5][0]+75,1) and carCentroid[1] in range(quadCentroids[5][1]-75, quadCentroids[5][1]+75,1):
            quad[5] = "car"
            carQuad = 5
    if carCentroid[0] in range(quadCentroids[6][0]-75, quadCentroids[6][0]+75,1) and carCentroid[1] in range(quadCentroids[6][1]-75, quadCentroids[6][1]+75,1):
            quad[6] = "car"
            carQuad = 6
    if carCentroid[0] in range(quadCentroids[7][0]-75, quadCentroids[7][0]+75,1) and carCentroid[1] in range(quadCentroids[7][1]-75, quadCentroids[7][1]+75,1):
            quad[7] = "car"
            carQuad = 7
    if carCentroid[0] in range(quadCentroids[8][0]-75, quadCentroids[8][0]+75,1) and carCentroid[1] in range(quadCentroids[8][1]-75, quadCentroids[8][1]+75,1):
            quad[8] = "car"
            carQuad = 8

    # Determine Initial Orientation of car
    orientation = None
    if carCentroid[1] > (quadCentroids[carQuad][1]) and carCentroid[0] in range(quadCentroids[carQuad][0]-30, quadCentroids[carQuad][0]+30):
        orientation = 'D'
    if carCentroid[0] > (quadCentroids[carQuad][0]) and carCentroid[1] in range(quadCentroids[carQuad][1]-30, quadCentroids[carQuad][1]+30):
        orientation = 'R'
    if carCentroid[0] < (quadCentroids[carQuad][0]) and carCentroid[1] in range(quadCentroids[carQuad][1]-30, quadCentroids[carQuad][1]+30):
        orientation = 'L'
    if carCentroid[1] < (quadCentroids[carQuad][1]) and carCentroid[0] in range(quadCentroids[carQuad][0]-30, quadCentroids[carQuad][0]+30):
        orientation = 'U'



    # Determine quadrant of chz
    if chzCentroid[0] in range(quadCentroids[0][0]-50, quadCentroids[0][0]+50,1) and chzCentroid[1] in range(quadCentroids[0][1]-50, quadCentroids[0][1]+50,1):
            quad[0] = "chz"
    if chzCentroid[0] in range(quadCentroids[1][0]-50, quadCentroids[1][0]+50,1) and chzCentroid[1] in range(quadCentroids[1][1]-50, quadCentroids[1][1]+50,1):
            quad[1] = "chz"
    if chzCentroid[0] in range(quadCentroids[2][0]-50, quadCentroids[2][0]+50,1) and chzCentroid[1] in range(quadCentroids[2][1]-50, quadCentroids[2][1]+50,1):
            quad[2] = "chz"
    if chzCentroid[0] in range(quadCentroids[3][0]-50, quadCentroids[3][0]+50,1) and chzCentroid[1] in range(quadCentroids[3][1]-50, quadCentroids[3][1]+50,1):
            quad[3] = "chz"
    if chzCentroid[0] in range(quadCentroids[4][0]-50, quadCentroids[4][0]+50,1) and chzCentroid[1] in range(quadCentroids[4][1]-50, quadCentroids[4][1]+50,1):
            quad[4] = "chz"
    if chzCentroid[0] in range(quadCentroids[5][0]-50, quadCentroids[5][0]+50,1) and chzCentroid[1] in range(quadCentroids[5][1]-50, quadCentroids[5][1]+50,1):
            quad[5] = "chz"
    if chzCentroid[0] in range(quadCentroids[6][0]-50, quadCentroids[6][0]+50,1) and chzCentroid[1] in range(quadCentroids[6][1]-50, quadCentroids[6][1]+50,1):
            quad[6] = "chz"
    if chzCentroid[0] in range(quadCentroids[7][0]-50, quadCentroids[7][0]+50,1) and chzCentroid[1] in range(quadCentroids[7][1]-50, quadCentroids[7][1]+50,1):
            quad[7] = "chz"
    if chzCentroid[0] in range(quadCentroids[8][0]-50, quadCentroids[8][0]+50,1) and chzCentroid[1] in range(quadCentroids[8][1]-50, quadCentroids[8][1]+50,1):
            quad[8] = "chz"



    # Determine which quads are dead ends and which are possible paths
    if quad[0] != "car" and quad[0] != "chz":
        if ab1 or a12:
            quad[0] = "dead"
            #cv.line(im,[quadCentroids[0][0]-30,quadCentroids[0][1]-30],[quadCentroids[0][0]+30,quadCentroids[0][1]+30],(0,0,255),5)
            #cv.line(im,[quadCentroids[0][0]+30,quadCentroids[0][1]-30],[quadCentroids[0][0]-30,quadCentroids[0][1]+30],(0,0,255),5)
    if quad[1] != "car" and quad[1] != "chz":
        trueCount = 0
        if a12:
            trueCount += 1
        if a23:
            trueCount += 1
        if ab2:
            trueCount += 1
        if trueCount > 1:
            quad[1] = "dead"
            #cv.line(im,[quadCentroids[1][0]-30,quadCentroids[1][1]-30],[quadCentroids[1][0]+30,quadCentroids[1][1]+30],(0,0,255),5)
            #cv.line(im,[quadCentroids[1][0]+30,quadCentroids[1][1]-30],[quadCentroids[1][0]-30,quadCentroids[1][1]+30],(0,0,255),5)
    if quad[2] != "car" and quad[2] != "chz":
        if ab3 or a23:
            quad[2] = "dead"
            #cv.line(im,[quadCentroids[2][0]-30,quadCentroids[2][1]-30],[quadCentroids[2][0]+30,quadCentroids[2][1]+30],(0,0,255),5)
            #cv.line(im,[quadCentroids[2][0]+30,quadCentroids[2][1]-30],[quadCentroids[2][0]-30,quadCentroids[2][1]+30],(0,0,255),5)
    if quad[3] != "car" and quad[3] != "chz":
        trueCount = 0
        if ab1:
            trueCount += 1
        if b12:
            trueCount += 1
        if bc1:
            trueCount += 1
        if trueCount > 1:
            quad[3] = "dead"
            #cv.line(im,[quadCentroids[3][0]-30,quadCentroids[3][1]-30],[quadCentroids[3][0]+30,quadCentroids[3][1]+30],(0,0,255),5)
            #cv.line(im,[quadCentroids[3][0]+30,quadCentroids[3][1]-30],[quadCentroids[3][0]-30,quadCentroids[3][1]+30],(0,0,255),5)
    if quad[4] != "car" and quad[4] != "chz":
        trueCount = 0
        if ab2:
            trueCount += 1
        if b23:
            trueCount += 1
        if bc2:
            trueCount += 1
        if b12:
            trueCount += 1
        if trueCount > 2:
            quad[4] = "dead"
            #cv.line(im,[quadCentroids[4][0]-30,quadCentroids[4][1]-30],[quadCentroids[4][0]+30,quadCentroids[4][1]+30],(0,0,255),5)
            #cv.line(im,[quadCentroids[4][0]+30,quadCentroids[4][1]-30],[quadCentroids[4][0]-30,quadCentroids[4][1]+30],(0,0,255),5)
    if quad[5] != "car" and quad[5] != "chz":
        trueCount = 0
        if ab3:
            trueCount += 1
        if b23:
            trueCount += 1
        if bc3:
            trueCount += 1
        if trueCount > 1:
            quad[5] = "dead"
            #cv.line(im,[quadCentroids[5][0]-30,quadCentroids[5][1]-30],[quadCentroids[5][0]+30,quadCentroids[5][1]+30],(0,0,255),5)
            #cv.line(im,[quadCentroids[5][0]+30,quadCentroids[5][1]-30],[quadCentroids[5][0]-30,quadCentroids[5][1]+30],(0,0,255),5)
    if quad[6] != "car" and quad[6] != "chz":
        if bc1 or c12:
            quad[6] = "dead"
            #cv.line(im,[quadCentroids[6][0]-30,quadCentroids[6][1]-30],[quadCentroids[6][0]+30,quadCentroids[6][1]+30],(0,0,255),5)
            #cv.line(im,[quadCentroids[6][0]+30,quadCentroids[6][1]-30],[quadCentroids[6][0]-30,quadCentroids[6][1]+30],(0,0,255),5)
    if quad[7] != "car" and quad[7] != "chz":
        trueCount = 0
        if bc2:
            trueCount += 1
        if c12:
            trueCount += 1
        if c23:
            trueCount += 1
        if trueCount > 1:
            quad[7] = "dead"
            #cv.line(im,[quadCentroids[7][0]-30,quadCentroids[7][1]-30],[quadCentroids[7][0]+30,quadCentroids[7][1]+30],(0,0,255),5)
            #cv.line(im,[quadCentroids[7][0]+30,quadCentroids[7][1]-30],[quadCentroids[7][0]-30,quadCentroids[7][1]+30],(0,0,255),5)
    if quad[8] != "car" and quad[8] != "chz":
        if bc3 or c23:
            quad[8] = "dead"
            #cv.line(im,[quadCentroids[8][0]-30,quadCentroids[8][1]-30],[quadCentroids[8][0]+30,quadCentroids[8][1]+30],(0,0,255),5)
            #cv.line(im,[quadCentroids[8][0]+30,quadCentroids[8][1]-30],[quadCentroids[8][0]-30,quadCentroids[8][1]+30],(0,0,255),5)


    # Defining recursive function for finding shortest path from bot to chz
    def findPath(path, directions, currQuad):
        if currQuad == 0:
            dir1,dir2 = None,None
            if 3 in path and quad[1] == "dead":
                return None
            elif not a12 and quad[1] != "dead" and 1 not in path:
                path1 = path.copy()
                path1.append(currQuad)
                directions1 = directions.copy()
                directions1.append("R")
                if quad[1] == "chz":
                    return directions1
                else:
                    nextQuad = currQuad+1
                    dir1 = findPath(path1, directions1, nextQuad)
            if 1 in path and quad[3] == "dead":
                return None
            elif not ab1 and quad[3] != "dead" and 3 not in path:
                path2 = path.copy()
                path2.append(currQuad)
                directions2 = directions.copy()
                directions2.append("D")
                if quad[3] == "chz":
                    return directions2
                else:
                    nextQuad = currQuad+3
                    dir2 = findPath(path2, directions2, nextQuad)

            if dir1 == None and dir2 == None:
                return None
            shortestDir = 0
            if dir1 != None:
                shortestDir = dir1
            if dir2 != None:
                if shortestDir == 0:
                    shortestDir = dir2
                elif len(dir2) < len(shortestDir):
                    shortestDir = dir2
            return shortestDir

        if currQuad == 1:
            dir1,dir2,dir3 = None, None, None
            if ((2 in path and ab2) or (4 in path and a23)) and quad[0] == "dead":
                return None
            elif 0 not in path and not a12 and quad[0] != "dead":
                path1 = path.copy()
                path1.append(currQuad)
                directions1 = directions.copy()
                directions1.append("L")
                if quad[0] == "chz":
                    return directions1
                else:
                    nextQuad = currQuad-1
                    dir1 = findPath(path1, directions1, nextQuad)
            if ((0 in path and ab2) or (4 in path and a12)) and quad[2] == "dead":
                return None
            elif 2 not in path and not a23 and quad[2] != "dead":
                path2 = path.copy()
                path2.append(currQuad)
                directions2 = directions.copy()
                directions2.append("R")
                if quad[2] == "chz":
                    return directions2
                else:
                    nextQuad = currQuad+1
                    dir2 = findPath(path2, directions2, nextQuad)
            if ((0 in path and a23) or (2 in path and a12)) and quad[4] == "dead":
                return None
            elif 4 not in path and not ab2 and quad[4] != "dead":
                path3 = path.copy()
                path3.append(currQuad)
                directions3 = directions.copy()
                directions3.append("D")
                if quad[4] == "chz":
                    return directions3
                else:
                    nextQuad = currQuad+3
                    dir3 = findPath(path3, directions3, nextQuad)

            if dir1 == None and dir2 == None and dir3 == None:
                return None
            shortestDir = 0
            if dir1 != None:
                shortestDir = dir1
            if dir2 != None:
                if shortestDir == 0:
                    shortestDir = dir2
                elif len(dir2) < len(shortestDir):
                    shortestDir = dir2
            if dir3 != None:
                if shortestDir == 0:
                    shortestDir = dir3
                elif len(dir3) < len(shortestDir):
                    shortestDir = dir3
            return shortestDir

        if currQuad == 2:
            dir1, dir2 = None, None
            if 5 in path and quad[2] == "dead":
                return None
            elif not a23 and quad[2] != "dead" and 1 not in path:
                path1 = path.copy()
                path1.append(currQuad)
                directions1 = directions.copy()
                directions1.append("L")
                if quad[1] == "chz":
                    return directions1
                else:
                    nextQuad = currQuad-1
                    dir1 = findPath(path1, directions1, nextQuad)
            if 1 in path and quad[5] == "dead":
                return None
            elif not ab3 and quad[5] != "dead" and 5 not in path:
                path2 = path.copy()
                path2.append(currQuad)
                directions2 = directions.copy()
                directions2.append("D")
                if quad[5] == "chz":
                    return directions2
                else:
                    nextQuad = currQuad+3
                    dir2 = findPath(path2, directions2, nextQuad)
            if dir1 == None and dir2 == None:
                return None
            shortestDir = 0
            if dir1 != None:
                shortestDir = dir1
            if dir2 != None:
                if shortestDir == 0:
                    shortestDir = dir2
                elif len(dir2) < len(shortestDir):
                    shortestDir = dir2
            return shortestDir

        if currQuad == 3:
            dir1,dir2,dir3 = None, None, None
            if ((6 in path and b12) or (4 in path and bc1)) and quad[0] == "dead":
                return None
            elif 0 not in path and not a12 and quad[0] != "dead":
                path1 = path.copy()
                path1.append(currQuad)
                directions1 = directions.copy()
                directions1.append("U")
                if quad[0] == "chz":
                    return directions1
                else:
                    nextQuad = currQuad-3
                    dir1 = findPath(path1, directions1, nextQuad)
            if ((0 in path and bc1) or (6 in path and ab1)) and quad[4] == "dead":
                return None
            elif 4 not in path and not b12 and quad[4] != "dead":
                path2 = path.copy()
                path2.append(currQuad)
                directions2 = directions.copy()
                directions2.append("R")
                if quad[4] == "chz":
                    return directions2
                else:
                    nextQuad = currQuad+1
                    dir2 = findPath(path2, directions2, nextQuad)
            if ((0 in path and b12) or (4 in path and ab1)) and quad[6] == "dead":
                return None
            elif 6 not in path and not bc1 and quad[6] != "dead":
                path3 = path.copy()
                path3.append(currQuad)
                directions3 = directions.copy()
                directions3.append("D")
                if quad[6] == "chz":
                    return directions3
                else:
                    nextQuad = currQuad+3
                    dir3 = findPath(path3, directions3, nextQuad)

            if dir1 == None and dir2 == None and dir3 == None:
                return None
            shortestDir = 0
            if dir1 != None:
                shortestDir = dir1
            if dir2 != None:
                if shortestDir == 0:
                    shortestDir = dir2
                elif len(dir2) < len(shortestDir):
                    shortestDir = dir2
            if dir3 != None:
                if shortestDir == 0:
                    shortestDir = dir3
                elif len(dir3) < len(shortestDir):
                    shortestDir = dir3
            return shortestDir

        if currQuad == 4:
            dir1,dir2,dir3,dir4 = None, None, None, None
            if ((3 in path and bc2 and b23) or (5 in path and b12 and bc2) or (7 in path and b12 and b23)) and quad[1] == "dead":
                return None
            elif 1 not in path and not ab2 and quad[1] != "dead":
                path1 = path.copy()
                path1.append(currQuad)
                directions1 = directions.copy()
                directions1.append("U")
                if quad[1] == "chz":
                    return directions1
                else:
                    nextQuad = currQuad-3
                    dir1 = findPath(path1, directions1, nextQuad)
            if ((1 in path and b12 and bc2) or (3 in path and ab2 and bc2) or (7 in path and b12 and ab2)) and quad[5] == "dead":
                return None
            elif 5 not in path and not b23 and quad[5] != "dead":
                path2 = path.copy()
                path2.append(currQuad)
                directions2 = directions.copy()
                directions2.append("R")
                if quad[5] == "chz":
                    return directions2
                else:
                    nextQuad = currQuad+1
                    dir2 = findPath(path2, directions2, nextQuad)
            if ((1 in path and b12 and b23) or (3 in path and ab2 and b23) or (5 in path and ab2 and b12)) and quad[7] == "dead":
                return None
            elif 7 not in path and not bc2 and quad[7] != "dead":
                path3 = path.copy()
                path3.append(currQuad)
                directions3 = directions.copy()
                directions3.append("D")
                if quad[7] == "chz":
                    return directions3
                else:
                    nextQuad = currQuad+3
                    dir3 = findPath(path3, directions3, nextQuad)
            if ((1 in path and bc2 and b23) or (7 in path and ab2 and b23) or (5 in path and ab2 and bc2)) and quad[3] == "dead":
                return None
            elif 3 not in path and not b12 and quad[3] != "dead":
                path4 = path.copy()
                path4.append(currQuad)
                directions4 = directions.copy()
                directions4.append("L")
                if quad[7] == "chz":
                    return directions4
                else:
                    nextQuad = currQuad-1
                    dir4 = findPath(path4, directions4, nextQuad)

            if dir1 == None and dir2 == None and dir3 == None and dir4 == None:
                return None
            shortestDir = 0
            if dir1 != None:
                shortestDir = dir1
            if dir2 != None:
                if shortestDir == 0:
                    shortestDir = dir2
                elif len(dir2) < len(shortestDir):
                    shortestDir = dir2
            if dir3 != None:
                if shortestDir == 0:
                    shortestDir = dir3
                elif len(dir3) < len(shortestDir):
                    shortestDir = dir3
            if dir4 != None:
                if shortestDir == 0:
                    shortestDir = dir4
                elif len(dir4) < len(shortestDir):
                    shortestDir = dir4
            return shortestDir

        if currQuad == 5:
            dir1,dir2,dir3 = None, None, None
            if ((4 in path and bc3) or (8 in path and b23)) and quad[2] == "dead":
                return None
            elif 2 not in path and not ab3 and quad[2] != "dead":

                path1 = path.copy()
                path1.append(currQuad)
                directions1 = directions.copy()
                directions1.append("U")
                if quad[2] == "chz":
                    return directions1
                else:
                    nextQuad = currQuad-3
                    dir1 = findPath(path1, directions1, nextQuad)
            if ((2 in path and bc3) or (8 in path and ab3)) and quad[4] == "dead":
                return None
            elif 4 not in path and not b23 and quad[4] != "dead":
                path2 = path.copy()
                path2.append(currQuad)
                directions2 = directions.copy()
                directions2.append("L")
                if quad[4] == "chz":
                    return directions2
                else:
                    nextQuad = currQuad-1
                    dir2 = findPath(path2, directions2, nextQuad)
            if ((2 in path and b23) or (4 in path and ab3)) and quad[8] == "dead":
                return None
            elif 8 not in path and not bc3 and quad[8] != "dead":
                path3 = path.copy()
                path3.append(currQuad)
                directions3 = directions.copy()
                directions3.append("D")
                if quad[8] == "chz":
                    return directions3
                else:
                    nextQuad = currQuad+3
                    dir3 = findPath(path3, directions3, nextQuad)

            if dir1 == None and dir2 == None and dir3 == None:
                return None
            shortestDir = 0
            if dir1 != None:
                shortestDir = dir1
            if dir2 != None:
                if shortestDir == 0:
                    shortestDir = dir2
                elif len(dir2) < len(shortestDir):
                    shortestDir = dir2
            if dir3 != None:
                if shortestDir == 0:
                    shortestDir = dir3
                elif len(dir3) < len(shortestDir):
                    shortestDir = dir3
            return shortestDir

        if currQuad == 6:
            dir1, dir2 = None, None
            if 3 in path and quad[7] == "dead":
                return None
            elif not c12 and quad[7] != "dead" and 7 not in path:
                path1 = path.copy()
                path1.append(currQuad)
                directions1 = directions.copy()
                directions1.append("R")
                if quad[7] == "chz":
                    return directions1
                else:
                    nextQuad = currQuad+1
                    dir1 = findPath(path1, directions1, nextQuad)
            if 7 in path and quad[3] == "dead":
                return None
            elif not bc1 and quad[3] != "dead" and 3 not in path:
                path2 = path.copy()
                path2.append(currQuad)
                directions2 = directions.copy()
                directions2.append("U")
                if quad[3] == "chz":
                    return directions2
                else:
                    nextQuad = currQuad-3
                    dir2 = findPath(path2, directions2, nextQuad)
            if dir1 == None and dir2 == None:
                return None
            shortestDir = 0
            if dir1 != None:
                shortestDir = dir1
            if dir2 != None:
                if shortestDir == 0:
                    shortestDir = dir2
                elif len(dir2) < len(shortestDir):
                    shortestDir = dir2
            return shortestDir

        if currQuad == 7:
            dir1,dir2,dir3 = None, None, None
            if ((4 in path and c23) or (8 in path and bc2)) and quad[6] == "dead":
                return None
            elif 6 not in path and not c12 and quad[6] != "dead":
                path1 = path.copy()
                path1.append(currQuad)
                directions1 = directions.copy()
                directions1.append("L")
                if quad[6] == "chz":
                    return directions1
                else:
                    nextQuad = currQuad-1
                    dir1 = findPath(path1, directions1, nextQuad)
            if ((4 in path and c12) or (6 in path and bc2)) and quad[8] == "dead":
                return None
            elif 8 not in path and not c23 and quad[8] != "dead":
                path2 = path.copy()
                path2.append(currQuad)
                directions2 = directions.copy()
                directions2.append("R")
                if quad[8] == "chz":
                    return directions2
                else:
                    nextQuad = currQuad+1
                    dir2 = findPath(path2, directions2, nextQuad)
            if ((6 in path and c23) or (8 in path and c12)) and quad[4] == "dead":
                return None
            elif 4 not in path and not bc2 and quad[4] != "dead":
                path3 = path.copy()
                path3.append(currQuad)
                directions3 = directions.copy()
                directions3.append("U")
                if quad[4] == "chz":
                    return directions3
                else:
                    nextQuad = currQuad-3
                    dir3 = findPath(path3, directions3, nextQuad)

            if dir1 == None and dir2 == None and dir3 == None:
                return None
            shortestDir = 0
            if dir1 != None:
                shortestDir = dir1
            if dir2 != None:
                if shortestDir == 0:
                    shortestDir = dir2
                elif len(dir2) < len(shortestDir):
                    shortestDir = dir2
            if dir3 != None:
                if shortestDir == 0:
                    shortestDir = dir3
                elif len(dir3) < len(shortestDir):
                    shortestDir = dir3
            return shortestDir

        if currQuad == 8:
            dir1, dir2 = None, None
            if 5 in path and quad[7] == "dead":
                return None
            elif not c23 and quad[7] != "dead" and 7 not in path:
                path1 = path.copy()
                path1.append(currQuad)
                directions1 = directions.copy()
                directions1.append("L")
                if quad[7] == "chz":
                    return directions1
                else:
                    nextQuad = currQuad-1
                    dir1 = findPath(path1, directions1, nextQuad)
            if 7 in path and quad[5] == "dead":
                return None
            elif not bc3 and quad[5] != "dead" and 5 not in path:
                path2 = path.copy()
                path2.append(currQuad)
                directions2 = directions.copy()
                directions2.append("U")
                if quad[5] == "chz":
                    return directions2
                else:
                    nextQuad = currQuad-3
                    dir2 = findPath(path2, directions2, nextQuad)
            if dir1 == None and dir2 == None:
                return None
            shortestDir = 0
            if dir1 != None:
                shortestDir = dir1
            if dir2 != None:
                if shortestDir == 0:
                    shortestDir = dir2
                elif len(dir2) < len(shortestDir):
                    shortestDir = dir2
            return shortestDir

    solve = findPath([], [], carQuad)

    print(f"Initial Orientation: {orientation}")
    if solve == None:
        print("No possible paths to cheese")
    else:
        for index, dir in enumerate(solve):
            print(solve[index])
        currPoint = carCentroid.copy()
        for index, dir in enumerate(solve):
            if index == len(solve)-1:
                cv.line(im,currPoint,chzCentroid,(0,0,0),5)
            elif index == 0:
                if dir == "R":
                    cv.line(im,currPoint,(currPoint[0]+115,currPoint[1]),(0,0,0),5)
                    currPoint[0] += 115
                elif dir == "L":
                    cv.line(im,currPoint,(currPoint[0]-115,currPoint[1]),(0,0,0),5)
                    currPoint[0] -= 115
                elif dir == "U":
                    cv.line(im,currPoint,(currPoint[0],currPoint[1]-115),(0,0,0),5)
                    currPoint[1] -= 115
                elif dir == "D":
                    cv.line(im,currPoint,(currPoint[0],currPoint[1]+115),(0,0,0),5)
                    currPoint[1] += 115
            elif dir == "R":
                cv.line(im,currPoint,(currPoint[0]+140,currPoint[1]),(0,0,0),5)
                currPoint[0] += 140
            elif dir == "L":
                cv.line(im,currPoint,(currPoint[0]-140,currPoint[1]),(0,0,0),5)
                currPoint[0] -= 140
            elif dir == "U":
                cv.line(im,currPoint,(currPoint[0],currPoint[1]-140),(0,0,0),5)
                currPoint[1] -= 140
            elif dir == "D":
                cv.line(im,currPoint,(currPoint[0],currPoint[1]+140),(0,0,0),5)
                currPoint[1] += 140




    cv.imshow('Maze', im)
    k = cv.waitKey(0)
    cv.destroyAllWindows()


    if solve != None:
        #print("Writing orientation")
        ser.flushInput()
        ser.flushOutput()
        if orientation == "R":
            ser.write(b'R')
        elif orientation == "L":
            ser.write(b'L')
        elif orientation == "D":
            ser.write(b'D')
        elif orientation == "U":
            ser.write(b'U')
        data = ser.read()
        for index, dir in enumerate(solve):
            time.sleep(0.5)
            #print("Writing next dir")
            #ser.flushInput()
            #ser.flushOutput()
            if dir == "R":
                ser.write(b'R')
            elif dir == "L":
                ser.write(b'L')
            elif dir == "D":
                ser.write(b'D')
            elif dir == "U":
                ser.write(b'U')
            time.sleep(0.5)
            data = ser.read()
            
        #ser.flushInput()
        #ser.flushOutput()
        #print("Wrote Last dir")
        ser.write(b'F')
        #print("Reading Last dir ACK")
        data = ser.read()
        currQuad = carQuad
        #xAdjust = abs(carCentroid[0]-quadCentroids[currQuad][0])
        #yAdjust = abs(carCentroid[1]-quadCentroids[currQuad][1])
        #print(f"xAdjust: {xAdjust}  yAdjust{yAdjust}")
        for index, dir in enumerate(solve):
            if solve[index] == "R":
                destQuad = currQuad + 1
            if solve[index] == "L":
                destQuad = currQuad - 1
            if solve[index] == "U":
                destQuad = currQuad - 3
            if solve[index] == "D":
                destQuad = currQuad + 3
            print(f"Movement {index}")
            #print(f"Curr Quad: {currQuad}  Dest Quad: {destQuad}")
            time.sleep(0.5)
            #print("Reading gyro ack")
            data = ser.read() #gyro ack
            #cap = cv.VideoCapture(0, cv.CAP_DSHOW)
            ret, im = cap.read()
            time.sleep(1)
            ret, im = cap.read()
            ret2, newIm2 = cap.read()
            hsv2 = cv.cvtColor(newIm2, cv.COLOR_BGR2HSV)
            greenMask2 = cv.inRange(hsv2, lower_green, upper_green)
            carM2 = cv.moments(greenMask2)
            cX2 = int(carM2["m10"] / carM2["m00"])
            cY2 = int(carM2["m01"] / carM2["m00"])
            carCentroid = [cX2,cY2]
            time.sleep(0.5)
            #print("Writing post gyro pic ACK")
            ser.write(b'F')
            
            while True:
                #print("Reading Movement ACK")
                time.sleep(0.5)
                data = ser.read() #movement ACK
                #cap = cv.VideoCapture(0, cv.CAP_DSHOW)
                ret, im = cap.read()
                time.sleep(1)
                ret, im = cap.read()
                ret3, newIm3 = cap.read()
                #cap.release()
                hsv3 = cv.cvtColor(newIm3, cv.COLOR_BGR2HSV)
                greenMask3 = cv.inRange(hsv3, lower_green, upper_green)
                carM3 = cv.moments(greenMask3)
                cX3 = int(carM3["m10"] / carM3["m00"])
                cY3 = int(carM3["m01"] / carM3["m00"])
                newCarCentroid = [cX3,cY3]
                xDiff = abs(newCarCentroid[0] - carCentroid[0])
                yDiff = abs(newCarCentroid[1] - carCentroid[1])
                #xDiff = quadCentroids[destQuad][0] - newCarCentroid[0]
                #if solve[index] == "L":
                #    xDiff += xAdjust
                #if solve[index] == "R":
                #    xDiff -= xAdjust
                #yDiff = quadCentroids[destQuad][1] - newCarCentroid[1]
                #if solve[index] == "U":
                #    yDiff += yAdjust
                #if solve[index] == "D":
                #    yDiff -= yAdjust
                print(f"xDiff: {xDiff}  yDiff: {yDiff}")
                #cv.imshow("Pre Move Mask", greenMask2)
                #cv.imshow("Post Move Mask", greenMask3)
                #cv.imshow("Pre Move",newIm2)
                #cv.imshow("Post Move", newIm3)
                #k = cv.waitKey(0)
                #cv.destroyAllWindows()
                

                if solve[index] == "U" or solve[index] == "D":
                    if yDiff in range(138,143,1):
                        carCentroid = newCarCentroid
                        #ser.flushInput()
                        #ser.flushOutput()
                        print("Good")
                        ser.write(b'F')
                        break
                    else:
                        if yDiff > 142:
                            #ser.flushInput()
                            #ser.flushOutput()
                            print("High")
                            ser.write(b'H')
                        else:
                            #ser.flushInput()
                            #ser.flushOutput()
                            print("Low")
                            ser.write(b'L')
                        offset = abs(yDiff-140) * 0.134155 # cm per pixel
                        #offset = abs(yDiff) * 0.134155
                        print(f"Offset: {offset} cm")
                        ba = bytearray(struct.pack("f",offset))
                        data = ser.read()
                        #ser.flushInput()
                        #ser.flushOutput()
                        ser.write(ba)
                        data = ser.read()

                if solve[index] == "L" or solve[index] == "R":
                    if xDiff in range(138,143,1):
                        carCentroid = newCarCentroid
                        #ser.flushInput()
                        #ser.flushOutput()
                        print("Good")
                        ser.write(b'F')
                        break
                    else:
                        if xDiff > 142:
                            #ser.flushInput()
                            #ser.flushOutput()
                            print("High")
                            ser.write(b'H')
                        else:
                            #ser.flushInput()
                            #ser.flushOutput()
                            print("Low")
                            ser.write(b'L')
                        offset = abs(xDiff-140) * 0.134155
                        #offset = abs(xDiff) * 0.134155
                        ba = bytearray(struct.pack("f",offset))
                        print(f"Offset: {offset} cm")
                        data = ser.read()
                        #ser.flushInput()
                        #ser.flushOutput()
                        ser.write(ba)
        data = ser.read()
        cap.release()
    cont = input("Go Again? 1 for yes or 0 for no: ")
    while cont != "0" and cont != "1":
        cont = input("Please enter 1 for yes or 0 for no: ")

ser.close()
cap.release()
cv.destroyAllWindows()




