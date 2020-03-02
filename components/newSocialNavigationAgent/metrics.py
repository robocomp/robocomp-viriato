import math
import numpy as np
import csv
import copy
import sys
import matplotlib
import matplotlib.pyplot as plt

from os import listdir
from os.path import isfile, join
from PySide.QtCore import QRect, QRectF, Qt, QSize, QSizeF, QPointF, QLineF
from PySide.QtGui import QTransform, QPainter, QPolygonF


def readPolylinesFromFiles(f):
    polylinesI = []
    c = 0

    with open(f, 'r') as csvfile:
        polyline = []
        for i in csvfile.readlines():
            c += 1
            if len(i) < 3:
                polyline = np.array(polyline + [polyline[0]])
                polylinesI.append(polyline)
                polyline = []
            else:
                row = i.split(' ')
                polyline.append([float(row[0]), float(row[1])])
        if len(polyline) > 0:
            polyline = np.array(polyline + [polyline[0]])
            polylinesI.append(polyline)
            polyline = []
    return polylinesI


def readData(directory, id):
    personsFiles = join(directory, "personpose.txt")
    objectsFiles = join(directory, "objects.txt")
    polylineIFiles = join(directory, "polyline_intimate.txt")
    polylinePFiles = join(directory, "polyline_personal.txt")
    polylineSFiles = join(directory, "polyline_social.txt")

    polylineAffNormalFiles = join(directory, "object_normal.txt")
    polylineAffLowFiles = join(directory, "object_lowP.txt")
    polylineAffMedFiles = join(directory, "object_medP.txt")
    polylineAffHighFiles = join(directory, "object_highP.txt")

    experimentFiles = join(directory, "robotpose" + str(id) + ".txt")

    persons = []
    with open(personsFiles, 'r') as csvfile:
        spamreader = csv.reader(csvfile, delimiter=' ', quotechar='|')
        persons += [[float(row[0]), float(row[1])] for row in spamreader]
    persons = np.array(persons)

    objects = []
    with open(objectsFiles, 'r') as csvfile:
        spamreader2 = csv.reader(csvfile, delimiter=' ', quotechar='|')
        objects += [[float(row[0]), float(row[1])] for row in spamreader2]
    objects = np.array(objects)

    polylinesI = readPolylinesFromFiles(polylineIFiles)
    polylinesP = readPolylinesFromFiles(polylinePFiles)
    polylinesS = readPolylinesFromFiles(polylineSFiles)

    polylinesAffNormal = readPolylinesFromFiles(polylineAffNormalFiles)
    polylinesAffLow = readPolylinesFromFiles(polylineAffLowFiles)
    polylinesAffMed = readPolylinesFromFiles(polylineAffMedFiles)
    polylinesAffHigh = readPolylinesFromFiles(polylineAffHighFiles)

    with open(experimentFiles, 'r') as csvfile:
        spamreader3 = csv.reader(csvfile, delimiter=' ', quotechar='|')
        experiment = np.array([[float(row[0]), float(row[1])] for row in spamreader3])

    return persons, objects, polylinesI, polylinesP, polylinesS, polylinesAffNormal, polylinesAffLow, \
           polylinesAffMed, polylinesAffHigh, experiment


def get_avgDist_minDist_maxDist_CP_single(persons, objects, traj, polylinesI, polylinesP, polylinesS,
                                          polylinesAffNormal,
                                          polylinesAffLow, polylinesAffMed, polylinesAffHigh):
    dist_cperson = np.array([min([np.linalg.norm(x - person) for person in persons]) for x in traj])
    avgDist_cperson = np.average(dist_cperson)

    dist_cperson2 = np.array([min([np.linalg.norm(x - person) for x in traj]) for person in persons])

    minDist_cperson = np.min(dist_cperson)
    maxDist_cperson = np.max(dist_cperson)

    # To objects
    dist_cobject = np.array([min([np.linalg.norm(x - object) for object in objects]) for x in traj])
    avgDist_cobject = np.average(dist_cobject)

    dist_cobject2 = np.array([min([np.linalg.norm(x - object) for x in traj]) for object in objects])

    minDist_cobject = np.min(dist_cobject)
    maxDist_cobject = np.max(dist_cobject)

    social_cperson = {'intimate': 0., 'personal': 0., 'social': 0., 'pub': 0.}
    social_affordance = {'normal': 0., 'low': 0., 'medium': 0., 'high': 0., 'free': 0.}

    polygonI = QPolygonF()
    polygonP = QPolygonF()
    polygonS = QPolygonF()

    polygonAffNormal = QPolygonF()
    polygonAffLow = QPolygonF()
    polygonAffMed = QPolygonF()
    polygonAffHigh = QPolygonF()

    for pI in polylinesI:
        for x in pI:
            polygonI.append(QPointF(x[0], x[1]))

    for pP in polylinesP:
        for x in pP:
            polygonP.append(QPointF(x[0], x[1]))

    for pS in polylinesS:
        for x in pS:
            polygonS.append(QPointF(x[0], x[1]))

    ## Object affordances
    for pN in polylinesAffNormal:
        for x in pN:
            polygonAffNormal.append(QPointF(x[0], x[1]))

    for pL in polylinesAffLow:
        for x in pL:
            polygonAffLow.append(QPointF(x[0], x[1]))

    for pM in polylinesAffMed:
        for x in pM:
            polygonAffMed.append(QPointF(x[0], x[1]))

    for pM in polylinesAffHigh:
        for x in pM:
            polygonAffHigh.append(QPointF(x[0], x[1]))

    for x in traj:
        if polygonI.containsPoint(QPointF(x[0], x[1]), Qt.OddEvenFill):
            social_cperson['intimate'] += 1.
        elif polygonP.containsPoint(QPointF(x[0], x[1]), Qt.OddEvenFill):
            social_cperson['personal'] += 1.
        elif polygonS.containsPoint(QPointF(x[0], x[1]), Qt.OddEvenFill):
            social_cperson['social'] += 1.
        else:
            social_cperson['pub'] += 1.

    social_cperson['intimate'] /= len(traj) / 100.
    social_cperson['personal'] /= len(traj) / 100.
    social_cperson['social'] /= len(traj) / 100.
    social_cperson['pub'] /= len(traj) / 100.

    for x in traj:
        if polygonAffNormal.containsPoint(QPointF(x[0], x[1]), Qt.OddEvenFill):
            social_affordance['normal'] += 1.
        elif polygonAffLow.containsPoint(QPointF(x[0], x[1]), Qt.OddEvenFill):
            social_affordance['low'] += 1.
        elif polygonAffMed.containsPoint(QPointF(x[0], x[1]), Qt.OddEvenFill):
            social_affordance['medium'] += 1.
        elif polygonAffHigh.containsPoint(QPointF(x[0], x[1]), Qt.OddEvenFill):
            social_affordance['high'] += 1.
        else:
            social_affordance['free'] += 1.

    social_affordance['normal'] /= len(traj) / 100.
    social_affordance['low'] /= len(traj) / 100.
    social_affordance['medium'] /= len(traj) / 100.
    social_affordance['high'] /= len(traj) / 100.
    social_affordance['free'] /= len(traj) / 100.

    return avgDist_cperson, minDist_cperson, maxDist_cperson, avgDist_cobject, minDist_cobject, maxDist_cobject, social_cperson, social_affordance, dist_cperson2


def get_CHC(ps):
    deriv_ang = []
    ang_rets = []
    back_ang = None
    back_p = None
    count = 0
    length = 0
    for p in ps:
        count += 1
        if back_p is None:
            back_p = p
            continue
        inc = p - back_p
        ang = math.atan2(inc[0], inc[1])
        ang_rets.append(ang)
        if back_ang is None:
            back_ang = ang
            continue
        ang_inc = np.abs(ang - back_ang)
        length += np.linalg.norm(inc)
        if ang_inc > 1: ang_inc = 0
        deriv_ang.append(ang_inc)
        back_p = p
        back_ang = ang

    ret2 = []
    for i in xrange(4, len(deriv_ang) - 4):
        ret2.append((deriv_ang[i - 3] + deriv_ang[i - 2] + deriv_ang[i - 1] + deriv_ang[i] + deriv_ang[i + 1] +
                     deriv_ang[i + 2] + deriv_ang[i + 3]) / 7)
    acc = 0
    prev = ret2[0]
    for p in ret2:
        acc += np.abs(p - prev)
        prev = p

    return acc, length


def showExperimentsInDirectory(directory):
    meanValues = {'avgDist': 0., 'minDist': 0., 'maxDist': 0., 'avgDistObj': 0., 'minDistObj': 0., 'maxDistObj': 0.,
                  'CHC': 0., 'pathLength': 0., 'dInt': 0., 'dPer': 0.,
                  'dSoc': 0., 'dPub': 0., 'dAffNormal': 0., 'dAffLow': 0., 'dAffMed': 0., 'dAffHigh': 0.,
                  'dAffFree': 0., 'time': 0.}

    allValues = {'avgDist': [], 'minDist': [], 'maxDist': [], 'avgDistObj': [], 'minDistObj': [], 'maxDistObj': [],
                 'CHC': [], 'pathLength': [], 'dInt': [], 'dPer': [],
                 'dSoc': [], 'dPub': [], 'dAffNormal': [], 'dAffLow': [], 'dAffMed': [], 'dAffHigh': [], 'dAffFree': [],
                 'time': []}

    num_experiments = 4
    for i in range(1, num_experiments + 1):

        persons, objects, polylinesI, polylinesP, polylinesS, polylinesAffNormal, polylinesAffLow, \
        polylinesAffMed, polylinesAffHigh, exp = readData(directory, i)

        ttr = 0.0445 * (len(exp))  # a ojo para que cuadre con el tiempo del cronometro

        ##llamar a esto tantas veces como recorridos haya
        avgDist, minDist, maxDist, avgDistObj, minDistObj, \
        maxDistObj, dists_persons, dist_objects, minDists2 = get_avgDist_minDist_maxDist_CP_single(persons, objects,
                                                                                                   exp, polylinesI,
                                                                                                   polylinesP,
                                                                                                   polylinesS,
                                                                                                   polylinesAffNormal,
                                                                                                   polylinesAffLow,
                                                                                                   polylinesAffMed,
                                                                                                   polylinesAffHigh)
        plt.axis('equal')
        for p in polylinesI:
            plt.plot(p.T[0], p.T[1], '-', color="#FF2D00")
        for p in polylinesP:
            plt.plot(p.T[0], p.T[1], '-', color="#00D8FF")
        for p in polylinesS:
            plt.plot(p.T[0], p.T[1], '-', color="#C500FF")
        plt.plot(persons.T[0], persons.T[1], 'go')

        ax = []
        ay = []

        for x in exp:
            ax.append(x[0])
            ay.append(x[1])

        plt.plot(ax, ay, 'o-')
        # plt.show()

        chc, plength = get_CHC(exp)

        meanValues['avgDist'] += avgDist
        meanValues['minDist'] += minDist
        try:
            meanValues['minDist2'] += minDists2
        except KeyError:
            meanValues['minDist2'] = copy.deepcopy(minDists2)

        meanValues['maxDist'] += maxDist

        meanValues['avgDistObj'] += avgDistObj
        meanValues['minDistObj'] += minDistObj
        meanValues['maxDistObj'] += maxDistObj

        meanValues['CHC'] += chc
        meanValues['pathLength'] += plength
        meanValues['dInt'] += dists_persons['intimate']
        meanValues['dPer'] += dists_persons['personal']
        meanValues['dSoc'] += dists_persons['social']
        meanValues['dPub'] += dists_persons['pub']

        meanValues['dAffNormal'] += dist_objects['normal']
        meanValues['dAffLow'] += dist_objects['low']
        meanValues['dAffMed'] += dist_objects['medium']
        meanValues['dAffHigh'] += dist_objects['high']
        meanValues['dAffFree'] += dist_objects['free']

        meanValues['time'] += ttr

        allValues['avgDist'].append(avgDist)
        allValues['minDist'].append(minDist)
        try:
            allValues['minDist2'] = np.vstack((allValues['minDist2'], minDists2))
        except KeyError:
            allValues['minDist2'] = minDists2
        allValues['maxDist'].append(maxDist)

        allValues['avgDistObj'].append(avgDistObj)
        allValues['minDistObj'].append(minDistObj)
        allValues['maxDistObj'].append(maxDistObj)

        allValues['CHC'].append(chc)
        allValues['pathLength'].append(plength)
        allValues['dInt'].append(dists_persons['intimate'])
        allValues['dPer'].append(dists_persons['personal'])
        allValues['dSoc'].append(dists_persons['social'])
        allValues['dPub'].append(dists_persons['pub'])

        allValues['dAffNormal'].append(dist_objects['normal'])
        allValues['dAffLow'].append(dist_objects['low'])
        allValues['dAffMed'].append(dist_objects['medium'])
        allValues['dAffHigh'].append(dist_objects['high'])
        allValues['dAffFree'].append(dist_objects['free'])

        allValues['time'].append(ttr)

    for i in meanValues:
        meanValues[i] /= num_experiments

    print ('<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<')
    print ('<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<')
    print ('<<<<<<<<<<  R E S U L T A D O S   F I N A L E S  <<<<<<<<<<<', directory)
    print ('<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<')
    print ('<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<')

    for i in meanValues:
        if i == 'minDist2':
            print (i, meanValues[i], np.std(np.array(allValues[i]), axis=0))
        else:
            print (i, meanValues[i], np.std(np.array(allValues[i])))


showExperimentsInDirectory('results/recorrido1')
showExperimentsInDirectory('results/recorrido2')
