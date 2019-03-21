from dtw import dtw
import numpy as np
from scipy.spatial.distance import euclidean

root_num = 6
l_arm_start = 30
l_arm_end = 45
r_arm_start = 45
r_arm_end = 60
l_leg_start = 63
l_leg_end = 78
r_leg_start = 78
r_leg_end =93
body_start = 6
body_end = 30
crotch_start = 60
crotch_end =63

def pathFix(path):
    i = 0
    fixedPath = [[], []]
    while i < len(path[0]):
        fixedPath[0].append(path[0][i])
        fixedPath[1].append(path[1][i])
        tmp = i + 1
        while tmp < len(path[0]) and path[0][tmp] == path[0][i]:
            tmp += 1
        i = tmp
    return fixedPath

def warpingInterpol(origdata, path):
    def _interpol(index, points):
        # out of range safety
        if index == len(origdata):
            return []        
        # path : [ ... t-1, t, t+1, ... ]
        if points == 1:
            return [origdata[index]]
        # path : [ ... t-1, t, t, t+1, ... ]
        if points == 2:
            if (index + 1) == len(origdata):
                return [origdata[index], origdata[index]]
            else:
                return [origdata[index], origdata[index+1]]
        # path : [ ... t-1, t, t, ... , t, t+1, ... ]
        interpolData = []
        tmpInterpol = []
        if (index + 1) == len(origdata):
            tmpInterpol.extend([origdata[index] for j in xrange(0, points)])
        elif (index + 2) == len(origdata):
            diff = origdata[index+1] - origdata[index]
            tmpInterpol.extend([origdata[index] + diff * (( j * 1.0 ) / points) for j in xrange(0, points)])
        else:
            try:
                diff = origdata[index+1] - origdata[index]
            except:
                None
            tmpInterpol.extend([origdata[index+1] + diff * (( j * 1.0 ) / points ) for j in xrange(-(points), 0, 2)])
            diff = origdata[index+2] - origdata[index+1]
            begin = 0 if (points % 2 == 0) else 1
            tmpInterpol.extend([origdata[index+1] + diff * (( j * 1.0 ) / points ) for j in range(begin, points, 2)])
        return tmpInterpol  # transpose List of List

    # warpingInterpol main code
    start = 0
    end = 1
    warpData = []
    debug_SumPoints = 0
    while start < len(path) - 1:
        while (path[end] == path[start]) and (end != len(path) - 1):
            end += 1
        warpData.extend(_interpol(path[start]*2, end - start))
        debug_warpDataLength = len(warpData)
        debug_SumPoints += end - start
        start = end;
    warpData.append(origdata[len(origdata)-1])
    return warpData


def joint_dtw(source, reference, start, end):
    distList = []
    for i in xrange(start , end):
         dist, _, _, _ = dtw( source.channels[i].motion, reference.channels[i].motion, dist=lambda x, y: np.linalg.norm(x - y))
         distList.append(dist)
    index = start + distList.index(max(distList))
    downSampleMotion = [reference.channels[index].motion[j] for j in xrange(0, len(reference.channels[index].motion), 2)]
    _, _, _, path = dtw(source.channels[index].motion, downSampleMotion, dist=lambda x, y: np.linalg.norm(x - y))
    fix = pathFix(path)
    for i in xrange(start , end):
        alignedMotion = warpingInterpol(reference.channels[i].motion, fix[1])
        reference.channels[i].motion[:len(alignedMotion)] = alignedMotion
        del reference.channels[i].motion[len(alignedMotion):]

def body_dtw(source, reference):
    distList = []
    for i in xrange( body_start , body_end ):
         dist, _, _, _ = dtw( source.channels[i].motion, reference.channels[i].motion, dist=lambda x, y: np.linalg.norm(x - y))
         distList.append(dist)
    for i in xrange( crotch_start , crotch_end ):
         dist, _, _, _ = dtw( source.channels[i].motion, reference.channels[i].motion, dist=lambda x, y: np.linalg.norm(x - y))
         distList.append(dist)
    index = distList.index(max(distList))

    if index < 24:
        index += 6
    else:
        index += 36

    downSampleMotion = [reference.channels[index].motion[j] for j in xrange(0, len(reference.channels[index].motion), 2)]
    _, _, _, path = dtw(source.channels[index].motion, downSampleMotion, dist=lambda x, y: np.linalg.norm(x - y))
    fix = pathFix(path)

    for i in xrange(body_start , body_end):
        alignedMotion = warpingInterpol(reference.channels[i].motion, fix[1])
        reference.channels[i].motion[:len(alignedMotion)] = alignedMotion
        del reference.channels[i].motion[len(alignedMotion):]
    for i in xrange( crotch_start , crotch_end):
        alignedMotion = warpingInterpol(reference.channels[i].motion, fix[1])
        reference.channels[i].motion[:len(alignedMotion)] = alignedMotion
        del reference.channels[i].motion[len(alignedMotion):]

def timeWarp(source, reference):
    #root
    for i in xrange ( root_num ):
        reference.channels[i].motion[:len(source.channels[i].motion)] = source.channels[i].motion
        del reference.channels[i].motion[len(source.channels[i].motion):]

    #aligned
    #left_arm
    joint_dtw(source, reference, l_arm_start, l_arm_end)
    #right_arm
    joint_dtw(source, reference, r_arm_start, r_arm_end)
    #left_leg
    joint_dtw(source, reference, l_leg_start, l_leg_end)
    #right_leg
    joint_dtw(source, reference, r_leg_start, r_leg_end)

    #body&crotch
    body_dtw(source, reference)