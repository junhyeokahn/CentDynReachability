import numpy as np
import tensorflow as tf
import tqdm as tqdm
import os
import sys
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time
from sklearn.mixture import GaussianMixture

class PDF(object):
    def __init__(self, _mean, _var=0.1):
        self.mean = _mean
        self.var = _var

    def evaluate(self, _x):
        return (np.exp(-0.5*((np.linalg.norm((_x-self.mean))/self.var)**2))) \
                / (np.sqrt(2 * np.pi))


def readData(fileName_, dim_):
    if not os.path.isfile(fileName_):
        print("File path {} does not exist. Exiting...".format(fileName_))
        sys.exit()
    if dim_ == 1:
        ret = np.ndarray(1)
        tmp = np.ndarray(1)
        with open(fileName_) as fp:
            for line in fp:
                vecList = line.split("\t")
                vecSize = len(vecList)
                for i in range(vecSize-1):
                    tmp[0] = float(vecList[i])
                    if i == 0:
                        ret = np.copy(tmp)
                    else:
                        ret = np.concatenate((ret, tmp), axis=0)
    else:
        ret = np.ndarray(shape=(1,dim_), dtype='float')
        tmp = np.ndarray(shape=(1,dim_), dtype='float');
        isFirst = True
        with open(fileName_) as fp:
            for line in fp:
                for i in range(dim_):
                    tmp[0][i] = float(line.split("\t")[i])
                if isFirst:
                    ret[0] = np.copy(tmp);
                    isFirst = False
                else:
                    ret = np.concatenate((ret, tmp), axis=0)
    return ret

def plotTest(params):
    CoM_rFoot = readData(params.dataPath+"CoM_rFoot.txt", 3)
    dim = 3
    fig = plt.figure()

    if dim == 3:
        ax = plt.axes(projection='3d')
        ax.scatter(CoM_rFoot[:,0], CoM_rFoot[:,1], CoM_rFoot[:,2])
    else:
        plt.plot(CoM_rFoot[:,1], CoM_rFoot[:,2])

    plt.show()

def drawContourPlot(_pointList, _value, _x, _y):
    fig = plt.figure()
    cp = plt.contourf(_pointList[0], _pointList[1], _value, 7)
    plt.colorbar(cp)

def kernelDensityEstimation(_pdfList, _x):
    ret = 0.
    for pdf in _pdfList:
        ret += pdf.evaluate(_x)
    return ret

def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataPath", type=str, default="../cpp/ExperimentData/")
    args = parser.parse_args()

    CoM_rFoot = readData(args.dataPath+"CoM_rFoot.txt", 3)
    pdfList = []
    for com in CoM_rFoot:
        pdfList.append(PDF(com))
    print("%d of PDF are Generated!" % len(pdfList))

    ''' Validating '''
    # plotTest(args)
    # pdfTest = PDF(0, 0.1)
    # x = np.arange(-5, 5, 0.001)
    # y = pdfTest.evaluate(x)
    # plt.plot(x, y)
    # plt.show()

    numCoM = 50
    selectedAxis = [0,1]
    reducedAxis = 3-selectedAxis[0]-selectedAxis[1]

    sampleRange = [np.array([-1.2, 0.8]), np.array([-0.5, 0.5]), np.array([-0.3, 1.2])]
    comLinSpace = [np.linspace(sampleRange[selectedAxis[0]][0],
                               sampleRange[selectedAxis[0]][1], numCoM),
                   np.linspace(sampleRange[selectedAxis[1]][0],
                               sampleRange[selectedAxis[1]][1], numCoM)]
    comSelectedAxis = np.meshgrid(comLinSpace[0], comLinSpace[1])
    comAugmented = np.array([np.average(sampleRange[0]), np.average(sampleRange[1]),
                             np.average(sampleRange[2])])
    for i in range(numCoM):
        for j in range(numCoM):
            if reducedAxis == 0:
                comAugmented = np.vstack((comAugmented,
                                          np.array([np.average(sampleRange[reducedAxis]),
                                          comSelectedAxis[0][i, j],
                                          comSelectedAxis[1][i, j]] )))
            elif reducedAxis == 1:
                comAugmented = np.vstack((comAugmented,
                                          np.array([comSelectedAxis[0][i, j],
                                                    np.average(sampleRange[reducedAxis]),
                                                    comSelectedAxis[1][i, j]])))
            else:
                comAugmented = np.vstack((comAugmented,
                                          np.array([comSelectedAxis[0][i, j], \
                                                    comSelectedAxis[1][i, j], \
                                                    np.average(sampleRange[reducedAxis])]) ))
    kdeVal = []
    KDEStartTime = time.time()
    for i, com in enumerate(comAugmented):
        kdeVal.append(kernelDensityEstimation(pdfList, com))
        if i%10 == 0:
            print("\r {} out of {} are Evaluated".format(i, len(comAugmented)))
    KDEEndTime = time.time()
    print("%s Seconds took for Kernel Density Estimation" \
            % (KDEEndTime-KDEStartTime))

    drawContourPlot(comSelectedAxis, np.array(kdeVal[1:]).reshape(comSelectedAxis[0].shape), 0, 1) # X-Y Plane Projection

    ''' GMM '''
    gmm = GaussianMixture(n_components=10)
    gmm.fit(CoM_rFoot)
    print("Mean \n")
    print(gmm.means_)
    print('\n')
    print("Covariance \n")
    print(gmm.covariances_)
    print("Weights \n")
    print(gmm.weights_)
    # X, Y = np.meshgrid(np.linspace(-20, 20), np.linspace(-20,20))
    # XX = np.array([X.ravel(), Y.ravel()]).T
    # Z = gmm.score_samples(XX)
    # Z = Z.reshape((50,50))

    plt.show()

if __name__ == "__main__":
    main()
