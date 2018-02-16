import numpy as np
import tensorflow as tf
import os
import sys
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

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
    plt.contour(_pointList[_x], _pointList[_y], _value, 7)


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

    ''' Validating '''
    # plotTest(args)

    # pdfTest = PDF(0, 0.1)
    # x = np.arange(-5, 5, 0.001)
    # y = pdfTest.evaluate(x)
    # plt.plot(x, y)
    # plt.show()

    numCoM = 5
    comX = np.linspace(-0.8, 0.8, numCoM)
    comY = np.linspace(-0.5, 0.5, numCoM)
    comZ = np.linspace(-0.3, 1.2, numCoM)
    comXList, comYList, comZList = np.meshgrid(comX, comY, comZ)
    import ipdb
    ipdb.set_trace()
    exit()
    kdeVal = []
    for com in comList:
        kdeVal.append(kernelDensityEstimation(pdfList, com))
    print("Eval Done")

    drawContourPlot(comList, kdeVal, 0, 1) # X-Y Plane Projection

    plt.show()

if __name__ == "__main__":
    main()
