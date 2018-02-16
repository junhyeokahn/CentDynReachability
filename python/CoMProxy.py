import numpy as np
import tensorflow as tf
import os
import sys
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

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

def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataPath", type=str, default="../cpp/ExperimentData/")
    args = parser.parse_args()

    plotTest(args)

if __name__ == "__main__":
    main()
