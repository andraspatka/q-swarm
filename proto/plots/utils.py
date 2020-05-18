import numpy as np

def readAndClean(filePath):
    npmat = np.genfromtxt(filePath, delimiter=',')
    npmat = np.delete(npmat, np.s_[2::], 1) # delete states and actions
    return npmat

def createPathToLogs(fileName):
    return '../../measurements/' + fileName
