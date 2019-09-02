import numpy as np
import cv2, serial, time, os, sys
from scipy import stats

def getSerialPort(intASCII): # intASCII is the integer which can be recognized as a correct signal (e.g. laser level) by our Arduino UNO
    for portNum in range(1, 11):
        try:
            portName = 'COM' + str(portNum)
            ser = serial.Serial(portName, 9600)
            time.sleep(2) # Always wait for a couple seconds to make sure the serial is connected, otherwise following codes may get stuck
            ser.write(chr(intASCII))
            timezero = time.time()
            elapsedtime = 0.0
            while ser.inWaiting() == 0 and elapsedtime < 0.1: # Wait until the Arduino gets back a serial signal and also the timeout is 0.1 second
                time.sleep(0.001) # Somehow attenuating the loop speed prevents stuck
                elapsedtime = time.time() - timezero
            if elapsedtime < 0.1: # This condition is critical. If the elapsed time > 0.1, then the opened serial port is not Arduino and the following ser.read() will get stuck
                if ser.read() == chr(0):    # If a 0 (ASCII form) is sent back from the port, then it's really our Arduino UNO
                    ser.close()
                    return portName
        except:
            pass
    return None

def detectCamOperable(camIndexR, camIndexL, imgLongSide, imgShortSide):
    # Detect if camIndexR and camIndexL are operable cameras
    # Two criteria:
    # (1) camIndexR and camIndexL can both be opened by OpenCV
    # (2) camIndexR and camIndexL can both be set to the correct image size

    # (1) and (2)
    indices = [camIndexR, camIndexL]
    for i in range(2):
        cam = cv2.VideoCapture(indices[i])
        if cam.isOpened():
            cam.set(3, imgLongSide) # Set width
            cam.set(4, imgShortSide) # Set height
            img = cam.read()[1]
            cam.release()
            if img.shape[0] != imgShortSide or img.shape[1] != imgLongSide:
                return False
        else:
            return False

    return True

def serialSignal(serObj, intASCII):
        serObj.write(chr(intASCII))           # Convert int to ASCII character and write to Arduino
        while serObj.inWaiting() == 0:        # Wait until the Arduino gets back a serial signal
            pass
        return ord(serObj.read())             # Clean up the serial signal in the buffer, and convert from ASCII character to int

def configCam(camObj, width, height, brightness, contrast, saturation, hue, gain, exposure, whiteBalance, focus):
    # Logitech C525 camera
    camObj.set(3, width)
    camObj.set(4, height)
    camObj.set(10, brightness)
    camObj.set(11, contrast)
    camObj.set(12, saturation)
    camObj.set(13, hue)
    camObj.set(14, gain)
    camObj.set(15, exposure)
    camObj.set(17, whiteBalance)
    camObj.set(28, focus)

def combineImg(img1, img2):
    if img1.shape == img2.shape:
        height, width, channels = img1.shape
        imgShow = np.zeros((height, width*2, channels), dtype=np.uint8) + 255
        imgShow[:, 0:width        , :] = img1
        imgShow[:, width:(width*2), :] = img2
        return imgShow

def laserPosition(img, laserHalfWidth):
    height, width = img.shape
    hw = laserHalfWidth
    imgD = np.zeros((height, width), np.float)
    imgD[:,:] = img[:,:]
    z_array = np.zeros(height, np.float)          # create an array of center points (that is z position) of the laser along the y axis
    for y in range(height):
        z_max = imgD[y,:].argmax()                             # max point of horizontal line y
        pos = np.arange(z_max-hw, z_max+hw, dtype = np.float)  # the position array around the max point
        B = imgD[y, (z_max-hw):(z_max+hw)]                     # the intensity array around the max point, which could be a null object
        if not B.shape[0] == 0 and pos.shape == B.shape:
            B = B - B.min()                                    # maximize the difference between the high and low intensity points
            PMF = B / B.sum()                                  # convert the intensity array to probability mass function (PMF)
            z_array[y] = (pos * PMF).sum()                     # center point = expectance = x_0*PMF(x_0) + x_1*PMF(x_1) + ... + x_n*PMF(x_n)
        else:
            z_array[y] = z_max                                 # if expectance cannot be calculated, use max as the center point
    return z_array

def laserBrightness(img):
    height, width = img.shape
    imgD = np.zeros((height, width), np.float)
    imgD[:,:] = img[:,:]
    bright_array = np.zeros(height, np.float)          # create an array of brightness of the laser along the y axis
    for y in range(height):
        bright_array[y] = imgD[y,:].max()
    return bright_array

def thresholdTopograph(topoMat, brightMat, laserThres):
    if topoMat.shape != brightMat.shape:
        return
    rowN, colN = topoMat.shape
    newMat = np.zeros((rowN, colN), np.float)
    newMat[:,:] = topoMat[:,:]
    for iRow in range(rowN):
        for iCol in range(colN):
            if brightMat[iRow, iCol] < laserThres:
                newMat[iRow, iCol] = 0
    return newMat

def removeHalfLaser(mat, laserHalfwidth, isRightCam):
    hw = laserHalfwidth
    rowN, colN = mat.shape
    newMat = np.zeros((rowN, colN), np.float)
    newMat[:,:] = mat[:,:]

    if not isRightCam:
        newMat = newMat[:,::-1]

    for r in range(rowN):
        c = hw
        while c < colN - hw:
            if newMat[r, c] == 0 and newMat[r, c-1] > 0:
                newMat[r, (c-hw):c] = 0 # from (c-hw) to (c-1)
                c += 1
            elif newMat[r, c] == 0 and newMat[r, c+1] > 0:
                newMat[r, (c+1):(c+hw)] = 0 # from (c+1) to (c+hw-1)
                c = c + hw
            else:
                c += 1

    if not isRightCam:
        newMat = newMat[:,::-1]

    return(newMat)

def mergeTopographs(ZR, ZL):
    if ZR.shape != ZL.shape:
        return
    rows, cols = ZR.shape
    outputZ = np.zeros((rows, cols), np.float)
    for r in range(rows):
        for c in range(cols):
            if   ZR[r, c] != 0 and ZL[r, c] != 0:
                outputZ[r, c] = (ZR[r, c] + ZL[r, c])/2
            elif ZR[r, c] != 0 and ZL[r, c] == 0:
                outputZ[r, c] = ZR[r, c]
            elif ZR[r, c] == 0 and ZL[r, c] != 0:
                outputZ[r, c] = ZL[r, c]
    return outputZ

def generateHeatMap(mat):
    return( (mat/mat.max()*255).astype(np.uint8) )

def fillEmptyWell(mat, edgeMin):
    rowN, colN = mat.shape
    newMat = np.zeros((rowN, colN), np.float)
    newMat[:,:] = mat[:,:]
    newMat[:,0] = edgeMin
    newMat[:,colN-1] = edgeMin
    newMat[0,:] = edgeMin
    newMat[rowN-1,:] = edgeMin
    for iRow in range(rowN):
        for iCol in range(colN):
            if newMat[iRow, iCol] == 0:
                preRow = iRow
                while newMat[preRow, iCol] == 0 and preRow > 0:
                    preRow -= 1
                postRow = iRow
                while newMat[postRow, iCol] == 0 and postRow < rowN-1:
                    postRow += 1
                preCol = iCol
                while newMat[iRow, preCol] == 0 and preCol > 0:
                    preCol -= 1
                postCol = iCol
                while newMat[iRow, postCol] == 0 and postCol < colN-1:
                    postCol += 1
                interpolateV = newMat[preRow, iCol] + (newMat[postRow, iCol] - newMat[preRow, iCol])*(iRow-preRow)/(postRow-preRow)
                interpolateH = newMat[iRow, preCol] + (newMat[iRow, postCol] - newMat[iRow, preCol])*(iCol-preCol)/(postCol-preCol)
                newMat[iRow, iCol] = interpolateH # Use horizontal interpolation but not the vertical one
    return(newMat)

def removeOutlier(mat, windowRange, meanThres, noiseThres):
    w = windowRange
    rowN, colN = mat.shape
    newMat = np.zeros((rowN, colN), np.float)
    newMat[:,:] = mat[:,:]
    diffMat = np.zeros((rowN, colN), np.float)
    for iRow in range(1, rowN):
        for iCol in range(1, colN):
            diffMat[iRow, iCol] = (mat[iRow, iCol] - mat[iRow-1, iCol]) + (mat[iRow, iCol] - mat[iRow, iCol-1])
    for iRow in range(w, rowN-w):
        for iCol in range(w, colN-w):
            meanDeviace = mat[iRow, iCol] - np.average(mat[(iRow-w):(iRow+w), (iCol-w):(iCol+w)])
            avgAbsDiff = np.average( np.absolute( diffMat[(iRow-w):(iRow+w), (iCol-w):(iCol+w)] ) )
            absAvgDiff = abs( np.average( diffMat[(iRow-w):(iRow+w), (iCol-w):(iCol+w)] ) )
            noiseLevel = avgAbsDiff - absAvgDiff
            if abs(meanDeviace) > meanThres and noiseLevel > noiseThres:
                newMat[iRow, iCol] = 0
    return(newMat)

def generateSTL(filename, topoMat):
    rowN, colN, dimensions = topoMat.shape
    M = np.zeros((rowN, colN, dimensions), np.float)
    M[:,:,:] = topoMat[:,:,:]
    M[0,:,2] = 0
    M[:,0,2] = 0
    M[rowN-1,:,2] = 0
    M[:,colN-1,2] = 0
    with open(filename, 'w') as STLfile:
        STLfile.write('solid name')
        for r in range(rowN-1):
            for c in range(colN-1):
                STL = ''
                STL = STL + '\nfacet normal 0 0 0'
                STL = STL + '\nouter loop'
                STL = STL + '\nvertex ' + str(M[r,c,0])     + ' ' + str(M[r,c,1])     + ' ' + str(M[r,c,2])
                STL = STL + '\nvertex ' + str(M[r+1,c+1,0]) + ' ' + str(M[r+1,c+1,1]) + ' ' + str(M[r+1,c+1,2])
                STL = STL + '\nvertex ' + str(M[r,c+1,0])   + ' ' + str(M[r,c+1,1])   + ' ' + str(M[r,c+1,2])
                STL = STL + '\nendloop\nendfacet'
                STL = STL + '\nfacet normal 0 0 0'
                STL = STL + '\nouter loop'
                STL = STL + '\nvertex ' + str(M[r,c,0])     + ' ' + str(M[r,c,1])     + ' ' + str(M[r,c,2])
                STL = STL + '\nvertex ' + str(M[r,c,0])     + ' ' + str(M[r+1,c,1])   + ' ' + str(M[r+1,c,2])
                STL = STL + '\nvertex ' + str(M[r+1,c+1,0]) + ' ' + str(M[r+1,c+1,1]) + ' ' + str(M[r+1,c+1,2])
                STL = STL + '\nendloop\nendfacet'
                STLfile.write(STL)
        r += 1
        c += 1
        STL = ''
        STL = STL + '\nfacet normal 0 0 0'
        STL = STL + '\nouter loop'
        STL = STL + '\nvertex ' + str(M[0,0,0]) + ' ' + str(M[0,0,1]) + ' ' + str(M[0,0,2])
        STL = STL + '\nvertex ' + str(M[r,c,0]) + ' ' + str(M[r,c,1]) + ' ' + str(M[0,0,2])
        STL = STL + '\nvertex ' + str(M[r,c,0]) + ' ' + str(M[0,0,1]) + ' ' + str(M[0,0,2])
        STL = STL + '\nendloop\nendfacet'
        STL = STL + '\nfacet normal 0 0 0'
        STL = STL + '\nouter loop'
        STL = STL + '\nvertex ' + str(M[0,0,0]) + ' ' + str(M[0,0,1]) + ' ' + str(M[0,0,2])
        STL = STL + '\nvertex ' + str(M[0,0,0]) + ' ' + str(M[r,c,1]) + ' ' + str(M[0,0,2])
        STL = STL + '\nvertex ' + str(M[r,c,0]) + ' ' + str(M[r,c,1]) + ' ' + str(M[0,0,2])
        STL = STL + '\nendloop\nendfacet'
        STLfile.write(STL)
        STLfile.write('\nendsolid')

def findTurnPoints(img, laserThres, nPoints, window):
    # Input:
    #   img        --- grayscale numpy image
    #   laserThres --- threshold for detecting laser point z position
    #   nPoints    --- number of turnpoints of the laser line to be found
    #   window     --- length of window vector
    # Output:
    #   numpy matrix (nPoints by 2, dtype=np.float32):
    #     row      --- each row is the coordinate of a turnpoint
    #     column   --- first Y, second Z
    height, width = img.shape
    hw = 10 # half width of laser line
    imgD = np.zeros((height, width), np.float)
    imgD[:,:] = img[:,:]
    zArr = np.zeros(height, np.float)           # create an array of center points (that is z position) of the laser along the y axis
    for y in range(height):
        if (imgD[y,:].max() > laserThres):
            zMax = imgD[y,:].argmax()           # max point of horizontal line y
            pos = np.arange(zMax-hw, zMax+hw)   # the position array around the max point
            B = imgD[y, (zMax-hw):(zMax+hw)]    # the intensity array around the max point
            PMF = B / B.sum()                   # convert the intensity array to probability mass function (PMF)
            if (pos.shape == PMF.shape):
                zArr[y] = (pos * PMF).sum()     # center point = expectance = x_0*PMF(x_0) + x_1*PMF(x_1) + ... + x_n*PMF(x_n)
            else:
                zArr[y] = zMax                  # if expectance cannot be calculated, use max as the center point
    # now I have the z array with missing values (zeros)

    i = 0
    while zArr[i] == 0 and i < height:
        i += 1
    zArr[0] = zArr[i]        # fill the first element
    i = height - 1
    while zArr[i] == 0 and i > 0:
        i -= 1
    zArr[height-1] = zArr[i] # fill the last element
    for i in range(1, height-2):
        if zArr[i] == 0:
            j = i
            while zArr[j] == 0:
                j += 1
            zArr[i] = zArr[i-1] + (zArr[j] - zArr[i-1])/(j-i+1) # interpolate missing values
    # now I have the z array with missing values all filled up

    # the following sectoin uses window vector to detect local maximums of absolute slope change
    # and find the top nPoints local maximums
    w = window # length of window vector
    slopeChange = np.zeros(height, np.float)
    for i in range(w, height-w):
        slopeChange[i] = (zArr[i+w] - zArr[i]) - (zArr[i] - zArr[i-w])
    absSlopeChange = np.abs(slopeChange)
    localMax = np.zeros(height, np.bool)
    for i in range(w, height-w):
        if absSlopeChange[i] == absSlopeChange[(i-w/2):(i+w/2)].max(): # find local max with an interval of window length
            localMax[i] = 1
    argSort = np.argsort(absSlopeChange)[::-1] # argument sorted from the max to the min of absSlopeChange
    ithPoint = 0
    turnPointsY  = np.zeros(nPoints, np.int)
    turnPointsYZ = np.zeros((nPoints, 2), np.float32) # output np.float32 data type because the function cv2.getPerspectiveTransform does not accept np.float64
    for i in range(height):                    # the following loop is the most essential part of this function
        if localMax[argSort][i] == 1:          # sort the local max array (which is a boolean array) by the value of absSlopeChange
            turnPointsY[ithPoint] = argSort[i] # find the arguments of the top 7 (or nPoints) local maximums
            ithPoint += 1                      #
            if ithPoint == nPoints:            #
                break                          #
    turnPointsY = np.sort(turnPointsY)
    turnPointsYZ[:, 0] = turnPointsY
    turnPointsYZ[:, 1] = zArr[turnPointsY]
    return turnPointsYZ # dtype = np.float32

def homographyMapping(x, y, H):
    # Input:
    #   x --- 2D numpy array of source coordinate x
    #   y --- 2D numpy array of source coordinate y
    #   H --- (3,3) numpy array of the H matrix from cv2.getPerspectiveTransform()
    # Output:
    #   3D numpy array (2 layers) of destination coordinate X and Y

    # The coordinate system used in this function follows the conventional math x and y
    # Lower case x, y refer to input coordinates
    # Upper case X, Y refer to transformed coordinates
    # The scanner coordinate system is y and z, so I plug the scanner's y in the function's x, and scanner's z in function's y
    if x.shape == y.shape:
        h11, h12, h13, h21, h22, h23, h31, h32, _ = H.reshape((9, ))
        X = (h11*x + h12*y + h13) / (h31*x + h32*y + 1)
        Y = (h21*x + h22*y + h23) / (h31*x + h32*y + 1)
        rowN, colN = x.shape
        XY = np.zeros((rowN, colN, 2), np.float)
        XY[:,:,0] = X
        XY[:,:,1] = Y
        return XY

def linearDiscrepancy(matZR, matZL):
    if matZR.shape != matZL.shape:
        return

    rows, cols = matZR.shape
    arrZR      = np.zeros(rows*cols, np.float)
    arrZL      = np.zeros(rows*cols, np.float)
    arrZR[:]   = matZR.reshape(rows*cols)
    arrZL[:]   = matZL.reshape(rows*cols)

    boolean_mask = np.logical_and(arrZR > 0, arrZL > 0)

    slope, intercept, r_value, p_value, std_err = stats.linregress(x = arrZL[boolean_mask],
                                                                   y = arrZR[boolean_mask])

    return r_value, slope, intercept
