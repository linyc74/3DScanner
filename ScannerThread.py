import numpy as np
import cv2, serial, time, os, sys
from scipy import stats
from PyQt4 import QtCore, QtGui
from ScannerFunction import *

class ScanProcThread(QtCore.QThread):
    def __init__(self, Main):
        super(ScanProcThread, self).__init__()
        self.isLaserLeveled = False
        self.savingImg = False
        self.abort = False

        # The following lines transfer all parameters from the main GUI thread to the scan/process thread
        self.isDeveloper = Main.isDeveloper
        self.scanname = Main.scanname
        self.imgLongSide = Main.imgLongSide
        self.imgShortSide = Main.imgShortSide
        self.camIndexR = Main.camIndexR
        self.camIndexL = Main.camIndexL
        self.nFrame = Main.nFrame
        self.targetLaserBrightness = Main.targetLaserBrightness
        self.laserLevel = Main.laserLevel
        self.laserThresR = Main.laserThresR
        self.laserThresL = Main.laserThresL
        self.hMat1 = Main.hMat1
        self.hMat2 = Main.hMat2
        self.zMin = Main.zMin
        self.zMax = Main.zMax
        self.xStart = Main.xStart
        self.xEnd = Main.xEnd
        self.xRange = 0
        self.xCurrent = Main.xCurrent
        self.xScale = Main.xScale
        self.denoiseWindowRange = Main.denoiseWindowRange
        self.denoiseMeanThres = Main.denoiseMeanThres
        self.denoiseNoiseThres = Main.denoiseNoiseThres
        self.brightness = Main.brightness
        self.contrast = Main.contrast
        self.saturation = Main.saturation
        self.hue = Main.hue
        self.gainR = Main.gainR
        self.gainL = Main.gainL
        self.exposure = Main.exposure
        self.whiteBalance = Main.whiteBalance
        self.focus = Main.focus
        self.serialPort = Main.serialPort
        self.laserHalfWidth = Main.laserHalfWidth

        self.STATE = Main.STATE

        if self.STATE == 11:
            self.laserData = Main.laserData

    def __del__(self):
        self.exiting = True
        self.wait()

    def run(self):
        if ((self.STATE == 10 or self.STATE == 16) and not self.abort) and not self.isLaserLeveled:
            # By default the program detects the optimal
            #     (1) laser threshold R
            #     (2) laser threshold L
            #     (3) laser level
            # Developers can set the laser as "leveled" to skip the automatic image adjusting (brightness) and laser leveling steps
            self.emit(QtCore.SIGNAL("message(PyQt_PyObject)"), 'Tuning laser...')

            self.ser = serial.Serial(self.serialPort, 9600)
            time.sleep(2) # Always wait for a couple seconds to make sure the serial is connected, otherwise following codes may get stuck

            serialSignal(self.ser, 1) # Turn off laser

            if not self.abort:
                self.cam = cv2.VideoCapture(self.camIndexR)
                configCam(self.cam, self.imgLongSide, self.imgShortSide, self.brightness, self.contrast, self.saturation, self.hue, self.gainR, self.exposure, self.whiteBalance, self.focus)
                self.imgAverage = np.zeros((self.imgLongSide, self.imgShortSide), np.float) # from here---
                for i in range(self.nFrame):
                    self.imgIn = self.cam.read()[1]
                    self.imgIn = self.imgIn[:,:,2]
                    self.imgIn = np.rot90(self.imgIn, 1)
                    self.imgAverage = self.imgAverage + self.imgIn
                self.imgAverage = self.imgAverage/self.nFrame # ---to here: acquires an image
                self.laserThresR = np.average(self.imgAverage) + 20
                self.emit(QtCore.SIGNAL("bestLaserThresR(PyQt_PyObject)"), self.laserThresR)
                self.cam.release()

            if not self.abort:
                self.cam = cv2.VideoCapture(self.camIndexL)
                configCam(self.cam, self.imgLongSide, self.imgShortSide, self.brightness, self.contrast, self.saturation, self.hue, self.gainL, self.exposure, self.whiteBalance, self.focus)
                self.imgAverage = np.zeros((self.imgLongSide, self.imgShortSide), np.float) # from here---
                for i in range(self.nFrame):
                    self.imgIn = self.cam.read()[1]
                    self.imgIn = self.imgIn[:,:,2]
                    self.imgIn = np.rot90(self.imgIn, 1)
                    self.imgAverage = self.imgAverage + self.imgIn
                self.imgAverage = self.imgAverage/self.nFrame # ---to here: acquires an image
                self.laserThresL = np.average(self.imgAverage) + 20
                self.emit(QtCore.SIGNAL("bestLaserThresL(PyQt_PyObject)"), self.laserThresL)
                self.cam.release()

            # Start measuring average laser brightness in the right camera
            if not self.abort:
                self.cam = cv2.VideoCapture(self.camIndexR)
                configCam(self.cam, self.imgLongSide, self.imgShortSide, self.brightness, self.contrast, self.saturation, self.hue, self.gainR, self.exposure, self.whiteBalance, self.focus)

                laserResultsArray_R = np.zeros(16, np.float)

                for level in range(16): # Iterate from laser level 1 to 16
                    serialSignal(self.ser, level+1)

                    self.imgAverage = np.zeros((self.imgLongSide, self.imgShortSide), np.float) # from here---
                    for i in range(self.nFrame):
                        self.imgIn = self.cam.read()[1]
                        self.imgIn = self.imgIn[:,:,2]
                        self.imgIn = np.rot90(self.imgIn, 1)
                        self.imgAverage = self.imgAverage + self.imgIn
                    self.imgAverage = self.imgAverage/self.nFrame # ---to here: acquires an image

                    bright_array = laserBrightness(self.imgAverage)
                    z_array = laserPosition(self.imgAverage, self.laserHalfWidth)
                    # Exclude out-of-bound laser points
                    boolArr = np.logical_and(z_array >= self.zMin, z_array <= self.zMax)
                    # Exclude laser points that are dimmer than the laser threshold R
                    # , that is, excluding laser points in the blind side which shouldn't be included for average brightness
                    boolArr = np.logical_and(boolArr, bright_array > self.laserThresR)

                    if np.sum(boolArr) > 0:
                        laserResultsArray_R[level] = np.average(bright_array[boolArr])
                    else: # If there are not points, just assign zero
                        laserResultsArray_R[level] = 0
                self.cam.release()

            # Start measuring average laser brightness in the left camera
            if not self.abort:
                self.cam = cv2.VideoCapture(self.camIndexL)
                configCam(self.cam, self.imgLongSide, self.imgShortSide, self.brightness, self.contrast, self.saturation, self.hue, self.gainL, self.exposure, self.whiteBalance, self.focus)

                laserResultsArray_L = np.zeros(16, np.float)

                for level in range(16): # Iterate from laser level 1 to 16
                    serialSignal(self.ser, level+1)

                    self.imgAverage = np.zeros((self.imgLongSide, self.imgShortSide), np.float) # from here---
                    for i in range(self.nFrame):
                        self.imgIn = self.cam.read()[1]
                        self.imgIn = self.imgIn[:,:,2]
                        self.imgIn = np.rot90(self.imgIn, 1)
                        self.imgAverage = self.imgAverage + self.imgIn
                    self.imgAverage = self.imgAverage/self.nFrame # ---to here: acquires an image

                    self.imgAverage = cv2.warpPerspective(self.imgAverage, self.hMat1, (480,640)) # homography mapping from the left to the right image

                    bright_array = laserBrightness(self.imgAverage)
                    z_array = laserPosition(self.imgAverage, self.laserHalfWidth)

                    boolArr = np.logical_and(z_array >= self.zMin, z_array <= self.zMax)
                    # Exclude out-of-bound laser points
                    boolArr = np.logical_and(boolArr, bright_array > self.laserThresL)
                    # Exclude laser points that are dimmer than the laser threshold R
                    # , that is, excluding laser points in the blind side which shouldn't be included for average brightness

                    if np.sum(boolArr) > 0:
                        laserResultsArray_L[level] = np.average(bright_array[boolArr])
                    else: # If there are not points, just assign zero
                        laserResultsArray_L[level] = 0
                self.cam.release()

            self.ser.close()

            # Average the average laser brightness results from both cameras
            # The laserResultsArray corresponds to laser lever 1 to 16
            laserResultsArray = (laserResultsArray_R + laserResultsArray_L) / 2
            # The best laser level results in the average laser brightness that's closest to the target brightness
            # Algebraically it's done by subtraction -> absolute -> minimum
            bestLaserLevel = np.argmin(np.abs( laserResultsArray - self.targetLaserBrightness )) + 1
            self.emit(QtCore.SIGNAL("bestLaserLevel(PyQt_PyObject)"), bestLaserLevel)
            self.laserLevel = bestLaserLevel

        ### Automatic laser leveling ends here ###

        ### Scanning process starts here ###

        if self.STATE == 10 and not self.abort:
            self.ser = serial.Serial(self.serialPort, 9600) # Serial object is initiated anyway, regardless of self.abort. So it has to be closed later.
            time.sleep(2) # Always wait for a couple seconds to make sure the serial is connected, otherwise following codes may get stuck
            if not self.abort:
                serialSignal(self.ser, self.laserLevel) # Set to the correct laser level

                if self.xStart > self.xEnd:
                    self.xStart, self.xEnd = self.xEnd, self.xStart
                self.xRange = self.xEnd - self.xStart
                self.yRange = self.imgLongSide

            self.emit(QtCore.SIGNAL("message(PyQt_PyObject)"), 'Positioning stage...')
            while self.xCurrent < self.xStart and not self.abort:
                # For every time moving the stage,
                # constantly update the position in the UI object, in case the thread stops prematurely
                # yet the UI object still needs to know where the stage is.
                serialSignal(self.ser, 51)
                self.xCurrent += 1
                self.emit(QtCore.SIGNAL("stagePosition(PyQt_PyObject)"), self.xCurrent)

            while self.xCurrent > self.xStart and not self.abort:
                serialSignal(self.ser, 52)
                self.xCurrent -= 1
                self.emit(QtCore.SIGNAL("stagePosition(PyQt_PyObject)"), self.xCurrent)

            if not self.abort:
                self.TopoR = np.zeros((self.yRange, self.xRange, 2), np.float)
                self.TopoL = np.zeros((self.yRange, self.xRange, 2), np.float)
                if self.savingImg:
                    os.mkdir(self.scanname+'_camR')
                    os.mkdir(self.scanname+'_camL')

            if not self.abort:
                self.cam = cv2.VideoCapture(self.camIndexR)
                configCam(self.cam, self.imgLongSide, self.imgShortSide, self.brightness, self.contrast, self.saturation, self.hue, self.gainR, self.exposure, self.whiteBalance, self.focus)
                for step in range(self.xRange):
                    if not self.abort:
                        self.imgAverage = np.zeros((self.imgLongSide, self.imgShortSide), np.float)
                        for i in range(self.nFrame):
                            self.imgIn = self.cam.read()[1]
                            self.imgIn = self.imgIn[:,:,2]
                            # self.imgIn = cv2.cvtColor(self.imgIn, cv2.COLOR_BGR2GRAY)
                            self.imgIn = np.rot90(self.imgIn, 1)
                            self.imgAverage = self.imgAverage + self.imgIn
                        self.imgAverage = self.imgAverage/self.nFrame
                        if self.savingImg == True:
                            filepath = os.path.join(self.scanname+'_camR', 'camR_%04d'%(step+1)+'.jpg')
                            cv2.imwrite(filepath, self.imgAverage)
                        self.TopoR[:, step, 0] = laserPosition(self.imgAverage, self.laserHalfWidth)
                        self.TopoR[:, step, 1] = laserBrightness(self.imgAverage)
                        percent = str(int( (step+1) / float(self.xRange+1) * 50 ))
                        message = 'Capturing images... ' + percent + '%'
                        if self.isDeveloper:
                            message = message + '\n\nCurrent stage postion is ' + str(self.xCurrent)
                        self.emit(QtCore.SIGNAL("message(PyQt_PyObject)"), message)
                        serialSignal(self.ser, 51)
                        self.xCurrent += 1
                        self.emit(QtCore.SIGNAL("stagePosition(PyQt_PyObject)"), self.xCurrent)
                self.cam.release()

            if not self.abort:
                self.cam = cv2.VideoCapture(self.camIndexL)
                configCam(self.cam, self.imgLongSide, self.imgShortSide, self.brightness, self.contrast, self.saturation, self.hue, self.gainL, self.exposure, self.whiteBalance, self.focus)
                for step in range(self.xRange-1, -1, -1):
                    if not self.abort:
                        self.imgAverage = np.zeros((self.imgLongSide, self.imgShortSide), np.float)
                        for i in range(self.nFrame):
                            self.imgIn = self.cam.read()[1]
                            self.imgIn = self.imgIn[:,:,2]
                            # self.imgIn = cv2.cvtColor(self.imgIn, cv2.COLOR_BGR2GRAY)
                            self.imgIn = np.rot90(self.imgIn, 1)
                            self.imgAverage = self.imgAverage + self.imgIn
                        self.imgAverage = self.imgAverage/self.nFrame
                        if self.savingImg == True:
                            filepath = os.path.join(self.scanname+'_camL', 'camL_%04d'%(step+1)+'.jpg')
                            cv2.imwrite(filepath, self.imgAverage)

                        self.imgAverage = cv2.warpPerspective(self.imgAverage, self.hMat1, (480,640)) # homography mapping from the left to the right image

                        self.TopoL[:, step, 0] = laserPosition(self.imgAverage, self.laserHalfWidth)
                        self.TopoL[:, step, 1] = laserBrightness(self.imgAverage)
                        percent = str( 50 + int( (self.xRange-step+1) / float(self.xRange+1) * 50 ) )
                        message = 'Capturing images... ' + percent + '%'
                        if self.isDeveloper:
                            message = message + '\n\nCurrent stage postion is ' + str(self.xCurrent)
                        self.emit(QtCore.SIGNAL("message(PyQt_PyObject)"), message)
                        serialSignal(self.ser, 52)
                        self.xCurrent -= 1
                        self.emit(QtCore.SIGNAL("stagePosition(PyQt_PyObject)"), self.xCurrent)
                self.cam.release()

            self.ser.close()

            if not self.abort and self.isDeveloper:
                saveArray = np.zeros((self.yRange, self.xRange, 4), np.float)
                saveArray[:,:,0:2] = self.TopoR
                saveArray[:,:,2:4] = self.TopoL
                np.save(self.scanname+'.npy', saveArray)

        elif self.STATE == 12 and not self.abort:
            self.yRange = self.imgLongSide
            self.TopoR = np.zeros((self.yRange, self.xRange, 2), np.float)
            self.TopoL = np.zeros((self.yRange, self.xRange, 2), np.float)
            for step in range(self.xRange):
                if not self.abort:
                    filepath = os.path.join(self.scanname+'_camR', 'camR_%04d'%(step+1)+'.jpg')
                    self.imgAverage = cv2.imread(filepath, cv2.IMREAD_GRAYSCALE)
                    self.TopoR[:, step, 0] = laserPosition(self.imgAverage, self.laserHalfWidth)
                    self.TopoR[:, step, 1] = laserBrightness(self.imgAverage)
                    filepath = os.path.join(self.scanname+'_camL', 'camL_%04d'%(step+1)+'.jpg')
                    self.imgAverage = cv2.imread(filepath, cv2.IMREAD_GRAYSCALE)
                    self.imgAverage = cv2.warpPerspective(self.imgAverage, self.hMat1, (480,640)) # homography mapping from the left to the right image
                    self.TopoL[:, step, 0] = laserPosition(self.imgAverage, self.laserHalfWidth)
                    self.TopoL[:, step, 1] = laserBrightness(self.imgAverage)
                    percent = str( int( (step+1) / float(self.xRange+1) * 100.0 ) )
                    message = 'Processing image... ' + percent + '%'
                    self.emit(QtCore.SIGNAL("message(PyQt_PyObject)"), message)
            if not self.abort and self.isDeveloper:
                saveArray = np.zeros((self.yRange, self.xRange, 4), np.float)
                saveArray[:,:,0:2] = self.TopoR
                saveArray[:,:,2:4] = self.TopoL
                np.save(self.scanname+'.npy', saveArray)

        elif self.STATE == 11 and not self.abort:
            self.TopoR = self.laserData[:,:,0:2]
            self.TopoL = self.laserData[:,:,2:4]
            self.yRange, self.xRange, _ = self.laserData.shape

        elif self.STATE == 16:
            self.abort = True

        if not self.abort:
            self.emit(QtCore.SIGNAL("message(PyQt_PyObject)"), 'Thresholding with laser brightness...')
            self.TopoR[:,:,0] = thresholdTopograph(self.TopoR[:,:,0], self.TopoR[:,:,1], self.laserThresR)
            self.TopoL[:,:,0] = thresholdTopograph(self.TopoL[:,:,0], self.TopoL[:,:,1], self.laserThresL)

        if not self.abort:
            self.emit(QtCore.SIGNAL("message(PyQt_PyObject)"), 'Remove half laser...')
            self.TopoR[:,:,0] = removeHalfLaser(mat=self.TopoR[:,:,0], laserHalfwidth=self.laserHalfWidth, isRightCam=True)
            self.TopoL[:,:,0] = removeHalfLaser(mat=self.TopoL[:,:,0], laserHalfwidth=self.laserHalfWidth, isRightCam=False)

        if not self.abort:
            self.emit(QtCore.SIGNAL("message(PyQt_PyObject)"), 'Removing out-of-bound points...')
            for r in range(self.yRange):
                for c in range(self.xRange):
                    if self.TopoR[r,c,0] > self.zMax or self.TopoR[r,c,0] < self.zMin:
                        self.TopoR[r,c,0] = 0
                    if self.TopoL[r,c,0] > self.zMax or self.TopoL[r,c,0] < self.zMin:
                        self.TopoL[r,c,0] = 0

        if not self.abort:
            self.emit(QtCore.SIGNAL("message(PyQt_PyObject)"), 'Correct linear discrepancy...')
            r_value, slope, intercept = linearDiscrepancy(matZR = self.TopoR[:,:,0], matZL = self.TopoL[:,:,0],)
            bool_mask = self.TopoL[:,:,0] > 0
            self.TopoL[:,:,0][bool_mask] = self.TopoL[:,:,0][bool_mask]*slope + intercept

        if not self.abort and self.isDeveloper:
            cv2.imwrite(self.scanname + '_1R.jpg', generateHeatMap(self.TopoR[:,:,0]))
            cv2.imwrite(self.scanname + '_1L.jpg', generateHeatMap(self.TopoL[:,:,0]))

        if not self.abort:
            self.VERTEX = np.zeros((self.yRange, self.xRange, 3), np.float)
            for c in range(self.xRange):
                self.VERTEX[:,c,0] = c
            for r in range(self.yRange):
                self.VERTEX[r,:,1] = r

            self.emit(QtCore.SIGNAL("message(PyQt_PyObject)"), 'Merging two cameras...')
            self.VERTEX[:,:,2] = mergeTopographs(ZR=self.TopoR[:,:,0], ZL=self.TopoL[:,:,0])

        if not self.abort:
            self.emit(QtCore.SIGNAL("message(PyQt_PyObject)"), 'Filling empty points...')
            self.VERTEX[:,:,2] = fillEmptyWell(self.VERTEX[:,:,2], self.zMin)
            if self.isDeveloper:
                cv2.imwrite(self.scanname + '_2_merged.jpg', generateHeatMap(self.VERTEX[:,:,2]))

        if not self.abort:
            self.emit(QtCore.SIGNAL("message(PyQt_PyObject)"), 'Removing outliers...')
            self.VERTEX[:,:,2] = removeOutlier(self.VERTEX[:,:,2], windowRange=self.denoiseWindowRange, meanThres=self.denoiseMeanThres, noiseThres=self.denoiseNoiseThres)

        if not self.abort:
            self.emit(QtCore.SIGNAL("message(PyQt_PyObject)"), 'Filling empty points...')
            self.VERTEX[:,:,2] = fillEmptyWell(self.VERTEX[:,:,2], self.zMin)
            if self.isDeveloper:
                cv2.imwrite(self.scanname + '_3_denoised.jpg', generateHeatMap(self.VERTEX[:,:,2]))

        if not self.abort:
            # The scaling in x dimension is simple
            self.VERTEX[:,:,0] = self.VERTEX[:,:,0] * self.xScale
            # Perspective homography transformation for the z and y coordinates for real-world scales
            self.emit(QtCore.SIGNAL("message(PyQt_PyObject)"), 'Perspective transformation...')
            self.VERTEX[:,:,1:3] = homographyMapping(self.VERTEX[:,:,1], self.VERTEX[:,:,2], self.hMat2)

        if not self.abort:
            self.emit(QtCore.SIGNAL("message(PyQt_PyObject)"), 'Writing STL file...')
            if self.isDeveloper:
                for i in range(3):
                    self.VERTEX[:,:,i] = self.VERTEX[:,:,i] - np.min(self.VERTEX[:,:,i])
                generateSTL(self.scanname+'.stl', self.VERTEX)

        if not self.abort:
            self.emit(QtCore.SIGNAL("message(PyQt_PyObject)"), 'Centering model coordinates...')
            for i in range(3):
                self.VERTEX[:,:,i] = self.VERTEX[:,:,i] - np.average(self.VERTEX[:,:,i])

        if not self.abort:
            self.emit(QtCore.SIGNAL("vertices(PyQt_PyObject)"), self.VERTEX)

        self.emit(QtCore.SIGNAL("closeUp(PyQt_PyObject)"), self.abort)
        self.abort = False



class CalibThreadManual(QtCore.QThread):
    def __init__(self, Main):
        super(CalibThreadManual, self).__init__()
        self.imgLongSide = Main.imgLongSide
        self.imgShortSide = Main.imgShortSide
        self.camIndexR = Main.camIndexR
        self.camIndexL = Main.camIndexL
        self.nFrame = 10
        self.laserLevel = Main.laserLevel
        self.laserThresR = Main.laserThresR
        self.laserThresL = Main.laserThresL
        self.brightness = Main.brightness
        self.contrast = Main.contrast
        self.saturation = Main.saturation
        self.hue = Main.hue
        self.gainR = Main.gainR
        self.gainL = Main.gainL
        self.exposure = Main.exposure
        self.whiteBalance = Main.whiteBalance
        self.focus = Main.focus
        self.serialPort = Main.serialPort
        self.mousePos = None
        self.input = None

    def __del__(self):
        self.exiting = True
        self.wait()

    def run(self):
        if self.serialPort != None:
            self.ser = serial.Serial(self.serialPort, 9600)
            time.sleep(2) # Always wait for a couple seconds to make sure the serial is connected, otherwise following codes may get stuck
            serialSignal(self.ser, 16) # Set laser to the brightest

        # Capture right image
        self.cam = cv2.VideoCapture(self.camIndexR)
        configCam(self.cam, self.imgLongSide, self.imgShortSide, self.brightness, self.contrast, self.saturation, self.hue, self.gainR, self.exposure, self.whiteBalance, self.focus)
        self.imgR = np.zeros((self.imgLongSide, self.imgShortSide, 3), np.float)
        for i in range(self.nFrame):
            self.imgIn = self.cam.read()[1]
            self.imgIn = np.rot90(self.imgIn, 1)
            self.imgR = self.imgR + self.imgIn
        self.imgR = self.imgR/self.nFrame
        self.cam.release()

        # Capture left image
        self.cam = cv2.VideoCapture(self.camIndexL)
        configCam(self.cam, self.imgLongSide, self.imgShortSide, self.brightness, self.contrast, self.saturation, self.hue, self.gainL, self.exposure, self.whiteBalance, self.focus)
        self.imgL = np.zeros((self.imgLongSide, self.imgShortSide, 3), np.float)
        for i in range(self.nFrame):
            self.imgIn = self.cam.read()[1]
            self.imgIn = np.rot90(self.imgIn, 1)
            self.imgL = self.imgL + self.imgIn
        self.imgL = self.imgL/self.nFrame
        self.cam.release()
        if self.serialPort != None:
            self.ser.close()

        # Image capture and serial control are done

        # In the following start to manually and interactively locate points

        self.imgRL = np.zeros((self.imgLongSide, self.imgShortSide*2, 3), np.float)
        self.imgRL[:,0:480,:] = self.imgL
        self.imgRL[:,480:960,:] = self.imgR
        self.imgRL = self.imgRL.astype(np.uint8)
        self.imgRL = cv2.cvtColor(self.imgRL, cv2.COLOR_BGR2RGB)
        self.emit(QtCore.SIGNAL("image(PyQt_PyObject)"), self.imgRL)

        self.fourPointsR = np.zeros((4, 2), np.float32) # the function cv2.getPerspectiveTransform accepts np.float32 but not np.float64
        self.fourPointsL = np.zeros((4, 2), np.float32)
        self.realCoordinates = np.zeros((4, 2), np.float32)

        self.emit(QtCore.SIGNAL("msgbox(PyQt_PyObject)"), 'Please locate four points of the right camera.')

        # Locate the four points of the right camera
        # Also input the real coordinates of those four points
        for i in range(4):
            while self.mousePos == None:
                time.sleep(0.05)

            z = self.mousePos.x()
            y = self.mousePos.y() - 30
            w = 4 + 4 * i
            self.imgRL[y, (z-w):(z+w), :] = 255
            self.imgRL[(y-w):(y+w), z, :] = 255
            self.emit(QtCore.SIGNAL("image(PyQt_PyObject)"), self.imgRL)
            self.fourPointsR[i,:] = [y, (z-480)]

            self.mousePos = None

            # Wait for the input of the real Y coordinate of point i
            self.emit(QtCore.SIGNAL("waitForInput(PyQt_PyObject)"), 'Y (mm) = ')

            while self.input == None:
                time.sleep(0.05)

            # point i, real Y coordinate
            self.realCoordinates[i, 0] = self.input
            self.input = None

            # Wait for the input of the real Z coordinate of point i
            self.emit(QtCore.SIGNAL("waitForInput(PyQt_PyObject)"), 'Z (mm) = ')

            while self.input == None:
                time.sleep(0.05)

            # point i, real Z coordinate
            self.realCoordinates[i, 1] = self.input
            self.input = None

        text = 'The input coordinates are\n' + \
               str(self.realCoordinates[0, :]) + '\n' + \
               str(self.realCoordinates[1, :]) + '\n' + \
               str(self.realCoordinates[2, :]) + '\n' + \
               str(self.realCoordinates[3, :]) + '\n\n' + \
               'Please locate four points of the left camera.'

        self.emit(QtCore.SIGNAL("msgbox(PyQt_PyObject)"), text)

        # Locate the four points of the left camera
        for i in range(4):
            while self.mousePos == None:
                time.sleep(0.05)

            z = self.mousePos.x()
            y = self.mousePos.y() - 30
            w = 4 + 4 * i
            self.imgRL[y, (z-w):(z+w), :] = 255
            self.imgRL[(y-w):(y+w), z, :] = 255
            self.emit(QtCore.SIGNAL("image(PyQt_PyObject)"), self.imgRL)

            self.fourPointsL[i,:] = [y, z]

            self.mousePos = None

        src = self.fourPointsL[:,::-1] # invert from (y, z) to (z, y) to follow the coordinate system of opencv
        dst = self.fourPointsR[:,::-1]
        self.hMat1 = cv2.getPerspectiveTransform(src, dst)

        self.imgL = cv2.warpPerspective(self.imgL, self.hMat1, (480,640))
        self.imgRL = np.zeros((self.imgLongSide, self.imgShortSide, 3), np.uint8)
        R = cv2.cvtColor(self.imgR.astype(np.uint8), cv2.COLOR_BGR2GRAY)
        L = cv2.cvtColor(self.imgL.astype(np.uint8), cv2.COLOR_BGR2GRAY)
        self.imgRL[:,:,0] = R
        self.imgRL[:,:,1] = L
        for i in range(4):
            y = int(self.fourPointsR[i,0])
            z = int(self.fourPointsR[i,1])
            self.imgRL[y, (z-8):(z+8), 0] = 255
            self.imgRL[(y-8):(y+8), z, 0] = 255

        src = self.fourPointsR     # four right-camera cooridinates (y, z)
        dst = self.realCoordinates # four real world coordinates (y, z)
        self.hMat2 = cv2.getPerspectiveTransform(src, dst)

        self.emit(QtCore.SIGNAL("image(PyQt_PyObject)"), self.imgRL)
        self.emit(QtCore.SIGNAL("hMat1(PyQt_PyObject)"), self.hMat1)
        self.emit(QtCore.SIGNAL("hMat2(PyQt_PyObject)"), self.hMat2)



class CamSelectThread(QtCore.QThread):
    def __init__(self, Main):
        super(CamSelectThread, self).__init__()
        self.imgLongSide = Main.imgLongSide
        self.imgShortSide = Main.imgShortSide
        self.brightness = Main.brightness
        self.contrast = Main.contrast
        self.saturation = Main.saturation
        self.hue = Main.hue
        self.gainR = Main.gainR
        self.exposure = Main.exposure
        self.whiteBalance = Main.whiteBalance
        self.focus = Main.focus
        self.pause = True

    def __del__(self):
        self.exiting = True
        self.wait()

    def run(self):
        for camIndex in range(10):
            try:
                self.pause = True

                self.cam = cv2.VideoCapture(camIndex)
                configCam(self.cam, self.imgLongSide, self.imgShortSide, self.brightness, self.contrast, self.saturation, self.hue, self.gainR, self.exposure, self.whiteBalance, self.focus)
                self.img = self.cam.read()[1]
                self.img = np.rot90(self.img, 1)
                self.img = cv2.cvtColor(self.img, cv2.COLOR_BGR2RGB)
                self.cam.release()

                self.emit(QtCore.SIGNAL("image(PyQt_PyObject)"), self.img)
                time.sleep(0.2)
                self.emit(QtCore.SIGNAL("camIndex(PyQt_PyObject)"), camIndex)

                while self.pause:
                    time.sleep(0.05)
            except:
                pass



class StepperThread(QtCore.QThread):
    def __init__(self, Main):
        super(StepperThread, self).__init__()
        self.stop = False

        self.ser = Main.ser
        self.xCurrent = Main.xCurrent
        self.xCurrentActual = Main.xCurrentActual

    def __del__(self):
        self.exiting = True
        self.wait()

    def run(self):
        while not self.stop:
            while self.xCurrentActual < self.xCurrent:
                serialSignal(self.ser, 51)
                self.xCurrentActual += 1
            while self.xCurrentActual > self.xCurrent:
                serialSignal(self.ser, 52)
                self.xCurrentActual -= 1
            self.emit(QtCore.SIGNAL("xCurrentActual(PyQt_PyObject)"), self.xCurrentActual)
            time.sleep(0.01) # Prevent overloading CPU when running a continuous while loop



class CamTuningThread(QtCore.QThread):
    def __init__(self, Main):
        super(CamTuningThread, self).__init__()
        self.abort = False

        # The following lines transfer all parameters from the main GUI thread to the scan/process thread
        self.isDeveloper = Main.isDeveloper
        self.scanname = Main.scanname
        self.imgLongSide = Main.imgLongSide
        self.imgShortSide = Main.imgShortSide
        self.camIndexR = Main.camIndexR
        self.camIndexL = Main.camIndexL
        self.nFrame = Main.nFrame
        self.targetLaserBrightness = Main.targetLaserBrightness
        self.laserLevel = Main.laserLevel
        self.laserThresR = Main.laserThresR
        self.laserThresL = Main.laserThresL
        self.hMat1 = Main.hMat1
        self.hMat2 = Main.hMat2
        self.zMin = Main.zMin
        self.zMax = Main.zMax
        self.xStart = Main.xStart
        self.xEnd = Main.xEnd
        self.xRange = 0
        self.xCurrent = Main.xCurrent
        self.xScale = Main.xScale
        self.denoiseWindowRange = Main.denoiseWindowRange
        self.denoiseMeanThres = Main.denoiseMeanThres
        self.denoiseNoiseThres = Main.denoiseNoiseThres
        self.brightness = Main.brightness
        self.contrast = Main.contrast
        self.saturation = Main.saturation
        self.hue = Main.hue
        self.gainR = Main.gainR
        self.gainL = Main.gainL
        self.exposure = Main.exposure
        self.whiteBalance = Main.whiteBalance
        self.focus = Main.focus
        self.serialPort = Main.serialPort
        self.laserHalfWidth = Main.laserHalfWidth

    def __del__(self):
        self.exiting = True
        self.wait()

    def run(self):
        self.emit(QtCore.SIGNAL("message(PyQt_PyObject)"), 'Tuning camera parameters...')

        self.ser = serial.Serial(self.serialPort, 9600)
        time.sleep(2) # Always wait for a couple seconds to make sure the serial is connected, otherwise following codes may get stuck

        serialSignal(self.ser, 16) # Set the laser level to the strongest

        self.cam = cv2.VideoCapture(self.camIndexR)
        configCam(self.cam, self.imgLongSide, self.imgShortSide, self.brightness, self.contrast, self.saturation, self.hue, self.gainR, self.exposure, self.whiteBalance, self.focus)
        for i in range(10): # Read a few images to stabilize the camera
            self.imgIn = self.cam.read()[1]
        self.imgAverage = np.zeros((self.imgLongSide, self.imgShortSide), np.float) # from here---
        for i in range(self.nFrame):
            self.imgIn = self.cam.read()[1]
            self.imgIn = self.imgIn[:,:,2]
            self.imgIn = np.rot90(self.imgIn, 1)
            self.imgAverage = self.imgAverage + self.imgIn
        self.imgAverage = self.imgAverage/self.nFrame # ---to here: acquires an image
        self.cam.release()

        bright_array = laserBrightness(self.imgAverage)
        boolArr = bright_array > self.laserThresR
        # Exclude laser points that are dimmer than the laser threshold R
        # , that is, excluding laser points in the blind side which shouldn't be included for average brightness
        if np.sum(boolArr) > 0:
            averageLaserBrightness_R = np.average(bright_array[boolArr])
        else: # If there are not points, just assign zero
            averageLaserBrightness_R = 0

        averageLaserBrightness_L_array = np.zeros(128, np.float)
        gain_array = np.zeros(128, np.uint8) # corresponding to averageLaserBrightness_L_array

        self.cam = cv2.VideoCapture(self.camIndexL)
        gain = 0
        for ith in range(64):
            gain += 2 # iterate from gain = 2 to 128
            gain_array[ith] = gain

            configCam(self.cam, self.imgLongSide, self.imgShortSide, self.brightness, self.contrast, self.saturation, self.hue, gain, self.exposure, self.whiteBalance, self.focus)
            self.imgAverage = np.zeros((self.imgLongSide, self.imgShortSide), np.float) # from here---
            for i in range(self.nFrame):
                self.imgIn = self.cam.read()[1]
                self.imgIn = self.imgIn[:,:,2]
                self.imgIn = np.rot90(self.imgIn, 1)
                self.imgAverage = self.imgAverage + self.imgIn
            self.imgAverage = self.imgAverage/self.nFrame # ---to here: acquires an image

            bright_array = laserBrightness(self.imgAverage)
            boolArr = bright_array > self.laserThresR
            # Exclude laser points that are dimmer than the laser threshold R
            # , that is, excluding laser points in the blind side which shouldn't be included for average brightness
            if np.sum(boolArr) > 0:
                averageLaserBrightness_L_array[ith] = np.average(bright_array[boolArr])
            else: # If there are not points, just assign zero
                averageLaserBrightness_L_array[ith] = 0

        self.cam.release()

        # The best left camera gain results in the average laser brightness that's closest to
        # the laser brightness of the right camera.
        # Algebraically it's done by subtraction -> absolute -> minimum
        arg = np.argmin(np.abs( averageLaserBrightness_L_array - averageLaserBrightness_R ))
        self.gainL = gain_array[arg]
        self.emit(QtCore.SIGNAL("gainL(PyQt_PyObject)"), self.gainL)

        self.ser.close()



