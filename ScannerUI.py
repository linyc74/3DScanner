import numpy as np
import cv2, serial, time, os, sys, pickle
from scipy import stats
from PyQt4 import QtCore, QtGui, QtOpenGL
from OpenGL import GL
from ScannerFunction import *
from ScannerThread import *



class DentalGLWidget(QtOpenGL.QGLWidget):
    def __init__(self, parent=None):
        super(DentalGLWidget, self).__init__(parent)
        self.xRot = 0
        self.yRot = 0
        self.zRot = 0
        self.xMove = 0.0
        self.yMove = 0.0
        self.zoom = 10.0
        self.lastPos = QtCore.QPoint()

    def initializeGL(self):
        empty = np.random.random(size=(10,10,3))
        self.objectMale = self.makeObject(vertices=empty, isMale=True)
        self.objectFemale = self.makeObject(vertices=empty, isMale=False)

        self.activeObject = self.objectMale
        GL.glClearColor(1.0, 1.0, 1.0, 1.0)
        GL.glShadeModel(GL.GL_SMOOTH)
        GL.glEnable(GL.GL_DEPTH_TEST)

        GL.glLightfv(GL.GL_LIGHT0, GL.GL_POSITION, (0.0, 0.0, 0.0, 1.0))
        GL.glLightfv(GL.GL_LIGHT0, GL.GL_AMBIENT, (0.2, 0.2, 0.2, 1.0))
        GL.glLightfv(GL.GL_LIGHT0, GL.GL_DIFFUSE, (1.0, 1.0, 1.0, 1.0))
        # GL.glLightfv(GL.GL_LIGHT0, GL.GL_DIFFUSE, (0.0, 0.0, 0.0, 1.0))
        GL.glEnable(GL.GL_LIGHTING)
        GL.glEnable(GL.GL_LIGHT0)

        GL.glColorMaterial(GL.GL_FRONT_AND_BACK, GL.GL_AMBIENT_AND_DIFFUSE)
        GL.glEnable(GL.GL_COLOR_MATERIAL)

    def resizeGL(self, width, height):
        GL.glViewport(0, 0, 960, 640)
        GL.glMatrixMode(GL.GL_PROJECTION)
        GL.glLoadIdentity()
        GL.glOrtho(-self.zoom*960/640, +self.zoom*960/640, +self.zoom, -self.zoom, 1.0, 200.0)
        GL.glMatrixMode(GL.GL_MODELVIEW)

    def paintGL(self):
        GL.glClear(GL.GL_COLOR_BUFFER_BIT | GL.GL_DEPTH_BUFFER_BIT)
        GL.glLoadIdentity()
        GL.glTranslated(self.xMove, self.yMove, -100.0)
        GL.glRotated(self.xRot / 16.0, 1.0, 0.0, 0.0)
        GL.glRotated(self.yRot / 16.0, 0.0, 1.0, 0.0)
        GL.glRotated(self.zRot / 16.0, 0.0, 0.0, 1.0)
        GL.glCallList(self.activeObject)

    def makeObject(self, vertices, isMale):
        V = vertices
        yRange, xRange, _ = V.shape

        genList = GL.glGenLists(1)
        GL.glNewList(genList, GL.GL_COMPILE)

        GL.glBegin(GL.GL_TRIANGLES)

        if isMale:
            GL.glColor3f(0.5, 0.6, 0.7)
        else:
            GL.glColor3f(0.5, 0.7, 0.6)

        for r in xrange(yRange-1):
            for c in xrange(xRange-1):
                P1 = V[r  ,c  ,0:3]
                P2 = V[r  ,c+1,0:3]
                P3 = V[r+1,c  ,0:3]
                P4 = V[r+1,c+1,0:3]

                N1 = np.cross(P2-P1, P4-P1)

                N1 = N1 / np.sqrt(np.sum(N1*N1))

                N2 = np.cross(P4-P1, P3-P1)
                N2 = N2 / np.sqrt(np.sum(N2*N2))

                if not isMale:
                    N1 = N1 * (-1)
                    N2 = N2 * (-1)

                GL.glNormal3f(N1[0], N1[1], N1[2])
                GL.glVertex3f(P1[0], P1[1], P1[2])
                GL.glVertex3f(P4[0], P4[1], P4[2])
                GL.glVertex3f(P2[0], P2[1], P2[2])

                GL.glNormal3f(N2[0], N2[1], N2[2])
                GL.glVertex3f(P1[0], P1[1], P1[2])
                GL.glVertex3f(P3[0], P3[1], P3[2])
                GL.glVertex3f(P4[0], P4[1], P4[2])

        P1 = V[0       , 0       , 0:3]
        P2 = V[yRange-1, 0       , 0:3]
        P3 = V[0       , xRange-1, 0:3]
        P4 = V[yRange-1, xRange-1, 0:3]

        N1 = np.cross(P2-P1, P4-P1)
        N1 = N1 / np.sqrt(np.sum(N1*N1))

        N2 = np.cross(P4-P1, P3-P1)
        N2 = N2 / np.sqrt(np.sum(N2*N2))

        if not isMale:
            N1 = N1 * (-1)
            N2 = N2 * (-1)

        GL.glNormal3f(N1[0], N1[1], N1[2])
        GL.glVertex3f(P1[0], P1[1], P1[2])
        GL.glVertex3f(P4[0], P4[1], P4[2])
        GL.glVertex3f(P2[0], P2[1], P2[2])

        GL.glNormal3f(N2[0], N2[1], N2[2])
        GL.glVertex3f(P1[0], P1[1], P1[2])
        GL.glVertex3f(P3[0], P3[1], P3[2])
        GL.glVertex3f(P4[0], P4[1], P4[2])

        GL.glEnd()

        GL.glEndList()
        return genList

    def mousePressEvent(self, event):
        self.lastPos = QtCore.QPoint(event.pos())

    def mouseMoveEvent(self, event):
        dx = event.x() - self.lastPos.x()
        dy = event.y() - self.lastPos.y()
        if event.buttons() & QtCore.Qt.LeftButton:
            self.setXRotation(self.xRot - 4 * dy)
            self.setZRotation(self.zRot - 4 * dx)
        elif event.buttons() & QtCore.Qt.RightButton:
            self.xMove = self.xMove + dx*self.zoom/200.0
            self.yMove = self.yMove + dy*self.zoom/200.0
            self.updateGL()
        self.lastPos = QtCore.QPoint(event.pos())

    def wheelEvent(self, event):
        if self.zoom - event.delta()/600.0 > 0:
            self.zoom = self.zoom - event.delta()/600.0
            GL.glMatrixMode(GL.GL_PROJECTION)
            GL.glLoadIdentity()
            GL.glOrtho(-self.zoom*960/640, +self.zoom*960/640, +self.zoom, -self.zoom, 1.0, 200.0)
            GL.glMatrixMode(GL.GL_MODELVIEW)
            self.updateGL()

    def setXRotation(self, angle):
        angle = self.normalizeAngle(angle)
        if angle != self.xRot:
            self.xRot = angle
            self.updateGL()

    def setZRotation(self, angle):
        angle = self.normalizeAngle(angle)
        if angle != self.zRot:
            self.zRot = angle
            self.updateGL()

    def normalizeAngle(self, angle):
        while angle < 0:
            angle += 360 * 16
        while angle > 360 * 16:
            angle -= 360 * 16
        return angle

    def renewObject(self, vertices, isMale):
        if isMale:
            self.objectMale = self.makeObject(vertices, isMale)
        else:
            self.objectFemale = self.makeObject(vertices, isMale)

    def showMale(self):
        self.activeObject = self.objectMale
        self.updateGL()

    def showFemale(self):
        self.activeObject = self.objectFemale
        self.updateGL()



class MainWindow(QtGui.QMainWindow):
    def __init__(self):
        super(MainWindow, self).__init__()
        self.setWindowTitle('TERO')
        self.setWindowIcon(QtGui.QIcon('icon_bear.png'))
        self.setGeometry(50, 50, 960, 670)
        self.setFixedSize(960, 670)

        self.isDeveloper = False
        QtGui.QShortcut(QtGui.QKeySequence("Ctrl+Shift+D"), self, self.toggleDeveloper)

        self.defaultParameters()
        self.UIfonts()
        self.UIbasic()
        self.UItoolBar()
        self.UIparameterSlots()
        self.displayParameters()
        self.UIparameterChange()

        self.STATE = -1
        self.configureSTATE(state=0)

        self.setMouseTracking(True)

        self.show()

    def defaultParameters(self):
        P = pickle.load(open('ScannerParameters.txt', 'r'))
        self.scanname = P['scanname']
        self.imgLongSide = P['imgLongSide']
        self.imgShortSide = P['imgShortSide']
        self.camIndexR = P['camIndexR']
        self.camIndexL = P['camIndexL']
        self.targetLaserBrightness = P['targetLaserBrightness']
        self.laserLevel = P['laserLevel']
        self.laserThresR = P['laserThresR']
        self.laserThresL = P['laserThresL']
        self.nFrame = P['nFrame']
        self.hMat1 = P['hMat1']
        self.hMat2 = P['hMat2']
        self.zMin = P['zMin']
        self.zMax = P['zMax']
        self.xScale = P['xScale']
        self.denoiseWindowRange = P['denoiseWindowRange']
        self.denoiseMeanThres = P['denoiseMeanThres']
        self.denoiseNoiseThres = P['denoiseNoiseThres']
        self.brightness = P['brightness']
        self.contrast = P['contrast']
        self.saturation = P['saturation']
        self.hue = P['hue']
        self.gainR = P['gainR']
        self.gainL = P['gainL']
        self.exposure = P['exposure']
        self.whiteBalance = P['whiteBalance']
        self.focus = P['focus']
        self.xCurrent = P['xCurrent']
        self.laserHalfWidth = P['laserHalfWidth']

        self.xStart = None
        self.xEnd = None
        self.xCurrentActual = self.xCurrent

        self.serialPort = getSerialPort(self.laserLevel) # Use the signal of the laser level to find the correct serial port of our Arduino UNO
        if not self.serialPort == None:
            print 'Arduino UNO detected on serial port ' + self.serialPort + '.'
        else:
            print 'Arduino UNO not detected.'

        # Detect if camIndexR and camIndexL are operable cameras
        # Two criteria:
        # (1) camIndexR and camIndexL can both be opened by OpenCV
        # (2) camIndexR and camIndexL can both be set to the correct image size
        # These criteria do not ensure the right and left cameras are really correct,
        # but they at least ensures the image sizes are correct so can be displayed in the UI
        self.isCamOperable = detectCamOperable(self.camIndexR, self.camIndexL, self.imgLongSide, self.imgShortSide)

        self.isLedOn = False

        # self.hMat1 = np.array([[1,2,3],[1,2,3],[0,0,1]], np.float) # Calibration
        # self.hMat2 = np.array([[1,2,3],[1,2,3],[1,1,1]], np.float) # Calibration

    def UIfonts(self):
        self.font1 = QtGui.QFont()
        self.font1.setFamily("Segoe UI")
        self.font1.setBold(False)
        self.font1.setPixelSize(14)
        self.font2 = QtGui.QFont()
        self.font2.setFamily("Calibri")
        self.font2.setBold(False)
        self.font2.setPixelSize(12)
        self.font3 = QtGui.QFont()
        self.font3.setFamily("Eras Medium ITC")
        self.font3.setBold(False)
        self.font3.setPixelSize(32)
        self.font4 = QtGui.QFont()
        self.font4.setFamily("Segoe UI")
        self.font4.setBold(True)
        self.font4.setPixelSize(14)

    def UIbasic(self):
        self.GLWidget = DentalGLWidget(self)
        self.GLWidget.setGeometry(0, 30, 960, 640)
        self.scanMonitor = QtGui.QLabel(self)
        self.scanMonitor.setGeometry(0, 30, 960, 640)
        self.scanMonitor.setAlignment(QtCore.Qt.AlignCenter)
        self.btnXStart = QtGui.QPushButton('From', self)
        self.btnXStart.setGeometry(0, 640+30-50, 80, 50)
        self.btnXStart.setFont(self.font1)
        self.btnXStart.clicked.connect(self.setXStart)
        self.btnXEnd = QtGui.QPushButton('To', self)
        self.btnXEnd.setGeometry(80, 640+30-50, 80, 50)
        self.btnXEnd.setFont(self.font1)
        self.btnXEnd.clicked.connect(self.setXEnd)
        self.btnStartScan = QtGui.QPushButton('SCAN', self)
        self.btnStartScan.setGeometry(860, 640+30-50, 100, 50)
        self.btnStartScan.setFont(self.font1)
        self.btnStartScan.clicked.connect(self.STATE_10_launchScan)
        self.btnToggleMode = QtGui.QPushButton('SCANNER', self)
        self.btnToggleMode.setGeometry(860, 640+30-50, 100, 50)
        self.btnToggleMode.setFont(self.font1)
        self.btnToggleMode.clicked.connect(self.setSTATE_0)
        self.instantInfo = QtGui.QLabel(self)
        self.instantInfo.setGeometry(0, 670, 960, 60)
        self.instantInfo.setAlignment(QtCore.Qt.AlignLeft)
        self.instantInfo.setFont(self.font2)

    def UItoolBar(self):
        self.action_OpenScanModel = QtGui.QAction(QtGui.QIcon('icon_open_model.png'), 'Open Scanned Model', self)
        self.action_OpenScanModel.triggered.connect(self.openScanModel)
        self.action_ProcLaserData = QtGui.QAction(QtGui.QIcon('icon_open_laser_data.png'), 'Process Laser Data', self)
        self.action_ProcLaserData.triggered.connect(self.STATE_11_procLaserData)
        self.action_ProcRawImg = QtGui.QAction(QtGui.QIcon('icon_open_raw_img.png'), 'Process Camera Images', self)
        self.action_ProcRawImg.triggered.connect(self.STATE_12_procCamImg)
        self.action_SaveParameters = QtGui.QAction(QtGui.QIcon('icon_save_parameters.png'), 'Save Parameters', self)
        self.action_SaveParameters.triggered.connect(self.saveParameters)
        self.action_CamCalibManual = QtGui.QAction(QtGui.QIcon('icon_calibration.png'), 'Camera Calibration Manual', self)
        self.action_CamCalibManual.triggered.connect(self.STATE_14_camCalibManual)
        self.action_SwitchCam = QtGui.QAction(QtGui.QIcon('icon_switch_cam.png'), 'Switch Camera', self)
        self.action_SwitchCam.triggered.connect(self.switchCam)
        self.action_CamSelectManual = QtGui.QAction(QtGui.QIcon('icon_select_cam.png'), 'Select Camera', self)
        self.action_CamSelectManual.triggered.connect(self.STATE_15_camSelectManual)
        self.action_AutoLaserLeveling = QtGui.QAction(QtGui.QIcon('icon_laser_level.png'), 'Auto Laser Level', self)
        self.action_AutoLaserLeveling.triggered.connect(self.STATE_16_laserLevelingAuto)
        self.action_Snapshot = QtGui.QAction(QtGui.QIcon('icon_snap.png'), 'Snapshot', self)
        self.action_Snapshot.triggered.connect(self.snapshot)
        self.action_CamTuning = QtGui.QAction(QtGui.QIcon('icon_cam_tuning.png'), 'Tune Camera Parameters', self)
        self.action_CamTuning.triggered.connect(self.STATE_17_camTuning)
        self.action_ToggleLED = QtGui.QAction(QtGui.QIcon('icon_LED.png'), 'LED On/Off', self)
        self.action_ToggleLED.triggered.connect(self.toggleLED)


        self.toolbar1 = self.addToolBar('Scanner Tools')
        self.toolbar1.setMovable(False)
        self.toolbar1.setStyleSheet("QToolBar { background:white; }")
        self.toolbar1.setIconSize(QtCore.QSize(45, 30))
        self.toolbar1_btn01 = self.toolbar1.addAction(self.action_OpenScanModel)
        self.toolbar1_sep10 = self.toolbar1.addSeparator()
        self.toolbar1_btn11 = self.toolbar1.addAction(self.action_CamSelectManual)
        self.toolbar1_btn12 = self.toolbar1.addAction(self.action_CamCalibManual)
        self.toolbar1_btn13 = self.toolbar1.addAction(self.action_CamTuning)
        self.toolbar1_btn14 = self.toolbar1.addAction(self.action_AutoLaserLeveling)
        self.toolbar1_btn15 = self.toolbar1.addAction(self.action_SaveParameters)
        self.toolbar1_sep20 = self.toolbar1.addSeparator()
        self.toolbar1_btn21 = self.toolbar1.addAction(self.action_SwitchCam)
        self.toolbar1_btn22 = self.toolbar1.addAction(self.action_ProcLaserData)
        self.toolbar1_btn23 = self.toolbar1.addAction(self.action_ProcRawImg)
        self.toolbar1_btn24 = self.toolbar1.addAction(self.action_Snapshot)
        self.toolbar1_btn25 = self.toolbar1.addAction(self.action_ToggleLED)

        self.toolbar1_sep10.setVisible(self.isDeveloper)
        self.toolbar1_sep20.setVisible(self.isDeveloper)
        self.action_SwitchCam.setVisible(self.isDeveloper)
        self.action_CamSelectManual.setVisible(self.isDeveloper)
        self.action_CamCalibManual.setVisible(self.isDeveloper)
        self.action_ProcLaserData.setVisible(self.isDeveloper)
        self.action_ProcRawImg.setVisible(self.isDeveloper)
        self.action_SaveParameters.setVisible(self.isDeveloper)
        self.action_AutoLaserLeveling.setVisible(self.isDeveloper)
        self.action_Snapshot.setVisible(self.isDeveloper)
        self.action_CamTuning.setVisible(self.isDeveloper)
        self.action_ToggleLED.setVisible(self.isDeveloper)

        self.action_showMaleCast = QtGui.QAction(QtGui.QIcon('icon_show_male.png'), 'Show Male Cast', self)
        self.action_showMaleCast.triggered.connect(self.showMaleCast)
        self.action_generateStent = QtGui.QAction(QtGui.QIcon('icon_generate_stent.png'), 'Generate Stent', self)
        self.action_generateStent.triggered.connect(self.generateStent)
        self.action_blank = QtGui.QAction(QtGui.QIcon('icon_blank.png'), 'Blank', self)

        self.toolbar2 = self.addToolBar('CAD/CAM Tools')
        self.toolbar2.setMovable(False)
        self.toolbar2.setStyleSheet("QToolBar { background:white; }")
        self.toolbar2.setIconSize(QtCore.QSize(45, 30))
        self.toolbar2_btn01 = self.toolbar2.addAction(self.action_OpenScanModel)
        self.toolbar2_btn02 = self.toolbar2.addAction(self.action_showMaleCast)
        self.toolbar2_btn03 = self.toolbar2.addAction(self.action_generateStent)
        # self.toolbar2_btn99 = self.toolbar2.addAction(self.action_blank)

    def UIparameterSlots(self):
        self.param_01 = QtGui.QLabel(self)
        self.param_01.setGeometry(1000, 30, 280, 30)
        self.param_02 = QtGui.QLabel(self)
        self.param_02.setGeometry(1000, 60, 280, 30)
        self.param_03 = QtGui.QLabel(self)
        self.param_03.setGeometry(1000, 90, 280, 30)
        self.param_04 = QtGui.QLabel(self)
        self.param_04.setGeometry(1000, 120, 280, 30)
        self.param_05 = QtGui.QLabel(self)
        self.param_05.setGeometry(1000, 150, 280, 30)
        self.param_06 = QtGui.QLabel(self)
        self.param_06.setGeometry(1000, 180, 280, 30)
        self.param_07 = QtGui.QLabel(self)
        self.param_07.setGeometry(1000, 210, 280, 30)
        self.param_08 = QtGui.QLabel(self)
        self.param_08.setGeometry(1000, 240, 280, 30)
        self.param_09 = QtGui.QLabel(self)
        self.param_09.setGeometry(1000, 270, 280, 30)
        self.param_10 = QtGui.QLabel(self)
        self.param_10.setGeometry(1000, 300, 280, 30)
        self.param_11 = QtGui.QLabel(self)
        self.param_11.setGeometry(1000, 330, 280, 30)
        self.param_12 = QtGui.QLabel(self)
        self.param_12.setGeometry(1000, 360, 280, 30)
        self.param_13 = QtGui.QLabel(self)
        self.param_13.setGeometry(1000, 390, 280, 30)
        self.param_14 = QtGui.QLabel(self)
        self.param_14.setGeometry(1000, 420, 280, 30)
        self.param_15 = QtGui.QLabel(self)
        self.param_15.setGeometry(1000, 450, 280, 30)
        self.param_16 = QtGui.QLabel(self)
        self.param_16.setGeometry(1000, 480, 280, 30)
        self.param_17 = QtGui.QLabel(self)
        self.param_17.setGeometry(1000, 510, 280, 30)
        self.param_18 = QtGui.QLabel(self)
        self.param_18.setGeometry(1000, 540, 280, 30)
        self.param_19 = QtGui.QLabel(self)
        self.param_19.setGeometry(1000, 570, 280, 30)
        self.param_20 = QtGui.QLabel(self)
        self.param_20.setGeometry(1000, 600, 280, 30)
        self.param_21 = QtGui.QLabel(self)
        self.param_21.setGeometry(1000, 630, 280, 30)
        self.param_22 = QtGui.QLabel(self)
        self.param_22.setGeometry(1000, 660, 280, 30)
        self.param_23 = QtGui.QLabel(self)
        self.param_23.setGeometry(1000, 690, 280, 30)
        self.param_24 = QtGui.QLabel(self)
        self.param_24.setGeometry(1000, 720, 280, 30)
        self.param_25 = QtGui.QLabel(self)
        self.param_25.setGeometry(1000, 750, 280, 30)
        self.param_26 = QtGui.QLabel(self)
        self.param_26.setGeometry(1000, 780, 280, 30)

        self.param_01.setFont(self.font2)
        self.param_02.setFont(self.font2)
        self.param_03.setFont(self.font2)
        self.param_04.setFont(self.font2)
        self.param_05.setFont(self.font2)
        self.param_06.setFont(self.font2)
        self.param_07.setFont(self.font2)
        self.param_08.setFont(self.font2)
        self.param_09.setFont(self.font2)
        self.param_10.setFont(self.font2)
        self.param_11.setFont(self.font2)
        self.param_12.setFont(self.font2)
        self.param_13.setFont(self.font2)
        self.param_14.setFont(self.font2)
        self.param_15.setFont(self.font2)
        self.param_16.setFont(self.font2)
        self.param_17.setFont(self.font2)
        self.param_18.setFont(self.font2)
        self.param_19.setFont(self.font2)
        self.param_20.setFont(self.font2)
        self.param_21.setFont(self.font2)
        self.param_22.setFont(self.font2)
        self.param_23.setFont(self.font2)
        self.param_24.setFont(self.font2)
        self.param_25.setFont(self.font2)
        self.param_26.setFont(self.font2)

    def displayParameters(self):
        self.param_01.setText('Scan name = ' + str(self.scanname))
        self.param_02.setText('Cam IndexR = ' + str(self.camIndexR))
        self.param_03.setText('Cam IndexL = ' + str(self.camIndexL))
        self.param_04.setText('Frame Average = ' + str(self.nFrame) + ' (1 - 10)')

        self.param_05.setText('----- Laser Threshold R = ' + str(self.laserThresR) + ' (0 - 255)')
        self.param_06.setText('----- Laser Threshold L = ' + str(self.laserThresL) + ' (0 - 255)')
        self.param_07.setText('----- Target Laser Brightness = ' + str(self.targetLaserBrightness) + ' (0 - 255)')
        self.param_08.setText('----- Laser Level = ' + str(self.laserLevel) + ' (1 - 16)')

        self.param_09.setText('X Start = ' + str(self.xStart))
        self.param_10.setText('X End = ' + str(self.xEnd))
        self.param_11.setText('X Current = ' + str(self.xCurrent) + ' (' + str(self.xCurrentActual) + ')')

        self.param_12.setText('----- Camera Brightness = ' + str(self.brightness) + ' (0 - 255)')
        self.param_13.setText('----- Camera Contrast = ' + str(self.contrast) + ' (0 - 255)')
        self.param_14.setText('----- Camera Gain R = ' + str(self.gainR) + ' (0 - 255)')
        self.param_15.setText('----- Camera Gain L = ' + str(self.gainL) + ' (0 - 255)')
        self.param_16.setText('----- Camera Exposure = ' + str(self.exposure) + ' (-1 - -7)')
        self.param_17.setText('----- Camera Focus = ' + str(self.focus) + ' (0 - 255)')

        self.param_18.setText('X Scale = ' + str(self.xScale) + ' mm/step')
        t = str(self.hMat1.astype(np.int).reshape((9, )))
        self.param_19.setText('hMat1 = ' + t)
        t = str(self.hMat2.astype(np.int).reshape((9, )))
        self.param_20.setText('hMat2 = ' + t)

        self.param_21.setText('Denoise Window Range = ' + str(self.denoiseWindowRange) + ' (smaller = finer)')
        self.param_22.setText('Denoise Mean Threshold = ' + str(self.denoiseMeanThres) + ' (smaller = stricter)')
        self.param_23.setText('Denoise Noise Threshold = ' + str(self.denoiseNoiseThres) + ' (smaller = stricter)')

        self.param_24.setText('Z Max = ' + str(self.zMax) + ' (0 - 479)')
        self.param_25.setText('Laser Half Width = ' + str(self.laserHalfWidth) + ' (1 - 20)')

    def UIparameterChange(self):
        self.param_01c = QtGui.QPushButton('...', self)
        self.param_01c.setGeometry(960, 30, 30, 30)
        self.param_01c.clicked.connect(self.changeScanname)
        self.param_04c = QtGui.QPushButton('...', self)
        self.param_04c.setGeometry(960, 30*4, 30, 30)
        self.param_04c.clicked.connect(self.changeNFrame)

        self.param_05c = QtGui.QPushButton('...', self)
        self.param_05c.setGeometry(960, 30*5, 30, 30)
        self.param_05c.clicked.connect(self.changeLaserThresR)
        self.param_06c = QtGui.QPushButton('...', self)
        self.param_06c.setGeometry(960, 30*6, 30, 30)
        self.param_06c.clicked.connect(self.changeLaserThresL)
        self.param_07c = QtGui.QPushButton('...', self)
        self.param_07c.setGeometry(960, 30*7, 30, 30)
        self.param_07c.clicked.connect(self.changeTargetLaserBrightness)
        self.param_08c = QtGui.QPushButton('...', self)
        self.param_08c.setGeometry(960, 30*8, 30, 30)
        self.param_08c.clicked.connect(self.changeLaserLevel)

        self.param_11c = QtGui.QPushButton('...', self)
        self.param_11c.setGeometry(960, 30*11, 30, 30)
        self.param_11c.clicked.connect(self.changeXCurrent)

        self.param_12c = QtGui.QPushButton('...', self)
        self.param_12c.setGeometry(960, 30*12, 30, 30)
        self.param_12c.clicked.connect(self.changeBrightness)
        self.param_13c = QtGui.QPushButton('...', self)
        self.param_13c.setGeometry(960, 30*13, 30, 30)
        self.param_13c.clicked.connect(self.changeContrast)
        self.param_14c = QtGui.QPushButton('...', self)
        self.param_14c.setGeometry(960, 30*14, 30, 30)
        self.param_14c.clicked.connect(self.changeGainR)
        self.param_15c = QtGui.QPushButton('...', self)
        self.param_15c.setGeometry(960, 30*15, 30, 30)
        self.param_15c.clicked.connect(self.changeGainL)
        self.param_16c = QtGui.QPushButton('...', self)
        self.param_16c.setGeometry(960, 30*16, 30, 30)
        self.param_16c.clicked.connect(self.changeExposure)
        self.param_17c = QtGui.QPushButton('...', self)
        self.param_17c.setGeometry(960, 30*17, 30, 30)
        self.param_17c.clicked.connect(self.changeFocus)

        self.param_18c = QtGui.QPushButton('...', self)
        self.param_18c.setGeometry(960, 30*18, 30, 30)
        self.param_18c.clicked.connect(self.changeXScale)

        self.param_21c = QtGui.QPushButton('...', self)
        self.param_21c.setGeometry(960, 30*21, 30, 30)
        self.param_21c.clicked.connect(self.changeDenoiseWindowRange)
        self.param_22c = QtGui.QPushButton('...', self)
        self.param_22c.setGeometry(960, 30*22, 30, 30)
        self.param_22c.clicked.connect(self.changeDenoiseMeanThres)
        self.param_23c = QtGui.QPushButton('...', self)
        self.param_23c.setGeometry(960, 30*23, 30, 30)
        self.param_23c.clicked.connect(self.changeDenoiseNoiseThres)

        self.param_24c = QtGui.QPushButton('...', self)
        self.param_24c.setGeometry(960, 30*24, 30, 30)
        self.param_24c.clicked.connect(self.changeZMax)
        self.param_25c = QtGui.QPushButton('...', self)
        self.param_25c.setGeometry(960, 30*25, 30, 30)
        self.param_25c.clicked.connect(self.changeLaserHalfWidth)

    ###

    def setSTATE_0(self):
        self.configureSTATE(0)

    def setSTATE_10(self):
        self.configureSTATE(10)

    def setSTATE_11(self):
        self.configureSTATE(11)

    def setSTATE_12(self):
        self.configureSTATE(12)

    def setSTATE_13(self):
        self.configureSTATE(13)

    def setSTATE_14(self):
        self.configureSTATE(14)

    def setSTATE_15(self):
        self.configureSTATE(15)

    def setSTATE_16(self):
        self.configureSTATE(16)

    def setSTATE_17(self):
        self.configureSTATE(17)

    def setSTATE_20(self):
        self.configureSTATE(20)

    def configureSTATE(self, state):
        self.GLWidget.hide()
        self.scanMonitor.hide()
        self.btnXStart.hide()
        self.btnXEnd.hide()
        self.btnToggleMode.hide()
        self.btnStartScan.hide()
        self.toolbar1.hide()
        self.toolbar2.hide()

        if self.STATE == 0:
            self.closeIdleMode()

        self.STATE = state

        if state == 0:
            self.scanMonitor.show()
            self.btnXStart.show()
            self.btnXEnd.show()
            self.btnStartScan.show()
            self.btnStartScan.setText('SCAN')
            self.btnStartScan.clicked.disconnect()
            self.btnStartScan.clicked.connect(self.STATE_10_launchScan)
            self.toolbar1.show()

            self.initializeIdleMode()

        elif state >= 10 and state <= 19:
            self.scanMonitor.show()
            if state >= 10 and state <= 12:
                self.btnStartScan.show()
                self.btnStartScan.setText('Abort')
                self.btnStartScan.clicked.disconnect()
                self.btnStartScan.clicked.connect(self.quitScan)
            self.scanMonitor.setAlignment(QtCore.Qt.AlignCenter)
            self.scanMonitor.setFont(self.font3)

        elif state >= 20 and state <= 29:
            self.GLWidget.show()
            self.btnToggleMode.show()
            self.toolbar2.show()

    ###

    def initializeIdleMode(self):
        # There are four objects to be initialized:
        # (1) serial
        # (2) camera
        # (3) QTimer for image
        # (4) stepperThread

        self.rightOrLeft = True

        # (1)
        if self.serialPort != None:
            self.ser = serial.Serial(self.serialPort, 9600)
            time.sleep(2) # Always wait for a couple seconds to make sure the serial is connected, otherwise following codes may get stuck
            if self.isDeveloper:
                serialSignal(self.ser, self.laserLevel) # Set to the prescribed laser level if it's in developer mode
            else:
                serialSignal(self.ser, 16) # Set to the brightest laser level if it's in user mode
                serialSignal(self.ser, 61) # Turn on LED if it's in user mode
                self.isLedOn = True

        # (2)
        if self.isCamOperable == True:
            self.imgBuffer = np.zeros((self.imgLongSide, self.imgShortSide, 10), np.float)
            if self.rightOrLeft == True:
                self.cam = cv2.VideoCapture(self.camIndexR)
                gain = self.gainR
            else:
                self.cam = cv2.VideoCapture(self.camIndexL)
                gain = self.gainL
            configCam(self.cam, self.imgLongSide, self.imgShortSide, self.brightness, self.contrast, self.saturation, self.hue, gain, self.exposure, self.whiteBalance, self.focus)

        # (3)
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.refreshImg)
        self.timer.start(1)

        # (4)
        if self.serialPort != None:
            self.stepperThread = StepperThread(self)
            self.connect(self.stepperThread, QtCore.SIGNAL("xCurrentActual(PyQt_PyObject)"), self.updateXCurrentActual)
            self.stepperThread.start()

    def closeIdleMode(self):
        # Stop the things launched in initializeIdleMode()

        self.timer.stop()

        if self.serialPort != None:
            self.stepperThread.stop = True

        if self.isCamOperable:
            self.cam.release()

        if self.serialPort != None:
            if self.isLedOn: # Turn off the LED if it's on
                serialSignal(self.ser, 62)
                self.isLedOn = False
            self.ser.close()

    def refreshImg(self):
        self.displayParameters()

        if self.isCamOperable == True:
            self.imgIn = self.cam.read()[1]
            self.imgIn = np.rot90(self.imgIn, 1)
            rows, columns, _ = self.imgIn.shape

            self.imgShow = np.zeros((self.imgLongSide, self.imgShortSide, 3), np.uint8)
            self.imgShow[:,:,:] = self.imgIn[:,:,:]

            if self.isDeveloper:
                self.imgBuffer[:,:,1:9] = self.imgBuffer[:,:,0:8]
                self.imgBuffer[:,:,0] = self.imgIn[:,:,2]
                self.imgAverage= np.zeros((self.imgLongSide, self.imgShortSide), np.float)
                for i in range(self.nFrame):
                    self.imgAverage = self.imgAverage + self.imgBuffer[:,:,i]
                self.imgAverage = self.imgAverage/self.nFrame

                z_array = laserPosition(self.imgAverage, self.laserHalfWidth)
                bright_array = laserBrightness(self.imgAverage)

                if self.rightOrLeft:
                    laserThres = self.laserThresR
                else:
                    laserThres = self.laserThresL

                self.imgShow[:, [self.zMin, self.zMax], :] = 127 # Mark the lines for zMin and zMax

                for y in range(self.imgLongSide): # Painting green dots along the laser line
                    if bright_array[y] >= laserThres:
                        if z_array[y] >= self.zMin and z_array[y] <= self.zMax:
                            self.imgShow[y, z_array[y], 1] = 255
                            self.imgShow[y, z_array[y], [0,2]] = 0

                self.imgBrightProfile = np.zeros((self.imgLongSide, self.imgShortSide, 3), np.uint8) + 255 # Create a white image to show the laser brightness profile

                c0 = 110 # The left boundary (first column, hence c0) of the chart
                self.imgBrightProfile[:, [c0,c0+255], :] = 200 # Mark the left and the right boundary lines for brightness 0 and 255
                for y in range(self.imgLongSide): # Paint red dots according to the brightness at each row
                    b = int(bright_array[y]) + c0
                    self.imgBrightProfile[y, b, 0:2] = 0 # white -> red

                if self.rightOrLeft:
                    self.imgBrightProfile[:, self.laserThresR+c0, :] = 0 # Mark the line of the laser threshold
                else:
                    self.imgBrightProfile[:, self.laserThresL+c0, :] = 0

                # Place the image on either the right or the left side of the larger image
                if self.rightOrLeft == True:
                    self.imgShow = combineImg(img1=self.imgBrightProfile, img2=self.imgShow)
                else:
                    self.imgShow = combineImg(img1=self.imgShow, img2=self.imgBrightProfile)

                # Calculate average brightness of the green dots, i.e. detected laser points
                # Green dots have positions between zMin and zMax, and are brighter than the threshold
                A = z_array >= self.zMin
                B = z_array <= self.zMax
                if self.rightOrLeft:
                    C = bright_array >= self.laserThresR
                else:
                    C = bright_array >= self.laserThresL
                bool_array = np.logical_and(A, B)
                bool_array = np.logical_and(bool_array, C)
                if np.sum(bool_array) > 0: # Check the presence of green dots
                    avg = np.average(bright_array[bool_array])
                else:
                    avg = 0 # If there's no green dots, then assign 0 to avg
                infoText = ' Average brightness of green dots = ' + str(avg)
                self.instantInfo.setText(infoText)

            self.imgShow = cv2.cvtColor(self.imgShow, cv2.COLOR_BGR2RGB) # convert from BGR to RGB for latter QImage
            height, width, bytesPerComponent = self.imgShow.shape
            bytesPerLine = bytesPerComponent * width
            Q_img = QtGui.QImage(self.imgShow, width, height, bytesPerLine, QtGui.QImage.Format_RGB888) # convert cv2 image to QImage
            Q_pixmap = QtGui.QPixmap.fromImage(Q_img) # Convert QImage to QPixmap
            self.scanMonitor.setPixmap(Q_pixmap) # Set the QLabel to display the QPixmap

        else:
            self.scanMonitor.setFont(self.font3)
            self.scanMonitor.setText('Camera not ready')

    def wheelEvent(self, event):
        if self.serialPort != None:
            if abs(self.xCurrent - self.xCurrentActual) < 50:
                self.xCurrent = self.xCurrent + event.delta()/120
                self.stepperThread.xCurrent = self.xCurrent

    def mousePressEvent(self, QMouseEvent):
        if self.STATE == 14:
            self.thread.mousePos = QMouseEvent.pos()
        else:
            pass

    def updateXCurrentActual(self, xCurrentActual):
        self.xCurrentActual = xCurrentActual

    ###

    def openScanModel(self):
        fname = QtGui.QFileDialog.getOpenFileName(parent=self, caption='Open scanned model', filter='Model (*.npy)')
        if fname != '':
            fname = str(fname)
            self.verticesMale = np.load(fname)
            self.setSTATE_20()
            self.GLWidget.renewObject(self.verticesMale, isMale=True)
            self.GLWidget.showMale()

    def toggleDeveloper(self):
        self.isDeveloper = not self.isDeveloper

        self.toolbar1_sep10.setVisible(self.isDeveloper)
        self.toolbar1_sep20.setVisible(self.isDeveloper)
        self.action_SwitchCam.setVisible(self.isDeveloper)
        self.action_CamSelectManual.setVisible(self.isDeveloper)
        self.action_CamCalibManual.setVisible(self.isDeveloper)
        self.action_ProcLaserData.setVisible(self.isDeveloper)
        self.action_ProcRawImg.setVisible(self.isDeveloper)
        self.action_SaveParameters.setVisible(self.isDeveloper)
        self.action_AutoLaserLeveling.setVisible(self.isDeveloper)
        self.action_Snapshot.setVisible(self.isDeveloper)
        self.action_CamTuning.setVisible(self.isDeveloper)
        self.action_ToggleLED.setVisible(self.isDeveloper)

        if self.isDeveloper:
            self.setFixedSize(1280, 810)
            self.setMaximumSize(1280, 810)
            self.setMinimumSize(960, 670)
            if self.serialPort != None:
                serialSignal(self.ser, self.laserLevel)
                if self.isLedOn:
                    serialSignal(self.ser, 62)
                    self.isLedOn = False

        else:
            self.setFixedSize(960, 670)
            serialSignal(self.ser, 16)
            if self.serialPort != None:
                if not self.isLedOn:
                    serialSignal(self.ser, 61)
                    self.isLedOn = True

    def showMaleCast(self):
        self.GLWidget.showMale()

    def generateStent(self):
        rowN, colN, dimN = self.verticesMale.shape

        self.verticesFemale = np.zeros((rowN, colN, dimN), np.float)
        self.verticesFemale[:,:,:] = self.verticesMale[:,:,:]
        self.verticesFemale[:,:,[0,2]] = self.verticesFemale[:,:,[0,2]] * (-1)

        baseHeight = np.min(self.verticesFemale[:,:,2]) - 2
        self.verticesFemale[0,     :,2] = baseHeight
        self.verticesFemale[rowN-1,:,2] = baseHeight
        self.verticesFemale[:,0     ,2] = baseHeight
        self.verticesFemale[:,colN-1,2] = baseHeight

        minY = np.min(self.verticesFemale[:,:,1])
        maxY = np.max(self.verticesFemale[:,:,1])
        self.verticesFemale[0,     :,1] = minY
        self.verticesFemale[rowN-1,:,1] = maxY

        for i in range(3):
            self.verticesFemale[:,:,i] = self.verticesFemale[:,:,i] - np.average(self.verticesFemale[:,:,i])

        fname = QtGui.QFileDialog.getSaveFileName(parent=self, directory='TERO Stent.npy', caption='Save stent', filter='Model (*.npy)')
        if fname != '':
            fname = str(fname)
            np.save(fname, self.verticesFemale)
            self.GLWidget.renewObject(self.verticesFemale, isMale=False)
            self.GLWidget.showFemale()

    def setXStart(self):
        if self.serialPort != None:
            if self.xStart == None:
                self.xStart = self.xCurrent
                self.btnXStart.setFont(self.font4)
            else:
                self.xStart = None
                self.btnXStart.setFont(self.font1)

    def setXEnd(self):
        if self.serialPort != None:
            if self.xEnd == None:
                self.xEnd = self.xCurrent
                self.btnXEnd.setFont(self.font4)
            else:
                self.xEnd = None
                self.btnXEnd.setFont(self.font1)

    def switchCam(self):
        if self.isCamOperable:
            self.rightOrLeft = not self.rightOrLeft
            self.cam.release()
            if self.rightOrLeft:
                self.cam = cv2.VideoCapture(self.camIndexR)
                gain = self.gainR
            else:
                self.cam = cv2.VideoCapture(self.camIndexL)
                gain = self.gainL
            configCam(self.cam, self.imgLongSide, self.imgShortSide, self.brightness, self.contrast, self.saturation, self.hue, gain, self.exposure, self.whiteBalance, self.focus)

    def saveParameters(self):
        P = {'scanname': self.scanname,
        'imgLongSide': self.imgLongSide,
        'imgShortSide': self.imgShortSide,
        'camIndexR': self.camIndexR,
        'camIndexL': self.camIndexL,
        'nFrame': self.nFrame,
        'targetLaserBrightness': self.targetLaserBrightness,
        'laserLevel': self.laserLevel,
        'laserThresR': self.laserThresR,
        'laserThresL': self.laserThresL,
        'hMat1': self.hMat1,
        'zMin': self.zMin,
        'zMax': self.zMax,
        'xScale': self.xScale,
        'hMat2': self.hMat2,
        'denoiseWindowRange': self.denoiseWindowRange,
        'denoiseMeanThres': self.denoiseMeanThres,
        'denoiseNoiseThres': self.denoiseNoiseThres,
        'brightness': self.brightness,
        'contrast': self.contrast,
        'saturation': self.saturation,
        'hue': self.hue ,
        'gainR': self.gainR,
        'gainL': self.gainL,
        'exposure': self.exposure,
        'whiteBalance': self.whiteBalance,
        'focus': self.focus,
        'xCurrent': self.xCurrent,
        'laserHalfWidth': self.laserHalfWidth}
        pickle.dump(P, open('ScannerParameters.txt', 'w'))

    def snapshot(self):
        fname = QtGui.QFileDialog.getSaveFileName(parent=self, directory='snapshot.jpg', caption='Save snapshot', filter='Image (*.jpg)')
        if fname != '':
            fname = str(fname)
            cv2.imwrite(fname, self.imgIn)

    def toggleLED(self):
        if self.serialPort != None:
            if self.isLedOn:
                serialSignal(self.ser, 62)
                self.isLedOn = False
            else:
                serialSignal(self.ser, 61)
                self.isLedOn = True

    ###

    def changeScanname(self):
        text, ok = QtGui.QInputDialog.getText(self, ' ', 'Scan Name', text=self.scanname)
        if ok:
            self.scanname = str(text)

    def changeNFrame(self):
        num, ok = QtGui.QInputDialog.getInt(self, ' ', 'Frame Average', self.nFrame)
        if ok:
            if num >= 1 and num <= 10:
                self.nFrame = num

    def changeTargetLaserBrightness(self):
        num, ok = QtGui.QInputDialog.getInt(self, ' ', 'Target Laser Brightness', self.targetLaserBrightness)
        if ok:
            if num >= 0 and num <= 255:
                self.targetLaserBrightness = num

    def changeLaserThresR(self):
        num, ok = QtGui.QInputDialog.getInt(self, ' ', 'Laser Threshold R', self.laserThresR)
        if ok:
            if num >= 0 and num <= 255:
                self.laserThresR = num

    def changeLaserThresL(self):
        num, ok = QtGui.QInputDialog.getInt(self, ' ', 'Laser Threshold L', self.laserThresL)
        if ok:
            if num >= 0 and num <= 255:
                self.laserThresL = num

    def changeXCurrent(self):
        num, ok = QtGui.QInputDialog.getInt(self, ' ', 'X Current Position', self.xCurrent)
        if ok:
            self.stepperThread.xCurrent = num
            self.stepperThread.xCurrentActual = num
            self.xCurrent = num
            self.xCurrentActual = num

    def changeXScale(self):
        num, ok = QtGui.QInputDialog.getDouble(self, ' ', 'X Scale (mm/step)', self.xScale, decimals=10)
        if ok:
            self.xScale = num

    def changeLaserLevel(self):
        num, ok = QtGui.QInputDialog.getInt(self, ' ', 'Laser Level', self.laserLevel)
        if ok:
            if num >=1 and num <=16: # Laser level is set between 1 and 16, the actual power output in Arduino is laserLevel*16-1, ranging from 15 to 255
                if self.serialPort != None:
                    self.laserLevel = num
                    serialSignal(self.ser, self.laserLevel)

    def changeBrightness(self):
        num, ok = QtGui.QInputDialog.getInt(self, ' ', 'Camera Brightness', self.brightness)
        if ok:
            if num >= 0 and num <= 255:
                self.brightness = num
                self.setCam()

    def changeContrast(self):
        num, ok = QtGui.QInputDialog.getInt(self, ' ', 'Camera Contrast', self.contrast)
        if ok:
            if num >= 0 and num <= 255:
                self.contrast = num
                self.setCam()

    def changeGainR(self):
        num, ok = QtGui.QInputDialog.getInt(self, ' ', 'Camera Gain R', self.gainR)
        if ok:
            if num >= 0 and num <= 255:
                self.gainR = num
                self.setCam()

    def changeGainL(self):
        num, ok = QtGui.QInputDialog.getInt(self, ' ', 'Camera Gain L', self.gainL)
        if ok:
            if num >= 0 and num <= 255:
                self.gainL = num
                self.setCam()

    def changeExposure(self):
        num, ok = QtGui.QInputDialog.getInt(self, ' ', 'Camera Exposure', self.exposure)
        if ok:
            if num >= -7 and num <=-1:
                self.exposure = num
                self.setCam()

    def changeFocus(self):
        num, ok = QtGui.QInputDialog.getInt(self, ' ', 'Camera Contrast', self.focus)
        if ok:
            if num >= 0 and num <= 255:
                self.focus = num / 5 * 5
                self.setCam()

    def setCam(self):
        if self.rightOrLeft:
            gain = self.gainR
        else:
            gain = self.gainL
        configCam(self.cam, self.imgLongSide, self.imgShortSide, self.brightness, self.contrast, self.saturation, self.hue, gain, self.exposure, self.whiteBalance, self.focus)

    def changeDenoiseWindowRange(self):
        num, ok = QtGui.QInputDialog.getInt(self, ' ', 'Denoise Window Range', self.denoiseWindowRange)
        if ok:
            self.denoiseWindowRange = num

    def changeDenoiseMeanThres(self):
        num, ok = QtGui.QInputDialog.getInt(self, ' ', 'Denoise Mean Threshold', self.denoiseMeanThres)
        if ok:
            self.denoiseMeanThres = num

    def changeDenoiseNoiseThres(self):
        num, ok = QtGui.QInputDialog.getInt(self, ' ', 'Denoise Noise Threshold', self.denoiseNoiseThres)
        if ok:
            self.denoiseNoiseThres = num

    def changeZMax(self):
        num, ok = QtGui.QInputDialog.getInt(self, ' ', 'Z Max', self.zMax)
        if ok:
            if num < 480 and num >= 0:
                self.zMax = num

    def changeLaserHalfWidth(self):
        num, ok = QtGui.QInputDialog.getInt(self, ' ', 'Laser Half Width', self.laserHalfWidth)
        if ok:
            if num >= 1 and num <= 20:
                self.laserHalfWidth = num

    ###

    def STATE_10_launchScan(self):
        if self.isCamOperable != True or self.serialPort == None:
            msgBox = QtGui.QMessageBox()
            msgBox.setWindowIcon(QtGui.QIcon('icon_bear.png'))
            msgBox.setWindowTitle('TERO')
            msgBox.setIcon(QtGui.QMessageBox.Warning)
            msgBox.setText('The scanner is not ready.')
            msgBox.addButton(QtGui.QPushButton('OK'), QtGui.QMessageBox.YesRole)
            msgBox.exec_()
            return

        if self.xStart == None or self.xEnd == None:
            msgBox = QtGui.QMessageBox()
            msgBox.setWindowIcon(QtGui.QIcon('icon_bear.png'))
            msgBox.setWindowTitle('TERO')
            msgBox.setIcon(QtGui.QMessageBox.Information)
            msgBox.setText('Please set the range to scan.')
            msgBox.addButton(QtGui.QPushButton('OK'), QtGui.QMessageBox.YesRole)
            msgBox.exec_()
            return

        if self.isDeveloper == True:
            reply1 = QtGui.QMessageBox.question(self, ' ', "Auto camera and laser tuning?", QtGui.QMessageBox.Yes, QtGui.QMessageBox.No)
            reply2 = QtGui.QMessageBox.question(self, ' ', "Save camera images?", QtGui.QMessageBox.Yes, QtGui.QMessageBox.No)
        else:
            reply1 = QtGui.QMessageBox.Yes # Do     auto laser level for the user
            reply2 = QtGui.QMessageBox.No  # Do not save raw images for the user

        self.setSTATE_10()

        self.thread = ScanProcThread(self)
        self.connect(self.thread, QtCore.SIGNAL("message(PyQt_PyObject)"), self.progressMessage)
        self.connect(self.thread, QtCore.SIGNAL("bestLaserThresR(PyQt_PyObject)"), self.updateLaserThresR)
        self.connect(self.thread, QtCore.SIGNAL("bestLaserThresL(PyQt_PyObject)"), self.updateLaserThresL)
        self.connect(self.thread, QtCore.SIGNAL("bestLaserLevel(PyQt_PyObject)"), self.updateLaserLevel)
        self.connect(self.thread, QtCore.SIGNAL("stagePosition(PyQt_PyObject)"), self.updateStagePosition)
        self.connect(self.thread, QtCore.SIGNAL("vertices(PyQt_PyObject)"), self.transferObjectToGLWidget)
        self.connect(self.thread, QtCore.SIGNAL("closeUp(PyQt_PyObject)"), self.closeUpScanThread)

        if reply1 == QtGui.QMessageBox.No:
            self.thread.isLaserLeveled = True
        if reply2 == QtGui.QMessageBox.Yes:
            self.thread.savingImg = True

        self.thread.start()

    def STATE_11_procLaserData(self):
        fname = QtGui.QFileDialog.getOpenFileName(parent=self, caption='Open laser data', filter='Laser points (*.npy)')
        if fname != '':
            self.laserData = np.load(str(fname))

            self.setSTATE_11()

            self.thread = ScanProcThread(self)
            self.connect(self.thread, QtCore.SIGNAL("message(PyQt_PyObject)"), self.progressMessage)
            self.connect(self.thread, QtCore.SIGNAL("vertices(PyQt_PyObject)"), self.transferObjectToGLWidget)
            self.connect(self.thread, QtCore.SIGNAL("closeUp(PyQt_PyObject)"), self.closeUpScanThread)
            self.thread.start()

    def STATE_12_procCamImg(self):
        self.setSTATE_12()

        self.thread = ScanProcThread(self)
        num, ok = QtGui.QInputDialog.getInt(self, ' ', 'Number of images')
        if ok:
            self.thread.xRange = num
            self.connect(self.thread, QtCore.SIGNAL("message(PyQt_PyObject)"), self.progressMessage)
            self.connect(self.thread, QtCore.SIGNAL("vertices(PyQt_PyObject)"), self.transferObjectToGLWidget)
            self.connect(self.thread, QtCore.SIGNAL("closeUp(PyQt_PyObject)"), self.closeUpScanThread)
            self.thread.start()

    def STATE_14_camCalibManual(self):
        self.setSTATE_14()

        self.thread = CalibThreadManual(self)
        self.connect(self.thread, QtCore.SIGNAL("image(PyQt_PyObject)"), self.displayCamCalib)
        self.connect(self.thread, QtCore.SIGNAL("waitForInput(PyQt_PyObject)"), self.inputRealCooridinates)
        self.connect(self.thread, QtCore.SIGNAL("msgbox(PyQt_PyObject)"), self.msgbox)
        self.connect(self.thread, QtCore.SIGNAL("hMat1(PyQt_PyObject)"), self.transferHMat1)
        self.connect(self.thread, QtCore.SIGNAL("hMat2(PyQt_PyObject)"), self.transferHMat2)
        self.connect(self.thread, QtCore.SIGNAL("finished()"), self.updateCamCalib)
        self.thread.start()

    def STATE_15_camSelectManual(self):
        self.setSTATE_15()

        self.isCamL_OK = False
        self.isCamR_OK = False

        self.thread = CamSelectThread(self)
        self.connect(self.thread, QtCore.SIGNAL("image(PyQt_PyObject)"), self.displayCamCalib)
        self.connect(self.thread, QtCore.SIGNAL("camIndex(PyQt_PyObject)"), self.specifyRightOrLeftCam)
        self.connect(self.thread, QtCore.SIGNAL("finished()"), self.decideIsCamOperable)
        self.connect(self.thread, QtCore.SIGNAL("finished()"), self.setSTATE_0)
        self.thread.start()

    def STATE_16_laserLevelingAuto(self):
        if self.isCamOperable != True or self.serialPort == None:
            msgBox = QtGui.QMessageBox()
            msgBox.setWindowIcon(QtGui.QIcon('icon_bear.png'))
            msgBox.setWindowTitle('TERO')
            msgBox.setIcon(QtGui.QMessageBox.Warning)
            msgBox.setText('The scanner is not ready.')
            msgBox.addButton(QtGui.QPushButton('OK'), QtGui.QMessageBox.YesRole)
            msgBox.exec_()
            return

        self.setSTATE_16()

        self.thread = ScanProcThread(self)
        self.connect(self.thread, QtCore.SIGNAL("message(PyQt_PyObject)"), self.progressMessage)
        self.connect(self.thread, QtCore.SIGNAL("bestLaserThresR(PyQt_PyObject)"), self.updateLaserThresR)
        self.connect(self.thread, QtCore.SIGNAL("bestLaserThresL(PyQt_PyObject)"), self.updateLaserThresL)
        self.connect(self.thread, QtCore.SIGNAL("bestLaserLevel(PyQt_PyObject)"), self.updateLaserLevel)
        self.connect(self.thread, QtCore.SIGNAL("finished()"), self.setSTATE_0)
        self.thread.start()

    def STATE_17_camTuning(self):
        if self.isCamOperable != True or self.serialPort == None:
            msgBox = QtGui.QMessageBox()
            msgBox.setWindowIcon(QtGui.QIcon('icon_bear.png'))
            msgBox.setWindowTitle('TERO')
            msgBox.setIcon(QtGui.QMessageBox.Warning)
            msgBox.setText('The scanner is not ready.')
            msgBox.addButton(QtGui.QPushButton('OK'), QtGui.QMessageBox.YesRole)
            msgBox.exec_()
            return

        self.setSTATE_17()

        self.thread = CamTuningThread(self)
        self.connect(self.thread, QtCore.SIGNAL("message(PyQt_PyObject)"), self.progressMessage)
        self.connect(self.thread, QtCore.SIGNAL("gainL(PyQt_PyObject)"), self.updateGainL)
        self.connect(self.thread, QtCore.SIGNAL("finished()"), self.setSTATE_0)
        self.thread.start()

    #

    def updateStagePosition(self, xPosition):
        self.xCurrent = xPosition
        self.xCurrentActual = xPosition

    def closeUpScanThread(self, isAborted):
        if not isAborted:
            self.setSTATE_20()
            self.GLWidget.showMale()
        else:
            self.setSTATE_0()

    def inputRealCooridinates(self, text):
        num, ok = QtGui.QInputDialog.getDouble(self, ' ', text)
        if ok:
            self.thread.input = num

    def msgbox(self, text):
        reply = QtGui.QMessageBox.information(self, 'TERO', text, QtGui.QMessageBox.Ok)

    def specifyRightOrLeftCam(self, camIndex):
        msgBox = QtGui.QMessageBox()
        msgBox.setWindowIcon(QtGui.QIcon('icon_bear.png'))
        msgBox.setWindowTitle('TERO')
        msgBox.setIcon(QtGui.QMessageBox.Question)
        msgBox.setText('Is this the LEFT or RIGHT camera?')
        msgBox.addButton(QtGui.QPushButton('Left'), QtGui.QMessageBox.YesRole)
        msgBox.addButton(QtGui.QPushButton('Right'), QtGui.QMessageBox.NoRole)
        msgBox.addButton(QtGui.QPushButton('None'), QtGui.QMessageBox.RejectRole)
        reply = msgBox.exec_()
        if reply == 0:
            self.camIndexL= camIndex
            self.isCamL_OK = True
        if reply == 1:
            self.camIndexR= camIndex
            self.isCamR_OK = True
        self.thread.pause = False

    def decideIsCamOperable(self):
        self.isCamOperable = self.isCamL_OK and self.isCamR_OK

    def displayCamCalib(self, img):
        height, width, bytesPerComponent = img.shape
        bytesPerLine = bytesPerComponent * width
        Q_img = QtGui.QImage(img, width, height, bytesPerLine, QtGui.QImage.Format_RGB888) # convert cv2 image to QImage
        Q_pixmap = QtGui.QPixmap.fromImage(Q_img) # Convert QImage to QPixmap
        self.scanMonitor.setPixmap(Q_pixmap) # Set the QLabel to display the QPixmap

    def transferHMat1(self, hMat1):
        self.new_hMat1 = hMat1

    def transferHMat2(self, hMat2):
        self.new_hMat2 = hMat2

    def updateCamCalib(self):
        reply = QtGui.QMessageBox.question(self, 'TERO', 'Update hMat1 and hMat2?', QtGui.QMessageBox.Yes, QtGui.QMessageBox.No)
        if reply == QtGui.QMessageBox.Yes:
            self.hMat1 = self.new_hMat1
            self.hMat2 = self.new_hMat2
        self.setSTATE_0()

    def updateLaserLevel(self, bestLevel):
        self.laserLevel = bestLevel
        self.displayParameters()

    def updateLaserThresR(self, bestThres):
        self.laserThresR = bestThres
        self.displayParameters()

    def updateLaserThresL(self, bestThres):
        self.laserThresL = bestThres
        self.displayParameters()

    def updateGainL(self, gainL):
        self.gainL = gainL
        self.displayParameters()

    def progressMessage(self, message):
        self.scanMonitor.setText(message)

    def transferObjectToGLWidget(self, vertices):
        self.verticesMale = vertices
        fname = QtGui.QFileDialog.getSaveFileName(parent=self, directory='TERO.npy', caption='Save scanned impression', filter='Model (*.npy)')
        if fname != '':
            fname = str(fname)
            np.save(fname, self.verticesMale)
        self.GLWidget.renewObject(self.verticesMale, isMale=True)

    def quitScan(self):
        self.thread.abort = True

    ###

    def closeEvent(self, event):
        reply = QtGui.QMessageBox.question(self, 'TERO', 'Are you sure you want to quit TERO?', QtGui.QMessageBox.Yes, QtGui.QMessageBox.No)
        if reply == QtGui.QMessageBox.Yes:

            if self.isDeveloper:
                reply2 = QtGui.QMessageBox.question(self, 'TERO', 'Save current parameters?', QtGui.QMessageBox.Yes, QtGui.QMessageBox.No)
                if reply2 == QtGui.QMessageBox.Yes:
                    self.saveParameters()
            else:
                self.saveParameters()

            self.timer.stop()

            if self.serialPort != None:
                self.stepperThread.stop = True
                if self.isLedOn:
                    serialSignal(self.ser, 62)
                    self.isLedOn = False
                self.ser.close()

            if self.isCamOperable:
                self.cam.release()

            event.accept()

        else:
            event.ignore()



if __name__ == '__main__':
    app = QtGui.QApplication(sys.argv)
    ScanWindow = MainWindow()
    sys.exit(app.exec_())
