import numpy as np
import cv2, time, sys, threading, json, os
from PyQt4 import QtCore, QtGui
from controller import *



class GUI(QtGui.QMainWindow):
    def __init__(self, controller_obj):
        super(GUI, self).__init__()

        # The controller object is the portal for GUI to call methods in the core object
        self.controller = controller_obj

        # Load gui parameters from the gui.json file
        pkg_dir = os.path.dirname(__file__)
        path = os.path.join(pkg_dir, 'parameters/gui.json')
        gui_parms = json.loads(open(path, 'r').read())
        w = gui_parms['window_width']
        h = gui_parms['window_height']

        self.setWindowTitle('Title')
        self.setWindowIcon(QtGui.QIcon('icons/cool.png'))
        self.setGeometry(100, 100, w, h)
        self.setFixedSize(w, h)
        self.setMouseTracking(True)

        self.monitor = QtGui.QLabel(self)
        self.monitor.setGeometry(0, 0, w, h)
        self.monitor.setAlignment(QtCore.Qt.AlignCenter)

        self.toolbar = QtGui.QToolBar('Tool Bar')
        self.toolbar.setMovable(True)
        self.toolbar.setStyleSheet("QToolBar { background:white; }")
        self.toolbar.setIconSize(QtCore.QSize(30, 45))
        self.addToolBar(QtCore.Qt.LeftToolBarArea, self.toolbar)

        # These are optional extra windows for additional interfaces
        self.info_window = TextWindow()
        self.tuner_window = CameraTunerWindow( controller_obj = self.controller )

        self.__init__toolbtns()

    def __init__toolbtns(self):
        # Each action has a unique key and a name
        # key = icon filename = method name
        # name = text of the action/button

        #    (    keys           ,   names                )
        K = [('action_1_key'     , 'action_1_name'        ),
             ('action_2_key'     , 'action_2_name'        ),
             ('action_3_key'     , 'action_3_name'        ),
             ('action_4_key'     , 'action_4_name'        )]

        self.actions = {}
        self.toolbtns = {}

        # Create actions and tool buttons
        for key, name in K:
            pkg_dir = os.path.dirname(__file__)
            path = os.path.join(pkg_dir, 'icons/' + key + '.png')
            icon = QtGui.QIcon(path)
            self.actions[key] = QtGui.QAction(icon, name, self)
            self.toolbtns[key] = self.toolbar.addAction(self.actions[key])

        # For actions that needs to be connected to the core object,
        K = ['action_1_key', 'action_2_key']

        # In this loop I defined a standard way of
        # connecting each action to a method in the core object via the controller object.
        for key in K:
            # Get a argument-less method from the controller object.
            # Note that the method_name = key.
            method = self.controller.get_method( method_name = key )
            # The get_method() returns None
            # if a particular method is not found in the core object.
            if not method is None:
                # Connect the action to the method in the controller object
                self.actions[key].triggered.connect(method)

        # For actions that needs to be connected to the self gui object,
        keys = ['action_3_key', 'action_4_key']
        for key in keys:
            try:
                method = getattr(self, key)
                self.actions[key].triggered.connect(method)
            except Exception as exception_inst:
                print exception_inst

    # Overridden methods

    def wheelEvent(self, event):
        if event.delta() > 0:
            self.controller.call_method('zoom_in')
        else:
            self.controller.call_method('zoom_out')

    def closeEvent(self, event):
        reply = QtGui.QMessageBox.question(self,
                                           'CaMNIST',
                                           'Are you sure you want to quit CaMNIST?',
                                           QtGui.QMessageBox.Yes, QtGui.QMessageBox.No)

        if reply == QtGui.QMessageBox.Yes:
            self.controller.call_method('close')
            self.info_window.close()
            event.accept()

        else:
            event.ignore()

    # Methods for actions in this GUI object

    def action_3_key(self):
        if not self.info_window.isVisible():
            self.info_window.show()

    def action_4_key(self):
        self.camera_tuner_window.show()

    # Methods for incoming signals

    def connect_signals(self, thread, signal_name):
        'Called by an external object to connect signals.'

        # The suffix '(PyQt_PyObject)' means the argument to be transferred
        # could be any type of python objects,
        # not limited to Qt objects.
        signal = signal_name + '(PyQt_PyObject)'

        # The method name to be called upon signal arrival = the signal name
        try:
            method = getattr(self, signal_name)
            self.connect(thread, QtCore.SIGNAL(signal), method)
        except Exception as exception_inst:
            print "Try to connect PyQt signal '{}'".format(signal_name)
            print exception_inst + '\n'

    def core_signal_1(self):
        pass

    def core_signal_2(self):
        pass

    def thread_signal_1(self):
        pass

    def thread_signal_2(self):
        pass



class TextWindow(QtGui.QWidget):
    def __init__(self):
        super(TextWindow, self).__init__()

        self.setWindowTitle('Info')
        self.setWindowIcon(QtGui.QIcon('icons/cool.png'))
        self.setGeometry(150, 150, 512, 256)
        self.setFixedSize(512, 256)

        self.font = QtGui.QFont()
        self.font.setFamily('Segoe UI')
        self.font.setBold(False)
        self.font.setPixelSize(14)

        self.textbox = QtGui.QLabel(self)
        self.textbox.setGeometry(0, 0, 512, 256)
        self.textbox.setAlignment(QtCore.Qt.AlignLeft)
        self.textbox.setFont(self.font)

    def setText(self, text):
        self.textbox.setText(text)



class SliderWidget(QtGui.QWidget):
    '''
    This widget wraps a single parameter in the TunerWindow.

    Name, value, min, max, interval are stored in this object.

    Three gui elements are included to display the information of the parameter:
      1) QLabel showing name
      2) QLabel showing value
      3) QSlider
    '''
    def __init__(self, parent, name, min, max, value, interval):
        super(SliderWidget, self).__init__(parent)

        self.name = name
        self.min = min
        self.max = max
        self.value = value
        self.interval = interval

        self.hbox = QtGui.QHBoxLayout()
        self.QLabel_name = QtGui.QLabel(self)
        self.QLabel_value = QtGui.QLabel(self)
        self.QSlider = QtGui.QSlider(QtCore.Qt.Horizontal, self)

        self.setLayout(self.hbox)

        self.hbox.addWidget(self.QLabel_name)
        self.hbox.addWidget(self.QLabel_value)
        self.hbox.addWidget(self.QSlider)

        self.QLabel_name.setText(name)
        self.QLabel_value.setText(str(value))

        self.QSlider.setMinimum(min)
        self.QSlider.setMaximum(max)
        self.QSlider.setValue(value)
        self.QSlider.setSingleStep(interval)
        self.QSlider.setTickInterval(interval)
        self.QSlider.setTickPosition(QtGui.QSlider.TicksBelow)

        self.QSlider.valueChanged.connect(self.setValue)

    def setValue(self, value):

        # Round the value to fit the interval
        value = value - self.min
        value = round( value / float(self.interval) ) * self.interval
        value = int( value + self.min )

        self.value = value
        self.QSlider.setValue(value)
        self.QLabel_value.setText(str(value))



class TunerWindow(QtGui.QWidget):
    '''
    A gui template window for tuning parameters.

    This class does not contain any business logic.
    All it does is to provide an interface to adjust parameters through gui.

    Each parameter is wrapped in a 'block' of SliderWidget object.

    Properties (name, min, max, value, interval)
    of each parameter is stored in the SliderWidget object.
    '''
    def __init__(self):
        super(TunerWindow, self).__init__()

        # self.setMinimumWidth(600)
        # self.setMaximumWidth(600)

        self.main_vbox = QtGui.QVBoxLayout()
        self.setLayout(self.main_vbox)

        self.btn_hbox = QtGui.QHBoxLayout()
        self.main_vbox.addLayout(self.btn_hbox)

        K = [('ok'    ,'OK'    ),
             ('cancel','Cancel'),
             ('apply' ,'Apply' )]

        self.btn = {}

        for key, name in K:
            self.btn[key] = QtGui.QPushButton(name, self)
            self.btn[key].clicked.connect(getattr(self, key))
            self.btn_hbox.addWidget( self.btn[key] )

        self.parameters = []

    def apply_parameter(self):
        '''
        Supposed to be overridden.
        Defines what to do when ok() or apply() are called.
        '''
        pass

    def ok(self):
        self.apply_parameter()
        self.hide()

    def cancel(self):
        self.hide()

    def apply(self):
        self.apply_parameter()

    def add_parameter(self, name, min, max, value, interval):
        '''
        Add a new SliderWidget object holding all information of the new parameter.
        '''
        widget = SliderWidget(parent   = self,
                              name     = name,
                              min      = min,
                              max      = max,
                              value    = value,
                              interval = interval)

        self.parameters.append(widget)

        self.main_vbox.insertWidget(len(self.main_vbox)-1, widget)



class CameraTunerWindow(TunerWindow):
    '''
    Inherits from the TunerWindow class.

    The business logics for the camera imaging parameters
    is specified in this class.

    This class also manages the transfer of camera parameters
    to the core object.
    '''
    def __init__(self, controller_obj):
        super(CameraTunerWindow, self).__init__()

        self.controller = controller_obj

        self.setWindowIcon(QtGui.QIcon('icons/cool.png'))
        self.setWindowTitle('Stereo Depth Parameters')
        self.setMinimumWidth(600)

        self.__init__parameters()

    def __init__parameters(self):
        '''
        The business logic for the actual values of camera parameters
        is defined in the method.
        '''

        # Call the 'fetch_cam_parm' method in the core object to get
        # the current values of camera parameters
        P = self.controller.call_method(method_name = fetch_cam_parm)
        # 'P' should be a list of dictionaries,
        # with each dictionary storing all information of a single parameter

        # Just as a backup,
        # these are the default parameter values that I used for the 3D microscopy project
        P = [{'name':'brightness'    , 'min':0   , 'max':255 , 'value':150 , 'interval':5  },
             {'name':'contrast'      , 'min':0   , 'max':255 , 'value':64  , 'interval':5  },
             {'name':'saturation'    , 'min':0   , 'max':255 , 'value':80  , 'interval':5  },
             {'name':'gain'          , 'min':0   , 'max':255 , 'value':50  , 'interval':5  },
             {'name':'exposure'      , 'min':-7  , 'max':-1  , 'value':-4  , 'interval':1  },
             {'name':'white_balance' , 'min':3000, 'max':6500, 'value':5000, 'interval':100},
             {'name':'focus'         , 'min':0   , 'max':255 , 'value':0   , 'interval':5  }]

        for p in P:
            self.add_parameter(name     = p['name'],
                               min      = p['min'],
                               max      = p['max'],
                               value    = p['value'],
                               interval = p['interval'])

    def apply_parameter(self):
        '''
        Transfers parameters to the core object via the controller.
        '''
        parms = {}
        for p in self.parameters:
            parms[p.name] = p.value

        self.controller.call_method( method_name = 'apply_camera_parameters',
                                             arg = parms                   )






if __name__ == '__main__':
    app = QtGui.QApplication(sys.argv)

    # MockController does not need the core object to be injected upon instantiation.
    gui = GUI( controller_obj = MockController() )
    gui.show()

    sys.exit(app.exec_())


