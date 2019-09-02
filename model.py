import numpy as np
import cv2, time, sys, threading, json, os
from view import *
from controller import *



class Core(object):
    def __init__(self):
        # Instantiate a controller object.
        # Pass the core object into the controller object,
        # so the controller can call the core.
        self.controller = Controller(core_obj = self)

        # Instantiate a gui object.
        # Pass the controller object into the gui object,
        # so the gui can call the controller, which in turn calls the core
        self.gui = GUI(controller_obj = self.controller)
        self.gui.show()

        # The mediator is a channel to emit any signal to the gui object.
        # Pass the gui object into the mediator object,
        # so the mediator knows where to emit the signal.
        self.mediator = Mediator(self.gui)

        self.__init__connect_signals()

        # Start the default thread
        self.start_thread()

    def __init__connect_signals(self):
        '''
        Call the mediator to connect signals to the gui.
        These are the signals to be emitted dynamically during runtime.

        Each signal is defined by a unique str signal name.
        '''
        signal_names = ['core_signal_1', 'core_signal_2']
        self.mediator.connect_signals(signal_names)

    def start_thread(self):
        # Pass the mediator into the thread,
        #   so the thread object can talk to the gui.
        self.thread = MockThread(mediator_obj = self.mediator)
        self.thread.start()

    def stop_thread(self):
        self.thread.stop()

    def close(self):
        'Should be called upon software termination.'
        self.stop_thread()

    # Methods called by the controller object

    def mock_method(self):
        self.thread.mock_method()

    def action_1_key(self):
        pass

    def action_2_key(self):
        pass



class MockThread(threading.Thread):
    def __init__(self, mediator_obj):
        super(MockThread, self).__init__()

        # Mediator emits signal to the gui object
        self.mediator = mediator_obj

    def __init__connect_signals(self):
        '''
        Call the mediator to connect signals to the gui.
        These are the signals to be emitted dynamically during runtime.

        Each signal is defined by a unique str signal name.
        '''
        signal_names = ['thread_signal_1', 'thread_signal_2']
        self.mediator.connect_signals(signal_names)

    def __init__parms(self):

        # Parameters for business logic, stored in .json files
        pkg_dir = os.path.dirname(__file__)
        path = os.path.join(pkg_dir, 'parameters/parms.json')
        parms = json.loads(open(path, 'r').read())
        self.parm_1 = parms['parm_1']
        self.parm_2 = parms['parm_2']

        # Parameters for administrative logic
        self.isStop = False
        self.isPause = False

    def run(self):
        '''
        The main loop that runs in the thread
        '''

        while not self.isStop:

            if self.isPause:
                time.sleep(1)
                continue

            time.sleep(1)

    # Methods commanded by the high-level core object.

    def stop(self):
        'Called to terminate the video thread.'

        # Shut off main loop in self.run()
        self.isStop = True

    def pause(self):
        self.isPause = True

    def resume(self):
        self.isPause = False

    def mock_method(self):
        pass


