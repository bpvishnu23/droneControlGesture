import os, sys, inspect, thread, time
src_dir = os.path.dirname(inspect.getfile(inspect.currentframe()))
# Windows and Linux
arch_dir = '../x64' if sys.maxsize > 2**32 else '../x86'
# Mac
#arch_dir = os.path.abspath(os.path.join(src_dir, '../lib'))

sys.path.insert(0, os.path.abspath(os.path.join(src_dir, arch_dir)))
import Leap
from Leap import CircleGesture, KeyTapGesture, ScreenTapGesture, SwipeGesture

from PyQt5 import QtWidgets
from Crazyflie_Hover_Test import Ui_Dialog
import time
import sys
from threading import Thread
import logging

import cflib  # noqa
from cfclient.utils.logconfigreader import LogConfig  # noqa
from cflib.crazyflie import Crazyflie  # noqa
logging.basicConfig(level=logging.ERROR)

mthrust = 10001
mpitch = 0
mroll = 0
myaw = 0
mgrab = 1
run = 1

maxroll = 180           #max roll inp allowed in degres
maxpitch = 180          #max pitch inp allowed in degrees
relative_no_power = 20  #Leap sensor 20mm unused offset distance
maxthrust = 42000       #max thrust input allowed(range: 10000 to 60000)
minthrust = 35000       #min thrust input allowed

#Leap sensor listener
class SampleListener(Leap.Listener):
    finger_names = ['Thumb', 'Index', 'Middle', 'Ring', 'Pinky']
    bone_names = ['Metacarpal', 'Proximal', 'Intermediate', 'Distal']
    state_names = ['STATE_INVALID', 'STATE_START', 'STATE_UPDATE', 'STATE_END']

    def on_init(self, controller):
        print "Initialized"

    def on_connect(self, controller):
        print "Connected"

        # Enable gestures
        controller.enable_gesture(Leap.Gesture.TYPE_CIRCLE);
        controller.enable_gesture(Leap.Gesture.TYPE_KEY_TAP);
        controller.enable_gesture(Leap.Gesture.TYPE_SCREEN_TAP);
        controller.enable_gesture(Leap.Gesture.TYPE_SWIPE);

    def on_disconnect(self, controller):
        # Note: not dispatched when running in a debugger.
        print "Disconnected"

    def on_exit(self, controller):
        print "Exited"

    def on_frame(self, controller):
        global mpitch, mthrust, myaw, mroll, relative_no_power, maxthrust, minthrust, run, maxroll, maxpitch, mgrab
        # Get the most recent frame and report some basic information
        frame = controller.frame()

        #print "Frame id: %d, timestamp: %d, hands: %d, fingers: %d, tools: %d, gestures: %d" % (
        #      frame.id, frame.timestamp, len(frame.hands), len(frame.fingers), len(frame.tools), len(frame.gestures()))
        mthrust = 10001
        # Get hands
        for hand in frame.hands:

            handType = "Left hand" if hand.is_left else "Right hand"

            # print "  %s, id %d, position: %s" % (
            #     handType, hand.id, hand.palm_position)

            # Get the hand's normal vector and direction
            normal = hand.palm_normal
            direction = hand.direction

            mpitch = -1 * direction.pitch * Leap.RAD_TO_DEG
            mroll = -1 * normal.roll * Leap.RAD_TO_DEG
            myaw = direction.yaw * Leap.RAD_TO_DEG
            mthrust = ((hand.palm_position.y - relative_no_power) * (maxthrust - minthrust) / 200) + minthrust # 50000 max thrust over 200 mm
            if mthrust > maxthrust:
                mthrust = maxthrust
            elif mthrust < minthrust:
                mthrust = minthrust

            if mroll > maxroll:
                mroll = maxroll
            if mroll < (-1 * maxroll):
                mroll = -1 * maxroll

            if mpitch > maxpitch:
                mpitch = maxpitch
            if mpitch < (-1 * maxpitch):
                mpitch = -1 * maxpitch

            mgrab = hand.grab_strength

            # Calculate the hand's pitch, roll, and yaw angles
            # print "  pitch: %f, roll: %f, yaw: %f, mthrust: %f, run: %f" % (
            #     mpitch,
            #     mroll,
            #     myaw,
            #     mthrust,
            #     run)

            if mroll > 130:     #rotate  hand 130 degrees to get disconnected with crazy flie
                run = 0

           

        # if not (frame.hands.is_empty):
        #     print "-"

class HoverTest:
    """Example that connects to the first Crazyflie found, hovers, and disconnects."""

    def __init__(self, link_uri):
        """ Initialize and run the example with the specified link_uri """

        self._cf = Crazyflie()

        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)

        self._cf.open_link(link_uri)

        self._status = {}
        self._status['stabilizer.roll'] = 0
        self._status['stabilizer.thrust'] = 0
        self._status['stabilizer.yaw'] = 0
        self._status['stabilizer.pitch'] = 0

        print("Connecting to %s" % link_uri)

    def _connected(self, link_uri):
        """ This callback is called form the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""
        print("Connected to %s" % link_uri)

        # The definition of the logconfig can be made before connecting
        self._lg_stabilizer = LogConfig(name="stabilizer", period_in_ms=10)
        self._lg_stabilizer.add_variable("stabilizer.roll", "float")
        self._lg_stabilizer.add_variable("stabilizer.thrust", "uint16_t")
        self._lg_stabilizer.add_variable("stabilizer.yaw", "float")
        self._lg_stabilizer.add_variable("stabilizer.pitch", "float")

        # Adding the configuration cannot be done until a Crazyflie is
        # connected, since we need to check that the variables we
        # would like to log are in the TOC.
        try:
            self._cf.log.add_config(self._lg_stabilizer)
            # This callback will receive the data
            self._lg_stabilizer.data_received_cb.add_callback(self._stabilizer_log_data)
            # This callback will be called on errors
            self._lg_stabilizer.error_cb.add_callback(self._stabilizer_log_error)
            # Start the logging
            self._lg_stabilizer.start()
        except KeyError as e:
            print("Could not start log configuration,"
                  "{} not found in TOC".format(str(e)))
        except AttributeError:
            print("Could not add stabilizer log config, bad configuration.")

        # Start a separate thread to do the motor test.
        # Do not hijack the calling thread!
        Thread(target=self._hover).start()

    def _connection_failed(self, link_uri, msg):
        """Callback when connection initial connection fails (i.e no Crazyflie
        at the specified address)"""
        print("Connection to %s failed: %s" % (link_uri, msg))

    def _connection_lost(self, link_uri, msg):
        """Callback when disconnected after a connection has been made (i.e
        Crazyflie moves out of range)"""
        print("Connection to %s lost: %s" % (link_uri, msg))

    def _disconnected(self, link_uri):
        """Callback when the Crazyflie is disconnected (called in all cases)"""
        print("Disconnected from %s" % link_uri)

    def _stabilizer_log_error(self, logconf, msg):
        """Callback from the log API when an error occurs"""
        print("Error when logging %s: %s" % (logconf.name, msg))

    def _stabilizer_log_data(self, timestamp, data, logconf):

        #print("[%d][%s]: %s" % (timestamp, logconf.name, data))
        self._status['stabilizer.roll'] = data['stabilizer.roll']
        self._status['stabilizer.thrust'] = data['stabilizer.thrust']
        self._status['stabilizer.yaw'] = data['stabilizer.yaw']
        self._status['stabilizer.pitch'] = data['stabilizer.pitch']

    def _hover(self):
        global mpitch, mthrust, myaw, mroll, run
        
        # Unlock startup thrust protection.
        #self._cf.commander.send_setpoint(0, 0, 0, 0)

        # Turn on altitude hold.
        #self._cf.param.set_value("flightmode.althold", "True")
        #f = open("Log.txt", "w")
        self._cf.commander.send_setpoint(0, 0, 0, 0)
        time.sleep(0.5)
        print "Unlocked"
        while run:
            # Update the position.
            #print("Setting Point: ")
            if ((mthrust >= minthrust)&(mgrab == 0)):
                self._cf.commander.send_setpoint(mroll, mpitch, myaw, mthrust)
                time.sleep(0.01)
            else:
                self._cf.commander.send_setpoint(0, 0, 0, 0)        #turns off wothout delay and initially unlocks the thrust
                print "zero"                

            print "  pitch: %6.2f %6.2f, roll: %6.2f %6.2f, yaw: %6.2f %6.2f, mthrust: %6.2f %6.2f, grab: %2f" % (
                mpitch,
                self._status['stabilizer.pitch'],
                mroll,
                self._status['stabilizer.roll'],
                myaw,
                self._status['stabilizer.yaw'],
                mthrust,
                self._status['stabilizer.thrust'],
                mgrab)
            
            #f.write("%f %f %f %f\n" % (self._status['stabilizer.roll'], self._status['stabilizer.pitch'], mroll, mpitch))
        
        #self._cf.commander.send_setpoint(0, 0, 0, 0)
        #f.close()

        # Make sure that the last packet leaves before the link is closed
        # since the message queue is not flushed before closing
        self._cf.commander.send_setpoint(0, 0, 0, 0)
        time.sleep(0.1)

        print "run %f" % run
        time.sleep(0.1)
        self._cf.close_link()


def main():
    # Create a sample listener and controller
    listener = SampleListener()
    controller = Leap.Controller()

    # Have the sample listener receive events from the controller
    controller.add_listener(listener)

    cflib.crtp.init_drivers(enable_debug_driver=False)

    
    # Scan for Crazyflies and use the first one found
    print("Scanning interfaces for Crazyflies...")
    available = cflib.crtp.scan_interfaces()
    print("Crazyflies found:")
    for i in available:
        print(i[0])
    if len(available) > 0:
        le = HoverTest(available[0][0])
    else:
        print("No Crazyflies found, cannot run example")

    # Keep this process running until Enter is pressed
    print "Press Enter to quit..."
    try:
        sys.stdin.readline()
    except KeyboardInterrupt:
        pass
    finally:
        # Remove the sample listener when done
        controller.remove_listener(listener)


if __name__ == "__main__":
    main()