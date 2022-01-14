from pynput import keyboard
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil
import time
import math

FLIGHT_ALT = 3
droneSpeed = 0.5
# connection_string = 'tcp:192.168.86.182:5763' # simulation
connection_string = '/dev/ttyACM0, 57600' # pixhawk
aTargetAltitude = FLIGHT_ALT
ACTION = "NONE"
STATUS = "Initializing Program"
MODE = "NONE"
vehicle = None
targetLoc = None
homeLocation = None

def connectionFunc():
    global vehicle
    print("Connecting to vehicle on: %s" % (connection_string,))
    vehicle = connect(connection_string, wait_ready=True)

class Commander():
    
    def __init__(self):
        pass