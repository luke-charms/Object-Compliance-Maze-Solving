import os, sys
import time
from mpu9250_jmdev.registers import *
from mpu9250_jmdev.mpu_9250 import MPU9250

from mpu9250_i2c import *

import dbus
import dbus.mainloop.glib
import gi
gi.require_version("Gtk", "3.0")
from gi.repository import Gtk


def setup():
    os.system("asebamedulla \"ser:name=Thymio-II\" &")
    
    mpu = MPU9250(
        address_ak=AK8963_ADDRESS,
        address_mpu_master=MPU9050_ADDRESS_68,
        address_mpu_slave=None,
        bus=1,
        gfs=GFS_1000,
        afs=AFS_8G,
        mfs=AK8963_BIT_16,
        mode=AK8963_MODE_C100HZ)
    
    mpu.configure() # apply settings to registers
    
    # Set up the DBUS so the script may communicate with the Thymio-II
    dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)
    
    # Create Aseba network - make sure the Aseba Medulla service is running
    bus = dbus.SessionBus()
    global network
    network = dbus.Interface(bus.get_object('ch.epfl.mobots.Aseba', '/'),
                            dbus_interface='ch.epfl.mobots.AsebaNetwork')
    
    # Print in the terminal the name of each Aseba Node
    print(network.GetNodesList())


def drive():
    left_motor_target = 50
    right_motor_target = 50
    
    # Use the DBUS network to set the motor target speeds variable on the Thymio-II
    network.SetVariable('thymio-II', 'motor.left.target', [left_motor_target])
    network.SetVariable('thymio-II', 'motor.right.target', [right_motor_target])


def calcCompliance():
    drive()
    
    while True:
        
        ax,ay,az,wx,wy,wz = mpu6050_conv() # read and convert mpu6050 data
        
        dataStr = (
            "Acc(x): " + str(ax) + "\n" +
            "Acc(y): " + str(ay) + "\n" +
            "Acc(z): " + str(az) + "\n" +
            "Gyr(x): " + str(wx) + "\n" +
            "Gyr(y): " + str(wy) + "\n" +
            "Gyr(z): " + str(wz) + "\n")
        
        print(dataStr)
        
        time.sleep(1)
    

def main():
    setup()
    time.sleep(5)
    loop = gi.repository.GLib.MainLoop()
    
    try:
        # Call loop method every 1 millisecond
        handle = gi.repository.GLib.timeout_add(1, calcCompliance)
        loop.run()
    except KeyboardInterrupt:
        # When the user presses Ctrl-C, close everything down nicely
        loop.quit()
        loop = None
        
        # Remove the "C" so when Ctrl-C is pressed, it results in "^Cleaning up..."
        print('leaning up...')
        network.SetVariable('thymio-II', 'motor.left.target', [0])
        network.SetVariable('thymio-II', 'motor.right.target', [0])
        
        os.system("pkill -n asebamedulla")

if __name__ == '__main__':
    main()