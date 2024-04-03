import os, sys
import time
from mpu9250_jmdev.registers import *
from mpu9250_jmdev.mpu_9250 import MPU9250

from mpu9250_i2c import *

from scipy.integrate import simpson
from numpy import trapz

import dbus
import dbus.mainloop.glib

import matplotlib.pyplot as plt
import matplotlib.animation as animation

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

network = dbus.Interface(bus.get_object('ch.epfl.mobots.Aseba', '/'),
                            dbus_interface='ch.epfl.mobots.AsebaNetwork')

# Print in the terminal the name of each Aseba Node
print(network.GetNodesList())

fig = plt.figure()
axis = fig.add_subplot(1,1,1)

accel_x = []
accel_y = []
accel_z = []
gyro_x = []
gyro_y = []
gyro_z = []
gyro_z_adj = []
timestamps = []
counter=0.1


def turnRight():
    network.SetVariable('thymio-II', 'motor.left.target', [300])
    network.SetVariable('thymio-II', 'motor.right.target', [0])
    time.sleep(1.6)
    setThymioSpeed(0)


def turnLeft():
    network.SetVariable('thymio-II', 'motor.left.target', [0])
    network.SetVariable('thymio-II', 'motor.right.target', [300])
    time.sleep(1.6)
    setThymioSpeed(0)



def test_gyro():
    while True:
      ax,ay,az,wx,wy,wz = mpu6050_conv() # read and convert mpu6050 data
    
      accel_x.append(ax)
      accel_y.append(ay)
      accel_z.append(az)
      gyro_x.append(wx)
      gyro_y.append(wy)
      gyro_z.append(wz)
      
      print("GYRO X: ", wx, "\n"
            "GYRO Y: ", wy, "\n"
            "GYRO Z: ", wz, "\n")
      

def animate(i, timestamps, gyro_x, gyro_y, gyro_z):
      ax,ay,az,wx,wy,wz = mpu6050_conv() # read and convert mpu6050 data
    
      accel_x.append(ax)
      accel_y.append(ay)
      accel_z.append(az)
      gyro_x.append(wx)
      gyro_y.append(wy)
      gyro_z.append(wz)
      gyro_z_adj.append(wz-0.6)
      timestamps.append(i)
      
      #accel_x = accel_x[-20:]
      #accel_y = accel_y[-20:]
      #accel_z = accel_z[-20:]
      
      axis.clear()
      #axis.plot(timestamps, gyro_x, label="Gyro X")
      axis.plot(timestamps, gyro_z, label="Gyro Z")
      axis.plot(timestamps, gyro_z_adj, label="Gyro Z (adjusted)")
      
      #plt.xticks(rotation=45, ha='right')
      #plt.subplots_adjust(bottom=0.30)
      plt.ylabel('m/s')
      plt.legend()


def plotChart(n):
    gyro_x = []
    gyro_y = []
    gyro_z = []
    
    timeStampsTime = []
    
    TEST_TIME = 2
    end = time.time() + TEST_TIME
    
    print("SHAKE NOW!")
    
    time.sleep(0.2)
            
    while time.time() <= end:
        ax,ay,az,wx,wy,wz = mpu6050_conv()
        
        gyro_x.append(wx)
        gyro_y.append(wy)
        gyro_z.append(wz)
        
        timeStampsTime.append(time.time())
    
    area = trapz(gyro_z, dx=5)
    print("area =", area)
    
    if area > 300:
        print("TURN RIGHT!")
    elif area < -300:
        print("TURN LEFT!")

    # Compute the area using the composite Simpson's rule.
    #area = simpson(gyro_z, dx=5)
    #print("area =", area)
    
    plt.figure(n)
    plt.plot(timeStampsTime, gyro_z)
    
    

#ani = animation.FuncAnimation(fig, animate, fargs=(timestamps, gyro_x, gyro_y, gyro_z), interval=1)
#plt.show()

#plotChart(1)
#plotChart(2)
#plt.show()

# start calibration
time.sleep(0.2)

xs = []
end = time.time() + 1

while time.time() <= end:
    ax,ay,az,wx,wy,wz = mpu6050_conv()
    xs.append(wz)

avg = sum(xs) / len(xs)
error = (max(xs) - min(xs)) / 2

print("""calibration results:
        average value: {}
        error range +-: {}
        data points: {}""".format(avg, error, len(xs)))


gyro = []
time.sleep(0.2)
prev = time.time()

while True:
        ax,ay,az,wx,wy,wz = mpu6050_conv()
        
        #gyro_x.append(wx)
        #gyro_y.append(wy)
        gyro.append(wz-avg)
        
        
        if time.time() > prev + 2:
            prev = time.time()
            area = trapz(gyro, dx=5)
            print("area1 =", area)
                
            if area > 500:
                print("TURN RIGHT!")
            elif area < -500:
                print("TURN LEFT!")
            
        
        # 90 degrees is roughly ≈ 30000m area!
    

"""
time.sleep(0.2)

xs = []
end = time.time() + 1

while time.time() <= end:
    ax,ay,az,wx,wy,wz = mpu6050_conv()
    xs.append(ax)

avg = sum(xs) / len(xs)
error = (max(xs) - min(xs)) / 2

print("""
    #calibration results:
    #average value: {}
    #error range +-: {}
    #data points: {}
""".format(avg, error, len(xs)))


c_mean, c_error = avg, error
noise_floor = 2 * (c_error - c_mean)

print("IMU sensor noise floor: ", noise_floor)

MIN_ACCELERATION_CHANGE = noise_floor
MAX_ACCELERATION_CHANGE = noise_floor


network.SetVariable('thymio-II', 'motor.left.target', [400])
network.SetVariable('thymio-II', 'motor.right.target', [400])
plotChart(c_mean)
network.SetVariable('thymio-II', 'motor.left.target', [0])
network.SetVariable('thymio-II', 'motor.right.target', [0])
plt.show()
network.SetVariable('thymio-II', 'motor.left.target', [-400])
network.SetVariable('thymio-II', 'motor.right.target', [-400])
time.sleep(2)
network.SetVariable('thymio-II', 'motor.left.target', [0])
network.SetVariable('thymio-II', 'motor.right.target', [0])
"""