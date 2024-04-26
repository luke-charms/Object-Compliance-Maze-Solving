# import necessary modules
import os
import cv2
import dbus
import dbus.mainloop.glib
import RPi.GPIO as GPIO
from picamera2 import Picamera2
from mpu9250_jmdev.registers import *
from mpu9250_jmdev.mpu_9250 import MPU9250
from mpu9250_i2c import *

from scipy.integrate import simpson
from numpy import trapz

GPIO.setwarnings(False) # Ignore warnings
GPIO.setmode(GPIO.BOARD) # Use physical pin numbering

GPIO.setup(10, GPIO.IN, pull_up_down=GPIO.PUD_DOWN) # Set pin 10 to be an input pin and set initial value to be pulled low (off)
GPIO.setup(12, GPIO.OUT) # set pin 12 to be an output pin for LED

OBJECT_COMPLIANCE = {} #dictionary to hold object compliance values, e.g., 'bowl':0.2, 'cup':0.9, 'bottle':0.9...

CONFIDENCE_THRESHOLD = 0.5 # threshold used to classify an image. if under, classification is not strong enough
NMS_THRESHOLD = 0.2 # threshold used to distint between two objects' classificaitons
SEARCH_OBJECTS = ['cup', 'bottle', 'bowl', 'sports ball'] #objects used to search for when classifying, in order to speed up process

TRAINING_SPEED = 50
TEST_TIME = 2
TESTING_SPEED = 400

#This is to pull the information about what each object is called
classNames = []
classFile = "/home/pi/3rdYearProject/object detection/Object_Detection_Files/coco.names"
with open(classFile,"rt") as f:
    classNames = f.read().rstrip("\n").split("\n")

#This is to pull the information about what each object should look like
configPath = "/home/pi/3rdYearProject/object detection/Object_Detection_Files/ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt"
weightsPath = "/home/pi/3rdYearProject/object detection/Object_Detection_Files/frozen_inference_graph.pb"

#This is some set up values to get good results
net = cv2.dnn_DetectionModel(weightsPath,configPath)
net.setInputSize(320,320)
net.setInputScale(1.0/ 127.5)
net.setInputMean((127.5, 127.5, 127.5))
net.setInputSwapRB(True)

# main function that switches between "Compliance Search Mode" and "Go To Goal" functions
def switchMode(channel):
    SEARCH_MODE = None
    if GPIO.input(12) == True:
        SEARCH_MODE = False
        GPIO.output(12, GPIO.LOW)
        print("FAST moving to charging port!")
        goToGoal(SEARCH_MODE)
        #complianceSearchMode(SEARCH_MODE)
    else:
        SEARCH_MODE = True
        GPIO.output(12, GPIO.HIGH)
        print("MEASURING nearby object compliance")
        #moveToChargingPort(SEARCH_MODE)
        complianceSearchMode(SEARCH_MODE)

# function to setup all the Thymio robot and image classification stuff
def setupThymio():
    os.system("asebamedulla \"ser:name=Thymio-II\" &")
    
    time.sleep(2)
    
    # Set up the DBUS so the script may communicate with the Thymio-II
    dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)
    
    # Create Aseba network - make sure the Aseba Medulla service is running
    bus = dbus.SessionBus()
    
    global network
    network = dbus.Interface(bus.get_object('ch.epfl.mobots.Aseba', '/'),
                            dbus_interface='ch.epfl.mobots.AsebaNetwork')
    
    global picam2
    picam2=Picamera2()
    camera_config = picam2.create_still_configuration(main={"size": (1920, 1080)}, buffer_count=2)
    picam2.configure(camera_config)
    picam2.start()
    
    global mpu
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

# function to calibrate the IMU to make sure acceleration value are accurate
def calibrateThymio():
    # start calibration
    time.sleep(0.2)

    xs = []
    zs = []
    end = time.time() + 1

    while time.time() <= end:
        ax,ay,az,wx,wy,wz = mpu6050_conv()
        xs.append(ax)
        zs.append(wz)

    avg_xs = sum(xs) / len(xs)
    error_xs = (max(xs) - min(xs)) / 2
    
    avg_zs = sum(zs) / len(zs)
    error_zs = (max(zs) - min(zs)) / 2

    print("""calibration results:
        average value acceleration: {}
        error range acceleration (+-): {}
        
        average value gyroscope: {}
        error range gyroscope (+-): {}
        data points: {}""".format(avg_xs, error_xs, avg_zs, error_zs,len(xs)))


    #noise_floor = 2 * (error_xs - avg_xs)
    return avg_xs, error_xs, avg_zs, error_zs

    print("IMU sensor noise floor: ", noise_floor)

def setThymioSpeed(speed):
    network.SetVariable('thymio-II', 'motor.left.target', [speed])
    network.SetVariable('thymio-II', 'motor.right.target', [speed])

def turnRight():
    network.SetVariable('thymio-II', 'motor.left.target', [150])
    network.SetVariable('thymio-II', 'motor.right.target', [0])
    time.sleep(1.6)
    setThymioSpeed(0)

def turnLeft():
    network.SetVariable('thymio-II', 'motor.left.target', [0])
    network.SetVariable('thymio-II', 'motor.right.target', [150])
    time.sleep(1.6)
    setThymioSpeed(0)

def turnAround():
    network.SetVariable('thymio-II', 'motor.left.target', [300])
    network.SetVariable('thymio-II', 'motor.right.target', [0])
    time.sleep(4)
    setThymioSpeed(0)

def driveForwardXcm(cms):
    drive_time = round(cms / 10  , 0)
    setThymioSpeed(250)
    time.sleep(drive_time)
    setThymioSpeed(0)

def driveBackwardXcm(cms):
    drive_time = round(cms / 10  , 0)
    setThymioSpeed(-250)
    time.sleep(drive_time)
    setThymioSpeed(0)

# Go To Goal function
def goToGoal(search_mode):
    setThymioSpeed(TESTING_SPEED)
    movable_object = False

    # array to collect gyroscope readings
    gyro_z = []
    flip = True

    print("getting to goal...")
    prev = time.time()
    

    while not search_mode:
            ax,ay,az,wx,wy,wz = mpu6050_conv()
            gyro_z.append(wz-0.6)
            
            indexes = [1,2,3]
            front_LIDAR = [network.GetVariable('thymio-II', 'prox.horizontal')[index] for index in indexes]
            if any((x > 0 for x in front_LIDAR)) and not movable_object:
                if flip:
                    driveBackwardXcm(10)
                    turnRight()
                    driveForwardXcm(10)
                    turnLeft()
                    setThymioSpeed(TESTING_SPEED)
                    flip = False
                else:
                    driveBackwardXcm(10)
                    turnLeft()
                    driveForwardXcm(10)
                    turnRight()
                    setThymioSpeed(TESTING_SPEED)
                    flip = True
            elif any((x > 0 for x in front_LIDAR)) and movable_object:
                driveForwardXcm(15)
                movable_object = False
                setThymioSpeed(TESTING_SPEED)
            
            
            image1 = picam2.capture_array("main")
            image1 = cv2.flip(image1, -1)
            rgb_image = cv2.cvtColor(image1, cv2.COLOR_BGR2RGB)
            result, objectInfo = getObjects(rgb_image, CONFIDENCE_THRESHOLD, NMS_THRESHOLD, objects=SEARCH_OBJECTS)
            
            print(objectInfo)
            
            if len(objectInfo) != 0:
                LAST_SEEN_OBJECT = objectInfo[0][1]
                if objectInfo[0][0][0] > 600 and objectInfo[0][0][0] <= 1200:
                    print("OBJECT IN FRONT OF ROBOT")
                    if LAST_SEEN_OBJECT in OBJECT_COMPLIANCE.keys():
                        if OBJECT_COMPLIANCE[LAST_SEEN_OBJECT] < 0.8:
                            movable_object = True
                            
            
            if time.time() > prev + 2:
                prev = time.time()
                area = trapz(gyro_z, dx=5)
                print("area1 =", area)
                
                if area > 600:
                    print("TURN RIGHT!")
                    turnRight()
                elif area < -600:
                    print("TURN LEFT!")
                    turnLeft()
            
            
            #setThymioSpeed(TRAINING_SPEED)
 


# drive forward slowly until an 'object' is detected in front of robot
def complianceSearchMode(search_mode):
    setThymioSpeed(TRAINING_SPEED)
    LAST_SEEN_OBJECT = None
    gyro_z = []

    print("searching for objects...")
    prev = time.time()

    while search_mode:
            ax,ay,az,wx,wy,wz = mpu6050_conv()
            gyro_z.append(wz-0.6)
            
            # if there is an object in front of the robot (max. 10cm), test compliance
            indexes = [1,2,3]
            front_LIDAR = [network.GetVariable('thymio-II', 'prox.horizontal')[index] for index in indexes]
            if any((x > 0 for x in front_LIDAR)) and LAST_SEEN_OBJECT != None:
                setThymioSpeed(0)
                driveBackwardXcm(10)
                setThymioSpeed(0)
                time.sleep(1)
                compliance = testCompliance()
                if LAST_SEEN_OBJECT != None:
                    OBJECT_COMPLIANCE[LAST_SEEN_OBJECT] = compliance
                setThymioSpeed(0)
                time.sleep(1)
                driveBackwardXcm(30)
                turnRight()
                setThymioSpeed(TRAINING_SPEED)
                LAST_SEEN_OBJECT = None
            
            front_LIDAR = [network.GetVariable('thymio-II', 'prox.horizontal')[index] for index in indexes]
            if any((x > 0 for x in front_LIDAR)):
                driveBackwardXcm(10)
                turnLeft()
                driveForwardXcm(10)
                turnRight()
                setThymioSpeed(TRAINING_SPEED)
            
            
            image1 = picam2.capture_array("main")
            image1 = cv2.flip(image1, -1)
            rgb_image = cv2.cvtColor(image1, cv2.COLOR_BGR2RGB)
            result, objectInfo = getObjects(rgb_image, CONFIDENCE_THRESHOLD, NMS_THRESHOLD, objects=SEARCH_OBJECTS)
            
            print(objectInfo)
            
            if len(objectInfo) != 0:
                LAST_SEEN_OBJECT = objectInfo[0][1]
                if objectInfo[0][0][0] < 850:
                    # turn left
                    network.SetVariable('thymio-II', 'motor.right.target', [TRAINING_SPEED+15])
                    #network.SetVariable('thymio-II', 'motor.left.target', [0])
                elif objectInfo[0][0][0] >= 980:
                    # turn right
                    network.SetVariable('thymio-II', 'motor.left.target', [TRAINING_SPEED+15])
                    #network.SetVariable('thymio-II', 'motor.right.target', [0])
            else:
                setThymioSpeed(TRAINING_SPEED)
                
            
            if time.time() > prev + 2:
                prev = time.time()
                area = trapz(gyro_z, dx=5)
                print("area1 =", area)
                
                if area > 20:
                    print("TURN RIGHT!")
                    network.SetVariable('thymio-II', 'motor.right.target', [TRAINING_SPEED+15])
                elif area < -20:
                    print("TURN LEFT!")
                    network.SetVariable('thymio-II', 'motor.left.target', [TRAINING_SPEED+15])
            
            
            #
            
            #cv2.imshow("Output",rgb_image)
            #cv2.waitKey(1)
            
def calcCompliance(mpu_values):    
    lrgest_acc_index = mpu_values.index(max(mpu_values))
    
    small_acc = []
    next_acc = []
    
    counter = 1
    
    while len(next_acc) != 5:
        current = mpu_values[lrgest_acc_index + counter]
        next_num = mpu_values[lrgest_acc_index + (counter + 1)]
        if next_num > current:
            # add number to acceleration list
            small_acc.append(current)
        else:
            # add number to acceleration list and...
            small_acc.append(current)

            # check if the size of the array is just 1 value
            if len(small_acc) != 1 and len(next_acc) < 5:
                next_acc.append(small_acc)
                small_acc = []
            else:
                small_acc = []
        counter += 1
    
    next_acc_diff = []

    for i in range(0,5):
        difference_acc = (max(next_acc[i]) - min(next_acc[i]))
        next_acc_diff.append(difference_acc)

    initial_acc = next_acc_diff[0]
    incr_acc = 0
    decr_acc = 0
    
    for i in range(1,5):
        if next_acc_diff[i] > initial_acc:
            incr_acc += (next_acc_diff[i] - initial_acc)
        else:
            decr_acc += (initial_acc - next_acc_diff[i])
    
        initial_acc = next_acc_diff[i]
        
    
    changeInAcc = round((incr_acc /decr_acc),1)
    
    compliance = changeInAcc
    
    # cap compliance at 0
    if compliance > 1:
        compliance = 1

    return compliance

# function is called when an object is detected right in front of robot (~10cm)
def testCompliance():
    time.sleep(2)
    setThymioSpeed(TESTING_SPEED)
    
    accel_Xs = []
    
    end = time.time() + TEST_TIME
        
    while time.time() <= end:
        ax,ay,az,wx,wy,wz = mpu6050_conv()
        
        accel_Xs.append(ax)
    
    # calculate object's compliance value (0-1.0) from decceleration numbers
    objectCompliance = calcCompliance(accel_Xs)
    
    print("Object's Compliance: ", objectCompliance)
    
    return objectCompliance
    

# using object detection, find objects in robot's vision
def getObjects(img, thres, nms, objects=[]):
    classIds, confs, bbox = net.detect(img,confThreshold=thres,nmsThreshold=nms)
    if len(objects) == 0: objects = classNames
    objectInfo = []
    if len(classIds) != 0:
        for classId, confidence, box in zip(classIds.flatten(),confs.flatten(),bbox):
            className = classNames[classId - 1]
            if className in objects: 
                objectInfo.append([box,className])
    
    return img,objectInfo


def main():
    setupThymio()
    setThymioSpeed(0)
    
    print("Standby mode activated, waiting for signal...")
    GPIO.output(12, GPIO.HIGH)
    GPIO.add_event_detect(10, GPIO.RISING, callback=switchMode, bouncetime=1000) # Setup event on pin 10 rising edge
    
    while True:
        try:
            pass
        except (KeyboardInterrupt, SystemExit):
            GPIO.cleanup()
            setThymioSpeed(0)
            #os.system("pkill -n asebamedulla")
            print(OBJECT_COMPLIANCE)
            raise
    

main()
