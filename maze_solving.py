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

battery = 100


GPIO.setwarnings(False) # Ignore warnings
GPIO.setmode(GPIO.BOARD) # Use physical pin numbering

GPIO.setup(10, GPIO.IN, pull_up_down=GPIO.PUD_DOWN) # Set pin 10 to be an input pin and set initial value to be pulled low (off)
GPIO.setup(12, GPIO.OUT) # set pin 12 to be an output pin for LED

OBJECT_COMPLIANCE = {}

CONFIDENCE_THRESHOLD = 0.5
NMS_THRESHOLD = 0.2
SEARCH_OBJECTS = ['cup', 'bottle']

TRAINING_SPEED = 50
TEST_TIME = 2
TEST_SPEED = 400

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

def switchMode(channel):
    SEARCH_MODE = None
    if GPIO.input(12) == True:
        SEARCH_MODE = False
        GPIO.output(12, GPIO.LOW)
        print("Maze solving mode activated")
        mazeSolvingMode(SEARCH_MODE)
        #complianceSearchMode(SEARCH_MODE)
    else:
        SEARCH_MODE = True
        GPIO.output(12, GPIO.HIGH)
        print("Object compliance measure mode activated")
        #mazeSolvingMode(SEARCH_MODE)
        complianceSearchMode(SEARCH_MODE)

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

def calibrateThymio():
    # start calibration
    time.sleep(0.2)

    xs = []
    end = time.time() + 1

    while time.time() <= end:
        ax,ay,az,wx,wy,wz = mpu6050_conv()
        xs.append(ax)

    avg = sum(xs) / len(xs)
    error = (max(xs) - min(xs)) / 2

    print("""calibration results:
        average value: {}
        error range +-: {}
        data points: {}""".format(avg, error, len(xs)))


    c_mean, c_error = avg, error
    noise_floor = 2 * (c_error - c_mean)
    return c_mean, c_error

    print("IMU sensor noise floor: ", noise_floor)

def setThymioSpeed(speed):
    network.SetVariable('thymio-II', 'motor.left.target', [speed])
    network.SetVariable('thymio-II', 'motor.right.target', [speed])

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


def mazeSolvingMode(search_mode):
    while not search_mode:
        setThymioSpeed(200)
        if network.GetVariable('thymio-II', 'prox.horizontal')[2] > 0:
            turnLeft()
            setThymioSpeed(200)
        elif network.GetVariable('thymio-II', 'prox.horizontal')[4] >= 0:
            turnRight()
            setThymioSpeed(200)
        elif network.GetVariable('thymio-II', 'prox.horizontal')[4] > 0 and network.GetVariable('thymio-II', 'prox.horizontal')[2] > 0 and network.GetVariable('thymio-II', 'prox.horizontal')[0] > 0:
            turnAround()
            setThymioSpeed(200)
        
        image1 = picam2.capture_array("main")
        image1 = cv2.flip(image1, -1)
        rgb_image = cv2.cvtColor(image1, cv2.COLOR_BGR2RGB)
        result, objectInfo = getObjects(rgb_image, CONFIDENCE_THRESHOLD, NMS_THRESHOLD, objects=SEARCH_OBJECTS)
            
        print(objectInfo)
            
        if len(objectInfo) != 0:
            if objectInfo[0][1] in OBJECT_COMPLIANCE.values():
                compliance = OBJECT_COMPLIANCE[LAST_SEEN_OBJECT]
                if (compliance < 0.8 and battery > 50) or (compliance < 0.55 and battery <= 50):
                    if objectInfo[0][0][0] < 800:
                        # turn left
                        network.SetVariable('thymio-II', 'motor.right.target', [30])
                        network.SetVariable('thymio-II', 'motor.left.target', [0])
                    elif objectInfo[0][0][0] >= 1120:
                        # turn right
                        network.SetVariable('thymio-II', 'motor.left.target', [30])
                        network.SetVariable('thymio-II', 'motor.right.target', [0])
                    else:
                        driveForwardXcm(20)
            
                    


# drive forward slowly until an 'object' is detected in front of robot
def complianceSearchMode(search_mode):
    setThymioSpeed(TRAINING_SPEED)
    LAST_SEEN_OBJECT = None
    turned_left = False

    print("searching for objects...")
    
    while search_mode:
    #if search_mode:
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
                turnLeft()
                setThymioSpeed(TRAINING_SPEED+15)
                LAST_SEEN_OBJECT = None
            
            if network.GetVariable('thymio-II', 'prox.horizontal')[2] > 0:
                if turned_left:
                    driveBackwardXcm(10)
                    turnAround()
                    turned_left = False
                else:
                    driveBackwardXcm(10)
                    turnLeft()
                    turned_left = True
                setThymioSpeed(TRAINING_SPEED+15)
            #elif network.GetVariable('thymio-II', 'prox.horizontal')[4] > 0:
            #     turnRight()
            #     setThymioSpeed(35)
            
            
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
            
            setThymioSpeed(TRAINING_SPEED)
            
            #cv2.imshow("Output",rgb_image)
            #cv2.waitKey(1)
            
def calcCompliance(mpu_values, c_mean):    
    lrgest_acc_index = mpu_values.index(max(mpu_values))
    
    small_acc = []
    next_acc = []
    small_deacc = []
    next_deacc = []
    
    counter = 1
    
    while len(next_acc) != 5 or len(next_deacc) != 5:
        current = mpu_values[lrgest_acc_index + counter]
        next_num = mpu_values[lrgest_acc_index + (counter + 1)]
        if next_num > current:
            # add number to acceleration list
            small_acc.append(current)
            
            small_deacc.append(current)
            if len(small_deacc) != 1 and len(next_deacc) < 5:
                next_deacc.append(small_deacc)
                small_deacc = []
            else:
                small_deacc = []
        else:
            # add number to de-acceleration list
            small_deacc.append(current)
            
            small_acc.append(current)
            if len(small_acc) != 1 and len(next_acc) < 5:
                next_acc.append(small_acc)
                small_acc = []
            else:
                small_acc = []
        counter += 1
    
    next_acc_diff = []
    next_deacc_diff = []

    for i in range(0,5):
        difference_acc = (max(next_acc[i]) - min(next_acc[i]))
        difference_deacc = (max(next_deacc[i]) - min(next_deacc[i]))

        next_acc_diff.append(difference_acc)
        next_deacc_diff.append(difference_deacc)

    initial_acc = next_acc_diff[0]
    initial_deacc = next_deacc_diff[0]
    incr_acc = 0
    decr_acc = 0
    incr_deacc = 0
    decr_deacc = 0
    
    for i in range(1,5):
        if next_acc_diff[i] > initial_acc:
            incr_acc += (next_acc_diff[i] - initial_acc)
        else:
            decr_acc += (initial_acc - next_acc_diff[i])
        if next_deacc_diff[i] < initial_deacc:
            incr_deacc += (initial_deacc - next_deacc_diff[i])
        else:
            decr_deacc += (next_deacc_diff[i] - initial_deacc)
        initial_acc = next_acc_diff[i]
        initial_deacc = next_deacc_diff[i]
        
    weighted_multip = 5

    avg_magn_nextAcc = sum(next_acc_diff) / len(next_acc_diff)
    percentage_high_acc = round(100 - ((avg_magn_nextAcc / mpu_values[lrgest_acc_index]) * 100),1)
    percentage_avg_val_acc = round(100 - (((abs(c_mean) * weighted_multip) / avg_magn_nextAcc) * 100),1)
    
    avg_magn_nextDeacc = sum(next_deacc_diff) / len(next_deacc_diff)
    percentage_high_deacc = round(100 - ((avg_magn_nextDeacc / mpu_values[lrgest_acc_index]) * 100),1)
    percentage_avg_val_deacc = round(100 - (((abs(c_mean) * weighted_multip) / avg_magn_nextDeacc) * 100),1)
    
    zeroToOne_acc = round((incr_acc /decr_acc),1)
    zeroToOne_deacc = round((decr_deacc / incr_deacc),1)
    
    compliance = zeroToOne_acc
    
    # cap compliance at 0
    if compliance < 0:
        compliance = 0
    elif compliance > 1:
        compliance = 1

    return compliance

# function is called when an object is detected right in front of robot (~10cm)
def testCompliance():
    c_mean, c_error = calibrateThymio()
    time.sleep(2)
    setThymioSpeed(TEST_SPEED)
    
    accel_Xs = []
    accel_Ys = []
    accel_Zs = []
    
    end = time.time() + TEST_TIME
        
    while time.time() <= end:
        ax,ay,az,wx,wy,wz = mpu6050_conv()
        
        accel_Xs.append(ax)
        accel_Ys.append(ay)
        accel_Zs.append(az)
    
    # calculate object's compliance value (0-1.0) from decceleration numbers
    objectCompliance = calcCompliance(accel_Xs, c_mean)
    
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


def main(battery_level):
    setupThymio()
    
    print("Standby mode activated, waiting for signal...")
    GPIO.output(12, GPIO.LOW)
    GPIO.add_event_detect(10, GPIO.RISING, callback=switchMode, bouncetime=1000) # Setup event on pin 10 rising edge
    
    while True:
        try:
            print("Battery at:", battery_level)
            time.sleep(3)
            battery_level -= 1
        except (KeyboardInterrupt, SystemExit):
            GPIO.cleanup()
            setThymioSpeed(0)
            os.system("pkill -n asebamedulla")
            print(OBJECT_COMPLIANCE)
            raise
    

main(battery)