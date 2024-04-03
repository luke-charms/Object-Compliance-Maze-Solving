#Import the Open-CV extra functionalities
import cv2

from picamera2 import Picamera2

import dbus, os
import dbus.mainloop.glib

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

def setupThymio():
    os.system("asebamedulla \"ser:name=Thymio-II\" &")
    
    # Set up the DBUS so the script may communicate with the Thymio-II
    dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)
    
    # Create Aseba network - make sure the Aseba Medulla service is running
    bus = dbus.SessionBus()
    
    global network
    network = dbus.Interface(bus.get_object('ch.epfl.mobots.Aseba', '/'),
                            dbus_interface='ch.epfl.mobots.AsebaNetwork')


#This is to set up what the drawn box size/colour is and the font/size/colour of the name tag and confidence label   
def getObjects(img, thres, nms, draw=True, objects=[]):
    classIds, confs, bbox = net.detect(img,confThreshold=thres,nmsThreshold=nms)
#Below has been commented out, if you want to print each sighting of an object to the console you can uncomment below     
#print(classIds,bbox)
    if len(objects) == 0: objects = classNames
    objectInfo = []
    if len(classIds) != 0:
        for classId, confidence, box in zip(classIds.flatten(),confs.flatten(),bbox):
            className = classNames[classId - 1]
            if className in objects: 
                objectInfo.append([box,className])
                if (draw):
                    cv2.rectangle(img,box,color=(0,255,0),thickness=2)
                    cv2.putText(img,classNames[classId-1].upper(),(box[0]+10,box[1]+30),
                    cv2.FONT_HERSHEY_COMPLEX,1,(0,255,0),2)
                    cv2.putText(img,str(round(confidence*100,2)),(box[0]+200,box[1]+30),
                    cv2.FONT_HERSHEY_COMPLEX,1,(0,255,0),2)
    
    return img,objectInfo

#Below determines the size of the live feed window that will be displayed on the Raspberry Pi OS
if __name__ == "__main__":
    setupThymio()
    picam2=Picamera2()
    camera_config = picam2.create_still_configuration(main={"size": (1920, 1080)}, buffer_count=2)
    picam2.configure(camera_config)
    picam2.start()
   
    while True:
        image1 = picam2.capture_array("main")
        image1 = cv2.flip(image1, -1)
        rgb_image = cv2.cvtColor(image1, cv2.COLOR_BGR2RGB)
        result, objectInfo = getObjects(rgb_image,0.45,0.2,objects=['cup', 'pottedplant'])
        print(objectInfo)
        cv2.imshow("Output",rgb_image)
        cv2.waitKey(1)