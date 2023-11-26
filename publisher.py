"""publisher.py -- send PiCamera image stream, compressing images.

publisher
"""

import sys
import numpy as np
import socket
import time
from datetime import datetime
import traceback
import cv2
import cv2.aruco as aruco
import imagezmq
import jsonpickle
import subprocess
from message import Message

#import yaml



class Publisher():
    def __init__(self) -> None: 
        # use either of the formats below to specifiy address of display computer
        # sender = imagezmq.ImageSender(connect_to='tcp://*:5555')

        # PUB/SUB:
        self.sender = None

        self.jpeg_quality = 50  # 0 to 100, higher is better quality, 95 is cv2 default

    def __enter__(self):
        self.sender = imagezmq.ImageSender(connect_to='tcp://*:5555', REQ_REP=False)

    # ---------------------------------------------------------------------------------------------------------------------------

    def findAruco(self,img,aruco_size=5,draw=True):
        
        aruco_dict      = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)              # Aruco Dictionary
        aruco_params    = aruco.DetectorParameters()                                            # Aruco Parameters
        #imgwritten = cv2.imwrite('debugimg.png',img)
        #print("imgwritten: ",imgwritten)
        corners, ids, _ = aruco.detectMarkers(img,aruco_dict,parameters=aruco_params) 
        if ids is None: return
        total_markers   = range(0,ids.size)
        #print(type(ids))
        #print(ids.shape)
        
        if draw:
            aruco.drawDetectedMarkers(img, corners)                                             # Draw detected Marker

        # Pose Estimation
        camera_matrix   = [635.93,0.0,444.27,0.0,634.263,256.835,0.0,0.0,1.0]                   # camera_matrix
        camera_matrix   = np.array(camera_matrix).reshape(3,3)                                  
        dist_coeffs     = [-0.372403,0.157163,-0.00424351,-0.00231644,-0.0522724]               # dist_coeffs
        dist_coeffs     = np.array(dist_coeffs).reshape(5,1)                                   
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners                               # rvec, tvec
                                                                  ,aruco_size
                                                                  ,camera_matrix
                                                                  ,dist_coeffs)
        if draw:
            
            font      = cv2.FONT_HERSHEY_PLAIN
            fontsize = 1.5
            color     = (184,242,156)
            thickness = 2

            for rvec,tvec,corner,ID in zip(rvecs,tvecs,corners,ids):
                
                rot_matrix  = cv2.Rodrigues(rvec)                                               # Rotation with Rodrigues
                #print(f"Rodrigues rot_matrix: {rot_matrix}\n")
                corner       = corner.reshape(4,2)  
                corner       = corner.astype(int)
                top_right    = corner[0].ravel()
                bottom_left  = corner[2].ravel()
                
                cv2.putText(img
                            ,f"x: {round(tvec[0][0],2)} y: {round(tvec[0][1],2)} z: {round(tvec[0][2],2)}"
                            ,top_right
                            ,font
                            ,fontsize
                            ,color
                            ,thickness
                            ,cv2.LINE_AA)               

                cv2.putText(img
                            ,f"ID:{ID}"
                            ,bottom_left
                            ,font
                            ,fontsize
                            ,color
                            ,thickness
                            ,cv2.LINE_AA)

                cv2.drawFrameAxes(img
                                  ,camera_matrix
                                  ,dist_coeffs
                                  ,rvec
                                  ,tvec
                                  ,3
                                  ,2)

    # ------------------------------------------------------------------------------------------------------------------------------------------


    def publish_img(self, msg, image):

        
        # image = self.print_on_image(image)
        # processing of image before sending would go here.
        # for example, rotation, ROI selection, conversion to grayscale, etc.        
        # -----------------------------------------------------------------------------------------------------------
        
        self.findAruco(image) 

        # -----------------------------------------------------------------------------------------------------------
        ret_code, jpg_buffer = cv2.imencode(".jpg", image, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
        self.sender.send_jpg(msg, jpg_buffer)


    def publish_str(self, msg):
        self.sender.zmq_socket.send_string(msg)

    def close(self):
        self.sender.close()  # close the ZMQ socket and context

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        self.close()
    

if __name__ == '__main__':
    publisher = Publisher()

    rpi_name = socket.gethostname()  # send RPi hostname with each image

    subprocess.call("v4l2-ctl -d /dev/video0 -c exposure_auto=1 -c exposure_auto_priority=0 -c exposure_absolute=100", shell=True)

    vc = cv2.VideoCapture(0)
    vc.set(cv2.CAP_PROP_FRAME_WIDTH, 848) # more FOV than with 640
    vc.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    vc.set(cv2.CAP_PROP_FPS, 30)

    #time.sleep(1.0)  # allow camera sensor to warm up

    count = 0
    try:
        while True:  # send images as stream until Ctrl-C
            rval, image = vc.read()
            msg = Message()
            msg.id = count
            # msg = rpi_name + " " + str(count)
            msg_str = jsonpickle.encode(msg)
            
            publisher.publish_img(msg_str, image)
            count += 1
    except (KeyboardInterrupt, SystemExit):
        pass  # Ctrl-C was pressed to end program
    except Exception as ex:
        print('Python error with no Exception handler:')
        print('Traceback error:', ex)
        traceback.print_exc()
    finally:
        vc.release()  # stop the camera thread
        publisher.close()
        sys.exit()

