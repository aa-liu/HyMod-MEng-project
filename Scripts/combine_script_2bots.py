#######################################################################
# Hand gesture script to control 2 HyMod units
# Author: Aaron Liu
# Date: 24th May 2020
# Script to control two simulated HyMod units in CoppeliaSim
# with hand gesture recognition python script.
#
#######################################################################
# IMPORTANT 1: 
# Make sure to have the server side running in CoppeliaSim: 
# in a child script of a CoppeliaSim scene, add following command
# to be executed just once, at simulation start:
#
# simRemoteApi.start(19999)
#
# then start simulation, and run this program.
#
#######################################################################
# IMPORTANT 2: for each successful call to simxStart, there
# should be a corresponding call to simxFinish at the end!
#
#######################################################################

try:
    import sim
except:
    print ('--------------------------------------------------------------')
    print ('"sim.py" could not be imported. This means very probably that')
    print ('either "sim.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "sim.py"')
    print ('--------------------------------------------------------------')
    print ('')

import time
import b0RemoteApi
import math
import cv2
import numpy as np

print ('Program started')
sim.simxFinish(-1) # just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to CoppeliaSim
if clientID!=-1:
    print ('Connected to remote API server')

    # Now try to retrieve data in a blocking fashion (i.e. a service call):
    res,objs=sim.simxGetObjects(clientID,sim.sim_handle_all,sim.simx_opmode_blocking)
    if res==sim.simx_return_ok:
        print ('Number of objects in the scene: ',len(objs))
    else:
        print ('Remote API function call returned with error code: ',res)

    time.sleep(2)

    # Now retrieve streaming data (i.e. in a non-blocking fashion):
    startTime=time.time()
    sim.simxGetIntegerParameter(clientID,sim.sim_intparam_mouse_x,sim.simx_opmode_streaming) # Initialize streaming
    while time.time()-startTime < 5:
        returnCode,data=sim.simxGetIntegerParameter(clientID,sim.sim_intparam_mouse_x,sim.simx_opmode_buffer) # Try to retrieve the streamed data
        if returnCode==sim.simx_return_ok: # After initialization of streaming, it will take a few ms before the first value arrives, so check the return code
            print ('Mouse position x: ',data) # Mouse position x is actualized when the cursor is over CoppeliaSim's window


        # get handles for the wheels and the main joint
        ret1,mainHinge_Handle=sim.simxGetObjectHandle(clientID,'main_Hinge',sim.simx_opmode_blocking)
        ret1,mainHingeB_Handle=sim.simxGetObjectHandle(clientID,'main_Hinge#0',sim.simx_opmode_blocking)
        ret2,motorA_handle=sim.simxGetObjectHandle(clientID,'Wheel_A_Motor',sim.simx_opmode_blocking)
        ret3,motorB_handle=sim.simxGetObjectHandle(clientID,'Wheel_B_Motor',sim.simx_opmode_blocking)



        cap = cv2.VideoCapture(1)
        while True:
                
            try:

                # Capture frame by frame
                ret, frame = cap.read()

                # flip the mirrored camera
                frame=cv2.flip(frame,1)

                # size of kernal to slide through the image
                kernel = np.ones((3,3),np.uint8)

                # define region of interest
                x_start,x_end, y_start, y_end= 400,600,100,300
                roi=frame[y_start:y_end, x_start:x_end]
                
                # draw the associated rectange
                cv2.rectangle(frame,(x_start,y_start),(x_end,y_end),(255,255,0),0)    
                hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
                
                # define skin range filter and apply
                lower_skin = np.array([0,30,80], dtype=np.uint8)
                upper_skin = np.array([30,255,255], dtype=np.uint8)
                mask = cv2.inRange(hsv, lower_skin, upper_skin)
                
                # update mask with gaussian blur to make the image smoother
                mask = cv2.GaussianBlur(mask,(5,5),100) 
                
                # find contours
                contours,hierarchy= cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
                cnt = max(contours, key = lambda x: cv2.contourArea(x))
                
                # find contour to hull ratio to filter between 0 and 1 fingers
                hull = cv2.convexHull(cnt)
                a_hull = cv2.contourArea(hull)
                a_cnt = cv2.contourArea(cnt)
                a_ratio=((a_hull-a_cnt)/a_cnt)*100
            
                # find the defects in convex hull with respect to hand
                hull = cv2.convexHull(cnt, returnPoints=False)
                defects = cv2.convexityDefects(cnt, hull)

                
                # number of defects
                l=0
                for i in range(defects.shape[0]):
                    s,e,f,d = defects[i,0]
                    start = tuple(cnt[s][0])
                    end = tuple(cnt[e][0])
                    far = tuple(cnt[f][0])
                    pt= (100,180)
                    
                    
                    # lengths and area of triangle
                    a = math.sqrt((end[0] - start[0])**2 + (end[1] - start[1])**2)
                    b = math.sqrt((far[0] - start[0])**2 + (far[1] - start[1])**2)
                    c = math.sqrt((end[0] - far[0])**2 + (end[1] - far[1])**2)
                    s = (a+b+c)/2
                    ar = math.sqrt(s*(s-a)*(s-b)*(s-c))
                    angle = math.acos((b**2 + c**2 - a**2)/(2*b*c)) * 57
                    
                    # distance between point and convex hull
                    d = (2*ar)/a 
                    
                
                    # only look at angles less than 90 which form the defect
                    if angle <= 80 and d>30:
                        l += 1
                        cv2.circle(roi, far, 3, [255,255,255], -1)
                    
                    # draw boundary lines around hand
                    cv2.line(roi,start, end, [0,255,0], 2)
                    
                # iterate for each detected finger   
                l+=1
                

                org = (x_start+80,y_start-10)
                font = cv2.FONT_HERSHEY_SIMPLEX
                fontScale = 2
                fontColour = (0,255,255)
                fontThickness= 3
                lineType = cv2.LINE_8

                # print number of fingers detected
                if l==1:
                    if a_cnt<2000:
                        cv2.putText(frame,'None',org, font, fontScale, fontColour, fontThickness, lineType)
                    else:
                        if a_ratio<12:
                            cv2.putText(frame,'0',org, font, fontScale, fontColour, fontThickness, lineType)
                            sim.simxSetJointTargetPosition(clientID,mainHinge_Handle,0*math.pi/180,sim.simx_opmode_streaming)
                        else:
                            cv2.putText(frame,'1',org, font, fontScale, fontColour, fontThickness, lineType)
                            sim.simxSetJointTargetPosition(clientID,mainHinge_Handle,45*math.pi/180,sim.simx_opmode_streaming)
                            
                elif l==2:
                    cv2.putText(frame,'2',org, font, fontScale, fontColour, fontThickness, lineType)
                    sim.simxSetJointTargetPosition(clientID,mainHinge_Handle,-45*math.pi/180,sim.simx_opmode_streaming)
                    
                    
                elif l==3:
                    cv2.putText(frame,'3',org, font, fontScale, fontColour, fontThickness, lineType)
                    #sim.simxSetJointTargetPosition(clientID,mainHinge_Handle,90*math.pi/180,sim.simx_opmode_streaming)
                    sim.simxSetJointTargetVelocity(clientID,motorA_handle,-0.1/0.046,sim.simx_opmode_streaming)
                            
                elif l==4:
                    cv2.putText(frame,'4',org, font, fontScale, fontColour, fontThickness, lineType)
                    #sim.simxSetJointTargetPosition(clientID,mainHinge_Handle,-90*math.pi/180,sim.simx_opmode_streaming)
                    sim.simxSetJointTargetVelocity(clientID,motorA_handle,0.1/0.046,sim.simx_opmode_streaming)
                    
                elif l==5:
                    cv2.putText(frame,'5',org, font, fontScale, fontColour, fontThickness, lineType)
                    
                else :
                    cv2.putText(frame,'None',org, font, fontScale, fontColour, fontThickness, lineType)
                    
                    
                # display view
                cv2.imshow('ROI',mask)
                cv2.imshow('Final',frame)
            except:
                pass
                
            
            k = cv2.waitKey(5) & 0xFF
            if k == ord('z'):
                break
                # able to get the value
        cv2.destroyAllWindows()
        cap.release()    

        
        #sim.simxSetJointTargetPosition(clientID,mainHinge_Handle,45*math.pi/180,sim.simx_opmode_streaming)
        #sim.simxSetJointTargetVelocity(clientID,motorA_handle,0*0.1/0.046,sim.simx_opmode_streaming)
        #sim.simxSetJointTargetVelocity(clientID,motorB_handle,0*0.1/0.046,sim.simx_opmode_streaming)
        #time.sleep(0.005)


    # Before closing the connection to CoppeliaSim, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
    sim.simxGetPingTime(clientID)

    # Now close the connection to CoppeliaSim:
    sim.simxFinish(clientID)
else:
    print ('Failed connecting to remote API server')
print ('Program ended')
