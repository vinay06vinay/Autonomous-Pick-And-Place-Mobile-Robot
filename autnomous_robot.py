# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import numpy as np
import cv2
import matplotlib.pyplot as plt
import imutils
import RPi.GPIO as gpio
import serial
#import email
from email01 import *
#Halt the robot
def gameover():
        gpio.output(31, False)
        gpio.output(33, False)
        gpio.output(35, False)
        gpio.output(37, False)
        gpio.output(16, False)
        gpio.output(18, False)
        gpio.output(36, False)
        gpio.cleanup()
        
# Initialize GPIO pins
def init():
    gpio.setmode(gpio.BOARD)
    gpio.setup(31, gpio.OUT) # IN1
    gpio.setup(33, gpio.OUT) # IN2
    gpio.setup(35, gpio.OUT) # IN3
    gpio.setup(37, gpio.OUT) # IN4
    
    gpio.setup(7, gpio.IN, pull_up_down = gpio.PUD_UP)
    gpio.setup(12, gpio.IN, pull_up_down = gpio.PUD_UP)

def distance_in_ultrasonic():
    global trig,echo    
    try:
        #Ensure output has no value
        gpio.output(trig,False)
        time.sleep(0.01)
        #Generate Pulse
        gpio.output(trig,True)
        time.sleep(0.00001)
        gpio.output(trig,False)
        #Generate echo time signal
        while gpio.input(echo) ==0 :
            pulse_start = time.time()
        while gpio.input(echo) == 1:
            pulse_end  = time.time()

        pulse_duration = pulse_end - pulse_start
        # Convert time to distance
        distance = pulse_duration* 17150
        distance =  round(distance,2)
        #Cleanup gpio pins & return distance estimate
        #gpio.cleanup()
        return distance,True
    except :
        pass
    return -1, False
def get_distance_from_ultrasonic():
    distance_list_flag = True
    distance_list = []
    while (distance_list_flag):
        for k in range(3):
            distance_i , flag = distance_in_ultrasonic()
            if(flag == True):
                distance_list.append(distance_i)
                distance_list_flag = False
    average_distance = sum(distance_list)/len(distance_list)
    distance_to_meter = average_distance/100
    return (distance_to_meter)
    
'''
Getting the current position of the IMU
'''
def get_position_imu():
    global ser,count
    ser.reset_input_buffer()
    while (ser.in_waiting == 0):
        continue
        #Read serial stream
    line = ser.readline()
    #Avoid 1st n lines of serial information
    #Setup serial stream of extra characters
    line = line.rstrip().lstrip()
    line = str(line)
    line = line.strip("'")
    line = line.strip("b'")
    #Return float
    angle = float(line)
    angle = round(angle)
    angle = int(angle)
    #print("current: ",angle)
    return angle
    #print(line,"\n")
'''
Calculation of ticks required for distance in meters
'''
def dist_ticks(d):
    wheel_rev = ((1/(np.pi*0.065)) * d)
    d_ticks = 120*8* wheel_rev
    return int(d_ticks)
'''
Calculation of ticks required for angle given in degrees
'''
def deg_ticks(deg):
    wheel_rev = (1/(np.pi*0.065)) * (((2*np.pi*85*1.4)/(1000*360))*deg)
    #print(wheel_rev)
    #print(deg,np.deg2rad(deg))
    a_ticks = 120*8* wheel_rev
    return int(a_ticks) 
'''
forward() - Moves forward based on difference in number of ticks and applying a control speed change given by the controller used on the base speed.
reverse() - Moves reverse based on difference in number of ticks and applying a control speed change given by the controller used on the base speed.
turn_right() - Turns right based on difference in number of ticks and applying a control speed change given by the controller used on the base speed.
turn_left() - Turns  Left based on difference in number of ticks and applying a control speed change given by the controller used on the base speed.
'''
def forward():
    global counterBR, counterFL, pwm_right_1, pwm_left_2,movement_speed,prev_error, kd, ki, total_error
    '''
    #Left wheels
    gpio.output(31, True)
    gpio.output(33, False)

    # Right Wheels
    gpio.output(35, False)
    gpio.output(37, True)
    '''
    #Increasing the speed of one wheel and decreasing the other based on difference of ticks
    diff_ticks = counterFL - counterBR
    #control_change = (kp * abs(diff_ticks)) + (kd * (diff_ticks - prev_error)) 
    #control_change = (kp * abs(diff_ticks)) + (kd * (diff_ticks - prev_error)) + (ki * total_error)
    #control_change = (kp * abs(diff_ticks)) + (ki * total_error)
    control_change = (kp * abs(diff_ticks))
    total_error += diff_ticks
    prev_error = diff_ticks
    final_speed = movement_speed + control_change
    final_speed1 = movement_speed - control_change
    if(final_speed <0 or final_speed > 100):
        final_speed = movement_speed
    if(final_speed1 <0 or final_speed1 > 100):
        final_speed1 = movement_speed
    if(diff_ticks < 0):
        pwm_right_1.ChangeDutyCycle(final_speed)
        pwm_left_2.ChangeDutyCycle(final_speed1)
    elif(diff_ticks > 0):
        pwm_right_1.ChangeDutyCycle(final_speed1)
        pwm_left_2.ChangeDutyCycle(final_speed)
    else:
        pwm_right_1.ChangeDutyCycle(movement_speed)
        pwm_left_2.ChangeDutyCycle(movement_speed)
def reverse():
    global counterBR, counterFL, pwm_left_1, pwm_right_2,movement_speed,prev_error, kd, ki, total_error
    diff_ticks = counterFL - counterBR
    #control_change = (kp * abs(diff_ticks)) + (kd * (diff_ticks - prev_error)) 
    #control_change = (kp * abs(diff_ticks)) + (kd * (diff_ticks - prev_error)) + (ki * total_error)
    #control_change = (kp * abs(diff_ticks)) + (ki * total_error)
    control_change = (kp * abs(diff_ticks))
    total_error += diff_ticks
    prev_error = diff_ticks
    final_speed = movement_speed + control_change
    final_speed1 = movement_speed - control_change
    if(final_speed <0 or final_speed > 100):
        final_speed = movement_speed
    if(final_speed1 <0 or final_speed1 > 100):
        final_speed1 = movement_speed
    if(diff_ticks < 0):
        pwm_left_1.ChangeDutyCycle(final_speed)
        pwm_right_2.ChangeDutyCycle(final_speed1)
    elif(diff_ticks > 0):
        pwm_left_1.ChangeDutyCycle(final_speed1)
        pwm_right_2.ChangeDutyCycle(final_speed)
    else:
        pwm_left_1.ChangeDutyCycle(movement_speed)
        pwm_right_2.ChangeDutyCycle(movement_speed)
        
def turn_right():
    global counterBR, counterFL, pwm_right_1, pwm_right_2,turn_speed, kp,kd,total_error,prev_error
    diff_ticks = counterFL - counterBR
    #control_change = kp * abs(diff_ticks)
    control_change = (kp * abs(diff_ticks)) + (kd * (diff_ticks - prev_error))
    total_error += diff_ticks
    prev_error = diff_ticks
    f_speed = turn_speed + control_change
    if(f_speed < 0 or f_speed > 100):
        f_speed = turn_speed
    if(diff_ticks < 0):
        pwm_right_1.ChangeDutyCycle(f_speed)
        pwm_right_2.ChangeDutyCycle(turn_speed)
    elif(diff_ticks > 0):
        pwm_right_1.ChangeDutyCycle(turn_speed)
        pwm_right_2.ChangeDutyCycle(f_speed)
    else:
        pwm_right_1.ChangeDutyCycle(turn_speed)
        pwm_right_2.ChangeDutyCycle(turn_speed)
def turn_left():
    global counterBR, counterFL, pwm_left_1, pwm_left_2,turn_speed, kp,kd,total_error,prev_error
    #print("Turning_left",counterBR,counterFL)
    diff_ticks = counterFL - counterBR
    #print("Difference of Ticks",diff_ticks)
    #control_change = kp * abs(diff_ticks)
    control_change = (kp * abs(diff_ticks)) + (kd * (diff_ticks - prev_error))
    total_error += diff_ticks
    prev_error = diff_ticks
    f_speed = turn_speed + control_change
    if(f_speed < 0 or f_speed > 100):
        f_speed = turn_speed
    if(diff_ticks < 0):
        #print("change_speed",f_speed,"turn_speed",turn_speed)
        pwm_left_1.ChangeDutyCycle(f_speed)
        pwm_left_2.ChangeDutyCycle(turn_speed)
    elif(diff_ticks > 0):
        pwm_left_1.ChangeDutyCycle(turn_speed)
        pwm_left_2.ChangeDutyCycle(f_speed)
    else:
        pwm_left_1.ChangeDutyCycle(turn_speed)
        pwm_left_2.ChangeDutyCycle(turn_speed)
'''
move_forward_by_ticks(distance_ticks) - calls the forward() until the distance ticks for a specified distance is achieved.
move_reverse_by_ticks(distance_ticks) - calls the reverse() until the distance ticks for a specified distance is achieved.
turn_left_by_ticks(angle_ticks) - calls the turn_left() until the angle ticks for a specified distance is achieved.
turn_right_by_ticks(angle_ticks) - calls the turn_right() until the distance ticks for a specified distance is achieved.
'''
def move_forward_by_ticks(distance_ticks):
    global counterBR,counterFL,pwm_right_1,pwm_left_2,buttonFL,buttonBR
    counterBR = 0
    counterFL = 0
    i=0
    while (counterBR < distance_ticks or counterFL < distance_ticks):
        if (gpio.input(12) != buttonBR):
            buttonBR = int(gpio.input(12)) #holds the state
            counterBR += 1

        if (gpio.input(7) != buttonFL):
            buttonFL = int(gpio.input(7)) #holds the state
            counterFL += 1
        forward()
    pwm_right_1.ChangeDutyCycle(0)
    pwm_left_2.ChangeDutyCycle(0)
    return

def move_reverse_by_ticks(distance_ticks):
    global counterBR,counterFL,pwm_left_1,pwm_right_2,buttonFL,buttonBR
    counterBR = 0
    counterFL = 0
    i=0
    while (counterBR < distance_ticks or counterFL < distance_ticks):
        if (gpio.input(12) != buttonBR):
            buttonBR = int(gpio.input(12)) #holds the state
            counterBR += 1

        if (gpio.input(7) != buttonFL):
            buttonFL = int(gpio.input(7)) #holds the state
            counterFL += 1
        reverse()
    pwm_left_1.ChangeDutyCycle(0)
    pwm_right_2.ChangeDutyCycle(0)
    return
    
def turn_left_by_ticks(angle_ticks):
    global counterBR,counterFL,buttonFL,buttonBR
    counterBR = 0
    counterFL = 0
    #print(angle_ticks)
    while (counterBR < angle_ticks or counterFL < angle_ticks):
        if (gpio.input(12) != buttonBR):
            buttonBR = int(gpio.input(12)) #holds the state
            counterBR += 1
        if (gpio.input(7) != buttonFL):
            buttonFL = int(gpio.input(7)) #holds the state
            counterFL += 1
        turn_left()
    pwm_left_1.ChangeDutyCycle(0)
    pwm_left_2.ChangeDutyCycle(0)
    return
def turn_right_by_ticks(angle_ticks):
    global counterBR,counterFL,buttonBR,buttonFL
    counterBR = 0
    counterFL = 0
    while (counterBR < angle_ticks or counterFL < angle_ticks):
        if (gpio.input(12) != buttonBR):
            buttonBR = int(gpio.input(12)) #holds the state
            counterBR += 1
        if (gpio.input(7) != buttonFL):
            buttonFL = int(gpio.input(7)) #holds the state
            counterFL += 1
        turn_right()
    pwm_right_1.ChangeDutyCycle(0)
    pwm_right_2.ChangeDutyCycle(0)
    return
'''
left(desired_turn) - Performs left turning based on the current imu position feedback continuously till the desired turn is achieved leading to desired angle
right(desired_turn) - Performs right turning based on the current imu position feedback continuously till the desired turn is achieved leading to desired angle
'''
def left(desired_turn):
    global pwm_left_1, pwm_left_2,turn_speed
    current_imu_pose = get_position_imu()
    if (current_imu_pose == None):
        return None
    if (current_imu_pose == 0):
        current_imu_pose = 360
    desired_angle = current_imu_pose - int(round(desired_turn))
    if(desired_angle < 0):
        desired_angle = 360 - abs(desired_angle)
    turn_speed_copy = turn_speed
    if(desired_turn >= 16):
        turn_speed_copy = turn_speed_copy
    elif(5 <= desired_turn <=15):
        turn_speed_copy = turn_speed_copy - 5
    desired_angle_range = list(range(desired_angle-4,desired_angle+2))
    for i in range(len(desired_angle_range)):
        if(desired_angle_range[i] < 0):
            desired_angle_range[i] = 360 - desired_angle_range[i]
        elif(desired_angle_range[i] > 360):
            desired_angle_range[i] = abs(360 - desired_angle_range[i])
    if(0<= current_imu_pose <= 60):
        while(current_imu_pose not in desired_angle_range):
            pwm_left_1.ChangeDutyCycle(turn_speed_copy)
            pwm_left_2.ChangeDutyCycle(turn_speed_copy)
            current_imu_pose = get_position_imu()
    else:
        while(current_imu_pose >= desired_angle):
            pwm_left_1.ChangeDutyCycle(turn_speed_copy)
            pwm_left_2.ChangeDutyCycle(turn_speed_copy)
            current_imu_pose = get_position_imu()
    current_imu_pose = get_position_imu()
    if( 8 <= abs(current_imu_pose - desired_angle) <=20):
        if((current_imu_pose - desired_angle) < 0):
            angle_ticks = deg_ticks(abs(current_imu_pose - desired_angle))
            turn_right_by_ticks(angle_ticks)
        else:
            angle_ticks = deg_ticks(abs(current_imu_pose - desired_angle))
            turn_left_by_ticks(angle_ticks)
    return
def right(desired_turn):
    global pwm_right_1, pwm_right_2,turn_speed
    current_imu_pose = get_position_imu()
    if (current_imu_pose == None):
        return None
    desired_angle = current_imu_pose + int(round(desired_turn))
    if(desired_angle > 360):
        desired_angle = abs(360 - desired_angle)
    turn_speed_copy = turn_speed
    if(desired_turn >= 16):
        turn_speed_copy = turn_speed_copy
    elif(5 <= desired_turn <=15):
        turn_speed_copy = turn_speed_copy - 5
    desired_angle_range = list(range(desired_angle-1,desired_angle+5))
    for i in range(len(desired_angle_range)):
        if(desired_angle_range[i] < 0):
            desired_angle_range[i] = 360 - desired_angle_range[i]
        elif(desired_angle_range[i] > 360):
            desired_angle_range[i] = abs(360 - desired_angle_range[i])
    if(270 <= current_imu_pose <= 360):
        while(current_imu_pose not in desired_angle_range):
            pwm_right_1.ChangeDutyCycle(turn_speed_copy)
            pwm_right_2.ChangeDutyCycle(turn_speed_copy)
            current_imu_pose = get_position_imu()
    else:
        while(current_imu_pose <= desired_angle):
            pwm_right_1.ChangeDutyCycle(turn_speed_copy)
            pwm_right_2.ChangeDutyCycle(turn_speed_copy)
            current_imu_pose = get_position_imu()
    current_imu_pose = get_position_imu()
    if( 8 <= abs(current_imu_pose - desired_angle) <=20):
        if((current_imu_pose - desired_angle) < 0):
            angle_ticks = deg_ticks(abs(current_imu_pose - desired_angle))
            turn_left_by_ticks(angle_ticks)
        else:
            angle_ticks = deg_ticks(abs(current_imu_pose - desired_angle))
            turn_right_by_ticks(angle_ticks)   
    return
'''
Calls the required action to perform based on direction decision and used for imu based turning.
'''
def perform_action_turn(turn_degree , direction):
    global pwm_right_1, pwm_right_2, pwm_left_1, pwm_left_2
    if(direction == 'left'):
        
        desired_turn = abs(turn_degree)
        #print("Have to go Left",desired_turn)
        left(desired_turn)
        pwm_left_1.ChangeDutyCycle(0)
        pwm_left_2.ChangeDutyCycle(0)
    else :
        #print("Have to go Right")
        desired_turn = abs(turn_degree)
        right(desired_turn)
        pwm_right_1.ChangeDutyCycle(0)
        pwm_right_2.ChangeDutyCycle(0)
    return
'''
Calculate Degree turn based on the x center of contour.
'''
def calculate_degree_turn(x):
    direction = ""
    degree = (x - (640/2))*0.061
    if(degree < 0):
        direction = 'left'
    else:
        direction = 'right'
    return degree,direction
def close_gripper():
    global pwm_gripper
    pwm_gripper.ChangeDutyCycle(3.0)
    time.sleep(0.5)
    #pwm_gripper.stop()
    return
def open_gripper():
    print("opening_gripper")
    global pwm_gripper
    pwm_gripper.ChangeDutyCycle(7.0)
    time.sleep(0.5)
    #pwm_gripper.stop()
    return
def calculate_dis2image (width,x_center,focal_length,known_width):
    return (known_width*focal_length)/width
def offset_turn (current_imu_offset,exact_desired_pose):
    if( (current_imu_offset - exact_desired_pose) < 0 ):
        angle_ticks = deg_ticks(abs(current_imu_offset - exact_desired_pose))
        turn_right_by_ticks(angle_ticks)
    elif((current_imu_offset - exact_desired_pose) > 0):
        angle_ticks = deg_ticks(abs(current_imu_offset - exact_desired_pose))
        turn_left_by_ticks(angle_ticks)
def perform_next_steps():
    move_reverse_first = dist_ticks(0.2)
    move_reverse_by_ticks(move_reverse_first)
    current_pose = get_position_imu()
    if( 0 <= current_pose <= 179):
        desired_pose_after_close = 180 - current_pose
        angle_ticks = deg_ticks(desired_pose_after_close)
        turn_right_by_ticks(angle_ticks)
    elif ( 181 <= current_pose <= 360):
        desired_pose_after_close =  current_pose - 180
        angle_ticks = deg_ticks(desired_pose_after_close)
        turn_left_by_ticks(angle_ticks)
    print("The 180 Degree Position Actioned")
    if((abs(get_position_imu() - 180 )) > 3):
        print(f"The after 180 {((abs(get_position_imu() - 180 )))}")
        offset_turn (get_position_imu(),180)
    distance_after_180 = get_distance_from_ultrasonic() - 0.45
    print("Distance",distance_after_180)
    if(distance_after_180 > 0 ):
        move_forward_after_180 = dist_ticks(distance_after_180)
        move_forward_by_ticks(move_forward_after_180)
    #Taking right to make imu pose to 270 after 180
    turn_to_270 = abs(270 - get_position_imu())
    angle_ticks = deg_ticks(turn_to_270)
    turn_right_by_ticks(angle_ticks)
    print("The 270 Degree Position Actioned")
    #after_270_change = 
    if((abs(get_position_imu() - 270 ) > 3)):
        print(f"The after 270 {((abs(get_position_imu() - 270 )))}")
        offset_turn (get_position_imu(),270)
    distance_after_270 = get_distance_from_ultrasonic() - 0.45
    print("Distance After 270 Turn",distance_after_270)
    if(distance_after_270 > 0 ):
        move_forward_after_270 = dist_ticks(distance_after_270)
        move_forward_by_ticks(move_forward_after_270)
    open_gripper()
    move_reverse_last = dist_ticks(0.15)
    move_reverse_by_ticks(move_reverse_last)
    turn_to_15 = 125
    angle_ticks = deg_ticks(turn_to_15)
    turn_right_by_ticks(angle_ticks)
    print("The current Imu pose after asking to turn to 15 ", get_position_imu())
    move_forward_after_place = dist_ticks(0.4)
    move_forward_by_ticks(move_forward_after_place)
    return 

def grab_contours(image,low_range,high_range):
    global r
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    #low_range = np.array([146,79,73],np.uint8)
    #high_range = np.array([255,255,255],np.uint8)
    #Applying in range to get a masked image consists of black and white pixels only using the lower and upper bounds
    masked_image= cv2.inRange(hsv,low_range,high_range)
    erode_image = cv2.erode(masked_image, None, iterations = 1)
    #The masked image applied on the original image to extract the region of interest. By giving mask command you will get the colored image from original image around the mask area(This means 
    # pixel value in which mask pixel has other than zero value that is considered)
    detected_image = cv2.bitwise_and(image,image, mask= masked_image)
    #Now Using Contours we draw a rectangle or circle around the region of interest
    contours = cv2.findContours(erode_image,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
    try:
        if(len(contours[0]) > 0):
            contour_list = imutils.grab_contours(contours)
            sorted_contours = sorted(contour_list, key=cv2.contourArea, reverse=True)[:4]
            max_contour = max(sorted_contours, key = cv2.contourArea)
            ((max_x_center, max_y_center), max_radius) = cv2.minEnclosingCircle(max_contour)
            # Rectangle contours
            max_rect_contour_area = cv2.contourArea(max_contour)
            x, y, w, h = cv2.boundingRect(max_contour)
            M = cv2.moments(max_contour)
            if(M["m00"] != 0 ): 
                #finding center method 1
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            else:
                #finding center method 2
                center = (int(x + (w//2)), int(y+(h//2)))
            cv2.rectangle(image,(x,y), (x+w,y+h), (0,0,255), 2)
            cv2.circle(image,center,1,(255,0,0),2)
            return (image,max_x_center,w)
        else:
            #Plan to turn by angle ticks
            r += 0.1
            distance_ticks = dist_ticks(0.05)
            move_forward_by_ticks(distance_ticks)
            angle_ticks = deg_ticks(10)
            #turn_left_by_ticks(angle_ticks)
            turn_left_by_ticks(angle_ticks)
    except:
        pass
    return (image,-1,-1)
image_list = []
global_position_x = open('global_position_x.txt','w+') 
global_position_y = open('global_position_y.txt','w+') 

ser = serial.Serial('/dev/ttyUSB0', 9600)
# Flush initial readings
while (ser.in_waiting == 0):
    continue
ser.reset_input_buffer()

r = 2.6
current_iteration_pose =[]

global_position_x.write(str(r)+'\n')
global_position_y.write(str(0.3)+'\n')
counterBR = np.uint64(0)
counterFL = np.uint64(0)
buttonBR = int(0)
buttonFL = int(0)
movement_speed = 45 #foward and reverse speed
kp = 2
ki = 0.0001
kd = 0.4
prev_error = 0
total_error = 0
#Motor Initialisation
init()
#Ultrasonic Pins Setup
trig = 16
echo = 18
gpio.setup(trig, gpio.OUT)
gpio.setup(echo, gpio.IN)
#Gripper Initialisation
gpio.setup(36,gpio.OUT)
pwm_gripper = gpio.PWM(36,50)
#Gripper Open position
pwm_gripper.start(6.5)
time.sleep(0.5)
#Initialise pwm for motors
turn_speed = 45 #turning speed
#Pins to turn left
pwm_left_1 = gpio.PWM(33, 50) #Left 
pwm_left_2 = gpio.PWM(37, 50) #right
val = 0
pwm_left_1.start(val)
pwm_left_2.start(val)
#Pins to turn right
pwm_right_1 = gpio.PWM(31, 50) #Left 
pwm_right_2 = gpio.PWM(35, 50) #right
pwm_right_1.start(val)
pwm_right_2.start(val)
time.sleep(0.1)
#count for distance calculation
count=0
known_width = 0.04
low_hsv_values = {'R':[146,79,73], 'G':[43,38,63], 'B':[68,113,94]}
high_hsv_values = {'R':[255,255,255], 'G':[100,255,255], 'B':[121,255,255]}
values = ['R','G','B','R','G','B','R','G','B']
def get_frames():
    # vide_capture = cv2.VideoCapture(-1, cv2.CAP_V4L)
    video_capture = cv2.VideoCapture(0)
    video_capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    video_capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    time.sleep(0.01)
    video_capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    ret, frame = video_capture.read()
    video_capture.release()
    return ret,frame
for j in range(9):
    current_color = values[j]
    #print(current_color)
    low_range = np.array(low_hsv_values[current_color],np.uint8)
    high_range = np.array(high_hsv_values[current_color],np.uint8)
    while(1):
        #i=0
        ret,frame = get_frames()
        if(ret):
            frame = cv2.flip(frame,-1)
            image_computed,x_center,width = grab_contours(frame,low_range,high_range)
            image_list.append(image_computed)
            if(x_center != -1):
                # show the frame to our screen
                cv2.imshow("Object Tracking", image_computed)
                turn_degree , direction = calculate_degree_turn(x_center)
                #print(turn_degree)
                if(abs(turn_degree) > 5):
                    print(turn_degree)
                    perform_action_turn(turn_degree , direction)
                else:
                    #if(count == 0):
                    
                    known_distance = 0.5
                    focal_length = (20*known_distance)/known_width
                    #print("focal _length",focal_length)
                    count = 5
                    distance_to_image = calculate_dis2image (width,x_center,focal_length,known_width)
                    print("Distance to image",distance_to_image,width)
                    if(distance_to_image <= 0.06):
                        ret,frame = get_frames()
                        frame = cv2.flip(frame,-1)
                        image_computed,x_center,width = grab_contours(frame,low_range,high_range)
                        image_list.append(image_computed)
                        turn_degree , direction = calculate_degree_turn(x_center)
                        if(abs(turn_degree) > 4):
                            print("Reverse from 2")
                            perform_action_turn(turn_degree , direction)
                            distance_ticks = dist_ticks(0.12)
                            move_reverse_by_ticks(distance_ticks)
                        else:
                            print("Performing closing")
                            close_gripper()
                            try:
                                email_send()
                            except:
                                pass
                            ret,frame = get_frames()
                            frame = cv2.flip(frame,-1)
                            image_computed,x_center,width = grab_contours(frame,low_range,high_range)
                            image_list.append(image_computed)
                            #count = 0
                            perform_next_steps()
                        break
                    else:
                        distance_ticks = dist_ticks(distance_to_image)
                        move_forward_by_ticks(distance_ticks)
                        ret,frame = get_frames()
                        frame = cv2.flip(frame,-1)
                        image_computed,x_center,width = grab_contours(frame,low_range,high_range)
                        image_list.append(image_computed)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q") :       
                break
            # if(i == 10):
            #     break
            # i+=1
gameover()
pwm_left_1.stop()
pwm_left_2.stop()
pwm_right_1.stop()
pwm_right_2.stop()
pwm_gripper.stop()
#video_capture.release()
for i in range(len(image_list)):
    cv2.imwrite(f"images/image_3_{i}.jpg",image_list[i])
cv2.destroyAllWindows()
