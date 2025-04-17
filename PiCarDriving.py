# SEP742 Project Group2 Note:
# For this module:
# We kept the orignal AutoRC code offered by the course, for consideration of possible utilization of 
# functions such as X-Box controller as override to correct the RC Car action if it lose control in
# our test.
# New code added for our project mainly include:
# - Configuration variables
# - def update_steering
# - def update_throttle
# - def calculate_coordinates (from Assignment1)
# - def detect_lanes
# - def calculate_lane_center_offset
# - def calculate_steering
# - def visualize_lines (from Assignment1)
# - def process_object_detection
# - def DetectionHandler_1 (simplified version of def DetectionHandler_2 for testing sign detection)
# - def DetectionHandler_2

import pigpio
import pygame as pg
import subprocess
import PiShared
import PiCapture as PiCap
import PiRecording as Pir
import PiDetection as PiDet
import time
import threading
import queue
from queue import Queue
from PiShared import detection_queue, record_queue, stop_flag
import numpy as np
import cv2
from ultralytics.utils.plotting import Annotator, colors

#______________________________GPT___________________________________
def ensure_pigpiod_running():
    try:
        result = subprocess.run(['pgrep', 'pigpiod'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        
        if result.returncode != 0:
            print("pigpiod is not running. Starting it now...")
            subprocess.run(['sudo', 'pigpiod'], check=True)
        else:
            print("pigpiod is already running.")
    except Exception as e:
        print(f"Error ensuring pigpiod is running: {e}")
    
    

ensure_pigpiod_running()
time.sleep(1)
pi = pigpio.pi()
if not pi.connected:
    raise RuntimeError("Failed to connect to pigpio daemon.")

def apply_deadzone(value, deadzone): #Returns a value as long as its bigger than the gien deadzone
    """Applies a deadzone to joystick axis input."""
    if abs(value) < deadzone:
        return 0
    return value
def ControllerConectionCheck() : # Returns True or False if a controler is connected or not
    if pg.joystick.get_count() == 0:
        return False
    else :
        return True

#_____________________________GPT_________________________

stop_flag.clear()
camConnected = False #This variable is used to manage camera connection adn avoid errors, if there is a camera then camera functions will be called. if not, skip them. Avoids the intrinsic error of the picamera2 library and there not being a camera
modelLoaded = False
try:
    #threading.Thread(target=Pir.CameraSetupHandler()).start() #Multithread the Functions relating to the camera
    threading.Thread(target=PiCap.captureFrame, daemon=True).start()
    camConnected = True     
except :
    if(camConnected != True) :
        print("No Camera Connected")
        camConnected = False

try:
    #threading.Thread(target=Pir.CameraSetupHandler()).start() #Multithread the Functions relating to the camera
    threading.Thread(target=PiDet.detectObject, daemon=True).start()
    modelLoaded = True  
except :
    if(modelLoaded != True) :
        print("Model loading failed!")
        modelLoaded = False

try:
    #threading.Thread(target=Pir.CameraSetupHandler()).start() #Multithread the Functions relating to the camera
    threading.Thread(target=Pir.RecordResult, daemon=True).start()
    modelLoaded = True  
except :
    if(modelLoaded != True) :
        print("Recording result failed!")
        modelLoaded = False

pg.init() #Initialzie the pygame library
pg.joystick. init() #Inititalize the pyganme controller reader
#pg.display.set_mode((300,300))
#pg.display.set_caption("Drivin Controller")
Clock = pg.time.Clock() #Needed to add an update rate to the program to reduce CPU oad
if (ControllerConectionCheck()) : # Initialzies the connection with a controller thats connected.
    joystick = pg.joystick.Joystick(0)
    joystick.init()

#PWM Variables
pin1 = 12 # GPIO Pin Number
pin2 = 13 # ^
Freq = 50 # in Hz

#REC Light Variables
pinREC = 25


def startup_signal(): #Siqi made this signal to tell when the car is ready
    for _ in range(3):
        pi.write(pinREC, 1)
        time.sleep(0.5)
        pi.write(pinREC, 0)
        time.sleep(0.5)


startup_signal()


pi.set_PWM_range(pin1,1000) #Scales the Range of the Steering PWM Function
pi.set_PWM_frequency(pin1, Freq)
pi.set_PWM_range(pin2,1000) #Scales the Range of the Throttle PWM Function
pi.set_PWM_frequency(pin2, Freq)

# GPIO引脚定义
STEERING_PIN = 12  # 转向舵机的GPIO引脚
THROTTLE_PIN = 13  # 电调的GPIO引脚
LED_PIN = 25       # 指示灯的GPIO引脚

# PWM参数
PWM_FREQ = 50      # 50Hz PWM频率

# 转向参数
STEERING_CENTER_MS = 1.07  # 直行的脉冲宽度（毫秒）
STEERING_RANGE_MS = 0.1702  # 转向范围（毫秒）

# 油门参数
THROTTLE_NEUTRAL_MS = 1.35  # 停止的脉冲宽度（毫秒）
THROTTLE_FORWARD_MS = 1.40 # 前进的脉冲宽度（毫秒）
THROTTLE_REVERSE_MS = 1.20 # 后退的脉冲宽度（毫秒）

# 状态变量
stop_detected = False       # 是否检测到停止信号（红灯或STOP标志）
stop_sign_disappeared = False  # STOP标志是否消失
stop_sign_disappear_time = 0  # STOP标志消失的时间
model = None                # YOLO模型
camera = None               # 相机对象
running = True              # 程序运行标志
current_steering = STEERING_CENTER_MS  # 当前转向值
# current_throttle = THROTTLE_FORWARD_MS  # 当前油门值
current_throttle = THROTTLE_FORWARD_MS
should_stop = 0

# 设置PWM范围和频率
pi.set_PWM_range(STEERING_PIN, 1000)
pi.set_PWM_frequency(STEERING_PIN, PWM_FREQ)
pi.set_PWM_range(THROTTLE_PIN, 1000)
pi.set_PWM_frequency(THROTTLE_PIN, PWM_FREQ)


operating = True #main loop variable
DEADZONE =  0.1 #Deadzone for the input of the controler sticks
inputValue = 0 #Used to differentiate between controller and keyboard inputs
button_states = {} #Used by gpt code to log a one time button press
#Control Variables, related to sterring
ControlMsOut = 1.0 # in milliseconds
ControlAxis = 0.0
ControlMsCentered = 1.07 #The point where the servo is centered
ControlMsRange = .1702 #How far the servo can go left or right to ot damage the car


#Throttle Variables
ThrottleMsOut = 0
ThrottleAxis = 0.0
ThrottleMsCentered = 1.35 # in between forward and reverse for the speed controller
ThrottleMsRange = .25 #How far back and forth for a limited FWD and REv speed
ThrottleKey = 0 #used to manage the Digital Transmission

#Camera Control Variables
Recording = False


#GPT---------------------------------------------------------------------------------------------
def ControllerReconnectionHandler():
    """Handle reconnection of the controller."""
    global joystick
    if(pg.joystick.get_count() == 0) :
        print("Controller Disconnected. Please Wait For Reconnection.")
    else:
        joystick = pg.joystick.Joystick(0)
        joystick.init()

def handle_one_shot_button(button_index):
    """
    Detect a one-shot press for a specific button.

    Args:
        button_index (int): The index of the button to track.

    Returns:
        bool: True if the button is pressed as a one-shot, False otherwise.
    """
    global button_states
    if joystick is not None:
        # Initialize the button state if not already tracked
        if button_index not in button_states:
            button_states[button_index] = False  # Initially not pressed
        
        # Get the current state of the button
        button_current = joystick.get_button(button_index)
        
        # Check for one-shot press
        if button_current == 1 and not button_states[button_index]:
            button_states[button_index] = True  # Update the state
            return True  # Button was pressed (one-shot)
        elif button_current == 0 and button_states[button_index]:
            button_states[button_index] = False  # Reset the state on release

    return False  # No one-shot press detected
#GPT-------------------------------------------------------------------------------------
def updateControlDuty() : #Upates teh steering based on the desired ms output calculated 
    global ControlMsOut,Freq
    Duty = ControlMsOut / (1/pi.get_PWM_frequency(pin1) * 1000)
    pi.set_PWM_dutycycle(pin1, (Duty*1000 ) ) # this is what changes the PWM duty, mathed to the given Ms output.
    #print(Duty * 1000)

def updateThrottleDuty() : #updates the drive PWM based of the desired ms output from calculations
    global ControlMsOut,Freq
    Duty = ThrottleMsOut / (1/pi.get_PWM_frequency(pin1) * 1000)
    if(ControllerConectionCheck()) : #Turns off the drive if the controller disconnects, emergency feature
         pi.set_PWM_dutycycle(pin2, (Duty*1000 ) )
    else :
        pi.set_PWM_dutycycle(pin2, (0 ) )
    #print(Duty * 1000)
    
def WindowHanlder() : # a window handler for a visual reference while debugging, unused as of now
    global operating
    for event in pg.event.get():  # Check for events
            if event.type == pg.QUIT:  # If the window close button is clicked
                operating = False  # Exit the loop and stop the program
    
def getKeyboardInputs() : # manages the control axis and throttle axis based off the arrowkey inputs
    
    global ControlAxis, ThrottleAxis, ThrottleKey 
    
    keys = pg.key.get_pressed() # gets input keys
    
    a = 0
    b = 0
    
    if (keys[pg.K_LEFT]) : # left key, etc
        #print("left is Pressed")
        a -= 1 
    if ((keys[pg.K_RIGHT]) ) :
        #print("Right is Pressed")
        a += 1    
    if (keys[pg.K_DOWN]) :
        #print("left is Pressed")
        b -= 1 
    if ((keys[pg.K_UP]) ) :
        #print("Right is Pressed")
        b += 1
    if(keys[pg.K_SPACE]) :
        ThrottleKey = 1
        
    ControlAxis = a # NEED TO BE HERE TO RESET, WILL NOT RESET IN FUNCTION
    ThrottleAxis = b
    
def SteeringHandler() : # Calculates the desired Steering Ms output basd of collected data and the user's input
    global ControlMsCentered, ControlAxis, ControlMsRange, ControlMsOut
    ControlMsOut = ControlMsCentered + ControlMsRange * ControlAxis
    #print(ControlAxis)
    #print(ControlMsOut)
def ThrottleHandler(): # ^^^ but for thottle
    global ThrottleMsCentered, ThrottleAxis, ThrottleMsRange,ThrottleMsOut, ThrottleKey
    ThrottleMsOut = ThrottleMsCentered + ThrottleMsRange * ThrottleAxis * ThrottleKey
    print("ThrottleMsOut:  ",ThrottleMsOut)

def getControllerInputs() : #obtains controller inputs with pygame
    global DEADZONE, ControlAxis, ThrottleAxis,ThrottleKey, ThrottleMsCentered,ThrottleMsOut
    ControlAxis = apply_deadzone(joystick.get_axis(0),DEADZONE)
    ThrottleAxis = joystick.get_axis(4)/2  +.5 #gets number here
    #print(ThrottleAxis)

    if(handle_one_shot_button(4)): #Start recording
        CameraFlipFlopHandler()

    dpad = joystick.get_hat(0) # Handles dpad input for transmission usage
    if(dpad[1] == 1) :
        ThrottleKey = 1
        
    elif(dpad[1] == -1) :
        ThrottleKey = -0.625
    elif(dpad[0] == 1 or dpad[0] == -1):
        ThrottleKey = 0

    a = ThrottleMsOut/(ThrottleMsCentered + ThrottleMsRange) #Controls rumble for fun
    #print(a)
    if(a > .85):
        joystick.rumble(0,a,0)
    elif(a < .83) :
        joystick.rumble(a,0,0)
    else :
        joystick.stop_rumble()
    
def InputHandler() : #Prioritizes instanced keyboard inputs over constant controller inputs
    global inputValue, ControlAxis, ThrottleAxis

    ControlAxis = 0 #NEED TO BE HERE TO RESET CONROL AXIS WHEN COTNROLLER IS NOT PLUGGED IN. 
    ThrottleAxis = 0

    keys = pg.key.get_pressed()    
    if any(keys) :
        inputValue = 0
        #print("Keyboard Input")
    
    else:
        inputValue = 1
        #print("Controller Input")

    match inputValue :
        case 0 :
            getKeyboardInputs()
        case 1:
            if(ControllerConectionCheck()) :
                getControllerInputs()
    
def CameraFlipFlopHandler() : # Starts recording using the GPT generated library
    global Recording

    if  not Recording :
        if(camConnected) : #only do it when cam is connected, avoids errors
            threading.Thread(target=Pir.start_recording(Pir.output_file)).start()
            
            print("Started Recording")
        
    else :
        if(camConnected) :
            threading.Thread(target=Pir.stop_recording()).start()
            print("Stopped Recording")
    
    Recording = not Recording

def RecordingLightHandler() : # Turns on an LED on when recording, off when not recording
    global Recording, pinREC
    if(Recording) :
        pi.write(pinREC, 1) # on
    else :
        pi.write(pinREC, 0) # off

# 更新转向PWM
def update_steering(ms_value):
    # 将毫秒值转换为占空比
    duty = ms_value / (1/pi.get_PWM_frequency(STEERING_PIN) * 1000)
    pi.set_PWM_dutycycle(STEERING_PIN, (duty * 1000))

# 更新油门PWM
def update_throttle(ms_value):
    # 将毫秒值转换为占空比
    # print("ms_value is ", ms_value)
    duty = ms_value / (1/pi.get_PWM_frequency(STEERING_PIN) * 1000)
    pi.set_PWM_dutycycle(THROTTLE_PIN, (duty * 1000))

# 控制LED指示灯
def set_led(state):
    pi.write(LED_PIN, state)

def calculate_coordinates(frame, parameters):
    if parameters is None or np.isnan(parameters).any():
        return None

    slope, intercept = parameters
    # Sets initial y-coordinate as height from top down (bottom of the frame)
    y1 = frame.shape[0]
    # Sets final y-coordinate as 150 above the bottom of the frame
    y2 = int(y1 - 150)

    if slope == 0:
        return None

    # Sets initial x-coordinate as (y1 - b) / m since y1 = mx1 + b
    x1 = int((y1 - intercept) / slope)
    # Sets final x-coordinate as (y2 - b) / m since y2 = mx2 + b
    x2 = int((y2 - intercept) / slope)
    return np.array([x1, y1, x2, y2])

# 车道线检测
def detect_lanes(frame):
    # 转换为灰度图
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # 使用二值化突出黑色线条
    _, thresh = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY_INV)
    
    # 应用高斯模糊
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    
    # 使用Canny边缘检测
    edges = cv2.Canny(blur, 30, 100)
    
    # 定义感兴趣区域 (ROI) - 通常是图像的下半部分
    height, width = edges.shape
    roi_vertices = np.array([
        [0, height], [width*0.5, height*0.0], [width, height]
        # [0, height], [width*0.2, height*0.2], [width*0.8, height*0.2], [width, height]
    ], dtype=np.int32)
    
    # 创建掩码并应用
    # mask = np.zeros_like(edges)
    mask = np.zeros(edges.shape[:2], dtype="uint8")
    cv2.fillPoly(mask, [roi_vertices], (255,255,255))
    masked_edges = cv2.bitwise_and(edges, edges, mask=mask)
    
    # 使用霍夫变换检测直线
    lines = cv2.HoughLinesP(masked_edges, 1, np.pi/180, threshold=100, 
                           minLineLength=100, maxLineGap=50)
    
    if lines is None:
        return None
    
    # 分离左右车道线
    left_lines = []
    right_lines = []
    for line in lines:
        # x1, y1, x2, y2 = line[0]
        x1, y1, x2, y2 = line.reshape(4)

        # Skip vertical lines (infinite slope)
        if x2 == x1:
            continue

        parameters = np.polyfit((x1, x2), (y1, y2), 1)
        slope = parameters[0]
        y_intercept = parameters[1]
      
        if slope < 0:
            left_lines.append((slope, y_intercept))
        else:
            right_lines.append((slope, y_intercept))

    lane_lines = []

    if len(left_lines) > 0:
        left_avg = np.average(left_lines, axis = 0)
        left_line = calculate_coordinates(frame, left_avg)
        lane_lines.append(left_line)
    else:
        lane_lines.append(None)
    
    # Calculate average right line if any detected
    if len(right_lines) > 0:
        right_avg = np.average(right_lines, axis=0)
        right_line = calculate_coordinates(frame, right_avg)
        lane_lines.append(right_line)
    else:
        lane_lines.append(None)
    
    return lane_lines

# 计算车道中心偏移
def calculate_lane_center_offset(frame, left_line, right_line):
    if left_line is None or right_line is None:
        return 0  # 如果未检测到车道线，返回0偏移
    
    # 获取图像中心x坐标
    height, width, _ = frame.shape
    image_center = width // 2
    
    # 在图像底部计算左右车道线的x坐标
    left_x = left_line[2]  # 左车道线在底部的x坐标
    right_x = right_line[2]  # 右车道线在底部的x坐标
    
    # 计算车道中心
    lane_center = (left_x + right_x) // 2
    
    # 计算车道中心与图像中心的偏移
    offset = lane_center - image_center
    
    return offset

# 计算转向角度
def calculate_steering(offset):
    # 根据偏移量计算转向角度
    # 这里使用简单的比例控制
    # 偏移量为正，需要向右转；偏移量为负，需要向左转
    steering_angle = STEERING_CENTER_MS + offset * 0.01  # 调整系数以获得合适的响应
    
    # 限制转向范围
    if steering_angle > STEERING_CENTER_MS + STEERING_RANGE_MS:
        steering_angle = STEERING_CENTER_MS + STEERING_RANGE_MS
    elif steering_angle < STEERING_CENTER_MS - STEERING_RANGE_MS:
        steering_angle = STEERING_CENTER_MS - STEERING_RANGE_MS
    
    return steering_angle

# from 742 assignment 1
def visualize_lines(frame, lines):
    # Creates an image filled with zero intensities with the same dimensions as the frame
    # Add one line of code here
    no_line = True
    image_black = np.zeros((frame.shape[0],frame.shape[1],3),dtype=np.uint8)
    # Checks if any lines are detected
    if lines is not None:
        for line in lines:
            if line is not None:
                try:
                    x1, y1, x2, y2 = map(int, line)
                # Draws lines between two coordinates with green color and 5 thickness
                # Add one line of code here
                    lines_visualize = cv2.line(image_black,(x1,y1),(x2,y2),(0,255,0),5)
                    no_line = False
                except (ValueError, TypeError, cv2.error):
                    continue
    
    if no_line is True:
        return frame
    else:
        return lines_visualize

# 对象检测处理
def process_object_detection(frame):
    global model, stop_detected, stop_sign_disappeared, stop_sign_disappear_time
    
    if model is None:
        return frame, False
    
    # 使用YOLO模型进行检测
    results = model(frame)
    
    # 在图像上绘制检测结果
    annotated_frame = results[0].plot()
    
    # 分析检测结果
    detected_red_light = False
    detected_green_light = False
    detected_stop_sign = False
    
    frame_area = frame.shape[0] * frame.shape[1]  # 总像素数
    
    for r in results:
        for box in r.boxes:
            # 获取类别
            cls_id = int(box.cls.item())
            cls_name = model.names[cls_id]
            
            # 获取边界框
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
            
            # 计算边界框面积
            box_area = (x2 - x1) * (y2 - y1)
            
            # 计算面积百分比
            area_percent = (box_area / frame_area) * 100
            
            # 根据检测到的对象类型和面积百分比处理逻辑
            if cls_name == "red_light" and area_percent >= 15:
                detected_red_light = True
                cv2.putText(annotated_frame, f"Red Light {area_percent:.1f}%", (x1, y1-10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            
            elif cls_name == "green_light" and area_percent >= 15:
                detected_green_light = True
                cv2.putText(annotated_frame, f"Green Light {area_percent:.1f}%", (x1, y1-10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            elif cls_name == "stop" and area_percent >= 35:
                detected_stop_sign = True
                cv2.putText(annotated_frame, f"STOP {area_percent:.1f}%", (x1, y1-10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
    
    # 更新停止状态
    if detected_red_light:
        stop_detected = True
        stop_sign_disappeared = False
    elif detected_green_light:
        stop_detected = False
        stop_sign_disappeared = False
    elif detected_stop_sign:
        stop_detected = True
        stop_sign_disappeared = False
    elif stop_detected and not detected_stop_sign and not detected_red_light:
        # 如果之前检测到STOP标志但现在没有检测到
        if not stop_sign_disappeared:
            stop_sign_disappeared = True
            stop_sign_disappear_time = time.time()
    
    # 处理STOP标志消失后的逻辑
    if stop_sign_disappeared and time.time() - stop_sign_disappear_time >= 1.0 and not detected_red_light:
        stop_detected = False
        stop_sign_disappeared = False
    
    # 在图像上显示状态
    status_text = "Stop" if stop_detected else "Go"
    cv2.putText(annotated_frame, status_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, 
                (0, 0, 255) if stop_detected else (0, 255, 0), 2)
    
    return annotated_frame, stop_detected

def DetectionHandler_1():
    try:
        class_names = ["stop", "turn left", "turn right"]

        if not detection_queue.empty():
            detection_package = detection_queue.get_nowait()  # N detections * [x1, y1, x2, y2, conf, class]   
            print(f"{time.time()} PiCarDriving: get a frame from detection_queue.")

            frame = detection_package[0]
            detections = detection_package[1]

            boxes = detections[:, :4]  #(N, 4)
            objectness = detections[:, 4] #(N, 1)
            class_scores = detections[:, 5:] #(N, 3)
            if len(class_scores[0]) != len(class_names):
                print("Error! Detected class scores and class names have different lengths!")
                return 0
            class_ids = np.argmax(class_scores, axis=1) #(N, 1)
            class_confidences = class_scores[np.arange(len(class_scores)), class_ids] #(N, 1)

            # Final confidence = objectness * class_confidence
            scores = objectness * class_confidences #(N, 1)

            confidence_threshold = 0.001
            iou_threshold = 0.99 # threshold for NMS
            mask = scores > confidence_threshold  #(d, 1)  d=number of detections with higher confidence than threshold
            boxes = boxes[mask] #(d, 1)
            class_ids = class_ids[mask] #(d, 1)
            scores = scores[mask] #(d, 1)

            print(f"PiCarDriving: After confidence mask shape: {class_ids.shape}.")

            indices = cv2.dnn.NMSBoxes(bboxes=boxes.tolist(), scores=scores.tolist(), \
                                   score_threshold=confidence_threshold, nms_threshold=iou_threshold)
            
            print(f"PiCarDriving: After NMS shape: {len(indices)}.")

            if len(indices) > 0:
                indices = np.array(indices).flatten()
                final_boxes = boxes[indices]
                final_scores = scores[indices]
                final_class_ids = class_ids[indices]                    

                for class_id in final_class_ids:
                    print("detected class: ", class_names[class_id])

    except queue.Empty:
        pass  # No new detections

def DetectionHandler_2():
    global should_stop, current_steering, current_throttle
    try:
        class_names = ["stop", "turn left", "turn right"]

        if not detection_queue.empty():
            detection_package = detection_queue.get_nowait()  # N detections * [x1, y1, x2, y2, conf, class]   
            print(f"{time.time()} PiCarDriving: get a frame from detection_queue.")

            frame = detection_package[0]
            detections = detection_package[1]
            result_frame = frame.copy()

            print(f"picardriving: detections shape: {len(detections)}")
            # 处理车道线检测
            lines_average = detect_lanes(frame)
            #print("lines_average shape: ", len(lines_average))
            if lines_average is not None:
                lines_visualize = visualize_lines(frame, lines_average)
                result_frame = cv2.addWeighted(frame, 0.9, lines_visualize, 1, 1)


                # 创建有效线列表
                valid_lines = []
                if lines_average[0] is not None:
                    valid_lines.append(lines_average[0]) #left line 
                if lines_average[1] is not None:
                    valid_lines.append(lines_average[1]) #right line

                # 计算车道中心偏移
                offset = calculate_lane_center_offset(frame, lines_average[0], lines_average[1])
            
                # 计算转向角度
                steering_angle = calculate_steering(offset)
                current_steering = steering_angle
            
            # 进行对象检测
            boxes = detections[:, :4]  #(N, 4)
            objectness = detections[:, 4] #(N, 1)
            class_scores = detections[:, 5:] #(N, 3)
            if len(class_scores[0]) != len(class_names):
                print("Error! Detected class scores and class names have different lengths!")
                return 0
            class_ids = np.argmax(class_scores, axis=1) #(N, 1)
            class_confidences = class_scores[np.arange(len(class_scores)), class_ids] #(N, 1)

            # Final confidence = objectness * class_confidence
            scores = objectness * class_confidences #(N, 1)

            confidence_threshold = 0.2
            iou_threshold = 0.9 # threshold for NMS
            mask = scores > confidence_threshold  #(d, 1)  d=number of detections with higher confidence than threshold
            boxes = boxes[mask] #(d, 1)
            class_ids = class_ids[mask] #(d, 1)
            scores = scores[mask] #(d, 1)

            print(f"PiCarDriving: After confidence mask shape: {class_ids.shape}.")

            indices = cv2.dnn.NMSBoxes(bboxes=boxes.tolist(), scores=scores.tolist(), \
                                   score_threshold=confidence_threshold, nms_threshold=iou_threshold)
            
            print(f"PiCarDriving: After NMS shape: {len(indices)}.")

            should_stop = 0

            if len(indices) > 0:
                indices = np.array(indices).flatten()
                final_boxes = boxes[indices] #[cx,cy,w,h]
                final_scores = scores[indices]
                final_class_ids = class_ids[indices] 

                final_boxes_draw = final_boxes.copy()
                final_boxes_draw[:,0]=final_boxes[:,0]-final_boxes[:,2]/2  #x1 = cx-w/2
                final_boxes_draw[:,1]=final_boxes[:,1]-final_boxes[:,3]/2  #y1 = cy-h/2
                final_boxes_draw[:,2]=final_boxes[:,0]+final_boxes[:,2]/2  #x2 = cx+w/2
                final_boxes_draw[:,3]=final_boxes[:,1]+final_boxes[:,3]/2  #y2 = cy+h/2  

                for class_id in final_class_ids:
                    print("detected class: ", class_names[class_id])

                annotator = Annotator(result_frame, line_width=2)               

                for box, class_id, confidence in zip(final_boxes_draw, final_class_ids, final_scores):
                    print("detected class: ", class_names[class_id])
                    label = f"{class_names[int(class_id)]} {confidence:.2f}"
                    annotator.box_label(box, color=colors(int(class_id), True), label=label)

                if final_class_ids[0] == 0:
                    should_stop =1
          
            # 更新停止状态
            if should_stop:
                current_throttle = THROTTLE_NEUTRAL_MS
                set_led(1)  # 亮灯表示停止
            else:
                current_throttle = THROTTLE_FORWARD_MS
                # current_throttle = THROTTLE_REVERSE_MS
                set_led(0)  # 灭灯表示行驶
            
            

            # 更新车辆控制
            update_steering(current_steering)
            update_throttle(current_throttle)

            # if the retriever is slower and the supplier is faster, there would be backlogs, in which case 
            #  the supplier would take out and discard the (only one, old) frame in the queue right 
            #  before it stores the new frame.
            try:
                record_queue.get_nowait()
            except queue.Empty:
                pass
            record_queue.put(result_frame)

    except queue.Empty:
        pass  # No new detections
#--------------------MAIN LOOP---------------------------------------------------

#ControllerConectionCheck()

#pg.display.update()

try:
    while operating: #calls functions over and over until the pi is turned off or commanded by the keyobard
        
        RecordingLightHandler()
        pg.event.pump()
        WindowHanlder()  
        #print(camConnected)

        DetectionHandler_2()
        # current_throttle = THROTTLE_FORWARD_MS
        # print("throttle key is  ", ThrottleKey)

        # ControllerReconnectionHandler()
        # InputHandler()
        # SteeringHandler()
        # ThrottleHandler()
        # updateThrottleDuty()
        # updateControlDuty()
        update_steering(current_steering)
        update_throttle(current_throttle)
        Clock.tick(60) # Limit it to 60 updates per second, can be reduced to spare processing power
except KeyboardInterrupt :
    print("\nProgram interrupted by user")
    stop_flag.set()

#Closing Commands, turns off PWMS, 

# notify other threads to stop
stop_flag.set()
time.sleep(0.5)

pg.quit()
pi.set_PWM_dutycycle(pin1,  0)
pi.set_PWM_dutycycle(pin2,  0)
pi.set_mode(pinREC, 0)
# if(camConnected) :
#     threading.Thread(target=Pir.CameraEnd()).start() #turn off camera


