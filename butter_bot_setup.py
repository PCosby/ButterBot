from gpio_utils import *
import subprocess
import time

#//////////////// VIDEO STUFF /////////////
from imutils.video import VideoStream
import numpy as np
import imutils
import time
import cv2
#///////////////////////////////////////////////////

#///////////////// Define Motor Driver GPIO Pins /////////////////

# Motor A, Left Side GPIO CONSTANTS
PWM_DRIVE_LEFT = 13		 # ENA - H-Bridge enable pin
FORWARD_LEFT_PIN = 5   # IN1 - Forward Drive
REVERSE_LEFT_PIN = 22	 # IN2 - Reverse Drive

# Motor B, Right Side GPIO CONSTANTS
PWM_DRIVE_RIGHT = 19		# ENB - H-Bridge enable pin
FORWARD_RIGHT_PIN = 6	# IN1 - Forward Drive
REVERSE_RIGHT_PIN = 26	# IN2 - Reverse Drive

# Motor C, Catapult GPIO CONSTANTS
PWM_DRIVE_CATAPULT = 18		# ENB - H-Bridge enable pin
FORWARD_CATAPULT_PIN = 14	# IN1 - Forward Drive
REVERSE_CATAPULT_PIN = 15	# IN2 - Reverse Drive

# Push Actuator GPIO CONSTANTS
PWM_DRIVE_PUSH = 25 		# ENB - H-Bridge enable pin
FORWARD_PUSH_PIN = 24 	# IN1 - Forward Drive
REVERSE_PUSH_PIN = 23 	# IN2 - Reverse Drive

# Blade Actuator GPIO CONSTANTS
PWM_DRIVE_BLADE = 12		# ENB - H-Bridge enable pin
FORWARD_BLADE_PIN = 20	# IN1 - Forward Drive
REVERSE_BLADE_PIN = 16	# IN2 - Reverse Drive

POWER_BUTTON_PIN = 11 # turn on/off the butter bot :)

US_PIN_ECHO = 21
US_PIN_TRIG = 7


# initialize motor components
L_MOTOR = Motor(PWM_DRIVE_LEFT, FORWARD_LEFT_PIN, REVERSE_LEFT_PIN)
R_MOTOR = Motor(PWM_DRIVE_RIGHT, FORWARD_RIGHT_PIN, REVERSE_RIGHT_PIN)
# For throwing butter
C_MOTOR = Motor(PWM_DRIVE_CATAPULT, FORWARD_CATAPULT_PIN, REVERSE_CATAPULT_PIN)

# For pushing butter
PUSH_ACTUATOR = Actuator(PWM_DRIVE_PUSH, FORWARD_PUSH_PIN, REVERSE_PUSH_PIN)
# For cutting butter
BLADE_ACTUATOR = Actuator(PWM_DRIVE_BLADE, FORWARD_BLADE_PIN, REVERSE_BLADE_PIN)

# Power Button
POWER_BUTTON = Button(POWER_BUTTON_PIN, debounce = .1)

# Ultrasonic
US_RANGE = 40
U_SONIC = Ultrasonic(US_PIN_ECHO, US_PIN_TRIG, thresholdCM = US_RANGE)

# //////////////////// ROBOT move methods (for both wheels/motors) //////////////////

def Wheels_Stop():
  L_MOTOR.Stop()
  R_MOTOR.Stop()
  
def Wheels_Forward(duty_cycle = 1):
  L_MOTOR.CW(duty_cycle)
  R_MOTOR.CW(duty_cycle)
  
def Wheels_Backwards(duty_cycle = 1):
  L_MOTOR.CCW(duty_cycle)
  R_MOTOR.CCW(duty_cycle)

def Wheels_Left(duty_cycle = 1):
  L_MOTOR.CCW(duty_cycle)
  R_MOTOR.CW(duty_cycle)
  
def Wheels_Right(duty_cycle = 1):
  L_MOTOR.CW(duty_cycle)
  R_MOTOR.CCW(duty_cycle)
    
Catapult_Spin_Reverse = C_MOTOR.CW
Catapult_Spin = C_MOTOR.CCW
Catapult_Stop = C_MOTOR.Stop
  
Push_Extend = PUSH_ACTUATOR.CW
Push_Retract = PUSH_ACTUATOR.CCW
Push_Stop = PUSH_ACTUATOR.Stop

Blade_Extend = BLADE_ACTUATOR.CW
Blade_Retract = BLADE_ACTUATOR.CCW
Blade_Stop = BLADE_ACTUATOR.Stop



# ////////////// AUDIO STUFF //////////////////
def PlayAudio(filename):
  subprocess.Popen("aplay Audio/"+filename+".wav", shell = True)
  
def PlayOmg():
  PlayAudio("omg")
  
def PlayWhatIsMyPurpose():
  PlayAudio("what-is-purpose")
  
def PlayButter():
  PlayAudio("butter")
  
def PlayNoFriendship():
  PlayAudio("no-friendship")

# ///////////////////////////////////////////////


# ////////////////// VIDEO STUFF ////////////////
PROTOTEXT = "Detection/deploy.prototxt.txt"
MODEL = "Detection/cmodel.caffemodel"
CONFIDENCE = 0.5

net = cv2.dnn.readNetFromCaffe(PROTOTEXT, MODEL)
vs = VideoStream(usePiCamera=True).start() 
time.sleep(2.0)

def UpdateCamera():
  frame = vs.read()
  frame = imutils.resize(frame, width=400)
  frame = cv2.convertScaleAbs(frame, alpha=5, beta=10)
  (h, w) = frame.shape[:2]
  blob = cv2.dnn.blobFromImage(cv2.resize(frame, (300, 300)), 1.0, (300, 300), (104.0, 177.0, 123.0))
  net.setInput(blob)
  detections = net.forward()
  

  # for i in range(0, detections.shape[2]):
  #   confidence = detections[0, 0, i, 2]
  #   if confidence < CONFIDENCE:
  #     continue
      
  #   box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
  #   (startX, startY, endX, endY) = box.astype("int")

  #   text = "{:.2f}%".format(confidence * 100)
  #   y = startY - 10 if startY - 10 > 10 else startY + 10
  #   cv2.rectangle(frame, (startX, startY), (endX, endY), (0, 255, 0), 2)
  #   cv2.putText(frame, text, (startX, y),
  #   cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 2)
  #   print(20000*6.5/((startX - endX)*(startY-endY)))

  # cv2.imshow("Frame", frame)
  
  closest = None
  closest_area = 0
  
  for i in range(0, detections.shape[2]):
    if detections[0, 0, i, 2] < CONFIDENCE:
      continue
    
    detect = (detections[0, 0, i, 3:7] * np.array([w, h, w, h])).astype("int")
    detect_area = (detect[2] - detect[0]) * (detect[3] - detect[1])
        
    if closest is None or detect_area > closest_area:
      closest = detect
      closest_area = detect_area
      
  return closest

thresh = 20

def IsOriented():
  detect = UpdateCamera()
  if detect is None:
    return None
  
  offset = (detect[0] + detect[2])/2 - 200
  
  return abs(thresh) < offset

def WaitUntilInPicture():
  while IsOriented() is None:
    time.sleep(0.1)

def OrientRotate():
  
  detect = None
  offset = 1000
  
  while True:
    
    detect = UpdateCamera()
    offset = (detect[0] + detect[2])/2 - 200 if not detect is None else offset
    
    if detect is None:
      Wheels_Stop()
      WaitUntilInPicture()
      continue
        
    if offset < -thresh:
      Wheels_Left(0.28)
    elif offset > thresh:
      Wheels_Right(0.28)
    else:
      Wheels_Stop()
      break
      

in_range = False      

def USRangeFunct(b):
  
  def funct():
    global in_range
    in_range = b
    
  return funct


U_SONIC.when_in_range(USRangeFunct(True))
U_SONIC.when_out_range(USRangeFunct(False))

def MoveForwardUS():
  
  if in_range:
    return
  
  Wheels_Forward(.3)
  
  while not in_range:
            
    if not IsOriented():
      
      Wheels_Stop()
      WaitUntilInPicture()
      OrientRotate()
      Wheels_Forward(.3)
      
    print(in_range)
            

  Wheels_Stop()
  print("done")
  
cut_time_extend = 3.5
cut_time_retract = 5
  
def CutButter():
  Blade_Extend()
  time.sleep(cut_time_extend)
  Blade_Retract()
  time.sleep(cut_time_retract)
  Blade_Stop()
  
Blade_Retract()
Push_Retract()

push_time = 4
def PushButter(i = 1):
  Push_Extend()
  time.sleep(push_time * i)
  Push_Stop()
  
reset_push_time = 12
def ResetPush():
  Push_Retract()
  time.sleep(reset_push_time)
  Push_Stop()

LOAD_CATAPULT_TIME = 3.8
CATAPULT_SPEED = 0.3


def LaunchCatapult():
  Catapult_Spin(CATAPULT_SPEED)
  time.sleep(LOAD_CATAPULT_TIME)
  Catapult_Stop()
  

if  True:
  Catapult_Spin(CATAPULT_SPEED)
  # Catapult_Spin_Reverse(CATAPULT_SPEED)
  time.sleep(2)
  Catapult_Stop()
else:
  LaunchCatapult()


      

# cv2.destroyAllWindows()
# vs.stop()