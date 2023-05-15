from gpiozero import PWMOutputDevice
from gpiozero import DigitalOutputDevice
from gpiozero import Button as zButton
from gpiozero import DistanceSensor
 
# NOTE: gpiozero and RPi.GPIO are incompatible
  
class Motor:
    
    default_freq = 1000 # by default, set freq to 1000

    # initialize Motor component
    def __init__(self, PWM_PIN, FORWARD_PIN, REVERSE_PIN, active_high = True, duty_cycle = 0, freq = default_freq):
        self.PWM_PIN = PWM_PIN
        self.FORWARD_PIN = FORWARD_PIN
        self.REVERSE_PIN = REVERSE_PIN
        self.active_high = active_high
        self.duty_cycle = duty_cycle
        self.freq = freq
        self.PWM = PWMOutputDevice(PWM_PIN, active_high, duty_cycle, freq)
        self.D_Forward = DigitalOutputDevice(FORWARD_PIN)
        self.D_Reverse = DigitalOutputDevice(REVERSE_PIN)
        
    # Stops motor
    def Stop(self):
        self.D_Forward.value = False
        self.D_Reverse.value = False
        self.PWM.value = 0
        
    # Move motor CW
    def CW(self, duty_cycle = 1):
        duty_cycle = min(abs(duty_cycle), 1)
        self.D_Forward.value = True
        self.D_Reverse.value = False
        self.PWM.value = duty_cycle
        
    # Move motor CCW
    def CCW(self, duty_cycle = 1):
        duty_cycle = min(abs(duty_cycle), 1)
        self.D_Forward.value = False
        self.D_Reverse.value = True
        self.PWM.value = duty_cycle
        
    # get PWM object
    def GetPWM(self):
        return self.PWM
    
    # get DigitalOutputDevice for Forward Pin
    def GetDigitalForward(self):
        return self.D_Forward
    
    # get DigitalOutputDevice for Reverse Pin
    def GetDigitalReverse(self):
        return self.D_Reverse

class Button:
    
    # initialize button component
    def __init__(self, PIN, pull_up = True, debounce = 0.1, hold_time = 0, hold_repeat = False):
        self.Button = zButton(PIN, pull_up, bounce_time = debounce, hold_time = hold_time, hold_repeat = hold_repeat)
        
    # hoist zButton events up a level
    def wait_for_press(self, timeout = 0):
        self.Button.wait_for_press(timeout)
        
    # hoist zButton events up a level
    def wait_for_release(self, timeout = 0):
        self.Button.wait_for_release(timeout)

    # set pressed event
    def pressed_event(self, f):
        self.Button.when_pressed = f
        
    # set released event
    def released_event(self, f):
        self.Button.when_released = f
        
class Ultrasonic:
    
    def __init__(self, ECHO_PIN, TRIGGER_PIN, thresholdCM = 50, maxDistanceCM = 100):
        self.Sensor = DistanceSensor(ECHO_PIN, TRIGGER_PIN, threshold_distance = thresholdCM/100, 
                                     max_distance = maxDistanceCM/100, queue_len = 3)
        
    def wait_for_in_range(self, timeout = None):
        self.Sensor.wait_for_in_range(timeout)
        
    def wait_for_out_range(self, timeout = None):
        self.Sensor.wait_for_out_of_range(timeout)
        
    def getDistance(self):
        return self.Sensor.distance * 100
    
    def when_in_range(self, f):
        self.Sensor.when_in_range = f
    
    def when_out_range(self, f):
        self.Sensor.when_out_of_range = f
        
class Actuator:
    
    def __init__(self, PWM_PIN, FORWARD_PIN, REVERSE_PIN):
        
        self.PWM_PIN = PWM_PIN
        self.FORWARD_PIN = FORWARD_PIN
        self.REVERSE_PIN = REVERSE_PIN
        
        self.PWM = DigitalOutputDevice(PWM_PIN)
        self.FORWARD = DigitalOutputDevice(FORWARD_PIN)
        self.REVERSE = DigitalOutputDevice(REVERSE_PIN)
        
    def CW(self):
        self.PWM.on()
        self.FORWARD.on()
        self.REVERSE.off()
        
    def CCW(self):
        self.PWM.on()
        self.FORWARD.off()
        self.REVERSE.on()
        
    def Stop(self):
        self.PWM.off()
        self.FORWARD.off()
        self.REVERSE.off()
        
    