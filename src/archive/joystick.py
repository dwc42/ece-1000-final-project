# This is an example code for using a joystick (X direction and Switch) with the ECE 1000 Raspberry Pi Pico Kit
# For any questions regarding this script, please email: jawilliams46@tntech.edu
# This script was inspired by the following source (YouTube video linked in website!): https://toptechboy.com/calibrating-joystick-to-show-positional-angle-in-micropython-on-the-raspberry-pi-pico-w/

# For this script, we utilize the Raspberry Pi Pico Kit that you are given along with the joystick from the sensors kit ... the joystick should be connected as follows:
# V_cc to the 3.3 Volt pin on the Pico Breadboard ... PLEASE USE 3.3 VOLT! 5 Volt will give incorrect values!
# GND to the GND pin on the Pico Breadboard
# VRx to the GPIO 26 pin on the Pico Breadboard (This is one of the ADC pins)
# SW to any GPIO pin ... let's say GPIO 16
# Using this code, try and figure out how you would connect the y-direction of the joystick

# The joystick utilizies two potentiometers and will output a voltage between two values depending on the position of the joystick (think of our Lab 1 session ... what happened when the potentiometer changed its resistance?)
# In order to read these ANALOG values (think real world, continuous values like temperature!), we must use an Analog to Digital Converter (ADC), which takes these continuous values and represents as signals the computer can understand (0, 1, etc...)

# How does this code work? Well, first we declare our libraries, then we delare what is hooked up to the Raspberry Pi Pico and to which pins they are connected, and finally we then ask the Pi Pico to show us the values of the joystick (FOREVER) .... (while True:)
from machine import PWM, ADC, Pin
import math
import utime

# What pins are hooked up to the joystick?
# This is saying that we have connected the x_direction of the joystick to the GPIO 26 pin (which is one of the ADC pins!)
adc_x_joystick = ADC(Pin(26))
adc_y_joystick = ADC(Pin(27))
# This is saying that we have conencted the switch pin of the joystick ... we are setting the Pin.PULL_UP meaning that the switch is normally at a 1 and then when pressed goes to 0 ... can also set this up reversed)
sw_joystick = Pin(28, Pin.IN, Pin.PULL_UP)


# This is saying that we have conencted the switch pin of the joystick ... we are setting the Pin.PULL_UP meaning that the switch is normally at a 1 and then when pressed goes to 0 ... can also set this up reversed)

# This is saying that we are initializing the Pulse Width Modulation pins that will be used to control the position of the servo motor (50 Hz is a standard frequency for the pulse used to control the servos)
servo_x_1 = PWM(Pin(0))
servo_x_1.freq(50)
servo_switch_1 = PWM(Pin(1), freq = 50)

servo_x_2 = PWM(Pin(4))
servo_x_2.freq(50)
servo_switch_2 = PWM(Pin(5), freq = 50)

servo_x_3 = PWM(Pin(8))
servo_x_3.freq(50)
servo_switch_3 = PWM(Pin(9), freq = 50)

servos = [[servo_x_1, servo_switch_1], [servo_x_2, servo_switch_2], [servo_x_3, servo_switch_3]]
# This function will map the ADC value for the joystick to a value between -100 and 100 for ease of viewing (creating a slope between two points (m) and then creating a line y = mx + b)
# joystick_position = value from ADC (VRx)
# joystick_min = Move the joystick and see what the value is all the way left
# joystick_max = Move the joystick and see what the value is all the way right
# desired_min = -100 or 100 (depending on which direction of the joystick you want to be top or bottom)
# desired_max = 100 or -100 (^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^)
JOYSTICK_MIN = 288 #416
JOYSTICK_MAX = 65535
JOYSTICK_MIDDLE = 2^15
def get_joystick_angle():

    hVal=adc_x_joystick.read_u16()
    vVal=adc_y_joystick.read_u16()
    
    hCal=int(-.00306*hVal+100.766)
    vCal=int(.00306*vVal-100.766)
    
    deg=math.atan2(vCal,hCal)*360/2/math.pi
    if hCal==0:
        hCal=1
    if deg<0:
        deg=deg+360
    
    mag=math.sqrt(hCal**2+vCal**2)
    if mag<=4:
        hCal=0
        vCal=0
    return deg, mag		

def get_joystick_value(joystick_position,  desired_min, desired_max, joystick_min = JOYSTICK_MIN, joystick_max = JOYSTICK_MAX):
    m = ((desired_min - desired_max)/(joystick_min - joystick_max))
    return int((m*joystick_position) - (m*joystick_max) + desired_max)
def get_joystick_value_corrected(joystick_position, desired_min, desired_max):
    if joystick_position < JOYSTICK_MIDDLE:
        return get_joystick_value(joystick_position, desired_min, 0, JOYSTICK_MIN, JOYSTICK_MIDDLE)
    else:
        return get_joystick_value(joystick_position, 0, desired_max, JOYSTICK_MIDDLE, JOYSTICK_MAX)
    
def get_joystick_vector_corrected(joystick_position_x, joystick_position_y, desired_min, desired_max):
    x = get_joystick_value_corrected(joystick_position_x, desired_min, desired_max)
    y = get_joystick_value_corrected(joystick_position_y, desired_min, desired_max)
    return (x,y)
def get_joystick_vector(joystick_position_x, joystick_position_y, desired_min, desired_max):
    x = get_joystick_value(joystick_position_x, desired_min, desired_max)
    y = get_joystick_value(joystick_position_y, desired_min, desired_max)
    return (x,y)
def now():
    return (utime.time_ns())

lastPress = now()
indexServo = 0
def sw_joystick_pressed(pin):
    global lastPress, index
    print(now(),now() - lastPress)
    if ((now() - lastPress) < 500e6):
        return
    indexServo += 1
    lastPress = now();
    print("Joystick button pressed!", index)

# Set up the interrupt on the falling edge (button press)
sw_joystick.irq(trigger=Pin.IRQ_FALLING, handler=sw_joystick_pressed)
MAX_DUTY = 7864
MIN_DUTY = 1802

# Function to set the servo angle
def set_servo_angle(servo, angle):
    # Ensure the angle is within the valid range
    if angle < 0:
        angle = 0
    elif angle > 180:
        angle = 180
    
    # Calculate the duty cycle for the given angle
    duty = int((angle / 180) * (MAX_DUTY - MIN_DUTY) + MIN_DUTY)
    servo.freq(50)
    servo.duty_u16(duty)

# Example usage: set the servo to 90 degrees
max_duty = 7864
min_duty = 1802
half_duty = int(max_duty/2)

def get_pwm(angle):
    return int((angle/18.0) + 2.5)
# In this while loop, we will continually read the status (position) of the x direction and switch ... the value shown for x will be between 0 and 65535 ... as the 65535 is the maximum number that can be represented with an unsigned 16 bit integer
# however, we will be representing this number in terms of -100 to 100 in order to read the value more clearly ... so, -100 will represent the servo -90 degrees and 100 will represent the servo +90 degrees
while True:
    servos[indexServo][1].duty_u16(min_duty)
    utime.sleep(2)
    #Servo at 90 degrees
    servos[indexServo][1].duty_u16(half_duty)
    utime.sleep(2)
    #Servo at 180 degrees
    servos[indexServo][1].duty_u16(max_duty)
    utime.sleep(2)
    (deg,mag) = get_joystick_angle()
    set_servo_angle(servos[indexServo][0], deg)
    print("angle: ", deg)
    utime.sleep(0.1)