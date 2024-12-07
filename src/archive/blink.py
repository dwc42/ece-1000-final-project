from machine import Pin, ADC, PWM
from utime import sleep
import random

# Get a random integer between 0 and 10
def button_handler(pin):
    led_index = button_pins.index(pin)
    led_pins[led_index].toggle()
        
# Set up PWM Pin for servo control
servo_pin = machine.Pin(0)
servo = PWM(servo_pin)

# Set Duty Cycle for Different Angles
max_duty = 7864
min_duty = 1802
half_duty = int(max_duty/2)

#Set PWM frequency
frequency = 50
servo.freq (frequency)

# Map LED names to their respective Pin objects using list comprehension
joy_stick_pins = [ADC(Pin(i)) for i in range(26,28)]
led_pins = [Pin(i, Pin.OUT) for i in range(2,6)]
button_pins = [Pin(i, Pin.IN, Pin.PULL_UP) for i in range(10, 14)]
buzzer_pin = Pin(Pin(15), Pin.OUT)
def read_joystick():
    x_value = joy_stick_pins[0].read_u16() // 64  # Read X-axis value
    y_value = joy_stick_pins[1].read_u16() // 64 # Read Y-axis value
    return x_value, y_value

for button in button_pins:
    button.irq(trigger=Pin.IRQ_FALLING, handler=button_handler)

def joystick_event_handler(x, y):
    print(f"Joystick changed - X: {x}, Y: {y}")
    
    
    
last_x, last_y = read_joystick()
threshold = 100
try:
    while True:
        delay =1
        #Servo at 0 degrees
        servo.duty_u16(min_duty)
        sleep(delay)
        #Servo at 90 degrees
        servo.duty_u16(half_duty)
        sleep(delay)
        #Servo at 180 degrees
        servo.duty_u16(max_duty)
        sleep(delay)    
      
except KeyboardInterrupt:
    print("Keyboard interrupt")
    # Turn off PWM 
    servo.deinit()
try:
    while True:
        servo.duty_u16(min_duty)
        sleep(2)
        #Servo at 90 degrees
        servo.duty_u16(half_duty)
        sleep(2)
        #Servo at 180 degrees
        servo.duty_u16(max_duty)
        sleep(2)
        x, y = read_joystick()
        if abs(x - last_x) > threshold or abs(y - last_y) > threshold:
            joystick_event_handler(x, y)
            last_x, last_y = x, y
        sleep(0.1)  # Polling delay
except KeyboardInterrupt:
    pass


# Example usage


# Turn off the motor
set_motor(0)
print("Finished.")
# Turn off all LEDs
for pin in led_pins:
    pin.off()
print("Finished.")
