import utime
from machine import Pin, PWM, ADC, DAC
from read_temp import init_temp_sensor, read_temp
from feeding_mussels import compute_concentration, feed_amount
import tcs34725
from pid import PID

#initialize PID
pid = PID(1.0, 1.0, 0.2, setpoint=17.5)
pid.tunings = (-1.0, -0.1, 0)
pid.output_limits = (0, 10)

# Initialize Input/Output devices
temp_sens = init_temp_sensor()

# Initialize PWM output for motor 1
step_pin1 = PWM(Pin(5, Pin.OUT))
# Initialize PWM output for motor 2
#step_pin2 = PWM(Pin(18, Pin.OUT))

# Initiliazes cooling pins
cooling_volts = Pin(4, Pin.OUT)
cooling_volts.value(0)

# Set PWM frequency
step_pin1.freq(500)
#step_pin2.freq(666)
# Set motor speed
step_pin1.duty(10)
#step_pin2.duty(10)

# Initialize LED rgb pins
r = Pin(32, Pin.OUT)
g = Pin(15, Pin.OUT)
b = Pin(33, Pin.OUT)
#Set desired colors on (TODO: move to another script)
def set_led_color(red, green, blue):
    r.value(red)
    g.value(green)
    b.value(blue)

set_led_color(1, 0,1)

# initialize i2c
i2c = I2C(scl=Pin(22), sda=Pin(21), freq=100000)

# Initialize rgb sensor
rgb_sensor = tcs34725.TCS34725(i2c)
rgb_sensor.integration_time(20) #value between 2.4 and 614.4.
rgb_sensor.gain(16) #must be a value of 1, 4, 16, 60

# Define sample time and last sample variable
SAMPLE_INTERVAL = 1000

pid.sample_time = SAMPLE_INTERVAL/1000

#Constants
STEPPER_MOTOR_FLOW = 1.408 # ml/s

MEASUREMENT_TIME = 3.7

TUBE_TIME = 2.2

BACK_TIME = 300 

MIN_CONCENTRATION = 100

MAX_CONCENTRATION = 2500


#pump control for PID
#TODO: move this to seperate document
# Changes flowrate of cooling pump
def change_flowrate(new_flow):
    step_pin1.freq(int(new_flow))

# Switches cooler voltage (True sets to 5v, False to 12v) 
def change_voltage(On5):
    if (On5):
        cooling_volts.value(1)
    else:
        cooling_volts.value(0)

# Gets frequency change for pump based on error from PID
def get_change(error):
    if(error >= 2 ):
        if(cooling_volts.value() == 0):
            change_voltage(True)
        error = (error - 2) * 53
        freq = error + 250
        if (freq > 666):
            freq = 666
        change_flowrate(freq)
        if(cooling_volts.value() == 1):
            change_voltage(False)
        print('freq: ' + str(freq) )
    else:
        if(step_pin1.freq != 0):
            change_flowrate(250)
        if(cooling_volts.value() == 0):
            change_voltage(True)
        print('no change')
    

sample_last_ms = 0
concentration_check = 0
is_feeding = False
feeding_time = 0
feeding_start = 0

while (True):
    # stops the feeding pump after feeding time has passed 
    if (is_feeding and utime.ticks_diff(utime.ticks_ms(), feeding_start) >= feeding_time):
        step_pin3.freq(1)
        is_feeding = False

    # check for pid sampling
    if utime.ticks_diff(utime.ticks_ms(), sample_last_ms) >= SAMPLE_INTERVAL:
        #gets temp from thermistor
        temp = read_temp(temp_sens)
        # computes error from pid and changes pumps accordingly
        control = pid(temp)
        get_change(control)

        # checks if concentration should be checked 
        if (concentration_check >= 5):
            concentration_check = 0
            
            concentration = compute_concentration(rgb_sensor)

            if (not is_feeding and concentration <= MIN_CONCENTRATION): 
                is_feeding = True
                feeding_time = feed_amount(concentration) / STEPPER_MOTOR_FLOW
                feeding_start = utime.ticks_ms()
                #TODO: move this to seperate function
                step_pin3.freq(500)
            elif (is_feeding and concentration >= MAX_CONCENTRATION):
                # stops the feeding
                step_pin3.freq(1)
                is_feeding = False




        concentration_check += 1

        print('Thermistor temperature: ' + str(temp) + ' & pid result: ' + str(control))
        sample_last_ms = utime.ticks_ms()


