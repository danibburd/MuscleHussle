import utime
from machine import Pin
from machine import ADC
from machine import DAC
from machine import PWM
from read_temp import init_temp_sensor, read_temp

#initialize PID
pid = PID(1.0, 0.1, 0.05, setpoint=17)
pid.tunings = (-1.0, -0.1, 0)
pid.output_limits = (0, 150)

#initialize Input/Output devices
temp_sens = init_temp_sensor()

##### Initialize PWM output for motor 1
step_pin1 = PWM(Pin(16, Pin.OUT))
dir_pin1 = Pin(17, Pin.OUT)
# Initialize PWM output for motor 2
step_pin2 = PWM(Pin(18, Pin.OUT))
dir_pin2 = Pin(19, Pin.OUT)

#sets the cooling
cooling_volts = Pin(4, Pin.OUT)
cooling_volts.value(0)

# Set motor direction clockwise
dir_pin1.value(1)
dir_pin2.value(0)

# Set PWM frequency
step_pin1.freq(500)
step_pin2.freq(666)
# Set motor speed
step_pin1.duty(10)
step_pin2.duty(10)


sample_last_ms = 0
SAMPLE_INTERVAL = 10000
pid.sample_time = SAMPLE_INTERVAL/1000

#pump control for PID
#TODO: move this to seperate document
def change_flowrate(new_flow):
    step_pin2.freq(int(new_flow))

def change_voltage(On5):
    if (On5):
        cooling_volts.value(1)
    else:
        cooling_volts.value(0)

def get_change(error):
    if(error >= 5 ):
        if(cooling_volts.value() == 0):
            change_voltage(True)
        error = (error - 5) * 2
        freq = error + 250
        if (freq > 666):
            freq = 666
        change_flowrate(freq)
        print('freq: ' + str(freq) )
    else:
        if(step_pin2.freq != 0):
            change_flowrate(10)
        if(cooling_volts.value() == 1):
            change_voltage(False)
        print('no change')


while (True):
    if utime.ticks_diff(utime.ticks_ms(), sample_last_ms) >= SAMPLE_INTERVAL:
        temp = read_temp(temp_sens)

        control = pid(temp)
        get_change(control)
        # update_pump(temp)

        print('Thermistor temperature: ' + str(temp) + ' & pid result: ' + str(control))
        sample_last_ms = utime.ticks_ms()