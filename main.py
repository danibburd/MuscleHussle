import utime
from machine import Pin, PWM, ADC, DAC
from read_temp import init_temp_sensor, read_temp
from pid import PID

#initialize PID
pid = PID(1.0, 1.0, 0.2, setpoint=17)
pid.tunings = (-1.0, -0.1, 0)
pid.output_limits = (0, 10)

#initialize Input/Output devices
temp_sens = init_temp_sensor()

# Initialize PWM output for motor 1
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

#Pin to which the LED sensor would be connected
adc = ADC(Pin(33))
adc.atten(ADC.ATTN_11DB)

#Pin to which the LED ligh is connected
pwm = PWM(Pin(15))
pwm.freq(70000)
pwm.duty(8)


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
    if(error >= 2 ):
        if(cooling_volts.value() == 0):
            change_voltage(True)
        error = (error - 2) * 42
        freq = error + 250
        if (freq > 666):
            freq = 666
        change_flowrate(freq)
        if(cooling_volts.value() == 1):
            change_voltage(False)
        print('freq: ' + str(freq) )
    else:
        if(step_pin2.freq != 0):
            change_flowrate(250)
        if(cooling_volts.value() == 0):
            change_voltage(True)
        print('no change')

#OD sensor control
#TODO: move to a seperate document
currentSamples = []
recentAvg = 0

def AvgSamples(samples):
    return (sum(samples) / len(samples))

def inputSample(sample):
    currentSamples.append(sample)

    if (len(currentSamples) < 10):
        return None
    elif ( len(currentSamples) == 10 ):
        avg = AvgSamples(currentSamples)
        currentSamples = []    
        return avg
    else:
        print("oop")
        currentSamples = []   
        return None

while (True):
    if utime.ticks_diff(utime.ticks_ms(), sample_last_ms) >= SAMPLE_INTERVAL:
        temp = read_temp(temp_sens)

        control = pid(temp)
        get_change(control)
        adcRead = adc.read()
        newAvg = inputSample(adcRead)
        if (newAvg):
            recentAvg = newAvg
        print("most recent average: " + str(recentAvg))


        # update_pump(temp)

        print('Thermistor temperature: ' + str(temp) + ' & pid result: ' + str(control))
        sample_last_ms = utime.ticks_ms()