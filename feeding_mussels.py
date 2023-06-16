#libraries
import machine
import time
import tcs34725
from machine import I2C, Pin, PWM


step_pin1 = PWM(Pin(17, Pin.OUT))
#dir_pin1 = Pin(17, Pin.OUT)


#Constants
STEPPER_MOTOR_FLOW = 1.408 # ml/s

MEASUREMENT_TIME = 3.7

TUBE_TIME = 2.2

BACK_TIME = 300  

#RGB sensor
def color_rgb_bytes_new(color_raw):
    r, g, b, clear = color_raw

    return [int(r), int(g), int(b)]


def measure(rgb_sensor):
    global r, g, b
    
    r.off()
    g.off()
    b.off()

    values_list = [0,0,0]

    for i in range(10):
        temp = color_rgb_bytes_new(rgb_sensor.read(True))
        for j in range(3):
            values_list[j] = values_list[j]+temp[j]

    for k in range(3):
            values_list[k] = int(values_list[k] / 10)

    return values_list





#Math model
def feed_amount(measured_Ca):

    #Questionable choices
    dt = 1/3; #time step in min
    N_m = 1; #How many mussels?
    t_max = 36 #Food every 12 min

    #Values from our data
    mu_m = 0.012/60; #Growth constant of Algae in mussel tank [min^-1]
    emu_m = 2.718**(mu_m*dt); #Another constant
    max_Cm = 7000; #Concentration in mussel tank cell/mL
    Vm = 2*1000; #Start volume in mussel tank [mL]

    #Initialize stuff
    Cm = [] #Simulated concentration in Mussel tank
    Cm.append(max_Cm)

    #Explicit Euler
    for t in range(int(t_max/dt)):
        Fm = (-0.00003163 * Cm[t]**2 + 0.5624*Cm[t] + 498.3370)/60#Feeding rate of a mussel in mL/min
        Cm.append(Cm[t]*(-Fm/Vm*dt*N_m+emu_m)) #Change in mussel tank
    Vr = Vm*(max_Cm-Cm[-1])/(measured_Ca-max_Cm) #Finding volume of bolus needed

    return Vr



#Compute concentration
def compute_concentration(rgb_sensor):
    list_of_values = measure(rgb_sensor)  

    RGB_sum = sum(list_of_values)

    concentration = -0.2079* RGB_sum + 19230
    print(concentration)

    return concentration #cells/ml





def perform_food_cycle():
    
    
    global feeding_time_hist, concentration_hist, time_feeding_hist

    # Setting the motor and starting for a few seconds
    # to get an accurate concentration
    #dir_pin1.value(0)
    step_pin1.freq(500)
    
    time.sleep(MEASUREMENT_TIME)
    step_pin1.freq(1)

    # Measuring the concentration and computing time to pump
    concentration = _compute_concentration()
    T = feed_amount(concentration) / STEPPER_MOTOR_FLOW


    
    step_pin1.freq(500)
    time.sleep(TUBE_TIME)
    step_pin1.freq(1)
    time.sleep(1)

   # Actually pumping stuff over
    step_pin1.freq(500)
    time.sleep(T)
    step_pin1.freq(1)


    time.sleep(BACK_TIME)
    #dir_pin1.value(1)
    step_pin1.freq(500)
    time.sleep(T + TUBE_TIME + MEASUREMENT_TIME)
    step_pin1.freq(1)
   
   
def manager_food_runner():
    print("Started food manager thread!")

    while True:
        _perform_food_cycle()
        time.sleep(414.1)
        
   
   
print(_manager_food_runner())