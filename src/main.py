import machine
import _thread
import time
import gc
from machine import I2C, Pin, PWM
from bno055 import BNO055
import telemetry
import lights

# --- Configuration ---
P_GAIN = 6.0 
I_GAIN = 0.05
I_LIMIT = 150 
D_GAIN = 0.0

# --- Pins ---
BNO_SDA = 26
BNO_SCL = 27
servo_out = PWM(Pin(18))
servo_out.freq(50)

# Init I2C
i2c = I2C(1, scl=Pin(BNO_SCL), sda=Pin(BNO_SDA), freq=100000)

# Init sensor
print("Initializing IMU...")
imu = BNO055(i2c, address=0x29)

# Shared memory
data_store = { "heading": 0, "last_ibus": time.ticks_ms() }

# Init lights
car_lights = lights.VehicleLights()
blink_state = False
last_blink_time = 0
BLINK_SPEED_MS = 300

# Start telemetry
print("Starting Telemetry Core...")
_thread.start_new_thread(telemetry.telemetry_task, (data_store,))

# Variables
target_heading = None
prev_error = 0.0
integral_error = 0.0
last_time = time.ticks_ms()

# --- INITIALIZE HEADING VARIABLE ---
h = 0 

# Helpers
def get_shortest_turn(target, current):
    if current is None or target is None: return 0
    try:
        error = target - current
        if error > 180: error -= 360
        elif error < -180: error += 360
        return error        
    except: return 0

# Watchdog
IBUS_TIMEOUT_MS = 500
last_ibus_check_time = 0 
ibus_healthy = True 

# GarbageCollector (GC)
gc.enable()
last_gc_time = time.ticks_ms()
GC_INTERVAL_MS = 5000 
last_debug_print = 0

# Main loop
while True:
    current_time = time.ticks_ms()
    
    # ---------------------------------------------------------
    # 0. Time Calculation
    # ---------------------------------------------------------
    dt = time.ticks_diff(current_time, last_time) / 1000.0 
    if dt <= 0: dt = 0.001 
    last_time = current_time

    # ---------------------------------------------------------
    # 1. SENSOR READING (With Last-Value Memory)
    # ---------------------------------------------------------
    new_reading = imu.read_heading()
    
    if new_reading is not None:
        bno_healthy = True
        h = new_reading # Update h to the new accurate value
        data_store["heading"] = h 
    else:
        bno_healthy = False
        # We do NOT update h. 
        # h stays at the last known good value.
        # This allows the variable to persist while lights warn of error.

    # ---------------------------------------------------------
    # 2. IBUS READING
    # ---------------------------------------------------------
    if time.ticks_diff(current_time, last_ibus_check_time) > 20: 
        last_ibus_update = data_store["last_ibus"]
        if time.ticks_diff(current_time, last_ibus_update) < IBUS_TIMEOUT_MS:
            ibus_healthy = True
        else:
            ibus_healthy = False
        last_ibus_check_time = current_time

    ch0 = telemetry.ibus_channels[0] 
    ch1 = telemetry.ibus_channels[1] 
    ch2 = telemetry.ibus_channels[2] 
    ch3 = telemetry.ibus_channels[3] 
    ch4 = telemetry.ibus_channels[4] 

    # ---------------------------------------------------------
    # 3. SERVO LOGIC
    # ---------------------------------------------------------
    output_pwm_us = 1500 
    is_manual_switch = (ch3 == 988)
    is_overriding = (ch0 < 1450 or ch0 > 1550)
    
    if ch4 == 2012: machine.reset()
    
    # NOTE: If BNO disconnects (bno_healthy=False), we enter this block.
    # The code will use 'h' (which is the last known value) to set target_heading.
    # This prevents the car from snapping wildly if it momentarily disconnects.
    if is_manual_switch or is_overriding or (not bno_healthy) or (not ibus_healthy):
        output_pwm_us = ch0
        target_heading = h 
        integral_error = 0.0
        prev_error = 0.0
            
    elif ch3 == 2012 and bno_healthy:
        # "AUTOPILOT"
        if target_heading is None: target_heading = h
        
        error = get_shortest_turn(target_heading, h)
        integral_error += error * dt
        if integral_error > I_LIMIT: integral_error = I_LIMIT
        if integral_error < -I_LIMIT: integral_error = -I_LIMIT

        derivative = (error - prev_error) / dt
        prev_error = error 

        pid_output = (error * P_GAIN) + (integral_error * I_GAIN) + (derivative * D_GAIN)

        if ch1 >= 1500: output_pwm_us = 1500 - pid_output 
        else: output_pwm_us = 1500 + pid_output

        if output_pwm_us > 1900: output_pwm_us = 1900
        if output_pwm_us < 1100: output_pwm_us = 1100

    # ---------------------------------------------------------
    # 4. SERVO OUTPUT
    # ---------------------------------------------------------
    duty = int(output_pwm_us * 3.2768)
    servo_out.duty_u16(duty)

    # ---------------------------------------------------------
    # 5. LIGHTS HANDLING
    # ---------------------------------------------------------
    if bno_healthy and ibus_healthy:
        car_lights.update(output_pwm_us, ch2) 
    else:
        # Even though we have the last value in 'h', we blink the lights
        # so you know the sensor is actually disconnected/recovering.
        lights.write_strip(0, car_lights.ERR_FRONT_STRIP)
        if (not bno_healthy) and ibus_healthy:
            lights.write_strip(1, car_lights.ERR_SIDE_BNO)
            lights.write_strip(2, car_lights.ERR_SIDE_BNO)
        elif bno_healthy and (not ibus_healthy):
            lights.write_strip(1, car_lights.ERR_SIDE_IBUS)
            lights.write_strip(2, car_lights.ERR_SIDE_IBUS)
        else:
            if time.ticks_diff(current_time, last_blink_time) > BLINK_SPEED_MS:
                blink_state = not blink_state
                last_blink_time = current_time
            if blink_state:
                lights.write_strip(1, car_lights.ERR_SIDE_BNO)
                lights.write_strip(2, car_lights.ERR_SIDE_BNO)
            else:
                lights.write_strip(1, car_lights.ERR_SIDE_IBUS)
                lights.write_strip(2, car_lights.ERR_SIDE_IBUS)

    # ---------------------------------------------------------
    # 6. GC
    # ---------------------------------------------------------
    if time.ticks_diff(current_time, last_gc_time) > GC_INTERVAL_MS:
        gc.collect()
        last_gc_time = current_time
