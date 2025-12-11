import machine
import neopixel
import time

# --- 1. CONFIGURATION ---
# Pin definitions
PIN_FRONT = 21
PIN_LEFT = 20  #Left Side
PIN_RIGHT = 19  #Right Side

# LED Counts
CNT_FRONT = 5
CNT_SIDE  = 7

# Brightness (0.0 to 1.0)
BRIGHTNESS = 1.0

# --- 2. HARDWARE INITIALIZATION ---
def init_neopixel(pin_num, count):
    """
    Initializes a NeoPixel strip with the Pico's internal PULL_DOWN resistor enabled.
    """
    # 1. Configure Pin: Output, Pull-Down enabled, Initial value 0
    p = machine.Pin(pin_num, machine.Pin.OUT, machine.Pin.PULL_DOWN, value=0)
    
    # 2. Wait 10ms for voltage to settle at 0V
    time.sleep_ms(10)
    
    # 3. Initialize NeoPixel object
    np = neopixel.NeoPixel(p, count)
    
    # 4. Immediately wipe the strip (Turn OFF) to clear any random startup noise
    for i in range(count):
        np[i] = (0, 0, 0)
    np.write()
    
    return np

# Create the objects using the initializer
np_front = init_neopixel(PIN_FRONT, CNT_FRONT)
np_right = init_neopixel(PIN_RIGHT, CNT_SIDE)
np_left  = init_neopixel(PIN_LEFT, CNT_SIDE)

def apply_brightness(color_tup):
    """Simple helper to dim the colors globally"""
    r, g, b = color_tup
    return (int(r * BRIGHTNESS), int(g * BRIGHTNESS), int(b * BRIGHTNESS))

# --- 3. LOGIC CLASS ---
class VehicleLights:
    def __init__(self):
        # --- Colors ---
        self.OFF = (0, 0, 0)
        self.WHITE = (255, 255, 255)
        self.RED = (255, 0, 0)
        self.YELLOW = (255, 150, 0)
        self.BLUE = (0, 0, 255)
        
        # Error Colors
        self.BNO_FAIL = (153, 203, 255) # Light Blue
        self.IBUS_FAIL = (69, 75, 27)   # Murky Green

        # Error Lists
        self.ERR_FRONT_STRIP = [self.RED] * CNT_FRONT
        self.ERR_SIDE_BNO    = [self.BNO_FAIL] * CNT_SIDE
        self.ERR_SIDE_IBUS   = [self.IBUS_FAIL] * CNT_SIDE

        # State Variables
        self.kitt_pos = 0
        self.kitt_dir = 1
        self.kitt_last_update = 0
        self.kitt_delay = 100

        self.police_state = False
        self.police_last_update = 0
        self.police_delay = 150

        # Buffers
        self.front_leds = [self.OFF] * CNT_FRONT
        self.left_leds  = [self.OFF] * CNT_SIDE
        self.right_leds = [self.OFF] * CNT_SIDE
        
        # Double check: clear hardware on class init
        self._clear_hardware()

    def _clear_hardware(self):
        for i in range(CNT_FRONT): np_front[i] = (0,0,0)
        for i in range(CNT_SIDE): np_right[i] = (0,0,0)
        for i in range(CNT_SIDE): np_left[i] = (0,0,0)
        np_front.write()
        np_right.write()
        np_left.write()

    def _run_kitt_logic(self):
        """Calculates KITT position"""
        now = time.ticks_ms()
        if time.ticks_diff(now, self.kitt_last_update) > self.kitt_delay:
            self.kitt_last_update = now
            
            self.front_leds = [self.OFF] * CNT_FRONT
            self.front_leds[self.kitt_pos] = self.RED

            self.kitt_pos += self.kitt_dir
            if self.kitt_pos >= (CNT_FRONT - 1):
                self.kitt_pos = CNT_FRONT - 1
                self.kitt_dir = -1
            elif self.kitt_pos <= 0:
                self.kitt_pos = 0
                self.kitt_dir = 1

    def _run_police_logic(self):
        """Alternates Red/Blue"""
        now = time.ticks_ms()
        if time.ticks_diff(now, self.police_last_update) > self.police_delay:
            self.police_last_update = now
            self.police_state = not self.police_state

            if self.police_state:
                # Red Left, Blue Right
                self.front_leds = [self.RED, self.RED, self.OFF, self.BLUE, self.BLUE]
            else:
                # Blue Left, Red Right
                self.front_leds = [self.BLUE, self.BLUE, self.OFF, self.RED, self.RED]

    def _write_to_hardware(self):
        """Copies the buffer lists to the actual NeoPixel objects"""
        for i in range(CNT_FRONT): np_front[i] = apply_brightness(self.front_leds[i])
        np_front.write()

        for i in range(CNT_SIDE): np_right[i] = apply_brightness(self.right_leds[i])
        np_right.write()

        for i in range(CNT_SIDE): np_left[i] = apply_brightness(self.left_leds[i])
        np_left.write()
        
    def set_strip_manual(self, index, color_list):
        """
        Manually writes a color list to a specific strip (Used for Error Modes in main.py).
        index 0 = Front, 1 = Right, 2 = Left
        """
        target_strip = None
        if index == 0: target_strip = np_front
        elif index == 1: target_strip = np_right
        elif index == 2: target_strip = np_left
        
        if target_strip:
            for i in range(min(len(color_list), target_strip.n)):
                target_strip[i] = apply_brightness(color_list[i])
            target_strip.write()

    def update(self, steering_pwm, light_pwm):
        # --- INPUT SANITIZATION  ---
        # If inputs are None or 0 (disconnected), force them to safe values.
        # This prevents "0" from being interpreted as a hard left turn.
        
        if steering_pwm is None or steering_pwm < 500:
            steering_pwm = 1500 # Center
            
        if light_pwm is None or light_pwm < 500:
            light_pwm = 2000 # Off

        # --- 1. SIDE LIGHTS LOGIC ---
        self.left_leds  = [self.OFF] * CNT_SIDE
        self.right_leds = [self.OFF] * CNT_SIDE

        if steering_pwm < 1400:
            # Turning Left -> Right Strip Yellow
            self.right_leds = [self.YELLOW] * CNT_SIDE
            
        elif steering_pwm > 1600:
            # Turning Right -> Left Strip Yellow
            self.left_leds = [self.YELLOW] * CNT_SIDE

        # --- 2. FRONT LIGHTS LOGIC ---
        if light_pwm < 1000:
            # Solid White
            self.front_leds = [self.WHITE] * CNT_FRONT
            
        elif light_pwm < 1500:
            # KITT Effect
            self._run_kitt_logic()
            
        elif light_pwm < 1900:
            # Police Effect
            self._run_police_logic()
            
        else:
            # OFF (Ensure we explicitly write OFF to prevent stuck pixels)
            self.front_leds = [self.OFF] * CNT_FRONT

        # --- 3. SEND TO STRIPS ---
        self._write_to_hardware()
