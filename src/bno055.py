import time
import struct

class BNO055:
    def __init__(self, i2c, address=0x29):
        self.i2c = i2c
        self.address = address
        self.connected = False
        self.last_init_attempt = 0
        
        # Try to init immediately on startup
        self.init_sensor()

    # ------------------------------
    #          LOW LEVEL I2C
    # ------------------------------
    def _write_register(self, reg, value):
        try:
            self.i2c.writeto_mem(self.address, reg, bytes([value]))
        except OSError:
            raise OSError("WRITE FAIL")

    def _read_register(self, reg, length):
        try:
            return self.i2c.readfrom_mem(self.address, reg, length)
        except OSError:
            raise OSError("READ FAIL")

    # ------------------------------
    #        SMART INIT
    # ------------------------------
    def init_sensor(self):
        try:
            # Check if already running (Hot Start)
            try:
                chip_id = self._read_register(0x00, 1)[0]
                mode = self._read_register(0x3D, 1)[0]
                if chip_id == 0xA0 and mode == 0x0C:
                    print("BNO055 Hot Start (Skipping Reset)")
                    self.connected = True
                    return True
            except:
                pass

            print("BNO055 Cold Start (Hard Reset)")
            self._write_register(0x3F, 0x20) # Reset
            time.sleep(0.7)
            self._write_register(0x3E, 0x00) # Normal Power
            time.sleep(0.01)
            self._write_register(0x07, 0x00) # Page ID 0
            time.sleep(0.01)
            self._write_register(0x3D, 0x00) # Config Mode
            time.sleep(0.02)
            self._write_register(0x3D, 0x0C) # NDOF Mode
            time.sleep(0.02)

            self.connected = True
            return True

        except Exception as e:
            print("BNO Init Fail:", e)
            self.connected = False
            return False

    def _try_recover(self):
        now = time.ticks_ms()
        if time.ticks_diff(now, self.last_init_attempt) < 500:
            return
        
        print("Attempting BNO Recovery...")
        self.last_init_attempt = now
        try:
             self._write_register(0x3F, 0x20)
             time.sleep(0.7)
             self._write_register(0x3D, 0x0C)
             self.connected = True
        except:
            self.connected = False

    # ------------------------------
    #        READ HEADING
    # ------------------------------
    def read_heading(self):
        if not self.connected:
            self._try_recover()
            return None

        try:
            data = self._read_register(0x1A, 2)
            heading_raw = struct.unpack('<h', data)[0]
            
            # Convert to degrees
            deg = heading_raw / 16.0
            
            # --- FIX: APPLY 90 DEGREE OFFSET ---
            corrected = (deg + 90.0) % 360.0
            
            return int(corrected)

        except Exception:
            self.connected = False
            return None

    # ------------------------------
    #      CHECK CALIBRATION
    # ------------------------------
    def get_calibration_status(self):
        if not self.connected: return (0,0,0,0)
        try:
            data = self._read_register(0x35, 1)
            val = data[0]
            sys = (val >> 6) & 0x03
            gyro = (val >> 4) & 0x03
            accel = (val >> 2) & 0x03
            mag = val & 0x03
            return (sys, gyro, accel, mag)
        except: return (0,0,0,0)
