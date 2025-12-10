import struct
import time
import gc
from machine import UART, Pin

# -------------------------
# TELEMETRY (UART0)
# -------------------------
TELEMETRY_TX_PIN = 16
TELEMETRY_RX_PIN = 17
SENSOR_ADDR = 1
SENSOR_TYPE = 0x08

# -------------------------
# IBUS INPUT (UART1)
# -------------------------
IBUS_RX_PIN = 5       
IBUS_CHANNELS = 16    

uart_ibus = UART(1, baudrate=115200, rx=Pin(IBUS_RX_PIN))

# --- MEMORY OPTIMIZATION 1: Pre-allocate the data storage ---
# We created this ONCE. We will never create a new list again.
ibus_channels = [1500] * IBUS_CHANNELS
ibus_channels[2] = 988 # Default switch low

# --- MEMORY OPTIMIZATION 2: Pre-allocate a reusable buffer ---
# We read data into this bucket instead of buying a new bucket every time.
ibus_buffer = bytearray(32)

# -------------------------
# Parse iBus packets (Optimized)
# -------------------------
def read_ibus_channels(uart):
    """
    Reads IBUS with zero memory allocation.
    Updates the global 'ibus_channels' list directly.
    """
    if uart.any() < 32:
        return False # No new data

    # 1. Peek/Read the header (Safe way to sync)
    # We read 1 byte into the start of our buffer
    n = uart.readinto(ibus_buffer, 1) 
    
    if n is None or ibus_buffer[0] != 0x20:
        return False

    # 2. Read the rest (31 bytes) into the buffer at index 1
    # We use a memoryview so we don't create a copy
    n_rest = uart.readinto(memoryview(ibus_buffer)[1:], 31)

    if n_rest != 31:
        return False

    # 3. Verify Command (0x40)
    if ibus_buffer[1] != 0x40:
        return False

    # 4. Checksum (Optional but good)
    # (Skipping strictly for speed, but you can add it back if you see glitches)

    # 5. Parse Channels (Zero Allocation)
    try:
        # Unpack directly from the buffer into a tuple
        # struct.unpack_from avoids creating a slice copy
        raw_vals = struct.unpack_from('<14H', ibus_buffer, 2)
        
        # Update the GLOBAL list in-place.
        for i in range(14):
            ibus_channels[i] = raw_vals[i]
            
        return True
            
    except Exception:
        return False

# -------------------------
# Telemetry task
# -------------------------
def telemetry_task(shared_data):
    uart = UART(0, baudrate=115200, tx=Pin(TELEMETRY_TX_PIN), rx=Pin(TELEMETRY_RX_PIN))

    # Pre-allocate reuse variables
    last_gc_time = time.ticks_ms()
    
    # Pre-allocate buffer for telemetry reading
    tele_cmd_buf = bytearray(3) 

    def checksum(data):
        return (0xFFFF - sum(data)) & 0xFFFF

    def send_packet(cmd, payload):
        # Construct packet
        l = len(payload)
        pkt = bytearray([l + 4, cmd]) + bytearray(payload)
        chk = checksum(pkt)
        pkt += struct.pack("<H", chk)
        uart.write(pkt)

    print("Telemetry task started (Optimized)")
    
    # Force clean start
    gc.collect()

    while True:
        # --------------------------------------
        # 1. READ IBUS INPUT (Zero Allocation)
        # --------------------------------------
        is_fresh = read_ibus_channels(uart_ibus)
        
        if is_fresh:
            shared_data["last_ibus"] = time.ticks_ms()

        # --------------------------------------
        # 2. HANDLE TELEMETRY REQUESTS
        # --------------------------------------
        try:
            if uart.any():
                # Read 1 byte to check for start
                header = uart.read(1)
                
                if header == b'\x04':
                    # Wait slightly for the rest of the command
                    start_wait = time.ticks_ms()
                    while uart.any() < 3:
                        if time.ticks_diff(time.ticks_ms(), start_wait) > 10:
                            break
                    
                    if uart.any() >= 3:
                        uart.readinto(tele_cmd_buf, 3) # Read into pre-allocated buffer
                        
                        cmd = tele_cmd_buf[0]
                        addr = cmd & 0x0F
                        ctype = cmd & 0xF0

                        if addr == SENSOR_ADDR:
                            if ctype == 0x80:
                                send_packet(cmd, [])
                            elif ctype == 0x90:
                                send_packet(cmd, bytes([SENSOR_TYPE, 2]))
                            elif ctype == 0xA0:
                                val = shared_data.get("heading", 0)
                                if val < 0: val = 0
                                if val > 360: val = 360
                                send_packet(cmd, struct.pack("<H", val))
                                
        except Exception as e:
            print("Telem Err:", e)

        # --------------------------------------
        # 3. GARBAGE COLLECTION (Safety)
        # --------------------------------------
        # Every 4 seconds, manually clean RAM on this core.
        # This prevents the "slow death" of the thread.
        now = time.ticks_ms()
        if time.ticks_diff(now, last_gc_time) > 4000:
            gc.collect()
            last_gc_time = now

        # Tiny sleep to let the CPU breathe
        time.sleep_ms(1)
