# OpenMV4 H7 Plus — Color Blob Tracking for 2-DOF Gimbal
# Sends target position to STM32F103C8T6 via UART3 (5-byte protocol).
# 30-60 fps with LAB color threshold tracking.

import sensor, image, time
from pyb import UART, LED

# ---- Configuration ----
SENSOR_FRAMESIZE  = sensor.QVGA       # 320x240
SENSOR_PIXFORMAT  = sensor.RGB565
SENSOR_HMIRROR    = True
SENSOR_VFLIP      = True

# LAB color threshold: (L_min, L_max, A_min, A_max, B_min, B_max)
COLOR_THRESHOLDS = [(0, 100, -128, -15, 0, 127)]

BLOB_AREA_THRESHOLD = 100
BLOB_MERGE          = True

UART_BAUDRATE = 115200
UART_CHANNEL  = 3

# Protocol: [0xFF][0xFE][hasBlob][tx][ty]
#   hasBlob: 0x01=detected, 0x00=lost
#   tx/ty: normalized 0-255, 128=center
HEADER1 = 0xFF
HEADER2 = 0xFE
IMAGE_W = 320
IMAGE_H = 240

uart = None
frame_count = 0
detect_count = 0
last_detect_time = 0

def init_camera():
    sensor.reset()
    sensor.set_framesize(SENSOR_FRAMESIZE)
    sensor.set_pixformat(SENSOR_PIXFORMAT)
    sensor.set_hmirror(SENSOR_HMIRROR)
    sensor.set_vflip(SENSOR_VFLIP)
    sensor.skip_frames(time=1000)
    sensor.set_auto_gain(False)
    sensor.set_auto_exposure(False, exposure_us=20000)
    return True

def init_uart():
    global uart
    try:
        uart = UART(UART_CHANNEL, UART_BAUDRATE, bits=8, stop=1, parity=None)
        print("UART OK, baud:", UART_BAUDRATE)
        return True
    except Exception as e:
        print("UART init failed:", e)
        return False

def send_blob_position(cx, cy, has_blob):
    if uart is None:
        return
    hasBlob = 1 if has_blob else 0
    tx = max(0, min(255, int(round((cx / IMAGE_W) * 255))))
    ty = max(0, min(255, int(round((cy / IMAGE_H) * 255))))
    data = bytes([HEADER1, HEADER2, hasBlob, tx, ty])
    for attempt in range(3):
        try:
            uart.write(data)
            break
        except Exception:
            if attempt == 2:
                print("UART write failed after 3 retries")

def detect_blob():
    global frame_count, detect_count
    img = sensor.snapshot()
    frame_count += 1
    blobs = img.find_blobs(COLOR_THRESHOLDS, area_threshold=BLOB_AREA_THRESHOLD, merge=BLOB_MERGE)
    if blobs:
        largest = max(blobs, key=lambda b: b.area())
        cx, cy = int(largest.cx()), int(largest.cy())
        img.draw_rectangle(largest.rect(), color=(0, 255, 0))
        img.draw_cross(cx, cy, color=(0, 255, 0))
        detect_count += 1
        return cx, cy, True
    else:
        return IMAGE_W // 2, IMAGE_H // 2, False

def set_led_status(status):
    LED(1).off(); LED(2).off(); LED(3).off()
    if status == 1:   LED(1).on()   # red
    elif status == 2: LED(2).on()   # green
    elif status == 3: LED(3).on()   # blue

# ---- Main ----
if __name__ == "__main__":
    print("=" * 50)
    print("OpenMV4 H7 Plus — Color Blob Tracking v4.0")
    print("=" * 50)
    set_led_status(1)
    time.sleep_ms(500)

    print("Init camera...")
    if not init_camera():
        print("Camera init failed!")
        while True:
            set_led_status(1); time.sleep_ms(500)

    print("Init UART...")
    if not init_uart():
        print("UART init failed!")
        while True:
            set_led_status(1); time.sleep_ms(500)

    set_led_status(2); time.sleep_ms(1000)
    print("Tracking active — LAB thresholds:", COLOR_THRESHOLDS)
    print("30-60 fps, 5-byte protocol to UART3")

    last_detect_time = time.time()
    while True:
        cx, cy, has_blob = detect_blob()
        send_blob_position(cx, cy, has_blob)
        set_led_status(2 if has_blob else 3)

        now = time.time()
        if now - last_detect_time >= 5.0:
            fps = frame_count / 5.0 if frame_count > 0 else 0
            rate = (detect_count / frame_count * 100) if frame_count > 0 else 0
            print("5s stats: frames=%d detected=%d rate=%.1f%% fps=%.1f" % (frame_count, detect_count, rate, fps))
            frame_count = 0
            detect_count = 0
            last_detect_time = now
