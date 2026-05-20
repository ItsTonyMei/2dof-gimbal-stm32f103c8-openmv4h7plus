# OpenMV4 H7 Plus — 颜色追踪固件 (Color Tracking Firmware, 旧硬件兼容 / Legacy)
# 通过 UART3 发送目标位置 (Target Position) 到 STM32F103C8T6（5 字节协议 / 5-Byte Protocol）。
# LAB 颜色阈值追踪 (Color Threshold Tracking), 30-60 fps。
# 注意: 当前主控已迁移到 N6 单板方案 (main.py), 此文件仅供 H7 Plus 使用。

import sensor, image, time
from pyb import UART, LED

# ---- 配置参数 (Configuration) ----
SENSOR_FRAMESIZE  = sensor.QVGA       # 320x240
SENSOR_PIXFORMAT  = sensor.RGB565
SENSOR_HMIRROR    = True              # 水平镜像 (Horizontal Mirror)
SENSOR_VFLIP      = True              # 垂直翻转 (Vertical Flip)

# LAB 颜色阈值 (Color Threshold): (L_min, L_max, A_min, A_max, B_min, B_max)
COLOR_THRESHOLDS = [(0, 100, -128, -15, 0, 127)]

BLOB_AREA_THRESHOLD = 100   # 色块面积阈值 (Blob Area Threshold)
BLOB_MERGE          = True  # 色块合并 (Blob Merge)

UART_BAUDRATE = 115200      # 波特率 (Baud Rate)
UART_CHANNEL  = 3

# 协议 (Protocol): [0xFF][0xFE][hasBlob][tx][ty]
#   hasBlob: 0x01=检测到目标 (Detected), 0x00=目标丢失 (Lost)
#   tx/ty:   归一化坐标 (Normalized Coordinates) 0-255, 128=中心 (Center)
HEADER1 = 0xFF
HEADER2 = 0xFE
IMAGE_W = 320
IMAGE_H = 240

uart = None
frame_count = 0
detect_count = 0
last_detect_time = 0

def init_camera():
    """初始化摄像头传感器 (Init Camera Sensor)"""
    sensor.reset()
    sensor.set_framesize(SENSOR_FRAMESIZE)
    sensor.set_pixformat(SENSOR_PIXFORMAT)
    sensor.set_hmirror(SENSOR_HMIRROR)
    sensor.set_vflip(SENSOR_VFLIP)
    sensor.skip_frames(time=1000)
    sensor.set_auto_gain(False)                         # 手动增益 (Manual Gain)
    sensor.set_auto_exposure(False, exposure_us=20000)  # 手动曝光 (Manual Exposure)
    return True

def init_uart():
    """初始化串口通信 (Init UART)"""
    global uart
    try:
        uart = UART(UART_CHANNEL, UART_BAUDRATE, bits=8, stop=1, parity=None)
        print("串口初始化成功 (UART Init OK), 波特率:", UART_BAUDRATE)
        return True
    except Exception as e:
        print("串口初始化失败 (UART Init Failed):", e)
        return False

def send_blob_position(cx, cy, has_blob):
    """发送 5 字节追踪数据帧 (Tracking Data Frame) 到 STM32"""
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
                print("串口发送失败 (UART Send Failed, 已重试 3 次)")

def detect_blob():
    """颜色色块检测 (Color Blob Detection), 返回 (cx, cy, has_blob)"""
    global frame_count, detect_count
    img = sensor.snapshot()
    frame_count += 1
    blobs = img.find_blobs(COLOR_THRESHOLDS, area_threshold=BLOB_AREA_THRESHOLD, merge=BLOB_MERGE)
    if blobs:
        largest = max(blobs, key=lambda b: b.area())  # 最大色块 (Largest Blob)
        cx, cy = int(largest.cx()), int(largest.cy())
        img.draw_rectangle(largest.rect(), color=(0, 255, 0))
        img.draw_cross(cx, cy, color=(0, 255, 0))
        detect_count += 1
        return cx, cy, True
    else:
        return IMAGE_W // 2, IMAGE_H // 2, False

def set_led_status(status):
    """设置 LED 状态 (LED Status): 0=关闭, 1=红 (Red), 2=绿 (Green), 3=蓝 (Blue)"""
    LED(1).off(); LED(2).off(); LED(3).off()
    if status == 1:   LED(1).on()
    elif status == 2: LED(2).on()
    elif status == 3: LED(3).on()

# ---- 主程序 (Main) ----
if __name__ == "__main__":
    print("=" * 50)
    print("OpenMV4 H7 Plus — 颜色追踪固件 (Color Tracking) v4.0")
    print("=" * 50)
    set_led_status(1)
    time.sleep_ms(500)

    print("正在初始化摄像头 (Init Camera)...")
    if not init_camera():
        print("摄像头初始化失败 (Camera Init Failed)!")
        while True:
            set_led_status(1); time.sleep_ms(500)

    print("正在初始化串口 (Init UART)...")
    if not init_uart():
        print("串口初始化失败 (UART Init Failed)!")
        while True:
            set_led_status(1); time.sleep_ms(500)

    set_led_status(2); time.sleep_ms(1000)
    print("追踪已启动 (Tracking Started) — LAB 阈值:", COLOR_THRESHOLDS)
    print("30-60 fps, 5 字节协议 (5-Byte Protocol) → UART3")

    last_detect_time = time.time()
    while True:
        cx, cy, has_blob = detect_blob()
        send_blob_position(cx, cy, has_blob)
        set_led_status(2 if has_blob else 3)

        now = time.time()
        if now - last_detect_time >= 5.0:
            fps = frame_count / 5.0 if frame_count > 0 else 0
            rate = (detect_count / frame_count * 100) if frame_count > 0 else 0
            print("5秒统计 (5s Stats): 总帧 (Frames)=%d 检测 (Detected)=%d 检测率 (Rate)=%.1f%% fps=%.1f" % (frame_count, detect_count, rate, fps))
            frame_count = 0
            detect_count = 0
            last_detect_time = now
