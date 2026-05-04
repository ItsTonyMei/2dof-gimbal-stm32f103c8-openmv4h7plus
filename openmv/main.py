#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
OpenMV颜色追踪固件
=================

功能说明:
    通过OpenMV4 H7 Plus摄像头追踪指定颜色的色块,将位置通过串口发送给STM32云台控制器,
    实现云台自动追踪功能。颜色追踪帧率可达30-60fps,远高于YOLO方案。

硬件连接:
    OpenMV4 H7 Plus    STM32F103C8T6
    TX (Pin 4)    <--->  PB11 (USART3_RX)
    GND           <--->  GND

串口参数: 115200 8N1

协议 (5字节):
    [0xFF][0xFE][hasBlob][tx][ty]
    hasBlob: 0x01=检测到目标, 0x00=未检测
    tx: 目标X坐标，归一化 0-255 (0=左边界, 128=中心, 255=右边界)
    ty: 目标Y坐标，归一化 0-255 (0=上边界, 128=中心, 255=下边界)

    QVGA 320x240: 中心像素(c=160,cy=120) → (128, 128)
    与 STM32 control.c 中 OPENMV_CENTER_X=OPENMV_CENTER_Y=128 对齐

版本: V4.0
日期: 2026-05-02

更新:
    V4.0: 切换为颜色追踪模式,帧率30-60fps (替代YOLO LC的~10fps)
    V3.0: 简化协议为5字节（去掉BCC），与STM32侧同步
    V2.0: 使用OpenMV内置 YOLO LC 神经网络进行人形检测 (完整人体)
    V1.2: 使用OpenMV内置/haarcascade_frontalface.cascade进行人脸检测
"""

import sensor, image, time
from pyb import UART, LED
import pyb

# =============================================================================
# 配置参数
# =============================================================================

# 摄像头配置
SENSOR_FRAMESIZE = sensor.QVGA    # 320x240
SENSOR_PIXFORMAT = sensor.RGB565

# 颜色追踪色块阈值 (LAB格式)
# 当前配置: 用户自定义绿色阈值
# ========== 常用参考阈值 (先用此值测试，再微调) ==========
#   绿色参考:  [(30, 50, -20, 20, 20, 70)]
#   蓝色参考:  [(20, 40, 10, 30, -60, -20)]
#   红色参考:  [(60, 80, 30, 70, 10, 50)]
# 如需多颜色追踪, 在列表中加多个元组即可
COLOR_THRESHOLDS = [
    (0, 100, -128, -15, 0, 127),   # 当前: 用户自定义绿色阈值
]

# 色块追踪参数
BLOB_AREA_THRESHOLD = 100         # 最小色块面积(像素),小于此忽略
BLOB_MERGE = True                  # 是否合并相邻色块

# 摄像头flip设置 (根据物理安装调整)
SENSOR_HMIRROR = True
SENSOR_VFLIP = True

# UART 配置
UART_BAUDRATE = 115200
UART_CHANNEL = 3

# 协议配置 (归一化到0-255, 128=中心)
# QVGA(320x240): 中心(160,120) → (160/320*255, 120/240*255) ≈ (128, 128)
HEADER1 = 0xFF
HEADER2 = 0xFE
IMAGE_W = 320    # 归一化基准(像素宽度)
IMAGE_H = 240    # 归一化基准(像素高度)
PIXEL_CENTER_X = IMAGE_W // 2
PIXEL_CENTER_Y = IMAGE_H // 2
PROTOCOL_CENTER = 128  # 归一化坐标中心，与STM32 control.c OPENMV_CENTER_X/Y=128 对齐

# =============================================================================
# 全局变量
# =============================================================================

uart = None
frame_count = 0
detect_count = 0
last_detect_time = 0
last_known_cx = PIXEL_CENTER_X
last_known_cy = PIXEL_CENTER_Y

# 目标丢失时立即回传画面中心且hasBlob=0, STM32端保持当前位置

# =============================================================================
# 函数: 初始化摄像头
# =============================================================================
def init_camera():
    """初始化摄像头传感器"""
    sensor.reset()
    sensor.set_framesize(SENSOR_FRAMESIZE)
    sensor.set_pixformat(SENSOR_PIXFORMAT)
    sensor.set_hmirror(SENSOR_HMIRROR)
    sensor.set_vflip(SENSOR_VFLIP)
    sensor.skip_frames(time=1000)
    sensor.set_auto_gain(False)
    sensor.set_auto_exposure(False, exposure_us=20000)
    return True

# =============================================================================
# 函数: 初始化串口
# =============================================================================
def init_uart():
    """初始化串口通信"""
    global uart

    try:
        uart = UART(UART_CHANNEL, UART_BAUDRATE, bits=8, stop=1, parity=None)
        uart.write("OpenMV OK\r\n")
        print("串口初始化成功, 波特率:", UART_BAUDRATE)
        return uart
    except Exception as e:
        print("串口初始化失败:", str(e))
        return None

# =============================================================================
# 函数: 发送追踪数据帧 (5字节协议)
# =============================================================================
def send_blob_position(cx, cy, has_blob=True):
    """发送5字节追踪数据帧到STM32
    格式: [0xFF][0xFE][hasBlob][target_x][target_y]
    """
    global uart

    if uart is None:
        return

    hasBlob = 1 if has_blob else 0

    # 转换为归一化坐标 (0-255)
    tx = int(round((cx / IMAGE_W) * 255))
    ty = int(round((cy / IMAGE_H) * 255))
    tx = max(0, min(255, tx))
    ty = max(0, min(255, ty))

    data = bytes([HEADER1, HEADER2, hasBlob, tx, ty])

    MAX_RETRIES = 3
    for attempt in range(MAX_RETRIES):
        try:
            uart.write(data)
            break
        except Exception as e:
            if attempt == MAX_RETRIES - 1:
                print("串口发送失败 (已重试{}次):".format(MAX_RETRIES), str(e))

# =============================================================================
# 函数: 色块检测
# =============================================================================
def detect_blob():
    """执行颜色追踪,返回(cx, cy, has_blob)
    使用LAB颜色空间阈值分割,帧率30-60fps
    """
    global frame_count, detect_count
    global last_known_cx, last_known_cy

    img = sensor.snapshot()
    frame_count += 1

    raw_cx = PIXEL_CENTER_X
    raw_cy = PIXEL_CENTER_Y
    has_blob = False

    # 查找色块
    blobs = img.find_blobs(
        COLOR_THRESHOLDS,
        area_threshold=BLOB_AREA_THRESHOLD,
        merge=BLOB_MERGE
    )

    if blobs:
        # 找最大面积的色块
        largest = max(blobs, key=lambda b: b.area())
        raw_cx = largest.cx()
        raw_cy = largest.cy()

        # 绘制检测框和十字
        img.draw_rectangle(largest.rect(), color=(0, 255, 0))
        img.draw_cross(raw_cx, raw_cy, color=(0, 255, 0))

        has_blob = True
        detect_count += 1
    else:
        # 色块丢失: 立即回中并发送hasBlob=0, 由STM32保持当前位置
        last_known_cx = PIXEL_CENTER_X
        last_known_cy = PIXEL_CENTER_Y

    # 更新 last-known-position
    if has_blob:
        last_known_cx = raw_cx
        last_known_cy = raw_cy

    cx = int(round(last_known_cx))
    cy = int(round(last_known_cy))
    return cx, cy, has_blob

# =============================================================================
# 函数: LED状态指示
# =============================================================================
def set_led_status(status):
    """设置LED状态: 0=关闭, 1=红, 2=绿, 3=蓝"""
    LED(1).off()
    LED(2).off()
    LED(3).off()
    if status == 1:
        LED(1).on()
    elif status == 2:
        LED(2).on()
    elif status == 3:
        LED(3).on()

# =============================================================================
# 函数: 主循环
# =============================================================================
def main_loop():
    """主循环 - 持续追踪色块并发送数据"""
    global last_detect_time, detect_count, frame_count

    print("进入主循环...")

    while True:
        cx, cy, has_blob = detect_blob()
        send_blob_position(cx, cy, has_blob)

        if has_blob:
            set_led_status(2)   # 绿色 = 检测到目标
        else:
            set_led_status(3)   # 蓝色 = 正常运行(未检测)

        current_time = time.time()
        if current_time - last_detect_time >= 5.0:
            fps = frame_count / 5.0 if frame_count > 0 else 0
            detection_rate = (detect_count / frame_count * 100) if frame_count > 0 else 0
            print("5秒统计: 帧数=%d, 检测成功=%d, 检测率=%.1f%%, FPS=%.1f" %
                  (frame_count, detect_count, detection_rate, fps))
            print("  最新数据: cx=%d, cy=%d, has_blob=%s" %
                  (cx, cy, has_blob))
            frame_count = 0
            detect_count = 0
            last_detect_time = current_time

        # 无需额外延时,循环全速运行(30-60fps)

# =============================================================================
# 主程序入口
# =============================================================================
if __name__ == "__main__":
    print("=" * 50)
    print("OpenMV4 H7 Plus 颜色追踪固件")
    print("版本: V4.0 (颜色追踪模式), 日期: 2026-05-02")
    print("=" * 50)

    set_led_status(1)
    time.sleep_ms(500)

    print("正在初始化摄像头...")
    if not init_camera():
        print("摄像头初始化失败!")
        while True:
            set_led_status(1)
            time.sleep_ms(500)
    print("摄像头初始化成功")

    print("正在初始化串口...")
    if not init_uart():
        print("串口初始化失败!")
        while True:
            set_led_status(1)
            time.sleep_ms(500)

    set_led_status(2)
    time.sleep_ms(1000)

    print("检测模式: 颜色追踪 (LAB阈值)")
    print("颜色阈值:", COLOR_THRESHOLDS)
    print("提示: 将目标对准摄像头,系统将自动追踪最大色块")
    print("帧率: 30-60fps (无需推理,纯视觉算法)")
    print("进入主循环...")

    main_loop()
