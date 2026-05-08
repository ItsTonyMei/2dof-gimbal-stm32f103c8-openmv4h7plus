# OpenMV N6 — YOLOv8 人形检测追踪固件（二自由度云台）
# 通过 UART3 发送目标位置到 STM32F103C8T6（5 字节协议）。
# YOLOv8n NPU 加速, COCO 预训练, 检测 "person" 类。
#
# ==== 模型烧录步骤 ====
# 1. OpenMV IDE → 连接 N6
# 2. 工具 → ROM文件系统 → 在OpenMV Cam上编辑romfs
# 3. 浏览模型库 → STMicroelectronics → Object Detection → YOLOv8
#    选择下面 PRESET 对应的 .tflite 文件
# 4. 勾选 → 提交烧录（IDE 会自动调用 ST Edge AI Core 编译）
# 5. 确认 /rom/ 下存在该 .tflite 文件

import csi, image, time, ml
from pyb import UART, LED
from ml.postprocessing.ultralytics import YoloV8

# ============================================================
# 配置参数 — 三选一预设（取消注释要用的那组，注释掉其余）
# ============================================================

MODEL_NMS_THRESHOLD = 0.1    # NMS 阈值

# -------- 预设 A: 画质优, ~20-23 fps --------
MODEL_PATH = "/rom/yolov8n_320.tflite"
CAMERA_WINDOW_W = 320
CAMERA_WINDOW_H = 320
MODEL_THRESHOLD = 0.4
MIN_AREA = 500

# -------- 预设 B: 均衡, ~28-33 fps --------
# MODEL_PATH = "/rom/yolov8n_256.tflite"
# CAMERA_WINDOW_W = 256
# CAMERA_WINDOW_H = 256
# MODEL_THRESHOLD = 0.55
# MIN_AREA = 300

# -------- 预设 C: 高速, ~40-48 fps --------
# MODEL_PATH = "/rom/yolov8n_192.tflite"
# CAMERA_WINDOW_W = 192
# CAMERA_WINDOW_H = 192
# MODEL_THRESHOLD = 0.6       # 192 分辨率最低，阈值需更高
# MIN_AREA = 200              # 最小检测面积 (px²)

# --- 摄像头 ---
CAMERA_HMIRROR = True        # 水平镜像（倒装安装）
CAMERA_VFLIP   = True        # 垂直翻转（倒装安装）
EXPOSURE_US    = 20000       # 固定曝光 (us), 室外可降低

# --- 平滑 ---
EMA_ALPHA = 0.3              # EMA 平滑系数 (0~1), 越小越平滑

# --- 上半身追踪 ---
BODY_CY_RATIO = 0.28         # 上半身中心在人体 bbox 顶部往下 % 处

# --- UART ---
UART_BAUDRATE = 115200
UART_CHANNEL  = 3            # N6 仅暴露 UART3: P4(TX) P5(RX)

# --- 协议 ---
# [0xFF][0xFE][hasPerson][tx][ty]  共 5 字节
#   hasPerson: 0x01=检测到人, 0x00=未检测到
#   tx/ty: 归一化坐标 0-255, 128=中心
HEADER1 = 0xFF
HEADER2 = 0xFE

# ============================================================
# 全局状态
# ============================================================
cam = None
model = None
uart = None
frame_count = 0
detect_count = 0
smooth_cx = 0
smooth_cy = 0
smooth_ready = False

# ============================================================
# 初始化
# ============================================================

def init_camera():
    global cam
    cam = csi.CSI()
    cam.reset()
    cam.pixformat(csi.RGB565)
    cam.framesize(csi.VGA)                    # 传感器工作分辨率
    cam.window((CAMERA_WINDOW_W, CAMERA_WINDOW_H))  # 实际输出 ROI
    cam.hmirror(CAMERA_HMIRROR)
    cam.vflip(CAMERA_VFLIP)
    cam.snapshot(time=2000)                   # 跳过启动帧
    cam.auto_gain(False)
    cam.auto_exposure(False, exposure_us=EXPOSURE_US)
    print("摄像头初始化成功 — 窗口 %dx%d" % (CAMERA_WINDOW_W, CAMERA_WINDOW_H))
    return True

def init_model():
    global model
    try:
        model = ml.Model(
            MODEL_PATH,
            postprocess=YoloV8(
                threshold=MODEL_THRESHOLD,
                nms_threshold=MODEL_NMS_THRESHOLD,
            ),
        )
        print("模型加载成功:", MODEL_PATH)
        print("检测类别:", model.labels)
        return True
    except Exception as e:
        print("模型加载失败:", e)
        print("请确认 ROMFS 已烧录模型文件")
        return False

def init_uart():
    global uart
    try:
        uart = UART(UART_CHANNEL, UART_BAUDRATE, bits=8, stop=1, parity=None)
        print("串口初始化成功 — UART%d @ %d" % (UART_CHANNEL, UART_BAUDRATE))
        return True
    except Exception as e:
        print("串口初始化失败:", e)
        return False

# ============================================================
# 核心逻辑
# ============================================================

def detect_person(img):
    """YOLOv8 推理 + 上半身位置估算, 返回 (body_cx, body_cy, has_person)
    从人体 bbox 顶部按 BODY_CY_RATIO 估算上半身中心。
    """
    global frame_count, detect_count
    frame_count += 1

    try:
        boxes = model.predict([img])
    except Exception:
        return CAMERA_WINDOW_W // 2, CAMERA_WINDOW_H // 2, False

    if not boxes or not boxes[0]:
        return CAMERA_WINDOW_W // 2, CAMERA_WINDOW_H // 2, False
    person_detections = boxes[0]

    largest = max(person_detections, key=lambda d: d[0][2] * d[0][3])
    rect, score = largest
    x, y, w, h = rect
    area = w * h

    if area < MIN_AREA:
        return CAMERA_WINDOW_W // 2, CAMERA_WINDOW_H // 2, False

    body_cx = x + w // 2
    body_cy = y + int(h * BODY_CY_RATIO)

    img.draw_rectangle((x, y, w, h), color=(0, 255, 0), thickness=2)
    br = max(8, w // 6)
    img.draw_rectangle((body_cx - br, body_cy - br, br * 2, br * 2),
                       color=(255, 0, 0), thickness=2)
    detect_count += 1
    return body_cx, body_cy, True

def send_position(cx, cy, has_target):
    """发送 5 字节追踪数据帧到 STM32"""
    if uart is None:
        return
    has = 1 if has_target else 0
    tx = max(0, min(255, int(round((cx / CAMERA_WINDOW_W) * 255))))
    ty = max(0, min(255, int(round((cy / CAMERA_WINDOW_H) * 255))))
    data = bytes([HEADER1, HEADER2, has, tx, ty])
    for attempt in range(3):
        try:
            uart.write(data)
            break
        except Exception:
            if attempt == 2:
                print("串口发送失败 (已重试 3 次)")

def set_led_status(status):
    """0=全灭 1=红 2=绿 3=蓝"""
    LED(1).off(); LED(2).off(); LED(3).off()
    if status == 1:   LED(1).on()
    elif status == 2: LED(2).on()
    elif status == 3: LED(3).on()

# ============================================================
# 主程序
# ============================================================

if __name__ == "__main__":
    print("=" * 50)
    print("OpenMV N6 — YOLOv8 人形追踪固件 v5.0")
    print("=" * 50)

    set_led_status(1)         # 红: 启动中
    time.sleep_ms(500)

    print("正在初始化摄像头...")
    if not init_camera():
        print("摄像头初始化失败!")
        while True:
            set_led_status(1); time.sleep_ms(500)

    print("正在加载模型...")
    if not init_model():
        print("模型加载失败!")
        while True:
            set_led_status(1); time.sleep_ms(500)

    print("正在初始化串口...")
    if not init_uart():
        print("串口初始化失败!")
        while True:
            set_led_status(1); time.sleep_ms(500)

    set_led_status(2)         # 绿: 就绪
    time.sleep_ms(1000)
    print("追踪已启动")
    print("  模型: %s" % MODEL_PATH)
    print("  阈值: %.2f  窗口: %dx%d" % (MODEL_THRESHOLD, CAMERA_WINDOW_W, CAMERA_WINDOW_H))

    clock = time.clock()
    last_report = time.time()

    while True:
        clock.tick()
        img = cam.snapshot()

        cx, cy, has_person = detect_person(img)

        if has_person:
            if not smooth_ready:
                smooth_cx, smooth_cy = cx, cy
                smooth_ready = True
            else:
                smooth_cx = smooth_cx * (1 - EMA_ALPHA) + cx * EMA_ALPHA
                smooth_cy = smooth_cy * (1 - EMA_ALPHA) + cy * EMA_ALPHA
            send_position(int(smooth_cx), int(smooth_cy), True)
        else:
            smooth_ready = False
            send_position(cx, cy, False)

        set_led_status(2 if has_person else 3)  # 绿: 检测到 / 蓝: 未检测到

        # 每 5 秒输出统计
        now = time.time()
        if now - last_report >= 5.0:
            fps = clock.fps()
            rate = (detect_count / frame_count * 100) if frame_count > 0 else 0
            print("5秒统计: 帧=%d 命中=%d 命中率=%.1f%% fps=%.1f" % (
                frame_count, detect_count, rate, fps))
            frame_count = 0
            detect_count = 0
            last_report = now
