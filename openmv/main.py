# OpenMV N6 — YOLOv8 人形检测追踪固件（二自由度云台）
# 通过 UART3 发送目标位置到 STM32F103C8T6（5 字节协议）。
# YOLOv8n NPU 加速, CrowdHuman 数据集, 检测 "head" + "person" 双类。
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
MODEL_PATH = "/rom/crowdhuman_head_person_int8.tflite"
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
EMA_ALPHA = 0.2              # EMA 平滑系数 (0~1), 越小越平滑, 0.2 抑制抖动

# --- 每类阈值与最小面积 ---
HEAD_THRESHOLD   = 0.20     # head 置信度阈值（近距大头置信度偏低，需更低阈值）
PERSON_THRESHOLD = 0.40     # person 置信度阈值（当前模型未训练 person，保留备用）
HEAD_MIN_AREA    = 50       # head 最小面积 (px²), 远处小人头 ~50-100
HEAD_MAX_AREA    = 30000    # head 最大面积 (px²), 超出视为误检（近距大头也到不了这值）
PERSON_MIN_AREA  = 1000     # person 最小面积 (px²), 备用

# --- 目标锁定 (tracking stability) ---
LOCK_DIST_PX    = 40        # 锁定距离 (px), 上一帧目标中心附近优先匹配
LOCK_TIMEOUT_S  = 1.0       # 锁定超时 (秒), 容忍短暂丢失, 避免抖动

# --- 追踪策略 ---
TRACK_MODE = "head"           # "head" / "person" / "both"
#   当前模型只输出 head, 故切换到 head 模式
#   "head":   仅追踪头部, 目标点 = bbox 中心
#   "person": 仅追踪人体, 目标点 = 上半身估算
#   "both":   追踪两者中面积最大的, 按类别决定目标点

# --- 上半身追踪 (TRACK_MODE = "person" 或 "both" 时生效) ---
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

# 目标锁定
locked_cx = 0
locked_cy = 0
locked_cls = -1
locked_score = 0.0
lock_last_s = 0.0

# 类别统计 (每 5 秒周期)
stat_head = 0
stat_person = 0

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

def _unpack(d):
    """解包 YOLOv8 检测结果, 兼容嵌套/扁平/有无 class_id 共 4 种格式。
    返回 (x, y, w, h, score, class_id)。缺少 class_id 时默认为 0。
    """
    if isinstance(d[0], (list, tuple)):
        # 嵌套: [(x,y,w,h), score] 或 [(x,y,w,h), score, class_id]
        x, y, w, h = d[0][0], d[0][1], d[0][2], d[0][3]
        score = d[1]
        cls = d[2] if len(d) > 2 else 0
    else:
        # 扁平: [x, y, w, h, score] 或 [x, y, w, h, score, class_id]
        x, y, w, h = d[0], d[1], d[2], d[3]
        score = d[4]
        cls = d[5] if len(d) > 5 else 0
    return x, y, w, h, score, cls


def detect_person(img):
    """YOLOv8 推理 + 目标锁定防抖。
    锁定期间即使短暂无候选也保持上一帧位置，避免 EMA 被重置。
    返回 (target_cx, target_cy, has_target)
    """
    global frame_count, detect_count, stat_head, stat_person
    global locked_cx, locked_cy, locked_cls, locked_score, lock_last_s
    frame_count += 1
    now_s = time.time()

    # 推理
    try:
        boxes = model.predict([img])
    except Exception:
        if locked_cls >= 0 and now_s - lock_last_s <= LOCK_TIMEOUT_S:
            return locked_cx, locked_cy, True
        return CAMERA_WINDOW_W // 2, CAMERA_WINDOW_H // 2, False

    if not boxes or not boxes[0]:
        # 无检测, 锁定未超时则保持
        if locked_cls >= 0 and now_s - lock_last_s <= LOCK_TIMEOUT_S:
            return locked_cx, locked_cy, True
        locked_cls = -1
        return CAMERA_WINDOW_W // 2, CAMERA_WINDOW_H // 2, False

    detections = boxes[0]

    # 按类别、阈值、面积筛选
    candidates = []
    for d in detections:
        x, y, w, h, score, cls = _unpack(d)
        label = model.labels[int(cls)] if int(cls) < len(model.labels) else ""
        area = w * h

        if label == "head":
            if score < HEAD_THRESHOLD or area < HEAD_MIN_AREA or area > HEAD_MAX_AREA:
                continue
        elif label == "person":
            if score < PERSON_THRESHOLD or area < PERSON_MIN_AREA:
                continue
        else:
            continue

        candidates.append((x, y, w, h, score, cls, area, label))

    if not candidates:
        if locked_cls >= 0 and now_s - lock_last_s <= LOCK_TIMEOUT_S:
            return locked_cx, locked_cy, True
        locked_cls = -1
        return CAMERA_WINDOW_W // 2, CAMERA_WINDOW_H // 2, False

    # 锁定匹配: 在锁定距离内优先选最近的候选
    if locked_cls >= 0 and now_s - lock_last_s <= LOCK_TIMEOUT_S:
        candidates.sort(key=lambda c: abs(c[0] + c[2] // 2 - locked_cx)
                                      + abs(c[1] + c[3] // 2 - locked_cy))
        nearest = candidates[0]
        dist = abs(nearest[0] + nearest[2] // 2 - locked_cx) \
             + abs(nearest[1] + nearest[3] // 2 - locked_cy)
        if dist <= LOCK_DIST_PX:
            x, y, w, h, score, cls, area, label = nearest
        else:
            candidates.sort(key=lambda c: c[6], reverse=True)
            x, y, w, h, score, cls, area, label = candidates[0]
    else:
        candidates.sort(key=lambda c: c[6], reverse=True)
        x, y, w, h, score, cls, area, label = candidates[0]

    # 更新锁定状态
    locked_cx = x + w // 2
    locked_cy = y + h // 2
    locked_cls = int(cls)
    locked_score = score
    lock_last_s = now_s

    # 统计
    if label == "head":
        stat_head += 1
    elif label == "person":
        stat_person += 1

    # 目标点
    if label == "head":
        target_cx = x + w // 2
        target_cy = y + h // 2
        color = (255, 255, 0)
    else:
        target_cx = x + w // 2
        target_cy = y + int(h * BODY_CY_RATIO)
        color = (0, 255, 0)

    # 绘制: bbox + 锁定十字
    img.draw_rectangle((x, y, w, h), color=color, thickness=2)
    br = max(8, w // 6)
    img.draw_rectangle((target_cx - br, target_cy - br, br * 2, br * 2),
                       color=(255, 0, 0), thickness=2)
    detect_count += 1
    return target_cx, target_cy, True

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
    print("OpenMV N6 — YOLOv8 CrowdHuman 双类追踪固件 v6.0")
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
    print("  阈值: %.2f  窗口: %dx%d  模式: %s" % (
        MODEL_THRESHOLD, CAMERA_WINDOW_W, CAMERA_WINDOW_H, TRACK_MODE))

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
            print("5秒统计: 帧=%d 命中=%d 命中率=%.1f%% fps=%.1f  head=%d person=%d" % (
                frame_count, detect_count, rate, fps, stat_head, stat_person))
            frame_count = 0
            detect_count = 0
            stat_head = 0
            stat_person = 0
            last_report = now
