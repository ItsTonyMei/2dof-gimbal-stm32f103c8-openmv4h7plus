# OpenMV N6 — CrowdHuman head+person 双类追踪固件（二自由度云台）
# 通过 UART3 发送目标位置到 STM32F103C8T6（5 字节协议）。
# 模型: YOLOv11n, 后处理器兼容 YoloV8

import csi, image, time, ml
from pyb import UART, LED
from ml.postprocessing.ultralytics import YoloV8  # YOLOv11 输出格式兼容 YoloV8 后处理

# ============================================================
# 配置参数
# ============================================================

MODEL_NMS_THRESHOLD = 0.1    # NMS 阈值
MODEL_PATH = "/rom/crowdhuman_head_person_int8.tflite"
CAMERA_WINDOW_W = 320
CAMERA_WINDOW_H = 320
MODEL_THRESHOLD = 0.4        # YOLOv11n NMS 置信度 (兼容 YoloV8 后处理)

# --- 摄像头 ---
CAMERA_HMIRROR = True        # 水平镜像（倒装安装）
CAMERA_VFLIP   = True        # 垂直翻转（倒装安装）
EXPOSURE_US    = 20000       # 固定曝光 (us), 室外可降低

# --- 平滑 ---
EMA_ALPHA = 0.2              # 目标点 EMA 平滑系数 (0~1), 越小越平滑
BBOX_EMA_ALPHA = 0.25        # 检测框 EMA 平滑系数, 低于 0.3 时抖动抑制明显

# --- 每类阈值与最小面积 ---
HEAD_THRESHOLD   = 0.20     # head 置信度阈值（近距大头置信度偏低，需更低阈值）
PERSON_THRESHOLD = 0.35     # person 置信度阈值
HEAD_MIN_AREA    = 50       # head 最小面积 (px²), 远处小人头 ~50-100
HEAD_MAX_AREA    = 30000    # head 最大面积 (px²), 超出视为误检（近距大头也到不了这值）
PERSON_MIN_AREA  = 1000     # person 最小面积 (px²)

# --- 目标锁定 (tracking stability) ---
LOCK_DIST_PX    = 40        # 锁定距离 (px), 上一帧目标中心附近优先匹配
LOCK_TIMEOUT_S  = 1.0       # 锁定超时 (秒), 容忍短暂丢失, 避免抖动

# --- 追踪策略 ---
TRACK_MODE = "both"           # "head" / "person" / "both"
#   "head":   仅追踪头部, 目标点 = bbox 中心
#   "person": 仅追踪人体, 目标点 = 上半身估算
#   "both":   person+head 配对: 有 person 时优先匹配其 head, 目标=头中心; 无双框同显

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
locked = False
lock_last_s = 0.0

# 类别统计 (每 5 秒周期)
stat_head = 0
stat_person = 0

# 检测框 EMA 平滑 (按角色分别追踪, 减少视觉抖动)
smooth_person_box = None  # (x, y, w, h) or None
smooth_head_box = None

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
            postprocess=YoloV8(  # YOLOv11n 输出兼容 YoloV8 后处理
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
    """解包 YOLOv11n 检测结果, 兼容嵌套/扁平/有无 class_id 共 4 种格式。
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


def _head_inside_person(hx, hy, hw, hh, px, py, pw, ph):
    """head 中心在 person 上半区域内 (top 50%) 且至少部分重叠"""
    hcx = hx + hw // 2
    hcy = hy + hh // 2
    if not (px <= hcx <= px + pw and py <= hcy <= py + ph):
        return False
    # head 中心在 person 上半部分 (top 50%)
    return hcy < py + ph * 0.5


def _match_head_to_person(heads, px, py, pw, ph):
    """在 heads 列表中找与 person 框配对的最佳 head。
    返回 (hx, hy, hw, hh, hscore) 或 None。
    """
    best = None
    best_area = 0
    for hx, hy, hw, hh, hscore in heads:
        if _head_inside_person(hx, hy, hw, hh, px, py, pw, ph):
            area = hw * hh
            if area > best_area:
                best_area = area
                best = (hx, hy, hw, hh, hscore)
    return best


def _ema_bbox(prev, curr, alpha):
    """对 bbox 四元组做 EMA 平滑, prev 为 None 时返回当前值"""
    if prev is None:
        return float(curr[0]), float(curr[1]), float(curr[2]), float(curr[3])
    return (prev[0] * (1 - alpha) + curr[0] * alpha,
            prev[1] * (1 - alpha) + curr[1] * alpha,
            prev[2] * (1 - alpha) + curr[2] * alpha,
            prev[3] * (1 - alpha) + curr[3] * alpha)


def detect_person(img):
    """YOLOv11n 推理 + 目标锁定防抖。
    both 模式: person 框 + 配对 head 框同时绘制, 目标点 = head 中心。
    返回 (target_cx, target_cy, has_target)
    """
    global frame_count, detect_count, stat_head, stat_person
    global locked_cx, locked_cy, locked, lock_last_s
    global smooth_person_box, smooth_head_box
    frame_count += 1
    now_s = time.time()

    try:
        boxes = model.predict([img])
    except Exception:
        if locked and now_s - lock_last_s <= LOCK_TIMEOUT_S:
            return locked_cx, locked_cy, True
        smooth_person_box = None
        smooth_head_box = None
        return CAMERA_WINDOW_W // 2, CAMERA_WINDOW_H // 2, False

    if not boxes or not boxes[0]:
        if locked and now_s - lock_last_s <= LOCK_TIMEOUT_S:
            return locked_cx, locked_cy, True
        locked = False
        smooth_person_box = None
        smooth_head_box = None
        return CAMERA_WINDOW_W // 2, CAMERA_WINDOW_H // 2, False

    # 合并所有类别的检测
    detections = []
    for cls_id, class_boxes in enumerate(boxes):
        for d in class_boxes:
            x, y, w, h, score, raw_cls = _unpack(d)
            if raw_cls == 0 and cls_id > 0:
                cls = cls_id
            else:
                cls = raw_cls
            detections.append((x, y, w, h, score, cls))

    # 按类别、阈值、面积筛选 → 分入 head / person 列表
    heads = []
    persons = []
    for (x, y, w, h, score, cls) in detections:
        label = model.labels[int(cls)] if int(cls) < len(model.labels) else ""
        area = w * h
        if label == "head":
            if score >= HEAD_THRESHOLD and HEAD_MIN_AREA <= area <= HEAD_MAX_AREA:
                heads.append((x, y, w, h, score))
                stat_head += 1
        elif label == "person":
            if score >= PERSON_THRESHOLD and area >= PERSON_MIN_AREA:
                persons.append((x, y, w, h, score))
                stat_person += 1

    # ---- 候选生成: 根据 TRACK_MODE 构建 (target_cx, target_cy, draw_list) ----
    candidates = []  # [(target_cx, target_cy, priority_key, draw_rects)]

    if TRACK_MODE == "both":
        # person + 配对 head
        for px, py, pw, ph, pscore in persons:
            paired_head = _match_head_to_person(heads, px, py, pw, ph)
            if paired_head:
                hx, hy, hw, hh, hscore = paired_head
                tc = hx + hw // 2
                ty = hy + hh // 2
                draws = [
                    (px, py, pw, ph, (0, 255, 0)),       # person 绿色
                    (hx, hy, hw, hh, (255, 255, 0)),      # head 黄色
                ]
                candidates.append((tc, ty, pw * ph, draws))
            else:
                tc = px + pw // 2
                ty = py + int(ph * BODY_CY_RATIO)
                draws = [(px, py, pw, ph, (0, 255, 0))]
                candidates.append((tc, ty, pw * ph, draws))

        # 没有 person 时, 用 head 单独
        if not persons:
            for hx, hy, hw, hh, hscore in heads:
                tc = hx + hw // 2
                ty = hy + hh // 2
                draws = [(hx, hy, hw, hh, (255, 255, 0))]
                candidates.append((tc, ty, hw * hh, draws))

    elif TRACK_MODE == "person":
        for px, py, pw, ph, pscore in persons:
            tc = px + pw // 2
            ty = py + int(ph * BODY_CY_RATIO)
            draws = [(px, py, pw, ph, (0, 255, 0))]
            candidates.append((tc, ty, pw * ph, draws))

    else:  # TRACK_MODE == "head"
        for hx, hy, hw, hh, hscore in heads:
            tc = hx + hw // 2
            ty = hy + hh // 2
            draws = [(hx, hy, hw, hh, (255, 255, 0))]
            candidates.append((tc, ty, hw * hh, draws))

    if not candidates:
        if locked and now_s - lock_last_s <= LOCK_TIMEOUT_S:
            return locked_cx, locked_cy, True
        locked = False
        smooth_person_box = None
        smooth_head_box = None
        return CAMERA_WINDOW_W // 2, CAMERA_WINDOW_H // 2, False

    # ---- 锁定匹配：优先选离上一帧目标最近的候选 ----
    if locked and now_s - lock_last_s <= LOCK_TIMEOUT_S:
        candidates.sort(key=lambda c: abs(c[0] - locked_cx) + abs(c[1] - locked_cy))
        nearest = candidates[0]
        dist = abs(nearest[0] - locked_cx) + abs(nearest[1] - locked_cy)
        if dist <= LOCK_DIST_PX:
            target_cx, target_cy, _, draws = nearest
        else:
            candidates.sort(key=lambda c: c[2], reverse=True)
            target_cx, target_cy, _, draws = candidates[0]
    else:
        candidates.sort(key=lambda c: c[2], reverse=True)
        target_cx, target_cy, _, draws = candidates[0]

    # 更新锁定
    locked_cx = target_cx
    locked_cy = target_cy
    locked = True
    lock_last_s = now_s

    # 绘制平滑后的检测框 + 瞄准十字
    display_draws = []
    smooth_head_center = None
    smooth_person_upper = None
    for rx, ry, rw, rh, rcolor in draws:
        if rcolor == (0, 255, 0):      # person 框
            sx, sy, sw, sh = _ema_bbox(smooth_person_box, (rx, ry, rw, rh), BBOX_EMA_ALPHA)
            smooth_person_box = (sx, sy, sw, sh)
            smooth_person_upper = (int(sx + sw / 2), int(sy + sh * BODY_CY_RATIO))
        elif rcolor == (255, 255, 0):  # head 框
            sx, sy, sw, sh = _ema_bbox(smooth_head_box, (rx, ry, rw, rh), BBOX_EMA_ALPHA)
            smooth_head_box = (sx, sy, sw, sh)
            smooth_head_center = (int(sx + sw / 2), int(sy + sh / 2))
        else:
            sx, sy, sw, sh = float(rx), float(ry), float(rw), float(rh)
        display_draws.append((int(sx), int(sy), int(sw), int(sh), rcolor))

    for rx, ry, rw, rh, rcolor in display_draws:
        img.draw_rectangle((rx, ry, rw, rh), color=rcolor, thickness=2)

    # 十字从平滑框推算 (优先 head, 否则 person 上半身)
    if smooth_head_center is not None:
        cross_cx, cross_cy = smooth_head_center
    elif smooth_person_upper is not None:
        cross_cx, cross_cy = smooth_person_upper
    else:
        cross_cx, cross_cy = target_cx, target_cy

    br = max(8, min(display_draws[0][2], display_draws[0][3]) // 6)
    img.draw_cross((cross_cx, cross_cy), size=br, color=(255, 0, 0), thickness=2)
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
    print("OpenMV N6 — YOLOv11n CrowdHuman 双类追踪固件 v6.1")
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
