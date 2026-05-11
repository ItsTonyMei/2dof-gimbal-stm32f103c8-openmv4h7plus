# OpenMV N6 — CrowdHuman head+person 双类追踪固件（二自由度云台单板方案）
# 视觉检测 + PD控制器 + 舵机PWM直驱，全部在 N6 上执行。
# 模型: YOLOv11n, 后处理器兼容 YoloV8
# 舵机: 通过 N6 传感器扩展板 P4 (TIM2_CH3, Base/Yaw), P5 (TIM2_CH4, Arm/Pitch)

import csi, image, time, ml
from machine import PWM, Pin
from pyb import LED
from ml.postprocessing.ultralytics import YoloV8

# ============================================================
# 视觉参数
# ============================================================

MODEL_NMS_THRESHOLD = 0.1
MODEL_PATH = "/rom/crowdhuman_head_person_int8.tflite"
CAMERA_WINDOW_W = 320
CAMERA_WINDOW_H = 320
MODEL_THRESHOLD = 0.4

CAMERA_HMIRROR = True
CAMERA_VFLIP   = True
EXPOSURE_US    = 20000

EMA_ALPHA = 0.2
BBOX_EMA_ALPHA = 0.25

HEAD_THRESHOLD   = 0.20
PERSON_THRESHOLD = 0.35
HEAD_MIN_AREA    = 50
HEAD_MAX_AREA    = 30000
PERSON_MIN_AREA  = 1000

LOCK_DIST_PX    = 40
LOCK_TIMEOUT_S  = 1.0

TRACK_MODE = "both"

BODY_CY_RATIO = 0.28

# ============================================================
# 舵机 PWM 参数
# ============================================================

SERVO_BASE_PIN = "P4"
SERVO_ARM_PIN  = "P5"
SERVO_FREQ = 50                     # 50 Hz, 20ms 周期

# PWM 脉冲宽度 (ns), 对应 STM32 端的 timer tick * 2us
SERVO_BASE_NEUTRAL = 1500000        # 1500us 中位
SERVO_BASE_MIN_NS  = 500000         # 500us
SERVO_BASE_MAX_NS  = 2500000        # 2500us
SERVO_ARM_NEUTRAL  = 1500000
SERVO_ARM_MIN_NS   = 600000         # 600us
SERVO_ARM_MAX_NS    = 2400000        # 2400us

# ============================================================
# PD 控制参数 (移植自 STM32 control.h)
# ============================================================

YAW_KP   = 0.05
YAW_KD   = 0.0                     # 微分增益默认禁用
PITCH_KP = 0.05
PITCH_KD = 0.0

PD_INNER_DEADZONE = 5              # px, 完全死区半径
PD_OUTER_DEADZONE = 15             # px, 过渡带外边界
PD_EMA_ALPHA      = 0.1            # 微分项 EMA 平滑

MAX_VELOCITY_BASE_NS = 20.0        # 每 ms 最大速度 (ns/ms)
MAX_VELOCITY_ARM_NS  = 20.0        # 设计基准: 10 ticks * 2us = 20ns per 1ms cycle

CENTER_X = 128.0                    # 归一化中心, 匹配 STM32 OPENMV_CENTER_X
CENTER_Y = 128.0

# ============================================================
# 全局状态
# ============================================================

cam = None
model = None
pwm_base = None
pwm_arm = None
frame_count = 0
detect_count = 0
smooth_cx = 0
smooth_cy = 0
smooth_ready = False

locked_cx = 0
locked_cy = 0
locked = False
lock_last_s = 0.0

stat_head = 0
stat_person = 0

smooth_person_box = None
smooth_head_box = None

# 舵机位置 (ns)
pos_base = SERVO_BASE_NEUTRAL
pos_arm  = SERVO_ARM_NEUTRAL

# PD 控制器状态
last_error_x = 0.0
last_error_y = 0.0
yaw_deriv = 0.0
pitch_deriv = 0.0
last_control_tick = 0

# ============================================================
# 初始化
# ============================================================

def init_camera():
    global cam
    cam = csi.CSI()
    cam.reset()
    cam.pixformat(csi.RGB565)
    cam.framesize(csi.VGA)
    cam.window((CAMERA_WINDOW_W, CAMERA_WINDOW_H))
    cam.hmirror(CAMERA_HMIRROR)
    cam.vflip(CAMERA_VFLIP)
    cam.snapshot(time=2000)
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

def init_servos():
    global pwm_base, pwm_arm
    try:
        pwm_base = PWM(Pin(SERVO_BASE_PIN), freq=SERVO_FREQ)
        pwm_arm  = PWM(Pin(SERVO_ARM_PIN), freq=SERVO_FREQ)
        pwm_base.duty_ns(SERVO_BASE_NEUTRAL)
        pwm_arm.duty_ns(SERVO_ARM_NEUTRAL)
        print("舵机初始化成功 — Base:%s Arm:%s @ %dHz" % (SERVO_BASE_PIN, SERVO_ARM_PIN, SERVO_FREQ))
        return True
    except Exception as e:
        print("舵机初始化失败:", e)
        return False

# ============================================================
# 核心逻辑
# ============================================================

def _unpack(d):
    if isinstance(d[0], (list, tuple)):
        x, y, w, h = d[0][0], d[0][1], d[0][2], d[0][3]
        score = d[1]
        cls = d[2] if len(d) > 2 else 0
    else:
        x, y, w, h = d[0], d[1], d[2], d[3]
        score = d[4]
        cls = d[5] if len(d) > 5 else 0
    return x, y, w, h, score, cls


def _head_inside_person(hx, hy, hw, hh, px, py, pw, ph):
    hcx = hx + hw // 2
    hcy = hy + hh // 2
    if not (px <= hcx <= px + pw and py <= hcy <= py + ph):
        return False
    return hcy < py + ph * 0.5


def _match_head_to_person(heads, px, py, pw, ph):
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
    if prev is None:
        return float(curr[0]), float(curr[1]), float(curr[2]), float(curr[3])
    return (prev[0] * (1 - alpha) + curr[0] * alpha,
            prev[1] * (1 - alpha) + curr[1] * alpha,
            prev[2] * (1 - alpha) + curr[2] * alpha,
            prev[3] * (1 - alpha) + curr[3] * alpha)


def detect_person(img):
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

    detections = []
    for cls_id, class_boxes in enumerate(boxes):
        for d in class_boxes:
            x, y, w, h, score, raw_cls = _unpack(d)
            if raw_cls == 0 and cls_id > 0:
                cls = cls_id
            else:
                cls = raw_cls
            detections.append((x, y, w, h, score, cls))

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

    candidates = []

    if TRACK_MODE == "both":
        for px, py, pw, ph, pscore in persons:
            paired_head = _match_head_to_person(heads, px, py, pw, ph)
            if paired_head:
                hx, hy, hw, hh, hscore = paired_head
                tc = hx + hw // 2
                ty = hy + hh // 2
                draws = [
                    (px, py, pw, ph, (0, 255, 0)),
                    (hx, hy, hw, hh, (255, 255, 0)),
                ]
                candidates.append((tc, ty, pw * ph, draws))
            else:
                tc = px + pw // 2
                ty = py + int(ph * BODY_CY_RATIO)
                draws = [(px, py, pw, ph, (0, 255, 0))]
                candidates.append((tc, ty, pw * ph, draws))

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

    else:
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

    locked_cx = target_cx
    locked_cy = target_cy
    locked = True
    lock_last_s = now_s

    display_draws = []
    smooth_head_center = None
    smooth_person_upper = None
    for rx, ry, rw, rh, rcolor in draws:
        if rcolor == (0, 255, 0):
            sx, sy, sw, sh = _ema_bbox(smooth_person_box, (rx, ry, rw, rh), BBOX_EMA_ALPHA)
            smooth_person_box = (sx, sy, sw, sh)
            smooth_person_upper = (int(sx + sw / 2), int(sy + sh * BODY_CY_RATIO))
        elif rcolor == (255, 255, 0):
            sx, sy, sw, sh = _ema_bbox(smooth_head_box, (rx, ry, rw, rh), BBOX_EMA_ALPHA)
            smooth_head_box = (sx, sy, sw, sh)
            smooth_head_center = (int(sx + sw / 2), int(sy + sh / 2))
        else:
            sx, sy, sw, sh = float(rx), float(ry), float(rw), float(rh)
        display_draws.append((int(sx), int(sy), int(sw), int(sh), rcolor))

    for rx, ry, rw, rh, rcolor in display_draws:
        img.draw_rectangle((rx, ry, rw, rh), color=rcolor, thickness=2)

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


# ============================================================
# PD 舵机控制器 (移植自 STM32 control.c)
# ============================================================

def _clamp(value, min_val, max_val):
    if value < min_val: return min_val
    if value > max_val: return max_val
    return value


def servo_control(cx, cy, has_target):
    """PD控制器 + 舵机PWM输出。每帧调用一次，dt自适应帧间隔。
    cx, cy: 目标在 320x320 图像中的像素坐标
    has_target: 是否检测到目标
    """
    global pos_base, pos_arm
    global last_error_x, last_error_y, yaw_deriv, pitch_deriv
    global last_control_tick

    now_us = time.ticks_us()
    dt_ms = 0.0
    if last_control_tick != 0:
        dt_ticks = time.ticks_diff(now_us, last_control_tick)
        dt_ms = dt_ticks / 1000.0
    last_control_tick = now_us

    if dt_ms <= 0:
        dt_ms = 33.0  # 默认 ~30fps

    if not has_target:
        # 目标丢失: 速度归零, 保持当前位置
        last_error_x = 0.0
        last_error_y = 0.0
        yaw_deriv = 0.0
        pitch_deriv = 0.0
        return

    # 归一化坐标 → 误差 (匹配 STM32 协议 tx/ty 量化路径)
    tx = int(round((cx / CAMERA_WINDOW_W) * 255))
    ty = int(round((cy / CAMERA_WINDOW_H) * 255))
    raw_dx = float(tx) - CENTER_X
    raw_dy = float(ty) - CENTER_Y

    abs_dx = raw_dx if raw_dx >= 0 else -raw_dx
    abs_dy = raw_dy if raw_dy >= 0 else -raw_dy

    # Yaw 软死区
    scale_x = 1.0
    if abs_dx <= PD_INNER_DEADZONE:
        scale_x = 0.0
    elif abs_dx < PD_OUTER_DEADZONE:
        t = (abs_dx - PD_INNER_DEADZONE) / (PD_OUTER_DEADZONE - PD_INNER_DEADZONE)
        scale_x = t * t

    # Pitch 软死区
    scale_y = 1.0
    if abs_dy <= PD_INNER_DEADZONE:
        scale_y = 0.0
    elif abs_dy < PD_OUTER_DEADZONE:
        t = (abs_dy - PD_INNER_DEADZONE) / (PD_OUTER_DEADZONE - PD_INNER_DEADZONE)
        scale_y = t * t

    new_error_x = (raw_dx / 255.0) * scale_x
    new_error_y = (raw_dy / 255.0) * scale_y

    # PD 控制 — 微分项 EMA 滤波
    alpha = PD_EMA_ALPHA
    yaw_deriv = alpha * ((new_error_x - last_error_x) / (dt_ms / 1000.0)) + (1.0 - alpha) * yaw_deriv
    pitch_deriv = alpha * ((new_error_y - last_error_y) / (dt_ms / 1000.0)) + (1.0 - alpha) * pitch_deriv

    yaw_out = YAW_KP * new_error_x + YAW_KD * yaw_deriv
    pitch_out = PITCH_KP * new_error_y + PITCH_KD * pitch_deriv

    # 速度缩放 (基准: 500 * dt_ms / 10.0, STM32 100Hz → 10ms/cycle)
    vel_scale = 500.0 * dt_ms / 10.0
    yaw_vel = yaw_out * vel_scale
    pitch_vel = pitch_out * vel_scale

    max_vel = MAX_VELOCITY_BASE_NS * dt_ms / 10.0
    yaw_vel = _clamp(yaw_vel, -max_vel, max_vel)
    pitch_vel = _clamp(pitch_vel, -max_vel, max_vel)

    # 位置积分
    pos_base += yaw_vel
    pos_arm  += pitch_vel
    pos_base = _clamp(pos_base, SERVO_BASE_MIN_NS, SERVO_BASE_MAX_NS)
    pos_arm  = _clamp(pos_arm,  SERVO_ARM_MIN_NS,  SERVO_ARM_MAX_NS)

    # PWM 输出
    pwm_base.duty_ns(int(pos_base))
    pwm_arm.duty_ns(int(pos_arm))

    last_error_x = new_error_x
    last_error_y = new_error_y


def set_led_status(status):
    LED(1).off(); LED(2).off(); LED(3).off()
    if status == 1:   LED(1).on()
    elif status == 2: LED(2).on()
    elif status == 3: LED(3).on()

# ============================================================
# 主程序
# ============================================================

if __name__ == "__main__":
    print("=" * 50)
    print("OpenMV N6 — YOLOv11n CrowdHuman 双类追踪 + PD舵机直驱 v7.0")
    print("=" * 50)

    set_led_status(1)
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

    print("正在初始化舵机...")
    if not init_servos():
        print("舵机初始化失败!")
        while True:
            set_led_status(1); time.sleep_ms(500)

    set_led_status(2)
    time.sleep_ms(1000)
    print("追踪已启动")
    print("  模型: %s" % MODEL_PATH)
    print("  阈值: %.2f  窗口: %dx%d  模式: %s" % (
        MODEL_THRESHOLD, CAMERA_WINDOW_W, CAMERA_WINDOW_H, TRACK_MODE))
    print("  舵机: Base=%s Arm=%s @ %dHz" % (SERVO_BASE_PIN, SERVO_ARM_PIN, SERVO_FREQ))
    print("  KP=%.3f KD=%.3f 死区=%d-%dpx" % (YAW_KP, YAW_KD, PD_INNER_DEADZONE, PD_OUTER_DEADZONE))

    clock = time.clock()
    last_report = time.time()

    while True:
        clock.tick()
        img = cam.snapshot()

        cx, cy, has_target = detect_person(img)

        if has_target:
            if not smooth_ready:
                smooth_cx, smooth_cy = cx, cy
                smooth_ready = True
            else:
                smooth_cx = smooth_cx * (1 - EMA_ALPHA) + cx * EMA_ALPHA
                smooth_cy = smooth_cy * (1 - EMA_ALPHA) + cy * EMA_ALPHA
            servo_control(int(smooth_cx), int(smooth_cy), True)
        else:
            smooth_ready = False
            servo_control(0, 0, False)

        set_led_status(2 if has_target else 3)

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
