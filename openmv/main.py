# OpenMV N6 — BlazeFace 人脸追踪 + PD 舵机控制（精简单板方案）
# 检测: Google MediaPipe BlazeFace (128×128, 186KB, ROM 内置)
# 追踪: 最近脸优先 + 跳变保护 + 丢失复位重捕获
# 控制: PD 控制器 + 线性死区 + 导数死区 + PWM 输出死区
# 舵机: P7 (Base/Yaw), P9 (Arm/Pitch)

import csi, time, gc
from machine import PWM, Pin
from pyb import LED

try:
    import ml
    from ml.postprocessing.mediapipe import BlazeFace
    from ml.utils import draw_keypoints
except ImportError as e:
    print("[FATAL] ml 模块加载失败:", e)
    LED(1).on()
    while True:
        time.sleep_ms(500)

# ============================================================
# 舵机方向 (调试: 改 1↔-1 翻转该轴)
# ============================================================

YAW_DIR   = -1   # 底舵机 P7:  1=正向, -1=反向
PITCH_DIR = 1   # 摇臂舵机 P9:  1=正向, -1=反向

# ============================================================
# 视觉参数
# ============================================================

CAMERA_WINDOW_W = 320  # VGA 640×480 → 缩小窗口提升模型空间分辨率
CAMERA_WINDOW_H = 320
BLAZEFACE_THRESHOLD = 0.25  # 人脸检测置信度 (降低=更远可检, 但可能误检)

CAMERA_HMIRROR = True
CAMERA_VFLIP   = True
EXPOSURE_US    = 20000  # 固定曝光 20ms
DRAW_ENABLE    = False  # IDE 调试时改为 True, 脱机运行保持 False

# ============================================================
# 追踪参数
# ============================================================

TRACK_LOST_MAX  = 30        # 连续丢失多少帧后彻底放弃
TRACK_MAX_JUMP  = 55        # 新目标与上次位置超过此距离视为误检 (px)

# ============================================================
# 舵机 PWM
# ============================================================

SERVO_BASE_PIN = "P7"       # 底舵机 (Yaw) — 扩展板舵机接口 1
SERVO_ARM_PIN  = "P9"       # 摇臂舵机 (Pitch) — 扩展板舵机接口 3
SERVO_FREQ = 50

SERVO_BASE_NEUTRAL = 1500000  # 中位 1500us
SERVO_BASE_MIN_NS  = 500000
SERVO_BASE_MAX_NS  = 2500000
SERVO_ARM_NEUTRAL  = 1500000
SERVO_ARM_MIN_NS   = 1000000
SERVO_ARM_MAX_NS   = 2000000

# ============================================================
# PD 控制参数
# ============================================================

CAM_CX = CAMERA_WINDOW_W // 2  # 160
CAM_CY = CAMERA_WINDOW_H // 2  # 160

KP = 0.8            # 比例增益: 误差→速度 (增大=更快响应)
KD = 0.1            # 微分增益: 抑制震荡 (增大=更强阻尼)
DERIV_DEAD = 5      # 导数死区 (pixels) — 过滤检测噪声防微抖

PD_DEAD_INNER = 13  # 完全死区 (pixels) — 加宽防微抖
PD_DEAD_OUTER = 27  # 过渡区外边界 (pixels)

SERVO_GAIN = 18000    # 像素误差→舵机速度 (ns/s per pixel, 320窗补偿)
MAX_STEP_NS = 50000   # 每 tick 最大步进 (ns)
PWM_DEAD_NS = 5000    # PWM 输出死区 (ns) — 不够 5us 不更新, 防微抖

CONTROL_EVERY_N = 1   # 每帧执行舵机控制 (~70Hz, 丝滑)

# ============================================================
# 回中参数
# ============================================================

RETURN_HOLD_TIME_MS = 1500  # 目标丢失后保持位置 (ms)
RETURN_SPEED_NS_MS  = 500   # 回中速度 (ns/ms) → ~500us/s

# ============================================================
# 全局状态
# ============================================================

cam = None
model = None
pwm_base = None
pwm_arm = None

# 舵机当前位置 (实际发送到 PWM 的值)
pos_base_ns = SERVO_BASE_NEUTRAL
pos_arm_ns  = SERVO_ARM_NEUTRAL
pwm_base_last = SERVO_BASE_NEUTRAL
pwm_arm_last  = SERVO_ARM_NEUTRAL

# PD 误差记忆 (像素)
last_err_px = 0.0
last_err_py = 0.0
last_ctrl_tick = 0

# 追踪状态
last_cx, last_cy = 0, 0      # 上一次检测到的人脸中心
track_lost = 0                # 连续丢失帧数
target_lost_tick = 0          # 丢失时刻 (us)

# 统计
frame_count  = 0
face_count   = 0

# ============================================================
# 初始化
# ============================================================

def init_camera():
    global cam
    cam = csi.CSI()
    cam.reset()
    cam.pixformat(csi.RGB565)
    cam.framesize(csi.VGA)                      # 640×480 传感器
    cam.window((CAMERA_WINDOW_W, CAMERA_WINDOW_H))  # 裁剪方形
    cam.hmirror(CAMERA_HMIRROR)
    cam.vflip(CAMERA_VFLIP)
    cam.snapshot(time=2000)                     # 等待传感器稳定
    cam.auto_gain(False)
    cam.auto_exposure(False, exposure_us=EXPOSURE_US)
    print("[CAM] %dx%d 初始化成功" % (CAMERA_WINDOW_W, CAMERA_WINDOW_H))
    return True

def init_model():
    global model
    try:
        model = ml.Model(
            "/rom/blazeface_front_128.tflite",        # ROM 内置模型
            postprocess=BlazeFace(threshold=BLAZEFACE_THRESHOLD),
        )
        print("[MODEL] BlazeFace 加载成功")
        print("  input :", model.input_shape)
        print("  size  :", model.len, "bytes")
        return True
    except Exception as e:
        print("[MODEL] 加载失败:", e)
        return False

def init_servos():
    global pwm_base, pwm_arm
    try:
        pwm_base = PWM(Pin(SERVO_BASE_PIN), freq=SERVO_FREQ)
        pwm_arm  = PWM(Pin(SERVO_ARM_PIN), freq=SERVO_FREQ)
        pwm_base.duty_ns(SERVO_BASE_NEUTRAL)
        pwm_arm.duty_ns(SERVO_ARM_NEUTRAL)
        print("[SERVO] Base=%s Arm=%s 初始化成功" % (SERVO_BASE_PIN, SERVO_ARM_PIN))
        return True
    except Exception as e:
        print("[SERVO] 初始化失败:", e)
        return False

# ============================================================
# 辅助函数
# ============================================================

def clamp(v, lo, hi):
    if v < lo: return lo
    if v > hi: return hi
    return v

def deadzone(pixels, inner, outer):
    a = abs(pixels)
    if a <= inner:
        return 0.0
    if a < outer:
        t = (a - inner) / (outer - inner)
        return pixels * t  # 线性过渡, 边界增益无尖峰
    return float(pixels)

def _pwm_write(pwm, value):
    """写入 PWM, 跳过 < PWM_DEAD_NS 的无效微调以防伺服死区狩猎。"""
    global pwm_base_last, pwm_arm_last
    last = pwm_base_last if pwm is pwm_base else pwm_arm_last
    if abs(value - last) >= PWM_DEAD_NS:
        pwm.duty_ns(value)
        if pwm is pwm_base:
            pwm_base_last = value
        else:
            pwm_arm_last = value

# ============================================================
# PD 舵机控制
# ============================================================

def servo_control(cx, cy, has_target):
    """PD 控制器 (像素误差 → 舵机速度)。
    cx, cy: 目标中心像素坐标 [0, 320]。
    """
    global pos_base_ns, pos_arm_ns
    global last_err_px, last_err_py, last_ctrl_tick
    global target_lost_tick

    now_us = time.ticks_us()
    if last_ctrl_tick == 0:
        last_ctrl_tick = now_us
        return
    dt_ticks = time.ticks_diff(now_us, last_ctrl_tick)
    dt_ms = dt_ticks / 1000.0
    if dt_ms < 1.0 or dt_ms > 100:
        dt_ms = 33.0
    last_ctrl_tick = now_us
    dt_s = dt_ms / 1000.0

    # ---- 目标丢失 ----
    if not has_target:
        if target_lost_tick == 0:
            target_lost_tick = now_us
        lost_ms = time.ticks_diff(now_us, target_lost_tick) / 1000.0

        if lost_ms < RETURN_HOLD_TIME_MS:
            return

        step = RETURN_SPEED_NS_MS * dt_ms
        if pos_base_ns > SERVO_BASE_NEUTRAL:
            pos_base_ns = max(SERVO_BASE_NEUTRAL, pos_base_ns - step)
        else:
            pos_base_ns = min(SERVO_BASE_NEUTRAL, pos_base_ns + step)
        if pos_arm_ns > SERVO_ARM_NEUTRAL:
            pos_arm_ns = max(SERVO_ARM_NEUTRAL, pos_arm_ns - step)
        else:
            pos_arm_ns = min(SERVO_ARM_NEUTRAL, pos_arm_ns + step)
        _pwm_write(pwm_base, int(pos_base_ns))
        _pwm_write(pwm_arm,  int(pos_arm_ns))
        return

    target_lost_tick = 0

    # ---- 像素误差 (带死区 + 方向校正) ----
    raw_px = cx - CAM_CX
    raw_py = cy - CAM_CY

    err_x = deadzone(raw_px, PD_DEAD_INNER, PD_DEAD_OUTER)
    err_y = deadzone(raw_py, PD_DEAD_INNER, PD_DEAD_OUTER)

    # ---- PD (导数用原始误差; 死区内强制切除 D 项防检测噪声) ----
    derr_x = raw_px - last_err_px if err_x != 0.0 else 0.0
    derr_y = raw_py - last_err_py if err_y != 0.0 else 0.0
    if abs(derr_x) <= DERIV_DEAD: derr_x = 0.0
    if abs(derr_y) <= DERIV_DEAD: derr_y = 0.0
    deriv_px = derr_x / dt_s if derr_x != 0.0 else 0.0
    deriv_py = derr_y / dt_s if derr_y != 0.0 else 0.0

    out_x = KP * err_x + KD * deriv_px
    out_y = KP * err_y + KD * deriv_py

    # ---- 像素输出 → 舵机速度 (ns/tick) ----
    step_x = out_x * SERVO_GAIN * dt_s * YAW_DIR
    step_y = out_y * SERVO_GAIN * dt_s * PITCH_DIR
    step_x = clamp(step_x, -MAX_STEP_NS, MAX_STEP_NS)
    step_y = clamp(step_y, -MAX_STEP_NS, MAX_STEP_NS)

    # ---- 位置积分 ----
    pos_base_ns = clamp(pos_base_ns + step_x, SERVO_BASE_MIN_NS, SERVO_BASE_MAX_NS)
    pos_arm_ns  = clamp(pos_arm_ns  + step_y, SERVO_ARM_MIN_NS,  SERVO_ARM_MAX_NS)

    _pwm_write(pwm_base, int(pos_base_ns))
    _pwm_write(pwm_arm,  int(pos_arm_ns))

    last_err_px = raw_px
    last_err_py = raw_py

# ============================================================
# 人脸追踪
# ============================================================

def get_target(faces):
    """返回 (cx, cy, bbox, has_target)。始终跟踪距离上次位置最近的脸。"""
    global last_cx, last_cy, track_lost

    if faces:
        if last_cx > 0:
            # 已锁定: 选离上次位置最近的脸, 防止多人/误检时跳目标
            best = min(faces, key=lambda f:
                abs(int(f[0][0] + f[0][2] // 2) - last_cx) +
                abs(int(f[0][1] + f[0][3] // 2) - last_cy))
        else:
            # 首次/复位后: 选最大脸
            best = max(faces, key=lambda f: f[0][2] * f[0][3])

        bbox, score, _kp = best
        cx = int(bbox[0] + bbox[2] // 2)
        cy = int(bbox[1] + bbox[3] // 2)

        # 边界保护: 丢弃越界检测框 (模型偶发异常输出)
        if not (0 <= cx < CAMERA_WINDOW_W and 0 <= cy < CAMERA_WINDOW_H):
            track_lost += 1
            if track_lost <= TRACK_LOST_MAX:
                return last_cx, last_cy, None, True
            last_cx, last_cy = 0, 0
            return 0, 0, None, False

        if last_cx > 0:
            dist = abs(cx - last_cx) + abs(cy - last_cy)
            if dist > TRACK_MAX_JUMP:
                # 跳变保护: 递增丢失计数, 超过阈值后彻底复位重捕获
                track_lost += 1
                if track_lost <= TRACK_LOST_MAX:
                    return last_cx, last_cy, bbox, True
                last_cx, last_cy = 0, 0
                return 0, 0, None, False

        last_cx, last_cy = cx, cy
        track_lost = 0
        return cx, cy, bbox, True

    track_lost += 1
    if track_lost <= TRACK_LOST_MAX:
        return last_cx, last_cy, None, True
    last_cx, last_cy = 0, 0
    return 0, 0, None, False

# ============================================================
# LED 状态
# ============================================================

def led_status(state):
    LED(1).off(); LED(2).off(); LED(3).off()
    if state == 1: LED(1).on()
    elif state == 2: LED(2).on()
    elif state == 3: LED(3).on()

# ============================================================
# 主程序
# ============================================================

if __name__ == "__main__":
    print("=" * 50)
    print("BlazeFace 人脸追踪 + PD 舵机控制")
    print("相机: %dx%d | 模型: BlazeFace 128" % (CAMERA_WINDOW_W, CAMERA_WINDOW_H))
    print("=" * 50)

    led_status(1)
    time.sleep_ms(500)

    if not init_camera():
        while True:
            led_status(1); time.sleep_ms(500)

    if not init_model():
        while True:
            led_status(1); time.sleep_ms(500)

    if not init_servos():
        while True:
            led_status(1); time.sleep_ms(500)

    led_status(2)
    time.sleep_ms(1000)
    print("追踪已启动")

    clock = time.clock()
    last_report = time.time()
    loop_cnt = 0

    while True:
        clock.tick()

        # 1. 采集图像
        img = cam.snapshot()

        # 2. BlazeFace 检测
        try:
            faces = model.predict([img])
            # faces: [((x,y,w,h), score, keypoints), ...]
        except Exception as e:
            faces = []

        # 3. 目标选取 (最近脸优先 + 跳变保护 + 超时复位)
        track_cx, track_cy, track_bbox, has_target = get_target(faces)

        # 4. 舵机控制 (每帧, ~70Hz 丝滑)
        loop_cnt += 1
        if loop_cnt % CONTROL_EVERY_N == 0:
            servo_control(track_cx, track_cy, has_target)

        # 5. 画面绘制 (DRAW_ENABLE=True 时启用, 脱机=False 省性能)
        if DRAW_ENABLE and loop_cnt % 2 == 0:
            if faces:
                for bbox, score, kp in faces:
                    x, y, w, h = bbox
                    img.draw_rectangle((int(x), int(y), int(w), int(h)),
                                       color=(0, 255, 0), thickness=2)
                    draw_keypoints(img, kp, color=(255, 0, 0))

            if has_target and track_bbox is not None:
                bx, by, bw, bh = track_bbox
                img.draw_rectangle((int(bx), int(by), int(bw), int(bh)),
                                   color=(255, 0, 0), thickness=3)
                img.draw_cross((int(track_cx), int(track_cy)),
                               color=(255, 0, 0), size=12, thickness=2)
            elif not has_target:
                img.draw_cross((CAMERA_WINDOW_W // 2, CAMERA_WINDOW_H // 2),
                               color=(0, 0, 255), size=20, thickness=1)

        # LED 状态 (每 4 帧更新, 节省 I2C)
        if loop_cnt % 4 == 0:
            led_status(2 if has_target else 3)

        # 6. 周期统计 (每 2 秒)
        frame_count += 1
        if faces:
            face_count += 1

        now = time.time()
        if now - last_report >= 2.0:
            fps = clock.fps()
            detect_rate = (face_count / frame_count * 100) if frame_count > 0 else 0
            print("[%.1ffps | 命中 %d/%d帧 (%.0f%%) | %s]" % (
                fps, face_count, frame_count, detect_rate,
                "锁定" if has_target else "丢失"))
            frame_count = 0
            face_count = 0
            last_report = now
            gc.collect()
