# OpenMV N6 — CrowdHuman head+person 双类追踪固件（二自由度云台单板方案）
# 视觉检测 + SORT 多目标追踪 + PD控制器 + 舵机PWM直驱
# 模型: YOLOv11n int8, 后处理器兼容 YoloV8
# 舵机: P4 (TIM2_CH3, Base/Yaw), P5 (TIM2_CH4, Arm/Pitch)

import csi, time, ml
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

HEAD_THRESHOLD   = 0.20
PERSON_THRESHOLD = 0.35
HEAD_MIN_AREA    = 50
HEAD_MAX_AREA    = 30000
PERSON_MIN_AREA  = 1000

# ============================================================
# 舵机 PWM 参数
# ============================================================

SERVO_BASE_PIN = "P4"
SERVO_ARM_PIN  = "P5"
SERVO_FREQ = 50

SERVO_BASE_NEUTRAL = 1500000
SERVO_BASE_MIN_NS  = 500000
SERVO_BASE_MAX_NS  = 2500000
SERVO_ARM_NEUTRAL  = 1500000
SERVO_ARM_MIN_NS   = 600000
SERVO_ARM_MAX_NS   = 2400000

# ============================================================
# PD 控制参数 (移植自 STM32 control.h)
# ============================================================

KP = 0.05                        # 两轴共用 PD 参数
KD = 0.01
PD_EMA_ALPHA = 0.1

PD_INNER_DEADZONE = 5
PD_OUTER_DEADZONE = 15

MAX_VELOCITY_BASE_NS = 20.0
MAX_VELOCITY_ARM_NS  = 20.0

CENTER_X = 128.0
CENTER_Y = 128.0

# ============================================================
# SORT 多目标追踪参数
# ============================================================

SORT_MAX_LOST = 60                   # 延长 track 存活时间 (~3.6s@16fps)
SORT_IOU_MIN  = 0.15                 # head bbox 小, IoU 需放宽
SORT_VELOCITY_SMOOTH = 0.7

# ============================================================
# Re-ID 颜色直方图参数
# ============================================================

REID_HIST_BINS = 64           # 4x4x4 RGB 量化
REID_DIST_THRESHOLD = 0.50    # 直方图 L1 距离阈值, 越小越严格
REID_SAMPLE_STEP = 8          # 像素采样步长 (加大减少 CPU 开销)
REID_MAX_LOST_MEMORY = 120    # 保存已丢失 track 直方图的最大帧数
REID_EVERY_N_FRAMES = 4       # 每 N 帧提取一次直方图
REID_DEBUG = False              # 调试打印开关

# ============================================================
# 舵机回中参数
# ============================================================

RETURN_HOLD_TIME_MS = 2000    # 目标丢失后保持当前位置的时间 (ms)
RETURN_SPEED_NS_PER_MS = 0.3  # 回中速度 (ns/ms)

# ============================================================
# 全局状态
# ============================================================

cam = None
model = None
pwm_base = None
pwm_arm = None
total_frame_count = 0            # 单调递增, 永不重置 (Re-ID 用)
frame_count = 0                  # 5秒统计窗口内帧数
detect_count = 0

stat_head = 0
stat_person = 0

pos_base = SERVO_BASE_NEUTRAL
pos_arm  = SERVO_ARM_NEUTRAL

last_error_x = 0.0
last_error_y = 0.0
yaw_deriv = 0.0
pitch_deriv = 0.0
last_control_tick = 0

sort_tracker = None
selected_track_id = None
sort_tracks = {}
detection_targets = []           # SORT 追踪目标 (head bbox)
lost_histograms = {}
target_lost_tick = 0

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
    print("摄像头初始化成功 — %dx%d" % (CAMERA_WINDOW_W, CAMERA_WINDOW_H))
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
        return False

def init_servos():
    global pwm_base, pwm_arm
    try:
        pwm_base = PWM(Pin(SERVO_BASE_PIN), freq=SERVO_FREQ)
        pwm_arm  = PWM(Pin(SERVO_ARM_PIN), freq=SERVO_FREQ)
        pwm_base.duty_ns(SERVO_BASE_NEUTRAL)
        pwm_arm.duty_ns(SERVO_ARM_NEUTRAL)
        print("舵机初始化成功 — Base:%s Arm:%s" % (SERVO_BASE_PIN, SERVO_ARM_PIN))
        return True
    except Exception as e:
        print("舵机初始化失败:", e)
        return False

# ============================================================
# 辅助函数
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

def _clamp(value, min_val, max_val):
    if value < min_val: return min_val
    if value > max_val: return max_val
    return value

# ============================================================
# Re-ID 颜色直方图
# ============================================================

def _extract_histogram(img, bbox):
    """从图像 bbox 区域采样提取 64-bin RGB 直方图。"""
    x, y, w, h = int(bbox[0]), int(bbox[1]), int(bbox[2]), int(bbox[3])
    x = max(0, x); y = max(0, y)
    w = min(w, CAMERA_WINDOW_W - x); h = min(h, CAMERA_WINDOW_H - y)
    if w < 4 or h < 4:
        return None

    hist = [0] * REID_HIST_BINS
    # 只采样上半身 (top 40%), 减少背景干扰
    sample_h = max(4, int(h * 0.4))
    count = 0
    for sy in range(y, y + sample_h, REID_SAMPLE_STEP):
        for sx in range(x, x + w, REID_SAMPLE_STEP):
            p = img.get_pixel((sx, sy))  # 返回 (R, G, B) 元组
            # 各分量量化为 4 级 (0-3)
            r = p[0] >> 6    # 0-255 → 0-3
            g = p[1] >> 6
            b = p[2] >> 6
            idx = (r << 4) | (g << 2) | b  # 4x4x4 = 64
            hist[idx] += 1
            count += 1

    if count > 0:
        for i in range(REID_HIST_BINS):
            hist[i] = hist[i] / count
    return hist


def _histogram_distance(h1, h2):
    """L1 距离 (0~2 之间, 越小越相似)。"""
    if h1 is None or h2 is None:
        return 2.0
    d = 0.0
    for i in range(REID_HIST_BINS):
        d += abs(h1[i] - h2[i])
    return d

# ============================================================
# SORT 多目标追踪器
# ============================================================

class SortTracker:

    def __init__(self, max_lost=30, iou_min=0.3, vel_smooth=0.7):
        self.tracks = {}
        self.next_id = 0
        self.max_lost = max_lost
        self.iou_min = iou_min
        self.vel_smooth = vel_smooth

    def _iou(self, a, b):
        ax1, ay1 = a[0], a[1]
        ax2, ay2 = a[0] + a[2], a[1] + a[3]
        bx1, by1 = b[0], b[1]
        bx2, by2 = b[0] + b[2], b[1] + b[3]
        ix1 = ax1 if ax1 > bx1 else bx1
        iy1 = ay1 if ay1 > by1 else by1
        ix2 = ax2 if ax2 < bx2 else bx2
        iy2 = ay2 if ay2 < by2 else by2
        if ix2 <= ix1 or iy2 <= iy1:
            return 0.0
        inter = (ix2 - ix1) * (iy2 - iy1)
        union = a[2] * a[3] + b[2] * b[3] - inter
        return inter / union if union > 0 else 0.0

    def update(self, detections, img=None, do_reid=True):
        """更新追踪器。do_reid=False 时跳过直方图提取以节省 CPU。"""

        # 无 track 时重置 ID 计数器
        if not self.tracks:
            self.next_id = 0

        predicted = {}
        for tid, t in self.tracks.items():
            px = t["bbox"][0] + t["vel"][0]
            py = t["bbox"][1] + t["vel"][1]
            pw = t["bbox"][2] + t["vel"][2]
            ph = t["bbox"][3] + t["vel"][3]
            predicted[tid] = (px, py, pw if pw > 10 else 10, ph if ph > 10 else 10)

        det_ids = list(range(len(detections)))
        trk_ids = list(predicted.keys())

        # 1) IoU 匹配
        ious = {}
        for di in det_ids:
            for ti in trk_ids:
                iou = self._iou(detections[di], predicted[ti])
                if iou >= self.iou_min:
                    ious[(di, ti)] = iou

        matched_dets = set()
        matched_trks = set()
        assignments = []
        for (di, ti), iou in sorted(ious.items(), key=lambda x: x[1], reverse=True):
            if di not in matched_dets and ti not in matched_trks:
                assignments.append((di, ti))
                matched_dets.add(di)
                matched_trks.add(ti)

        # 2) 中心距离回退匹配 (head bbox 小, IoU 不可靠)
        for di in det_ids:
            if di in matched_dets:
                continue
            dx, dy = detections[di][0] + detections[di][2] // 2, detections[di][1] + detections[di][3] // 2
            best_ti, best_dist = None, 30  # 30px 中心距离阈值
            for ti in trk_ids:
                if ti in matched_trks:
                    continue
                px, py = predicted[ti][0] + predicted[ti][2] // 2, predicted[ti][1] + predicted[ti][3] // 2
                dist = abs(dx - px) + abs(dy - py)
                if dist < best_dist:
                    best_dist = dist
                    best_ti = ti
            if best_ti is not None:
                assignments.append((di, best_ti))
                matched_dets.add(di)
                matched_trks.add(best_ti)

        for di, ti in assignments:
            det = detections[di]
            t = self.tracks[ti]
            old_bbox = t["bbox"]
            t["vel"][0] = self.vel_smooth * t["vel"][0] + (1 - self.vel_smooth) * (det[0] - old_bbox[0])
            t["vel"][1] = self.vel_smooth * t["vel"][1] + (1 - self.vel_smooth) * (det[1] - old_bbox[1])
            t["vel"][2] = self.vel_smooth * t["vel"][2] + (1 - self.vel_smooth) * (det[2] - old_bbox[2])
            t["vel"][3] = self.vel_smooth * t["vel"][3] + (1 - self.vel_smooth) * (det[3] - old_bbox[3])
            t["bbox"] = list(det)
            t["lost"] = 0
            # 更新直方图 (EMA), 仅每 N 帧执行
            if do_reid and img is not None:
                h = _extract_histogram(img, det)
                if h is not None:
                    if "hist" in t and t["hist"] is not None:
                        for i in range(REID_HIST_BINS):
                            t["hist"][i] = 0.8 * t["hist"][i] + 0.2 * h[i]
                    else:
                        t["hist"] = h

        # 新 track: 仅在 do_reid 且有丢失记录时提取直方图做 Re-ID
        if do_reid:
            for lost_id in list(lost_histograms.keys()):
                if total_frame_count - lost_histograms[lost_id][1] > REID_MAX_LOST_MEMORY:
                    del lost_histograms[lost_id]

        for di in det_ids:
            if di not in matched_dets:
                det = detections[di]
                new_hist = None
                if do_reid and lost_histograms and img is not None:
                    new_hist = _extract_histogram(img, det)
                reused_id = None

                # Re-ID: 与已丢失 track 的直方图比较
                if new_hist is not None and lost_histograms:
                    best_dist = REID_DIST_THRESHOLD
                    for lost_id, entry in list(lost_histograms.items()):
                        if not isinstance(entry, tuple) or len(entry) != 2:
                            continue
                        lost_hist, lost_frame = entry
                        if total_frame_count - lost_frame > REID_MAX_LOST_MEMORY:
                            del lost_histograms[lost_id]
                            continue
                        d = _histogram_distance(new_hist, lost_hist)
                        if d < best_dist:
                            best_dist = d
                            reused_id = lost_id

                tid = reused_id if reused_id is not None else self.next_id
                if reused_id is not None:
                    if REID_DEBUG: print("[ReID] matched lost track %d (dist=%.3f)" % (reused_id, best_dist))
                    del lost_histograms[reused_id]
                else:
                    self.next_id += 1
                t = {"bbox": list(det), "vel": [0.0, 0.0, 0.0, 0.0], "lost": 0}
                if new_hist is not None:
                    t["hist"] = new_hist
                self.tracks[tid] = t

        dead_ids = []
        for ti in trk_ids:
            if ti not in matched_trks:
                t = self.tracks[ti]
                t["lost"] += 1
                if t["lost"] > self.max_lost:
                    dead_ids.append(ti)

        for tid in dead_ids:
            t = self.tracks[tid]
            if "hist" in t and t["hist"] is not None:
                lost_histograms[tid] = (t["hist"], total_frame_count)
                if REID_DEBUG: print("[ReID] saved hist for track", tid)
            del self.tracks[tid]

        return self.tracks

# ============================================================
# 检测主函数
# ============================================================

def detect_person(img):
    global total_frame_count, frame_count, detect_count, stat_head, stat_person
    global detection_targets
    total_frame_count += 1
    frame_count += 1

    detection_targets = []  # 始终清空, 防止 SORT 收到过期数据

    try:
        boxes = model.predict([img])
    except Exception:
        return CAMERA_WINDOW_W // 2, CAMERA_WINDOW_H // 2, False

    if not boxes or not boxes[0]:
        return CAMERA_WINDOW_W // 2, CAMERA_WINDOW_H // 2, False

    # 筛选 head 检测
    best_cx, best_cy = CAMERA_WINDOW_W // 2, CAMERA_WINDOW_H // 2
    best_area = 0
    has_any = False

    for cls_id, class_boxes in enumerate(boxes):
        for d in class_boxes:
            x, y, w, h, score, raw_cls = _unpack(d)
            if raw_cls == 0 and cls_id > 0:
                cls = cls_id
            else:
                cls = raw_cls
            area = w * h
            label = model.labels[int(cls)] if int(cls) < len(model.labels) else ""
            if label == "head":
                if score >= HEAD_THRESHOLD and HEAD_MIN_AREA <= area <= HEAD_MAX_AREA:
                    detection_targets.append((x, y, w, h))
                    stat_head += 1
                    has_any = True
                    if area > best_area:
                        best_area = area
                        best_cx, best_cy = x + w // 2, y + h // 2
            elif label == "person":
                if score >= PERSON_THRESHOLD and area >= PERSON_MIN_AREA:
                    stat_person += 1

    if has_any:
        detect_count += 1
        return best_cx, best_cy, True
    return CAMERA_WINDOW_W // 2, CAMERA_WINDOW_H // 2, False

# ============================================================
# PD 舵机控制器 (移植自 STM32 control.c)
# ============================================================

def servo_control(cx, cy, has_target):
    global pos_base, pos_arm
    global last_error_x, last_error_y, yaw_deriv, pitch_deriv
    global last_control_tick, target_lost_tick

    now_us = time.ticks_us()
    dt_ms = 0.0
    if last_control_tick != 0:
        dt_ticks = time.ticks_diff(now_us, last_control_tick)
        dt_ms = dt_ticks / 1000.0
    last_control_tick = now_us

    if dt_ms <= 0 or dt_ms > 100.0:
        dt_ms = 33.0

    if not has_target:
        # 回中策略: 先保持 N ms, 然后缓慢滑回中位
        if target_lost_tick == 0:
            target_lost_tick = now_us

        lost_ms = time.ticks_diff(now_us, target_lost_tick) / 1000.0

        last_error_x = 0.0; last_error_y = 0.0
        yaw_deriv = 0.0; pitch_deriv = 0.0

        if lost_ms < RETURN_HOLD_TIME_MS:
            return  # 保持当前位置

        # 滑回中位
        step = RETURN_SPEED_NS_PER_MS * dt_ms
        if pos_base > SERVO_BASE_NEUTRAL:
            pos_base = max(SERVO_BASE_NEUTRAL, pos_base - step)
        else:
            pos_base = min(SERVO_BASE_NEUTRAL, pos_base + step)
        if pos_arm > SERVO_ARM_NEUTRAL:
            pos_arm = max(SERVO_ARM_NEUTRAL, pos_arm - step)
        else:
            pos_arm = min(SERVO_ARM_NEUTRAL, pos_arm + step)

        pwm_base.duty_ns(int(pos_base))
        pwm_arm.duty_ns(int(pos_arm))
        return

    # 有目标: 重置丢失计时
    target_lost_tick = 0

    tx = int(round((cx / CAMERA_WINDOW_W) * 255))
    ty = int(round((cy / CAMERA_WINDOW_H) * 255))
    raw_dx = float(tx) - CENTER_X
    raw_dy = float(ty) - CENTER_Y

    abs_dx = abs(raw_dx)
    abs_dy = abs(raw_dy)

    scale_x = 1.0
    if abs_dx <= PD_INNER_DEADZONE:
        scale_x = 0.0
    elif abs_dx < PD_OUTER_DEADZONE:
        t = (abs_dx - PD_INNER_DEADZONE) / (PD_OUTER_DEADZONE - PD_INNER_DEADZONE)
        scale_x = t * t

    scale_y = 1.0
    if abs_dy <= PD_INNER_DEADZONE:
        scale_y = 0.0
    elif abs_dy < PD_OUTER_DEADZONE:
        t = (abs_dy - PD_INNER_DEADZONE) / (PD_OUTER_DEADZONE - PD_INNER_DEADZONE)
        scale_y = t * t

    new_error_x = (raw_dx / 255.0) * scale_x
    new_error_y = (raw_dy / 255.0) * scale_y

    alpha = PD_EMA_ALPHA
    yaw_deriv = (1 - alpha) * yaw_deriv + alpha * ((new_error_x - last_error_x) / (dt_ms / 1000.0))
    pitch_deriv = (1 - alpha) * pitch_deriv + alpha * ((new_error_y - last_error_y) / (dt_ms / 1000.0))

    yaw_out = KP * new_error_x + KD * yaw_deriv
    pitch_out = KP * new_error_y + KD * pitch_deriv

    vel_scale = 500.0 * dt_ms / 10.0
    yaw_vel = yaw_out * vel_scale
    pitch_vel = pitch_out * vel_scale

    max_yaw = MAX_VELOCITY_BASE_NS * dt_ms / 10.0
    max_pitch = MAX_VELOCITY_ARM_NS * dt_ms / 10.0
    yaw_vel = _clamp(yaw_vel, -max_yaw, max_yaw)
    pitch_vel = _clamp(pitch_vel, -max_pitch, max_pitch)

    pos_base += yaw_vel
    pos_arm  += pitch_vel
    pos_base = _clamp(pos_base, SERVO_BASE_MIN_NS, SERVO_BASE_MAX_NS)
    pos_arm  = _clamp(pos_arm,  SERVO_ARM_MIN_NS,  SERVO_ARM_MAX_NS)

    pwm_base.duty_ns(int(pos_base))
    pwm_arm.duty_ns(int(pos_arm))

    last_error_x = new_error_x
    last_error_y = new_error_y

# ============================================================
# 目标选择
# ============================================================

def select_servo_target():
    global sort_tracks, selected_track_id

    if not sort_tracks:
        return CAMERA_WINDOW_W // 2, CAMERA_WINDOW_H // 2, None, False

    if selected_track_id is not None and selected_track_id in sort_tracks:
        t = sort_tracks[selected_track_id]
        if t["lost"] <= 5:
            b = t["bbox"]
            return int(b[0] + b[2] // 2), int(b[1] + b[3] // 2), selected_track_id, True

    # 自动锁最小 ID 的活跃 track
    best_id = None
    for tid in sorted(sort_tracks.keys()):
        if sort_tracks[tid]["lost"] == 0:
            best_id = tid
            break

    if best_id is not None:
        t = sort_tracks[best_id]
        b = t["bbox"]
        return int(b[0] + b[2] // 2), int(b[1] + b[3] // 2), best_id, True

    return CAMERA_WINDOW_W // 2, CAMERA_WINDOW_H // 2, None, False

# ============================================================
# LED 状态
# ============================================================

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
    print("OpenMV N6 — YOLOv11n CrowdHuman + SORT + PD 舵机 v8.0")
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

    sort_tracker = SortTracker(max_lost=SORT_MAX_LOST, iou_min=SORT_IOU_MIN, vel_smooth=SORT_VELOCITY_SMOOTH)

    print("追踪已启动")
    print("  模型: %s" % MODEL_PATH)
    print("  KP=%.3f KD=%.3f 死区=%d-%dpx" % (KP, KD, PD_INNER_DEADZONE, PD_OUTER_DEADZONE))

    clock = time.clock()
    last_report = time.time()

    while True:
        clock.tick()
        img = cam.snapshot()

        cx, cy, has_target = detect_person(img)

        sort_tracks = sort_tracker.update(detection_targets, img,
            do_reid=(frame_count % REID_EVERY_N_FRAMES == 0))

        # 按 head 面积从大到小重排显示 ID: 最大=0, 次大=1, ...
        display_id = {}
        sorted_tracks = sorted(sort_tracks.items(),
            key=lambda x: x[1]["bbox"][2] * x[1]["bbox"][3], reverse=True)
        for new_id, (orig_id, t) in enumerate(sorted_tracks):
            display_id[orig_id] = new_id

        # 始终锁定 ID 0 (面积最大的 head)
        has_track = False
        for orig_id, t in sorted_tracks:
            if t["lost"] == 0:
                selected_track_id = orig_id
                b = t["bbox"]
                target_cx, target_cy = int(b[0] + b[2] // 2), int(b[1] + b[3] // 2)
                has_track = True
                break
        if not has_track:
            selected_track_id = None

        # 绘制: 锁定目标红框, 其余绿框, 使用重排后的显示 ID
        for orig_id, t in sort_tracks.items():
            if t["lost"] == 0:
                bx, by, bw, bh = t["bbox"]
                is_sel = (selected_track_id is not None and orig_id == selected_track_id)
                color = (255, 0, 0) if is_sel else (0, 255, 0)
                img.draw_rectangle((int(bx), int(by), int(bw), int(bh)), color=color, thickness=2)
                img.draw_string((int(bx), int(by) - 10), "ID:%d" % display_id.get(orig_id, 0),
                    color=color, scale=2)

        # 舵机驱动: 优先 SORT track, 否则用 detect_person 单帧输出
        if has_track:
            pass  # target_cx/cy already set above
        elif has_target:
            target_cx, target_cy = cx, cy
        else:
            target_cx, target_cy = CAMERA_WINDOW_W // 2, CAMERA_WINDOW_H // 2

        servo_control(target_cx, target_cy, has_target or has_track)

        set_led_status(2 if (has_target or has_track) else 3)

        now = time.time()
        if now - last_report >= 5.0:
            fps = clock.fps()
            rate = (detect_count / frame_count * 100) if frame_count > 0 else 0
            track_count = 0
            for t in sort_tracks.values():
                if t["lost"] == 0:
                    track_count += 1
            print("5s: 帧=%d 命中=%d 命中率=%.1f%% fps=%.1f tracks=%d head=%d person=%d" % (
                frame_count, detect_count, rate, fps, track_count, stat_head, stat_person))
            frame_count = 0
            detect_count = 0
            stat_head = 0
            stat_person = 0
            last_report = now
