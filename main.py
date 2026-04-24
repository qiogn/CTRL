import gc
import math
import time

import aicube
import image
import nncase_runtime as nn
import ujson
import ulab.numpy as np
from libs.PipeLine import ScopedTiming
from libs.Utils import *
from media.display import *
from media.media import *
from media.sensor import *

# ========================= 串口配置 =========================
USE_UART = True
uart = None

if USE_UART:
    from machine import UART, FPIOA
    fpioa = FPIOA()

    # 庐山派 K230:
    # GPIO03 -> UART1_TXD -> 排针 Pin8
    # GPIO04 -> UART1_RXD -> 排针 Pin10
    fpioa.set_function(3, FPIOA.UART1_TXD)
    fpioa.set_function(4, FPIOA.UART1_RXD)
    uart = UART(UART.UART1, 115200)

UART_FUNC_POINT = 0xF2
UART_FUNC_LOST  = 0xF3
UART_FUNC_DRAW_READY = 0xE1  # K230 → STM32: notify circle mode
UART_FUNC_DRAW_POINT = 0xE2  # K230 → STM32: trajectory point

CIRCLE_POINTS = 256
CIRCLE_LOCK_FRAMES = 10

has_ever_locked = False
lost_reported = False

# ========================= 基本配置 =========================
display_mode = "lcd"   # "lcd" or "hdmi"
debug_mode = 0

if display_mode == "lcd":
    DISPLAY_WIDTH = ALIGN_UP(800, 16)
    DISPLAY_HEIGHT = 480
else:
    DISPLAY_WIDTH = ALIGN_UP(1920, 16)
    DISPLAY_HEIGHT = 1080

OUT_RGB888P_WIDTH = ALIGN_UP(640, 16)
OUT_RGB888P_HEIGH = 360

root_path = "/sdcard/mp_deployment_source/"
config_path = root_path + "deploy_config.json"

# ========================= lock 参数 =========================
LOCK_IOU_TH = 0.40
LOCK_SCORE_BONUS = 0.35
LOST_MAX = 6
GC_INTERVAL = 20
CROSS_SIZE = 12

# ========================= Kalman 参数 =========================
KF_ENABLE = True
KF_Q_POS = 0.50
KF_Q_VEL = 0.20
KF_R_BOX = 20
KF_GATE_DIST2 = 200 * 200
KF_VEL_DAMP = 0.80

# ========================= 固定中心参数 =========================
CENTER_LOCK_ENABLE = True
CENTER_DEAD_BAND = 16
CENTER_MAX_JUMP = 20
CENTER_SWITCH_CONFIRM = 3
AIM_OFFSET_X = 0
AIM_OFFSET_Y = 0
CENTER_SIZE_GATE = 0.30
AIM_OUTPUT_MAX_STEP_X = 10
AIM_OUTPUT_MAX_STEP_Y = 8
LOCKED_FILTER_GATE = 60
LOCKED_AIM_HOLD_BAND = 4
LOCKED_AIM_BLEND_BAND = 15

# ========================= 边缘精修参数 =========================
REFINE_ENABLE = False  # 开启后在 YOLO 框及其外扩区域内用阈值法精修白色矩形

# Tracking constants used by pick_best_det
TRACK_MATCH_MAX_DIST2 = 150 * 150
TRACK_REACQUIRE_SCORE_TH = 0.85

# ========================= 串口发送节流 =========================
UART_SEND_INTERVAL_MS = 15
NO_TARGET_SEND_CENTER = False
uart_last_send_ms = 0
UART_DEBUG_PRINT = False  # 串口打印坐标偏移调试信息（开启会降低FPS）
# ========================= 工具函数 =========================
def two_side_pad_param(input_size, output_size):
    ratio_w = output_size[0] / input_size[0]
    ratio_h = output_size[1] / input_size[1]
    ratio = min(ratio_w, ratio_h)

    new_w = int(ratio * input_size[0])
    new_h = int(ratio * input_size[1])

    dw = (output_size[0] - new_w) / 2
    dh = (output_size[1] - new_h) / 2

    top = int(round(dh - 0.1))
    bottom = int(round(dh + 0.1))
    left = int(round(dw - 0.1))
    right = int(round(dw + 0.1))
    return top, bottom, left, right, ratio


def read_deploy_config(path):
    with open(path, "r") as json_file:
        return ujson.load(json_file)


def clamp(v, vmin, vmax):
    if v < vmin:
        return vmin
    if v > vmax:
        return vmax
    return v


def abs_i(v):
    return -v if v < 0 else v


def stabilize_locked_aim(prev_center, target_x, target_y):
    if prev_center is None:
        return (target_x, target_y)

    dx = target_x - prev_center[0]
    dy = target_y - prev_center[1]
    dist2 = dx * dx + dy * dy

    if abs_i(dx) <= LOCKED_AIM_HOLD_BAND and abs_i(dy) <= LOCKED_AIM_HOLD_BAND:
        return prev_center

    if dist2 <= (LOCKED_AIM_BLEND_BAND * LOCKED_AIM_BLEND_BAND):
        return (
            int((prev_center[0] * 2 + target_x) / 3),
            int((prev_center[1] * 2 + target_y) / 3),
        )

    return (target_x, target_y)


def choose_locked_center(box_cx, box_cy, filt_cx, filt_cy):
    if center_dist2((box_cx, box_cy), (filt_cx, filt_cy)) <= (LOCKED_FILTER_GATE * LOCKED_FILTER_GATE):
        return filt_cx, filt_cy, "KF"
    return box_cx, box_cy, "BOX"


def calc_iou_xyxy(a, b):
    ax1, ay1, ax2, ay2 = a
    bx1, by1, bx2, by2 = b

    inter_x1 = ax1 if ax1 > bx1 else bx1
    inter_y1 = ay1 if ay1 > by1 else by1
    inter_x2 = ax2 if ax2 < bx2 else bx2
    inter_y2 = ay2 if ay2 < by2 else by2

    iw = inter_x2 - inter_x1
    ih = inter_y2 - inter_y1
    if iw <= 0 or ih <= 0:
        return 0.0

    inter = iw * ih
    area_a = (ax2 - ax1) * (ay2 - ay1)
    area_b = (bx2 - bx1) * (by2 - by1)
    union = area_a + area_b - inter
    if union <= 0:
        return 0.0

    return inter / union


def box_center_xyxy(box):
    return ((box[0] + box[2]) / 2.0, (box[1] + box[3]) / 2.0)


def det_to_xyxy(det):
    return [float(det[2]), float(det[3]), float(det[4]), float(det[5])]


def pick_best_det(det_boxes, last_box_xyxy):
    """
    det_boxe 格式:
    [class_id, score, x1, y1, x2, y2]
    """
    if not det_boxes:
        return None

    best_det = None
    best_value = -999.0

    for det in det_boxes:
        score = float(det[1])
        x1 = float(det[2])
        y1 = float(det[3])
        x2 = float(det[4])
        y2 = float(det[5])

        value = score

        if last_box_xyxy is not None:
            cand_box = [x1, y1, x2, y2]
            iou = calc_iou_xyxy(cand_box, last_box_xyxy)
            cand_cx, cand_cy = box_center_xyxy(cand_box)
            last_cx, last_cy = box_center_xyxy(last_box_xyxy)
            dx = cand_cx - last_cx
            dy = cand_cy - last_cy
            dist2 = dx * dx + dy * dy

            if iou >= LOCK_IOU_TH:
                value += LOCK_SCORE_BONUS * iou
            elif dist2 <= TRACK_MATCH_MAX_DIST2:
                value += 0.18 * (1.0 - dist2 / float(TRACK_MATCH_MAX_DIST2))
            elif score < TRACK_REACQUIRE_SCORE_TH:
                value -= 1.5

        if value > best_value:
            best_value = value
            best_det = det

    if best_det is not None and last_box_xyxy is not None:
        score = float(best_det[1])
        best_box = det_to_xyxy(best_det)
        iou = calc_iou_xyxy(best_box, last_box_xyxy)
        best_cx, best_cy = box_center_xyxy(best_box)
        last_cx, last_cy = box_center_xyxy(last_box_xyxy)
        dx = best_cx - last_cx
        dy = best_cy - last_cy
        dist2 = dx * dx + dy * dy

        if iou < LOCK_IOU_TH and dist2 > TRACK_MATCH_MAX_DIST2 and score < TRACK_REACQUIRE_SCORE_TH:
            return None

    return best_det


def draw_cross(img, cx, cy, size, color, thickness=2):
    img.draw_line(cx - size, cy, cx + size, cy, color=color, thickness=thickness)
    img.draw_line(cx, cy - size, cx, cy + size, color=color, thickness=thickness)


def ai_to_lcd_point(x, y):
    return (
        int(x * DISPLAY_WIDTH // OUT_RGB888P_WIDTH),
        int(y * DISPLAY_HEIGHT // OUT_RGB888P_HEIGH),
    )


def safe_label(labels, cls_id):
    if cls_id >= 0 and cls_id < len(labels):
        return labels[cls_id]
    return "target"


def uart_send_packet(func_code, payload_bytes):
    global uart_last_send_ms
    if (not USE_UART) or (uart is None):
        return

    now_ms = time.ticks_ms()
    if time.ticks_diff(now_ms, uart_last_send_ms) < UART_SEND_INTERVAL_MS:
        return

    length = len(payload_bytes)
    pkt = bytearray([0xAA, 0xFF, func_code, length] + payload_bytes + [0, 0])

    sc = 0
    ac = 0
    for i in range(4 + length):
        sc ^= pkt[i]
        ac = (ac + pkt[i]) & 0xFF

    pkt[-2] = sc
    pkt[-1] = ac
    uart.write(pkt)
    uart_last_send_ms = now_ms


def uart_send_point(x, y):
    dx = max(min(int(round(x)), OUT_RGB888P_WIDTH), 0)
    dy = max(min(int(round(y)), OUT_RGB888P_HEIGH), 0)
    payload = [dx & 0xFF, (dx >> 8) & 0xFF, dy & 0xFF, (dy >> 8) & 0xFF]
    uart_send_packet(UART_FUNC_POINT, payload)


def uart_send_lost():
    uart_send_packet(UART_FUNC_LOST, [])

def uart_send_draw_ready():
    uart_send_packet(UART_FUNC_DRAW_READY, [])

def uart_send_draw_point(x, y):
    dx = max(min(int(round(x)), OUT_RGB888P_WIDTH), 0)
    dy = max(min(int(round(y)), OUT_RGB888P_HEIGH), 0)
    payload = [dx & 0xFF, (dx >> 8) & 0xFF, dy & 0xFF, (dy >> 8) & 0xFF]
    uart_send_packet(UART_FUNC_DRAW_POINT, payload)

def generate_circle_points(center_x, center_y, radius):
    points = []
    for i in range(CIRCLE_POINTS):
        angle = 2.0 * math.pi * i / CIRCLE_POINTS
        px = center_x + radius * math.cos(angle)
        py = center_y - radius * math.sin(angle)  # Y向下
        points.append((int(px), int(py)))
    return points

def center_dist2(c1, c2):
    dx = c1[0] - c2[0]
    dy = c1[1] - c2[1]
    return dx * dx + dy * dy


def size_change_ratio(old_wh, new_wh):
    if old_wh is None:
        return 0.0
    ow, oh = old_wh
    nw, nh = new_wh
    if ow <= 0 or oh <= 0:
        return 0.0
    rw = abs(nw - ow) / float(ow)
    rh = abs(nh - oh) / float(oh)
    return rw if rw > rh else rh


def refine_box_edges(arr, x1, y1, x2, y2):
    """在 YOLO 框及其外扩区域内用阈值法找白色矩形。
    返回 (inner_x1, inner_y1, inner_x2, inner_y2)。
    arr 是 RGBP888 planar 格式 (3, H, W)。
    策略：外扩 ROI 30px，用固定高阈值(180)找白色区域边界。
    """
    # 外扩搜索范围，覆盖 YOLO 框外可能的白色目标
    pad = 30
    ex1 = max(0, x1 - pad)
    ey1 = max(0, y1 - pad)
    ex2 = min(OUT_RGB888P_WIDTH, x2 + pad)
    ey2 = min(OUT_RGB888P_HEIGH, y2 + pad)

    ew = ex2 - ex1
    eh = ey2 - ey1

    # 外扩区域太小则放弃
    if ew <= 8 or eh <= 8:
        return x1, y1, x2, y2

    # RGBP888 planar: (3, H, W)
    g0 = arr[0, ey1:ey2, ex1:ex2]
    g1 = arr[1, ey1:ey2, ex1:ex2]
    g2 = arr[2, ey1:ey2, ex1:ex2]
    gray = (g0 + g1 + g2) // 3

    # 固定高阈值：白色矩形区域灰度值应远高于暗色背景
    # 不用 avg+20 自适应阈值，因为 YOLO 框可能完全在暗色区域上
    threshold = 180

    # 找上边：从顶部向下找第一个灰度 >= threshold 的连续行
    found_top = False
    for row in range(eh):
        row_sum = int(np.sum(gray[row, :]))
        if row_sum >= threshold * ew:
            found_top = True
            refined_y1 = ey1 + row
            break
    if not found_top:
        return x1, y1, x2, y2

    # 找下边：从底部向上找
    found_bottom = False
    for row in range(eh - 1, -1, -1):
        row_sum = int(np.sum(gray[row, :]))
        if row_sum >= threshold * ew:
            found_bottom = True
            refined_y2 = ey1 + row + 1
            break
    if not found_bottom:
        return x1, y1, x2, y2

    # 找左边：从左向右找
    found_left = False
    for col in range(ew):
        col_sum = int(np.sum(gray[:, col]))
        if col_sum >= threshold * eh:
            found_left = True
            refined_x1 = ex1 + col
            break
    if not found_left:
        return x1, y1, x2, y2

    # 找右边：从右向左找
    found_right = False
    for col in range(ew - 1, -1, -1):
        col_sum = int(np.sum(gray[:, col]))
        if col_sum >= threshold * eh:
            found_right = True
            refined_x2 = ex1 + col + 1
            break
    if not found_right:
        return x1, y1, x2, y2

    if refined_x1 >= refined_x2 or refined_y1 >= refined_y2:
        return x1, y1, x2, y2

    return refined_x1, refined_y1, refined_x2, refined_y2


class Kalman1D:
    def __init__(self, q_pos=1.0, q_vel=0.2, r=10.0, vel_damp=1.0):
        self.q_pos = q_pos
        self.q_vel = q_vel
        self.r = r
        self.vel_damp = vel_damp
        self.inited = False

        self.x = 0.0
        self.v = 0.0

        self.p00 = 1.0
        self.p01 = 0.0
        self.p10 = 0.0
        self.p11 = 1.0

    def reset(self, z):
        self.inited = True
        self.x = float(z)
        self.v = 0.0

        self.p00 = 10.0
        self.p01 = 0.0
        self.p10 = 0.0
        self.p11 = 10.0

    def predict(self):
        if not self.inited:
            return

        self.v = self.v * self.vel_damp
        self.x = self.x + self.v

        p00 = self.p00 + self.p10 + self.p01 + self.p11 + self.q_pos
        p01 = self.p01 + self.p11
        p10 = self.p10 + self.p11
        p11 = self.p11 + self.q_vel

        self.p00 = p00
        self.p01 = p01
        self.p10 = p10
        self.p11 = p11

    def update(self, z, r_override=None):
        if not self.inited:
            self.reset(z)
            return

        r = self.r if r_override is None else r_override

        y = float(z) - self.x
        s = self.p00 + r
        if s <= 1e-6:
            return

        k0 = self.p00 / s
        k1 = self.p10 / s

        p00_old = self.p00
        p01_old = self.p01
        p10_old = self.p10
        p11_old = self.p11

        self.x = self.x + k0 * y
        self.v = self.v + k1 * y

        self.p00 = (1.0 - k0) * p00_old
        self.p01 = (1.0 - k0) * p01_old
        self.p10 = p10_old - k1 * p00_old
        self.p11 = p11_old - k1 * p01_old

    def get(self):
        return self.x



# ========================= 主逻辑 =========================
def detection():
    global has_ever_locked, lost_reported

    print("lock_light_fixed_center_uart start")

    # Circle mode state
    circle_mode = False
    circle_lock_count = 0
    circle_points = []
    circle_point_idx = 0

    deploy_conf = read_deploy_config(config_path)
    kmodel_name = deploy_conf["kmodel_path"]
    labels = deploy_conf["categories"]
    confidence_threshold = deploy_conf["confidence_threshold"]
    nms_threshold = deploy_conf["nms_threshold"]
    img_size = deploy_conf["img_size"]
    num_classes = deploy_conf["num_classes"]
    color_four = get_colors(num_classes)
    nms_option = deploy_conf["nms_option"]
    model_type = deploy_conf["model_type"]

    if model_type != "AnchorBaseDet":
        raise ValueError("This script is for AnchorBaseDet only")

    anchors = deploy_conf["anchors"][0] + deploy_conf["anchors"][1] + deploy_conf["anchors"][2]

    kmodel_frame_size = img_size
    frame_size = [OUT_RGB888P_WIDTH, OUT_RGB888P_HEIGH]
    strides = [8, 16, 32]

    top, bottom, left, right, ratio = two_side_pad_param(frame_size, kmodel_frame_size)

    print("model =", root_path + kmodel_name)
    print("img_size =", kmodel_frame_size)
    print("labels =", labels)

    # -------- KPU --------
    kpu = nn.kpu()
    kpu.load_kmodel(root_path + kmodel_name)

    # -------- AI2D --------
    ai2d = nn.ai2d()
    ai2d.set_dtype(nn.ai2d_format.NCHW_FMT, nn.ai2d_format.NCHW_FMT, np.uint8, np.uint8)
    ai2d.set_pad_param(True, [0, 0, 0, 0, top, bottom, left, right], 0, [114, 114, 114])
    ai2d.set_resize_param(True, nn.interp_method.tf_bilinear, nn.interp_mode.half_pixel)
    ai2d_builder = ai2d.build(
        [1, 3, OUT_RGB888P_HEIGH, OUT_RGB888P_WIDTH],
        [1, 3, kmodel_frame_size[1], kmodel_frame_size[0]]
    )

    # -------- Sensor / Display --------
    sensor = Sensor()
    sensor.reset()
    sensor.set_hmirror(False)
    sensor.set_vflip(False)

    sensor.set_framesize(width=DISPLAY_WIDTH, height=DISPLAY_HEIGHT)
    sensor.set_pixformat(PIXEL_FORMAT_YUV_SEMIPLANAR_420)

    sensor.set_framesize(width=OUT_RGB888P_WIDTH, height=OUT_RGB888P_HEIGH, chn=CAM_CHN_ID_2)
    sensor.set_pixformat(PIXEL_FORMAT_RGB_888_PLANAR, chn=CAM_CHN_ID_2)

    sensor_bind_info = sensor.bind_info(x=0, y=0, chn=CAM_CHN_ID_0)
    Display.bind_layer(**sensor_bind_info, layer=Display.LAYER_VIDEO1)

    if display_mode == "lcd":
        Display.init(Display.ST7701, to_ide=False)
    else:
        Display.init(Display.LT9611, to_ide=False)

    osd_img = image.Image(DISPLAY_WIDTH, DISPLAY_HEIGHT, image.ARGB8888)

    MediaManager.init()
    sensor.run()

    data = np.ones((1, 3, kmodel_frame_size[1], kmodel_frame_size[0]), dtype=np.uint8)
    ai2d_output_tensor = nn.from_numpy(data)

    fps = 0.0
    fps_count = 0
    fps_last_ms = time.ticks_ms()

    lost_cnt = 0
    last_box_xyxy = None
    last_box_wh = None

    stable_center_ai = None
    stable_box_wh = None
    candidate_center_ai = None
    candidate_confirm_cnt = 0
    aim_center_ai = None

    gc_cnt = 0

    kf_x = Kalman1D(KF_Q_POS, KF_Q_VEL, KF_R_BOX, KF_VEL_DAMP)
    kf_y = Kalman1D(KF_Q_POS, KF_Q_VEL, KF_R_BOX, KF_VEL_DAMP)

    try:
        while True:
            with ScopedTiming("total", debug_mode > 0):
                fps_count += 1
                now_ms = time.ticks_ms()
                diff_ms = time.ticks_diff(now_ms, fps_last_ms)
                if diff_ms >= 1000:
                    fps = fps_count * 1000.0 / diff_ms
                    fps_count = 0
                    fps_last_ms = now_ms
                    print("FPS = %.2f" % fps)

                rgb888p_img = sensor.snapshot(chn=CAM_CHN_ID_2)

                if rgb888p_img and rgb888p_img.format() == image.RGBP888:
                    ai2d_input_tensor = None

                    ai2d_input = rgb888p_img.to_numpy_ref()
                    ai2d_input_tensor = nn.from_numpy(ai2d_input)

                    ai2d_builder.run(ai2d_input_tensor, ai2d_output_tensor)

                    kpu.set_input_tensor(0, ai2d_output_tensor)
                    kpu.run()

                    results = []
                    for i in range(kpu.outputs_size()):
                        out_data = kpu.get_output_tensor(i)
                        result = out_data.to_numpy()
                        result = result.reshape(
                            (result.shape[0] * result.shape[1] * result.shape[2] * result.shape[3])
                        )
                        del out_data
                        results.append(result)

                    det_boxes = aicube.anchorbasedet_post_process(
                        results[0],
                        results[1],
                        results[2],
                        kmodel_frame_size,
                        frame_size,
                        strides,
                        num_classes,
                        confidence_threshold,
                        nms_threshold,
                        anchors,
                        nms_option,
                    )

                    best_det = pick_best_det(det_boxes, last_box_xyxy)

                    osd_img.clear()
                    osd_img.draw_string_advanced(
                        20, 20, 24, "FPS: %.1f" % fps, color=(255, 255, 0)
                    )

                    if best_det is not None:
                        cls_id = int(best_det[0])
                        score = float(best_det[1])

                        x1 = int(best_det[2])
                        y1 = int(best_det[3])
                        x2 = int(best_det[4])
                        y2 = int(best_det[5])

                        x1 = clamp(x1, 0, OUT_RGB888P_WIDTH - 1)
                        y1 = clamp(y1, 0, OUT_RGB888P_HEIGH - 1)
                        x2 = clamp(x2, x1 + 1, OUT_RGB888P_WIDTH)
                        y2 = clamp(y2, y1 + 1, OUT_RGB888P_HEIGH)

                        if REFINE_ENABLE and ai2d_input is not None:
                            rx1, ry1, rx2, ry2 = refine_box_edges(ai2d_input, x1, y1, x2, y2)
                            x1, y1, x2, y2 = rx1, ry1, rx2, ry2

                        last_box_xyxy = [x1, y1, x2, y2]
                        lost_cnt = 0

                        box_cx = (x1 + x2) // 2
                        box_cy = (y1 + y2) // 2
                        box_w = x2 - x1
                        box_h = y2 - y1
                        box_wh = (box_w, box_h)

                        meas_cx = box_cx
                        meas_cy = box_cy

                        if KF_ENABLE:
                            if kf_x.inited and kf_y.inited:
                                kf_x.predict()
                                kf_y.predict()

                            if kf_x.inited and kf_y.inited:
                                pred_x = kf_x.get()
                                pred_y = kf_y.get()
                                dxg = meas_cx - pred_x
                                dyg = meas_cy - pred_y

                                if (dxg * dxg + dyg * dyg) <= KF_GATE_DIST2 or score >= 0.90:
                                    kf_x.update(meas_cx, KF_R_BOX)
                                    kf_y.update(meas_cy, KF_R_BOX)
                                else:
                                    # Fast motion should snap to the current box center
                                    # instead of holding an old prediction off-target.
                                    kf_x.reset(meas_cx)
                                    kf_y.reset(meas_cy)
                            else:
                                kf_x.reset(meas_cx)
                                kf_y.reset(meas_cy)

                            filt_cx = int(round(kf_x.get()))
                            filt_cy = int(round(kf_y.get()))
                        else:
                            filt_cx = meas_cx
                            filt_cy = meas_cy

                        filt_cx = clamp(filt_cx, 0, OUT_RGB888P_WIDTH - 1)
                        filt_cy = clamp(filt_cy, 0, OUT_RGB888P_HEIGH - 1)

                        if CENTER_LOCK_ENABLE:
                            if stable_center_ai is None:
                                stable_center_ai = (filt_cx, filt_cy)
                                stable_box_wh = box_wh
                                candidate_center_ai = None
                                candidate_confirm_cnt = 0
                            else:
                                dx = filt_cx - stable_center_ai[0]
                                dy = filt_cy - stable_center_ai[1]
                                jump = dx * dx + dy * dy
                                size_ratio = size_change_ratio(stable_box_wh, box_wh)

                                if abs_i(dx) <= CENTER_DEAD_BAND and abs_i(dy) <= CENTER_DEAD_BAND:
                                    pass
                                elif jump > (CENTER_MAX_JUMP * CENTER_MAX_JUMP) or size_ratio > CENTER_SIZE_GATE:
                                    cand = (filt_cx, filt_cy)

                                    if candidate_center_ai is None:
                                        candidate_center_ai = cand
                                        candidate_confirm_cnt = 1
                                    else:
                                        cdx = cand[0] - candidate_center_ai[0]
                                        cdy = cand[1] - candidate_center_ai[1]

                                        if abs_i(cdx) <= CENTER_DEAD_BAND and abs_i(cdy) <= CENTER_DEAD_BAND:
                                            candidate_confirm_cnt += 1
                                        else:
                                            candidate_center_ai = cand
                                            candidate_confirm_cnt = 1

                                    if candidate_confirm_cnt >= CENTER_SWITCH_CONFIRM:
                                        stable_center_ai = candidate_center_ai
                                        stable_box_wh = box_wh
                                        candidate_center_ai = None
                                        candidate_confirm_cnt = 0
                                else:
                                    sx, sy = stable_center_ai
                                    stable_center_ai = (
                                        int((sx * 3 + filt_cx) / 4),
                                        int((sy * 3 + filt_cy) / 4)
                                    )
                                    stable_box_wh = box_wh
                                    candidate_center_ai = None
                                    candidate_confirm_cnt = 0
                        else:
                            stable_center_ai = (filt_cx, filt_cy)
                            stable_box_wh = box_wh

                        final_cx, final_cy, aim_source = choose_locked_center(
                            box_cx, box_cy, filt_cx, filt_cy
                        )
                        raw_aim_cx = clamp(final_cx + AIM_OFFSET_X, 0, OUT_RGB888P_WIDTH - 1)
                        raw_aim_cy = clamp(final_cy + AIM_OFFSET_Y, 0, OUT_RGB888P_HEIGH - 1)
                        aim_center_ai = stabilize_locked_aim(aim_center_ai, raw_aim_cx, raw_aim_cy)
                        aim_cx, aim_cy = aim_center_ai
                        last_box_wh = box_wh

                        if UART_DEBUG_PRINT:
                            offset_x = AIM_OFFSET_X
                            offset_y = AIM_OFFSET_Y
                            stab_dx = aim_cx - raw_aim_cx
                            stab_dy = aim_cy - raw_aim_cy
                            print("[UART] aim=(%d,%d) box=(%d,%d) kf=(%d,%d) offset=(%d,%d) stab=(%d,%d) src=%s" % (
                                aim_cx, aim_cy, box_cx, box_cy, filt_cx, filt_cy,
                                offset_x, offset_y, stab_dx, stab_dy, aim_source))

                        x_lcd = int(x1 * DISPLAY_WIDTH // OUT_RGB888P_WIDTH)
                        y_lcd = int(y1 * DISPLAY_HEIGHT // OUT_RGB888P_HEIGH)
                        w_lcd = int((x2 - x1) * DISPLAY_WIDTH // OUT_RGB888P_WIDTH)
                        h_lcd = int((y2 - y1) * DISPLAY_HEIGHT // OUT_RGB888P_HEIGH)

                        cx_lcd = int(aim_cx * DISPLAY_WIDTH // OUT_RGB888P_WIDTH)
                        cy_lcd = int(aim_cy * DISPLAY_HEIGHT // OUT_RGB888P_HEIGH)

                        osd_img.draw_rectangle(
                            x_lcd, y_lcd, w_lcd, h_lcd,
                            color=color_four[cls_id][1:],
                            thickness=3
                        )
                        draw_cross(osd_img, cx_lcd, cy_lcd, CROSS_SIZE, (255, 0, 255, 0), 2)

                        osd_img.draw_string_advanced(
                            20, 50, 24,
                            "LOCK %.2f" % score,
                            color=(255, 0, 255, 0)
                        )
                        osd_img.draw_string_advanced(
                            20, 80, 24,
                            "AI:(%d,%d)" % (aim_cx, aim_cy),
                            color=(255, 0, 255, 0)
                        )
                        osd_img.draw_string_advanced(
                            20, 110, 24,
                            "BOX:(%d,%d)" % (box_cx, box_cy),
                            color=(255, 255, 255, 0)
                        )
                        osd_img.draw_string_advanced(
                            20, 140, 24,
                            "%s %s" % (safe_label(labels, cls_id), aim_source),
                            color=(255, 255, 255, 0)
                        )

                        uart_send_point(aim_cx, aim_cy)
                        has_ever_locked = True
                        lost_reported = False

                        # --- Circle mode trigger: auto-enter when stable at center ---
                        if not circle_mode and has_ever_locked:
                            err_cx = aim_cx - K230_CENTER_X
                            err_cy = aim_cy - K230_CENTER_Y
                            if abs_i(err_cx) <= CENTER_DEAD_BAND and abs_i(err_cy) <= CENTER_DEAD_BAND:
                                circle_lock_count += 1
                                if circle_lock_count >= CIRCLE_LOCK_FRAMES:
                                    circle_mode = True
                                    circle_lock_count = 0
                                    circle_point_idx = 0

                                    # Circle center and radius
                                    circle_cx = float(aim_cx)
                                    circle_cy = float(aim_cy)

                                    # A4 paper ratio compensation
                                    # A4: 210×297mm (√2≈1.414), AI frame: 640×360 (16/9≈1.778)
                                    a4_comp = (16.0 / 9.0) / 1.4142
                                    radius_x = (box_w * 0.4) * a4_comp
                                    radius_y = (box_h * 0.4) / a4_comp
                                    circle_radius = max(min(radius_x, radius_y), 30.0)

                                    circle_points = generate_circle_points(circle_cx, circle_cy, circle_radius)
                                    uart_send_draw_ready()
                                    print("[CIRCLE] R=%.1f points=%d" % (circle_radius, len(circle_points)))
                            else:
                                circle_lock_count = 0
                        elif circle_mode:
                            circle_lock_count = 0

                        # --- Circle mode execution ---
                        if circle_mode and circle_points:
                            px, py = circle_points[circle_point_idx % CIRCLE_POINTS]
                            uart_send_draw_point(px, py)
                            circle_point_idx += 1

                            # Draw current trajectory point on LCD
                            px_lcd = int(px * DISPLAY_WIDTH // OUT_RGB888P_WIDTH)
                            py_lcd = int(py * DISPLAY_HEIGHT // OUT_RGB888P_HEIGH)
                            draw_cross(osd_img, px_lcd, py_lcd, CROSS_SIZE, (0, 255, 0, 0), 2)

                            osd_img.draw_string_advanced(
                                20, 170, 24, "CIRCLE MODE", color=(0, 255, 0)
                            )

                    else:
                        lost_cnt += 1
                        last_box_xyxy = None
                        last_box_wh = None

                        stable_center_ai = None
                        stable_box_wh = None
                        candidate_center_ai = None
                        candidate_confirm_cnt = 0
                        aim_center_ai = None

                        kf_x.inited = False
                        kf_y.inited = False

                        # Reset circle mode on target loss
                        circle_mode = False
                        circle_lock_count = 0

                        osd_img.draw_string_advanced(
                            20, 50, 24,
                            "NO TARGET",
                            color=(255, 0, 0)
                        )
                        if has_ever_locked and (not lost_reported) and (lost_cnt >= LOST_MAX):
                            uart_send_lost()
                            lost_reported = True
                            if UART_DEBUG_PRINT:
                                print("[UART] LOST reported")

                    Display.show_image(osd_img, 0, 0, Display.LAYER_OSD3)

                    if ai2d_input_tensor is not None:
                        del ai2d_input_tensor

                    gc_cnt += 1
                    if gc_cnt >= GC_INTERVAL:
                        gc.collect()
                        gc_cnt = 0

                rgb888p_img = None

    except KeyboardInterrupt:
        print("stop by keyboard")
    except BaseException as e:
        print("ERROR:", e)
        try:
            import sys
            sys.print_exception(e)
        except Exception:
            pass
    finally:
        try:
            sensor.stop()
        except Exception:
            pass

        try:
            Display.deinit()
        except Exception:
            pass

        try:
            MediaManager.deinit()
        except Exception:
            pass

        gc.collect()
        try:
            time.sleep_ms(100)
        except Exception:
            time.sleep(0.1)
        nn.shrink_memory_pool()
        print("lock_light_fixed_center_uart end")

    return 0


if __name__ == "__main__":
    detection()
