#!/usr/bin/env python3
"""
BEV Grid Visualizer Node
-------------------------
Subscribe TrackedObject3DArray (track_id, position, distance, grid_n, grid_m)
tu depth_projector_component, ve thanh anh BEV grid thoi gian thuc.

Anh nay dung cho 2 muc dich:
1. Debug truc quan (RViz / cv2.imshow)
2. Input hinh anh cho VLM trong pipeline VLA (node Prompt/VLM sau nay se lay anh nay)

Quy uoc truc (KHOP voi depth_projector_component.cpp):
  grid_n = floor(x / resolution)   -> x = forward (base_link), truc DOC anh
  grid_m = floor(y / resolution)   -> y = trai    (base_link), truc NGANG anh

Robot dat o MEP DUOI anh (khong phai tam), vi camera chi nhin duoc phia truoc (x >= 0).
Luoi chi trai rong ve phia truoc (forward), doi xung 2 ben theo chieu ngang (lateral).

NHAN HIEN THI (ĐÃ ĐỔI): thay vì in so co dau (m-3, m2...) - de VLM de nham
trai/phai - gio in dang chu: C (center, m=0), L1..Lk (ben TRAI, m duong),
R1..Rk (ben PHAI, m am). Gia tri THAT trong message/topic (obj.grid_m) VAN
LA SO CO DAU nhu cu, chi doi CACH VE CHU len anh, khong doi schema message.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import numpy as np
import cv2
from cv_bridge import CvBridge

from perception.msg import TrackedObject3DArray
from sensor_msgs.msg import Image


class BevVisualizerNode(Node):
    def __init__(self):
        super().__init__("bev_visualizer_node")

        # ==== Tham so cau hinh (declare de doc tu perception_params.yaml) ====
        self.declare_parameter("grid_range_forward_m", 12.0)  # do sau nhin thay phia truoc robot (met)
        self.declare_parameter("grid_range_lateral_m", 6.0)   # be ngang MOI BEN (trai/phai) tinh tu tam (met)
        self.declare_parameter("grid_resolution", 0.5)        # phai KHOP voi depth_projector_component
        self.declare_parameter("canvas_width_px", 500)
        self.declare_parameter("canvas_height_px", 700)
        self.declare_parameter("show_window", False)
        self.declare_parameter("input_topic", "/tracked_persons_depth")
        self.declare_parameter("output_topic", "/image_bev")
        self.declare_parameter("axis_label_step_m", 2.0)      # khoang cach giua 2 nhan truc lien tiep (met)
        self.declare_parameter("bottom_margin_px", 40)        # chua vach + nhan robot o day anh

        self.grid_range_forward_m_ = self.get_parameter("grid_range_forward_m").value
        self.grid_range_lateral_m_ = self.get_parameter("grid_range_lateral_m").value
        self.grid_resolution_ = self.get_parameter("grid_resolution").value
        self.canvas_w_ = self.get_parameter("canvas_width_px").value
        self.canvas_h_ = self.get_parameter("canvas_height_px").value
        self.show_window_ = self.get_parameter("show_window").value
        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        self.axis_label_step_m_ = self.get_parameter("axis_label_step_m").value
        self.bottom_margin_px_ = self.get_parameter("bottom_margin_px").value

        # Scale rieng cho 2 truc de tan dung het canvas (khong bat buoc vuong ty le)
        usable_h = self.canvas_h_ - self.bottom_margin_px_
        self.px_per_meter_x_ = usable_h / self.grid_range_forward_m_
        self.px_per_meter_y_ = (self.canvas_w_ / 2.0) / self.grid_range_lateral_m_
        # Dung 1 scale chung (nho hon trong 2) de luoi vuong deu, khong bi meo ty le
        self.px_per_meter_ = min(self.px_per_meter_x_, self.px_per_meter_y_)

        self.label_step_cells_ = max(
            1, round(self.axis_label_step_m_ / self.grid_resolution_)
        )

        # Goc robot (0,0) tren canvas: giua theo chieu ngang, sat day (tru margin)
        self.origin_px_ = self.canvas_w_ / 2.0
        self.origin_py_ = self.canvas_h_ - self.bottom_margin_px_

        self.bridge_ = CvBridge()

        self.sub_ = self.create_subscription(
            TrackedObject3DArray,
            input_topic,
            self.callback,
            QoSProfile(depth=10),
        )
        self.pub_ = self.create_publisher(Image, output_topic, 10)

        self.get_logger().info(
            f"BevVisualizerNode ready. Sub: {input_topic} -> Pub: {output_topic}"
        )

    def format_m_label(self, grid_m: int) -> str:
        """
        Doi gia tri grid_m (so co dau, THAT trong message) sang nhan chu de
        hien thi len anh: 0 -> 'C', duong -> 'L<k>' (trai), am -> 'R<k>' (phai).
        Chi dung de VE CHU, khong lam thay doi gia tri grid_m goc.
        """
        if grid_m == 0:
            return "C"
        return f"L{grid_m}" if grid_m > 0 else f"R{-grid_m}"

    def world_to_px(self, x_m: float, y_m: float):
        """
        Quy doi toa do (x,y) trong base_link (met) sang pixel tren canvas.
        Goc (0,0) = vi tri robot, dat o MEP DUOI, GIUA canvas theo chieu ngang.
        Truc x (forward) -> huong LEN TREN (py giam khi x tang)
        Truc y (trai)    -> huong SANG TRAI (px giam khi y tang)
        """
        px = self.origin_px_ - y_m * self.px_per_meter_
        py = self.origin_py_ - x_m * self.px_per_meter_
        return int(px), int(py)

    def draw_grid_lines(self, canvas: np.ndarray):
        """Ve luoi BEV chi phia truoc robot (x tu 0 -> grid_range_forward_m)."""
        step_px = self.grid_resolution_ * self.px_per_meter_
        color_grid = (60, 60, 60)
        color_major = (110, 110, 110)
        h, w = canvas.shape[:2]

        n_lateral_cells = int(np.ceil(self.grid_range_lateral_m_ / self.grid_resolution_))
        n_forward_cells = int(np.ceil(self.grid_range_forward_m_ / self.grid_resolution_))

        # Duong doc (ung voi cac gia tri m, chay tu day len tren = vung nhin thay)
        for k in range(-n_lateral_cells, n_lateral_cells + 1):
            px = self.origin_px_ - k * step_px
            if 0 <= px < w:
                is_major = (k % self.label_step_cells_ == 0)
                color = color_major if is_major else color_grid
                thickness = 2 if is_major else 1
                y_top = max(0, int(self.origin_py_ - n_forward_cells * step_px))
                cv2.line(canvas, (int(px), y_top), (int(px), int(self.origin_py_)), color, thickness)

        # Duong ngang (ung voi cac gia tri n, chi ve trong khoang lateral)
        for k in range(0, n_forward_cells + 1):
            py = self.origin_py_ - k * step_px
            if 0 <= py < h:
                is_major = (k % self.label_step_cells_ == 0)
                color = color_major if is_major else color_grid
                thickness = 2 if is_major else 1
                x_left = max(0, int(self.origin_px_ - n_lateral_cells * step_px))
                x_right = min(w, int(self.origin_px_ + n_lateral_cells * step_px))
                cv2.line(canvas, (x_left, int(py)), (x_right, int(py)), color, thickness)

    def draw_axis_labels(self, canvas: np.ndarray):
        """
        Nhan truc m: doc theo canh TREN cua vung luoi (vi ca vung deu forward, khong can nhan o day).
        Nhan truc n: doc theo mep TRAI, tu duoi (n0, tai robot) len tren (n tang dan).
        """
        step_px = self.grid_resolution_ * self.px_per_meter_
        h, w = canvas.shape[:2]
        color_text = (0, 255, 255)
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.4
        thickness = 1

        n_lateral_cells = int(np.ceil(self.grid_range_lateral_m_ / self.grid_resolution_))
        n_forward_cells = int(np.ceil(self.grid_range_forward_m_ / self.grid_resolution_))
        grid_top_py = max(0, int(self.origin_py_ - n_forward_cells * step_px))

        # -------- Legend LEFT / RIGHT --------
        cv2.putText(
            canvas,
            "LEFT",
            (10, grid_top_py - 8),
            font,
            0.5,
            (0, 255, 255),
            1,
            cv2.LINE_AA,
        )
        
        right_text = "RIGHT"
        (text_w, _), _ = cv2.getTextSize(
            right_text,
            font,
            0.5,
            1,
        )
        cv2.putText(
            canvas,
            right_text,
            (self.canvas_w_ - text_w - 10, grid_top_py - 8),
            font,
            0.5,
            (0, 255, 255),
            1,
            cv2.LINE_AA,
        )
        # --- Nhan truc m: dat ngay tren dinh luoi (gan grid_top_py) ---
        for k in range(-n_lateral_cells, n_lateral_cells + 1, self.label_step_cells_):
            px = self.origin_px_ - k * step_px
            if 0 <= px < w:
                label = self.format_m_label(k)
                cv2.putText(canvas, label, (int(px) - 12, grid_top_py + 12),
                            font, font_scale, color_text, thickness, cv2.LINE_AA)

        # --- Nhan truc n: dat o mep trai, ung voi tung hang ---
        x_left = max(0, int(self.origin_px_ - n_lateral_cells * step_px))
        for k in range(0, n_forward_cells + 1, self.label_step_cells_):
            py = self.origin_py_ - k * step_px
            if 0 <= py < h:
                label = f"n{k}"
                cv2.putText(canvas, label, (x_left + 2, int(py) - 3),
                            font, font_scale, color_text, thickness, cv2.LINE_AA)

    def draw_robot(self, canvas: np.ndarray):
        """Ve robot tai goc (0,0) = mep duoi canvas, mui ten chi huong forward (+x)."""

        cx_px, cy_px = self.world_to_px(0.0, 0.0)

        cv2.circle(canvas, (cx_px, cy_px), 8, (0, 255, 255), -1)

        tip_x, tip_y = self.world_to_px(0.6, 0.0)

        cv2.arrowedLine(
            canvas,
            (cx_px, cy_px),
            (tip_x, tip_y),
            (0, 255, 255),
            2,
            tipLength=0.3,
        )

        cv2.putText(
            canvas,
            "Robot",
            (cx_px - 22, cy_px + 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (0, 255, 255),
            1,
            cv2.LINE_AA,
        )

        cv2.putText(
            canvas,
            "n0,C",
            (cx_px + 10, cy_px + 12),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.40,
            (0, 255, 255),
            1,
            cv2.LINE_AA,
        )

    def draw_object(self, canvas: np.ndarray, obj):
        px, py = self.world_to_px(obj.position.x, obj.position.y)
        h, w = canvas.shape[:2]
        if not (0 <= px < w and 0 <= py < h):
            return  # object ngoai vung hien thi (vd sau lung robot), bo qua

        cv2.circle(canvas, (px, py), 10, (0, 0, 255), -1)
        m_label = self.format_m_label(obj.grid_m)
        label = (
            f"ID{obj.track_id}  "
            f"n{obj.grid_n}  "
            f"{m_label}  "
            f"{obj.distance:.1f}m"
        )
        cv2.putText(
            canvas, label, (px + 12, py),
            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA,
        )

    def callback(self, msg: TrackedObject3DArray):
        canvas = np.zeros((self.canvas_h_, self.canvas_w_, 3), dtype=np.uint8)

        self.draw_grid_lines(canvas)
        self.draw_axis_labels(canvas)
        self.draw_robot(canvas)
        for obj in msg.objects:
            self.draw_object(canvas, obj)

        img_msg = self.bridge_.cv2_to_imgmsg(canvas, encoding="bgr8")
        img_msg.header = msg.header
        self.pub_.publish(img_msg)

        if self.show_window_:
            cv2.imshow("BEV Grid", canvas)
            cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = BevVisualizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
