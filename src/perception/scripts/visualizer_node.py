#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from cv_bridge import CvBridge
import cv2
import message_filters

# Bảng màu cố định theo track ID, để cùng 1 ID luôn ra cùng 1 màu
# giữa các frame (dễ theo dõi bằng mắt xem ID có nhảy hay không).
_COLOR_PALETTE = [
    (66, 135, 245), (245, 66, 66), (66, 245, 129), (245, 191, 66),
    (191, 66, 245), (66, 245, 224), (245, 66, 197), (150, 245, 66),
]


def _color_for_id(track_id: int):
    return _COLOR_PALETTE[track_id % len(_COLOR_PALETTE)]


class VisualizerNode(Node):
    def __init__(self):
        super().__init__('visualizer_node')
        self.bridge = CvBridge()

        # image_raw dùng SensorDataQoS (Best Effort) bên usb_cam_component,
        # message_filters.Subscriber mặc định Reliable -> phải khai báo
        # đúng qos_profile, không sẽ không nhận được message nào.
        self.image_sub = message_filters.Subscriber(
            self, Image, '/image_raw',
            qos_profile=qos_profile_sensor_data)

        # /detections_output và /tracked_persons đều publish QoS mặc định
        # (Reliable), khớp default của message_filters.Subscriber -> không
        # cần truyền qos_profile riêng cho 2 cái này.
        self.det_sub = message_filters.Subscriber(
            self, Detection2DArray, '/detections_output')
        self.track_sub = message_filters.Subscriber(
            self, Detection2DArray, '/tracked_persons')

        # Đồng bộ CẢ 3 nguồn theo timestamp. Header của cả 3 topic đều
        # copy nguyên từ msg->header gốc của ảnh (yolov8_trt_component
        # và bytetrack_component đều giữ nguyên header), nên timestamp
        # khớp gần như tuyệt đối, slop=0.1 dư sức đủ.
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.image_sub, self.det_sub, self.track_sub],
            queue_size=10, slop=0.1)
        self.ts.registerCallback(self.sync_callback)

        self.image_pub = self.create_publisher(Image, '/image_visualized', 10)
        self.get_logger().info(
            "Visualizer Node (Python) đã sẵn sàng vẽ Box "
            "(detect=xám, track=màu theo ID)!")

    def sync_callback(self, img_msg, det_msg, track_msg):
        cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')

        # 1. Vẽ box detect GỐC (trước tracking) - viền xám mảnh, không nhãn.
        # Mục đích: soi bằng mắt xem detector có ra 2 box chồng nhau cho
        # cùng 1 vật thể không (nghi vấn NMS threshold hiện tại chưa đủ
        # gộp mạnh). Nếu thấy 2 viền xám gần trùng khít lên nhau ở cùng
        # 1 người -> đúng nghi vấn, cần tăng nms_threshold_ bên
        # yolov8_trt_component.
        for det in det_msg.detections:
            bbox = det.bbox
            cx, cy = bbox.center.position.x, bbox.center.position.y
            w, h = bbox.size_x, bbox.size_y
            x1, y1 = int(cx - w / 2), int(cy - h / 2)
            x2, y2 = int(cx + w / 2), int(cy + h / 2)
            cv2.rectangle(cv_image, (x1, y1), (x2, y2), (150, 150, 150), 1)

        # 2. Vẽ box đã TRACK - viền màu theo track ID, nhãn "ID:x".
        # Lưu ý: bytetrack_component đang tái dùng field class_id để
        # chứa track ID (xem hyp.hypothesis.class_id trong
        # bytetrack_component.cpp), không phải class COCO.
        for trk in track_msg.detections:
            bbox = trk.bbox
            cx, cy = bbox.center.position.x, bbox.center.position.y
            w, h = bbox.size_x, bbox.size_y
            x1, y1 = int(cx - w / 2), int(cy - h / 2)
            x2, y2 = int(cx + w / 2), int(cy + h / 2)

            if len(trk.results) == 0:
                continue

            track_id_str = trk.results[0].hypothesis.class_id
            score = trk.results[0].hypothesis.score

            try:
                track_id = int(track_id_str)
            except ValueError:
                track_id = 0

            color = _color_for_id(track_id)
            cv2.rectangle(cv_image, (x1, y1), (x2, y2), color, 2)

            label = f"ID:{track_id_str}  {score:.2f}"
            (tw, th), _ = cv2.getTextSize(
                label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 2)
            label_y1 = max(y1 - th - 8, 0)
            cv2.rectangle(
                cv_image, (x1, label_y1), (x1 + tw + 6, label_y1 + th + 8),
                color, -1)
            cv2.putText(
                cv_image, label, (x1 + 3, label_y1 + th + 2),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)

        # Chú thích góc trên: số lượng box mỗi nguồn, tiện đối chiếu
        # nhanh xem detect có ra nhiều box hơn số track hay không.
        info = f"detect={len(det_msg.detections)}  track={len(track_msg.detections)}"
        cv2.putText(
            cv_image, info, (10, 25),
            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        out_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        out_msg.header = img_msg.header
        self.image_pub.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = VisualizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
