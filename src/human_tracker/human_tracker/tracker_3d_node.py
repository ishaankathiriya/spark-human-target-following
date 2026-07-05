import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
import torch
torch.set_num_threads(1)
from ultralytics import YOLO
import message_filters
import numpy as np
import cv2
import time
cv2.setNumThreads(1)


class HumanTracker3DNode(Node):
    def __init__(self):
        super().__init__('human_tracker_3d')

        self.declare_parameter('model', 'yolov8n.pt')
        self.declare_parameter('confidence', 0.3)
        self.declare_parameter('tracker', 'bytetrack.yaml')
        self.declare_parameter('target_id', -1)
        self.declare_parameter('depth_patch_size', 5)
        self.declare_parameter('device', 'cuda:0')   # 'cuda:0' for Jetson GPU, 'cpu' to force CPU

        model_path = self.get_parameter('model').get_parameter_value().string_value
        self.conf = self.get_parameter('confidence').get_parameter_value().double_value
        self.tracker_cfg = self.get_parameter('tracker').get_parameter_value().string_value
        self.target_id = self.get_parameter('target_id').get_parameter_value().integer_value
        self.patch = self.get_parameter('depth_patch_size').get_parameter_value().integer_value
        self._device = self.get_parameter('device').get_parameter_value().string_value

        self.model = YOLO(model_path)
        if model_path.endswith('.pt'):
            self.model.to(self._device)
        self.bridge = CvBridge()
        self.intrinsics = None

        self.create_subscription(
            CameraInfo,
            '/camera/camera/color/camera_info',
            self._camera_info_cb,
            1,
        )

        rgb_sub = message_filters.Subscriber(
            self, Image, '/camera/camera/color/image_raw')
        depth_sub = message_filters.Subscriber(
            self, Image, '/camera/camera/aligned_depth_to_color/image_raw')

        self.sync = message_filters.ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub], queue_size=5, slop=0.05)
        self.sync.registerCallback(self._image_callback)

        self.pub_detections = self.create_publisher(
            Detection2DArray, '/tracker/detections', 10)
        self.pub_centroid = self.create_publisher(
            PointStamped, '/tracker/centroid_3d', 10)
        self.pub_debug = self.create_publisher(
            Image, '/tracker/debug_image', 10)

        self._prof_n = 0
        self._prof_conv = 0.0
        self._prof_track = 0.0
        self._prof_post = 0.0
        self._prof_pre = 0.0
        self._prof_infer = 0.0
        self._prof_postp = 0.0

        self.get_logger().info('Human tracker 3D node started')

    def _camera_info_cb(self, msg: CameraInfo):
        if self.intrinsics is None:
            self.intrinsics = {
                'fx': msg.k[0],
                'fy': msg.k[4],
                'cx': msg.k[2],
                'cy': msg.k[5],
            }
            self.get_logger().info(
                f"Intrinsics received: fx={self.intrinsics['fx']:.1f}, "
                f"fy={self.intrinsics['fy']:.1f}, "
                f"cx={self.intrinsics['cx']:.1f}, "
                f"cy={self.intrinsics['cy']:.1f}"
            )

    def _deproject(self, px, py, depth_m):
        fx = self.intrinsics['fx']
        fy = self.intrinsics['fy']
        cx = self.intrinsics['cx']
        cy = self.intrinsics['cy']
        x = (px - cx) * depth_m / fx
        y = (py - cy) * depth_m / fy
        return x, y, depth_m

    def _sample_depth(self, depth_image, x1, y1, x2, y2, encoding):
        """Sample depth from the inner 60% of the bounding box.

        Uses a large region rather than a small centre patch so that:
        - Hundreds of pixels are averaged instead of ~25
        - Edge pixels (likely background) are excluded by the 20% margin
        - Outliers are removed by taking the median of valid depths only
        """
        h, w = depth_image.shape

        bw = x2 - x1
        bh = y2 - y1
        mx = max(1, int(bw * 0.20))
        my = max(1, int(bh * 0.20))

        rx0 = max(0, x1 + mx);  rx1 = min(w, x2 - mx)
        ry0 = max(0, y1 + my);  ry1 = min(h, y2 - my)

        if rx1 <= rx0 or ry1 <= ry0:
            # Box too small for margin — fall back to a 5×5 centre patch
            cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
            rx0 = max(0, cx - 2);  rx1 = min(w, cx + 3)
            ry0 = max(0, cy - 2);  ry1 = min(h, cy + 3)

        region = depth_image[ry0:ry1, rx0:rx1].astype(np.float32)

        # 16UC1 = RealSense hardware (mm); 32FC1 = Gazebo simulation (metres)
        if encoding == '16UC1':
            valid = region[(region > 100) & (region < 10000)]   # 0.1 m – 10 m in mm
            if valid.size == 0:
                return None
            return float(np.median(valid)) * 0.001
        else:
            valid = region[(region > 0.1) & (region < 10.0) & np.isfinite(region)]
            if valid.size == 0:
                return None
            return float(np.median(valid))

    def _image_callback(self, rgb_msg: Image, depth_msg: Image):
        if self.intrinsics is None:
            return

        t0 = time.perf_counter()

        frame = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
        depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        depth_encoding = depth_msg.encoding
        h, w = frame.shape[:2]

        t1 = time.perf_counter()

        if self.target_id != -1:
            # Persistent track IDs are only needed to lock onto a specific person
            results = self.model.track(
                frame,
                persist=True,
                classes=[0],
                conf=self.conf,
                tracker=self.tracker_cfg,
                device=self._device,
                verbose=False,
            )
        else:
            # Auto-select (closest to image centre) doesn't need track IDs —
            # skip ByteTrack entirely (saves ~20% CPU) by using detection only
            results = self.model.predict(
                frame,
                classes=[0],
                conf=self.conf,
                device=self._device,
                verbose=False,
            )

        t2 = time.perf_counter()
        spd = results[0].speed  # ms: {'preprocess', 'inference', 'postprocess'} from ultralytics
        self._prof_pre += spd.get('preprocess', 0.0)
        self._prof_infer += spd.get('inference', 0.0)
        self._prof_postp += spd.get('postprocess', 0.0)

        det_array_msg = Detection2DArray()
        det_array_msg.header = rgb_msg.header

        target_centroid_3d = None
        min_dist = float('inf')
        cx_img, cy_img = w / 2.0, h / 2.0

        publish_debug = self.pub_debug.get_subscription_count() > 0
        annotated = frame.copy() if publish_debug else None

        if results[0].boxes is not None:
            for box in results[0].boxes:
                if self.target_id != -1 and box.id is None:
                    continue

                track_id = int(box.id[0]) if box.id is not None else -1
                x1, y1, x2, y2 = [int(v) for v in box.xyxy[0].tolist()]
                conf = float(box.conf[0])
                cx = (x1 + x2) / 2.0
                cy = (y1 + y2) / 2.0

                depth_m = self._sample_depth(depth, x1, y1, x2, y2, depth_encoding)
                if depth_m is None or depth_m <= 0:
                    X, Y, Z = None, None, None
                else:
                    X, Y, Z = self._deproject(cx, cy, depth_m)

                det = Detection2D()
                det.header = rgb_msg.header
                det.bbox.center.position.x = cx
                det.bbox.center.position.y = cy
                det.bbox.size_x = float(x2 - x1)
                det.bbox.size_y = float(y2 - y1)
                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = str(track_id)
                hyp.hypothesis.score = conf
                det.results.append(hyp)
                det_array_msg.detections.append(det)

                # Draw bounding box and track ID (only when someone is viewing the debug feed)
                if publish_debug:
                    cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.circle(annotated, (int(cx), int(cy)), 4, (0, 0, 255), -1)
                    cv2.putText(annotated, f'ID:{track_id}',
                                (x1, y1 - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)

                    if X is not None:
                        coord_text = f'X:{X:.2f} Y:{Y:.2f} Z:{Z:.2f}m'
                        cv2.putText(annotated, coord_text,
                                    (int(cx) - 60, int(cy) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
                    else:
                        cv2.putText(annotated, 'depth: N/A',
                                    (int(cx) - 40, int(cy) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 100, 255), 2)

                # Select target
                if self.target_id != -1:
                    if track_id == self.target_id and X is not None:
                        target_centroid_3d = (X, Y, Z)
                else:
                    dist = np.hypot(cx - cx_img, cy - cy_img)
                    if dist < min_dist and X is not None:
                        min_dist = dist
                        target_centroid_3d = (X, Y, Z)

        self.pub_detections.publish(det_array_msg)

        if target_centroid_3d is not None:
            pt = PointStamped()
            pt.header = rgb_msg.header
            pt.point.x = target_centroid_3d[0]
            pt.point.y = target_centroid_3d[1]
            pt.point.z = target_centroid_3d[2]
            self.pub_centroid.publish(pt)

        if publish_debug:
            self.pub_debug.publish(
                self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8'))

        t3 = time.perf_counter()
        self._prof_conv += t1 - t0
        self._prof_track += t2 - t1
        self._prof_post += t3 - t2
        self._prof_n += 1
        # if self._prof_n >= 60:
        #     n = self._prof_n
        #     bytetrack_ms = self._prof_track - (self._prof_pre + self._prof_infer + self._prof_postp) / 1000.0
        #     self.get_logger().info(
        #         f'PROFILE ms/frame (avg of {n}): '
        #         f'convert={1000 * self._prof_conv / n:.1f} '
        #         f'track_total={1000 * self._prof_track / n:.1f} '
        #         f'[pre={self._prof_pre / n:.1f} '
        #         f'infer={self._prof_infer / n:.1f} '
        #         f'yolo_post={self._prof_postp / n:.1f} '
        #         f'bytetrack≈{1000 * bytetrack_ms / n:.1f}] '
        #         f'post={1000 * self._prof_post / n:.1f} '
        #         f'total={1000 * (self._prof_conv + self._prof_track + self._prof_post) / n:.1f}'
        #     )
        #     self._prof_n = 0
            self._prof_conv = 0.0
            self._prof_track = 0.0
            self._prof_post = 0.0
            self._prof_pre = 0.0
            self._prof_infer = 0.0
            self._prof_postp = 0.0


def main(args=None):
    rclpy.init(args=args)
    node = HumanTracker3DNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
