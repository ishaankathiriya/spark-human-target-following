import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from vision_msgs.msg import Detection2DArray, Detection2D, BoundingBox2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
from ultralytics import YOLO
import numpy as np


class HumanTrackerNode(Node):
    def __init__(self):
        super().__init__('human_tracker')

        self.declare_parameter('model', 'yolov8n.pt')
        self.declare_parameter('confidence', 0.5)
        self.declare_parameter('tracker', 'bytetrack.yaml')
        self.declare_parameter('target_id', -1)   # -1 = closest to center
        
        model_path = self.get_parameter('model').get_parameter_value().string_value
        self.conf = self.get_parameter('confidence').get_parameter_value().double_value
        tracker_cfg = self.get_parameter('tracker').get_parameter_value().string_value
        self.target_id = self.get_parameter('target_id').get_parameter_value().integer_value


        self.model = YOLO(model_path)
        self.tracker_cfg = tracker_cfg
        self.bridge = CvBridge()

        self.sub = self.create_subscription( Image, '/camera/camera/color/image_raw', self.image_callback, 10)
        self.pub_detections = self.create_publisher(Detection2DArray, '/tracker/detections', 10)
        self.pub_centroid = self.create_publisher(PointStamped, 'tracker/centroid', 10)
        self.pub_debug = self.create_publisher(Image, '/tracker/debug_image', 10)

        self.get_logger().info('Human tracker node started')

    
    def image_callback(self, msg:Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w = frame.shape[:2]

        results = self.model.track(
            frame, 
            persist=True, 
            classes=[0], #person only 
            conf = self.conf,
            tracker = self.tracker_cfg,
            verbose = False,
        )

        det_array_msg = Detection2DArray()
        det_array_msg.header = msg.header

        target_box = None
        target_centroid = None
        min_dist = float('inf')
        cx_img, cy_img = w/2.0, h/2.0

        if results[0].boxes is not None:
            boxes = results[0].boxes
            for box in boxes:
                if box.id is None:
                    continue

                track_id = int(box.id[0])
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                conf = float(box.conf[0])

                cx = (x1+x2) / 2.0
                cy = (y1+y2) / 2.0

                det = Detection2D()
                det.header = msg.header
                det.bbox.center.position.x = cx
                det.bbox.center.position.y = cy
                det.bbox.size_x = float(x2 - x1)
                det.bbox.size_y = float(y2 - y1)

                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = str(track_id)
                hyp.hypothesis.score = conf
                det.results.append(hyp)
                det_array_msg.detections.append(det)

                # Select target: explicit ID or closest to image center
                if self.target_id != -1:
                    if track_id == self.target_id:
                        target_centroid = (cx, cy)

                else:
                    dist = np.hypot(cx - cx_img, cy - cy_img)
                    if dist < min_dist:
                        min_dist = dist
                        target_centroid = (cx, cy)

                

        self.pub_detections.publish(det_array_msg)

        if target_centroid is not None:
            pt = PointStamped()
            pt.header = msg.header
            pt.point.x = target_centroid[0]
            pt.point.y = target_centroid[1]
            pt.point.z = 0.0
            self.pub_centroid.publish(pt)


        annotated = results[0].plot()
        self.pub_debug.publish(self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8'))



def main(args=None):
    rclpy.init(args=args)
    node = HumanTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

    
if __name__ == '__main__':
    main()
