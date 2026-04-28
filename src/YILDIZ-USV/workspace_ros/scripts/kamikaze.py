#!/usr/bin/env python3

# ----------------------------------------------------------------------------------------------- #
#  Node that processes camera images using a YOLO segmentation model to detect the target buoy and
#  publish motion commands accordingly. It dynamically reloads the target color definition and
#  provides visual feedback for detection and navigation status.
# ----------------------------------------------------------------------------------------------- #

import os
import re
import cv2
import json
import rclpy
import numpy as np
from rclpy.node import Node
from ultralytics import YOLO
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from ament_index_python.packages import get_package_share_directory

IMAGE_TOPIC = "/roboboat/sensors/camera/image"
CMD_VEL_TOPIC = "/cmd_vel_nav"
TARGET_JSON_FILENAME = "target_buoy.json"
WINDOW_NAME = "KAMIKAZE"
WINDOW_WIDTH = 1280
WINDOW_HEIGHT = 720
FONT = cv2.FONT_HERSHEY_SIMPLEX
GREEN = (0, 150, 0)
RED = (0, 0, 150)
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
CAMERA_CHECK_SECONDS = 3.0
TARGET_RELOAD_INTERVAL = 2.0

class CameraSubscriber(Node):
    def __init__(self) -> None:
        super().__init__("kamikaze")
        self.bridge = CvBridge()
        self.model = self._load_model()
        self.image_topic = IMAGE_TOPIC
        self.cmd_vel_pub = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)
        self.create_subscription(Image, self.image_topic, self.image_callback, 10)
        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WINDOW_NAME, WINDOW_WIDTH, WINDOW_HEIGHT)
        self.win_w = WINDOW_WIDTH
        self.win_h = WINDOW_HEIGHT
        self.window_name = WINDOW_NAME
        self.latest_color = None
        self._image_received = False
        self._camera_check_counter = 0
        self._camera_check_logged = False
        self.last_detection_side = "right"
        self.target_json_path = self._resolve_target_json_path()
        self._load_target_from_file()
        self.create_timer(1.0, self._camera_check_timer_cb)
        self.create_timer(TARGET_RELOAD_INTERVAL, self._reload_target_timer_cb)

    def _load_model(self):
        try:
            env_path = os.environ.get("YOLO_MODEL_PATH")
            if env_path and os.path.isfile(env_path):
                try:
                    return YOLO(env_path, task="segment")
                except Exception as e:
                    self.get_logger().error(f"Failed to load YOLO model from YOLO_MODEL_PATH='{env_path}': {e}")
            try:
                base = get_package_share_directory("workspace_ros")
                model_path = os.path.join(base, "YOLOv11", "YOLOv11.pt")
                if os.path.isfile(model_path):
                    return YOLO(model_path, task="segment")
            except Exception:
                pass
            start = os.path.abspath(os.path.dirname(__file__))
            cur = start
            for _ in range(8):
                candidate = os.path.join(cur, "workspace_ros", "YOLOv11", "YOLOv11.pt")
                if os.path.isfile(candidate):
                    return YOLO(candidate, task="segment")
                candidate2 = os.path.join(cur, "YILDIZ-USV", "workspace_ros", "YOLOv11", "YOLOv11.pt")
                if os.path.isfile(candidate2):
                    return YOLO(candidate2, task="segment")
                parent = os.path.dirname(cur)
                if parent == cur:
                    break
                cur = parent
            fallback = os.path.join(os.getcwd(), "src", "YILDIZ-USV", "workspace_ros", "YOLOv11", "YOLOv11.pt")
            if os.path.isfile(fallback):
                return YOLO(fallback, task="segment")
            self.get_logger().error("YOLO model file not found. Tried env, package share, repo-relative and cwd fallbacks.")
        except Exception as e:
            self.get_logger().error(f"Model load exception: {e}")
        return None

    def _resolve_target_json_path(self) -> str:
        try:
            env_path = os.environ.get("TARGET_JSON_PATH")
            if env_path and os.path.isfile(env_path):
                return env_path
            try:
                base = get_package_share_directory("workspace_nav")
                candidate = os.path.join(base, "json", TARGET_JSON_FILENAME)
                if os.path.isfile(candidate):
                    return candidate
            except Exception:
                pass
        except Exception:
            pass
        start = os.path.abspath(os.path.dirname(__file__))
        cur = start
        for _ in range(8):
            candidate = os.path.join(cur, "workspace_nav", "json", TARGET_JSON_FILENAME)
            if os.path.isfile(candidate):
                return candidate
            candidate2 = os.path.join(cur, "YILDIZ-USV", "workspace_nav", "json", TARGET_JSON_FILENAME)
            if os.path.isfile(candidate2):
                return candidate2
            candidate3 = os.path.join(cur, "src", "YILDIZ-USV", "workspace_nav", "json", TARGET_JSON_FILENAME)
            if os.path.isfile(candidate3):
                return candidate3
            parent = os.path.dirname(cur)
            if parent == cur:
                break
            cur = parent
        cwd_candidate = os.path.join(os.getcwd(), "src", "YILDIZ-USV", "workspace_nav", "json", TARGET_JSON_FILENAME)
        if os.path.isfile(cwd_candidate):
            return cwd_candidate
        alt_cwd = os.path.join(os.getcwd(), "YILDIZ-USV", "workspace_nav", "json", TARGET_JSON_FILENAME)
        if os.path.isfile(alt_cwd):
            return alt_cwd
        return cwd_candidate

    def _read_target_file(self):
        try:
            path = self.target_json_path
            if not os.path.isfile(path):
                self.get_logger().warning(f"Target JSON not found at: {path}")
                return None
            with open(path, "r", encoding="utf-8") as f:
                data = json.load(f)
            target = data.get("target", {})
            color = target.get("color")
            if color is None:
                self.get_logger().warning("Target JSON missing 'color' field")
                return None
            return str(color).strip().lower()
        except Exception as e:
            self.get_logger().error(f"Failed to read target JSON: {e}")
            return None

    def _load_target_from_file(self) -> None:
        color = self._read_target_file()
        if color is not None:
            self.latest_color = color
            self.get_logger().info(f"Loaded target color: '{self.latest_color}' from {self.target_json_path}")

    def _reload_target_timer_cb(self) -> None:
        try:
            color = self._read_target_file()
            if color is None:
                return
            if self.latest_color != color:
                old = self.latest_color
                self.latest_color = color
                self.get_logger().info(f"Target color changed: '{old}' -> '{self.latest_color}'")
        except Exception as e:
            self.get_logger().error(f"Reload target error: {e}")

    def _to_numpy(self, x):
        try:
            return x.cpu().numpy()
        except Exception:
            try:
                return np.asarray(x)
            except Exception:
                return np.array([])

    def _column_of_x(self, x: int, width: int) -> int:
        c = int(x * 9 / width) + 1
        if c < 1:
            c = 1
        if c > 9:
            c = 9
        return c

    def _publish_cmd_vel(self, linear_x: float, angular_z: float) -> None:
        try:
            msg = Twist()
            msg.linear.x = float(linear_x)
            msg.linear.y = 0.0
            msg.linear.z = 0.0
            msg.angular.x = 0.0
            msg.angular.y = 0.0
            msg.angular.z = float(angular_z)
            self.cmd_vel_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"cmd_vel publish error: {e}")

    def _camera_check_timer_cb(self) -> None:
        if self._image_received or self._camera_check_logged:
            return
        self._camera_check_counter += 1
        if self._camera_check_counter * 1.0 >= CAMERA_CHECK_SECONDS:
            self.get_logger().warning(f"No images received on '{self.image_topic}' within {int(CAMERA_CHECK_SECONDS)} seconds.")
            self._camera_check_logged = True

    def _normalize(self, s: str) -> str:
        if s is None:
            return ""
        tmp = str(s).lower()
        tmp = tmp.replace("_", " ").replace("-", " ")
        tmp = re.sub(r"[^a-z0-9\s]", "", tmp)
        tmp = re.sub(r"\s+", " ", tmp).strip()
        return tmp

    def _tokens(self, s: str):
        n = self._normalize(s)
        if n == "":
            return set()
        return set(n.split())

    def _class_name_from_model(self, classes_array, idx, names_obj):
        try:
            if classes_array is None:
                return ""
            cls_idx = int(classes_array[idx])
            if names_obj is None:
                return str(cls_idx)
            if isinstance(names_obj, dict):
                return str(names_obj.get(cls_idx, str(cls_idx)))
            if isinstance(names_obj, (list, tuple)):
                if 0 <= cls_idx < len(names_obj):
                    return str(names_obj[cls_idx])
                else:
                    return str(cls_idx)
            return str(cls_idx)
        except Exception:
            try:
                return str(classes_array[idx])
            except Exception:
                return ""

    def _matches_target(self, cls_name_raw: str) -> bool:
        if not cls_name_raw:
            return False
        if not self.latest_color:
            return False
        cls_norm = self._normalize(cls_name_raw)
        target_norm = self._normalize(self.latest_color)
        if cls_norm == "" or target_norm == "":
            return False
        if cls_norm == target_norm:
            return True
        if target_norm in cls_norm:
            return True
        if cls_norm in target_norm:
            return True
        cls_tokens = self._tokens(cls_name_raw)
        target_tokens = self._tokens(self.latest_color)
        if cls_tokens & target_tokens:
            return True
        return False

    def image_callback(self, msg: Image) -> None:
        try:
            self._image_received = True
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            resized = cv2.resize(cv_image, (self.win_w, self.win_h))
            results = None
            if self.model is not None:
                try:
                    results = self.model(resized)
                except Exception as e:
                    self.get_logger().error(f"Model inference error: {e}")
            frame = resized.copy()
            h, w = frame.shape[:2]
            for i in range(1, 9):
                x = int(i * w / 9)
                cv2.line(frame, (x, 0), (x, h - 1), BLACK, 1)
            detections_present = False
            best_match = None
            best_conf = -1.0
            if results is not None and len(results) > 0:
                r = results[0]
                boxes_attr = getattr(r, "boxes", None)
                coords = None
                confs = None
                classes = None
                if boxes_attr is not None:
                    xyxy = getattr(boxes_attr, "xyxy", None)
                    conf_attr = getattr(boxes_attr, "conf", None)
                    cls_attr = getattr(boxes_attr, "cls", None)
                    if xyxy is not None:
                        coords = self._to_numpy(xyxy)
                    if conf_attr is not None:
                        confs = self._to_numpy(conf_attr)
                    if cls_attr is not None:
                        classes = self._to_numpy(cls_attr)
                    if coords is None:
                        data = getattr(boxes_attr, "data", None)
                        if data is not None:
                            arr = self._to_numpy(data)
                            if arr.size != 0:
                                coords = arr[:, 0:4]
                                confs = arr[:, 4]
                                classes = arr[:, 5]
                if coords is not None and coords.size != 0:
                    names = getattr(self.model, "names", None)
                    for idx, box in enumerate(coords):
                        if len(box) < 4:
                            continue
                        x1, y1, x2, y2 = int(box[0]), int(box[1]), int(box[2]), int(box[3])
                        conf = None
                        cls_name = ""
                        if confs is not None and idx < len(confs):
                            try:
                                conf = float(confs[idx])
                            except Exception:
                                conf = None
                        if classes is not None and idx < len(classes):
                            cls_name = self._class_name_from_model(classes, idx, names)
                        else:
                            if names is not None:
                                try:
                                    if isinstance(names, dict):
                                        cls_name = str(next(iter(names.values())))
                                    elif isinstance(names, (list, tuple)) and len(names) > 0:
                                        cls_name = str(names[0])
                                except Exception:
                                    cls_name = ""
                        if cls_name == "" and names is not None:
                            try:
                                cls_name = str(names.get(0, "0")) if isinstance(names, dict) else (str(names[0]) if len(names) > 0 else "")
                            except Exception:
                                cls_name = ""
                        label = cls_name if conf is None else f"{cls_name} {conf:.2f}"
                        match_this = False
                        if self.latest_color is not None and cls_name:
                            try:
                                if self._matches_target(cls_name):
                                    match_this = True
                                    self.get_logger().info(f"Match found: model='{cls_name}' target='{self.latest_color}' conf={conf}")
                                else:
                                    self.get_logger().debug(f"Detected class '{cls_name}' does not match target '{self.latest_color}'")
                            except Exception:
                                match_this = False
                        rect_color = BLACK
                        cv2.rectangle(frame, (x1, y1), (x2, y2), rect_color, 2)
                        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)
                        tx1, ty1 = x1, max(0, y1 - th - 6)
                        tx2, ty2 = x1 + tw, y1
                        cv2.rectangle(frame, (tx1, ty1), (tx2, ty2), BLACK, -1)
                        cv2.putText(frame, label, (tx1, ty2 - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.6, WHITE, 1, cv2.LINE_AA)
                        if match_this:
                            detections_present = True
                            conf_val = conf if conf is not None else 0.0
                            if conf_val > best_conf:
                                best_conf = conf_val
                                best_match = (x1, y1, x2, y2, cls_name, conf_val)
            linear_x = 0.0
            angular_z = 0.0
            status_text = "NOT DETECTED"
            status_color = RED
            mode_text = "MODE: RECOVERY"
            mode_color = RED
            if detections_present and best_match is not None:
                x1, y1, x2, y2, cls_name, conf_val = best_match
                center_x = int((x1 + x2) / 2)
                center_y = int((y1 + y2) / 2)
                col = self._column_of_x(center_x, w)
                status_text = "DETECTED"
                status_color = GREEN
                self.get_logger().debug(f"Best match: '{cls_name}', conf={conf_val:.3f}, col={col}, target='{self.latest_color}'")
                if col < 5:
                    self.last_detection_side = "left"
                elif col > 5:
                    self.last_detection_side = "right"
                plus_color = GREEN if col == 5 else RED
                l = 6
                cv2.line(frame, (center_x - l, center_y), (center_x + l, center_y), plus_color, 1)
                cv2.line(frame, (center_x, center_y - l), (center_x, center_y + l), plus_color, 1)
                if col == 5:
                    linear_x = 2.0
                    angular_z = 0.0
                    mode_text = "MODE: 0"
                    mode_color = GREEN
                elif col == 4:
                    linear_x = 0.85
                    angular_z = 0.25
                    mode_text = "MODE: -1"
                    mode_color = RED
                elif col == 3:
                    linear_x = 0.65
                    angular_z = 0.35
                    mode_text = "MODE: -2"
                    mode_color = RED
                elif col == 2:
                    linear_x = 0.45
                    angular_z = 0.45
                    mode_text = "MODE: -3"
                    mode_color = RED
                elif col == 1:
                    linear_x = 0.25
                    angular_z = 0.75
                    mode_text = "MODE: -4"
                    mode_color = RED
                elif col == 6:
                    linear_x = 0.85
                    angular_z = -0.25
                    mode_text = "MODE: 1"
                    mode_color = RED
                elif col == 7:
                    linear_x = 0.65
                    angular_z = -0.35
                    mode_text = "MODE: 2"
                    mode_color = RED
                elif col == 8:
                    linear_x = 0.45
                    angular_z = -0.45
                    mode_text = "MODE: 3"
                    mode_color = RED
                elif col == 9:
                    linear_x = 0.25
                    angular_z = -0.75
                    mode_text = "MODE: 4"
                    mode_color = RED
            else:
                if self.last_detection_side == "left":
                    angular_z = 1.0
                else:
                    angular_z = -1.0
                mode_text = "MODE: RECOVERY"
                mode_color = RED
            self._publish_cmd_vel(linear_x, angular_z)
            (stw, sth), _ = cv2.getTextSize(status_text, FONT, 1.0, 2)
            stx = (w - stw) // 2
            sty_top = 20
            rect_top_left = (stx - 8, sty_top - 6)
            rect_bottom_right = (stx + stw + 8, sty_top + sth + 6)
            cv2.rectangle(frame, rect_top_left, rect_bottom_right, BLACK, -1)
            cv2.putText(frame, status_text, (stx, sty_top + sth), FONT, 1.0, status_color, 2, cv2.LINE_AA)
            (mtw, mth), _ = cv2.getTextSize(mode_text, FONT, 0.9, 2)
            mtx = (w - mtw) // 2
            mty_bottom = h - 20
            mrect_top_left = (mtx - 8, mty_bottom - mth - 6)
            mrect_bottom_right = (mtx + mtw + 8, mty_bottom + 6)
            cv2.rectangle(frame, mrect_top_left, mrect_bottom_right, BLACK, -1)
            cv2.putText(frame, mode_text, (mtx, mty_bottom), FONT, 0.9, mode_color, 2, cv2.LINE_AA)
            cv2.imshow(self.window_name, frame)
            key = cv2.waitKey(1) & 0xFF
            if key == 27:
                try:
                    rclpy.shutdown()
                except Exception:
                    pass
        except Exception as e:
            self.get_logger().error(f"Image callback error: {e}")

def main() -> None:
    rclpy.init()
    node = CameraSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass

if __name__ == "__main__":
    main()