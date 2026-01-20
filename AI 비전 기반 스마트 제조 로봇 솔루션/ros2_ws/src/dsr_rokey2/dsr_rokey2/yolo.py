import os
import time
from collections import Counter
import rclpy
from ament_index_python.packages import get_package_share_directory
from ultralytics import YOLO
import numpy as np

# 패키지 이름
PACKAGE_NAME = "dsr_rokey2"

# 1. 모델 경로 (사용자분의 절대 경로)
YOLO_MODEL_PATH = r'/home/rokey/ros2_ws/src/Tutorial/Calibration_Tutorial/data/please/rokey_obb_project/run_1/weights/best.pt'

class YoloModel:
    def __init__(self):
        # 모델 로드
        self.model = YOLO(YOLO_MODEL_PATH)
        
        # [수정] JSON 파일 없이, 모델 내부에서 직접 클래스 이름 가져오기
        # 예: {'hammer': 0, 'wrench': 1, ...} 형태로 자동 변환
        self.reversed_class_dict = {v: k for k, v in self.model.names.items()}
        
        print(f"Loaded Classes: {self.reversed_class_dict}")

    def get_frames(self, img_node, duration=1.0):
        """get frames while target_time"""
        end_time = time.time() + duration
        frames = {}

        while time.time() < end_time:
            rclpy.spin_once(img_node)
            frame = img_node.get_color_frame()
            stamp = img_node.get_color_frame_stamp()
            if frame is not None:
                frames[stamp] = frame
            time.sleep(0.01)

        if not frames:
            print(f"No frames captured in {duration:.2f} seconds")

        print(f"{len(frames)} frames captured")
        return list(frames.values())

    def get_best_detection(self, img_node, target):
        rclpy.spin_once(img_node)
        frames = self.get_frames(img_node)
        if not frames:
            return None, None

        results = self.model(frames, verbose=False)
        detections = self._aggregate_detections(results)
        
        # 타겟 이름(예: hammer)이 모델 클래스 목록에 있는지 확인
        if target not in self.reversed_class_dict:
            print(f"Error: Target '{target}' is not in model classes!")
            return None, None

        label_id = self.reversed_class_dict[target]
        print(f"Target: {target} (ID: {label_id})")

        matches = [d for d in detections if d["label"] == label_id]
        if not matches:
            print("No matches found for the target label.")
            return None, None
            
        best_det = max(matches, key=lambda x: x["score"])
        return best_det["box"], best_det["score"]

    def _aggregate_detections(self, results, confidence_threshold=0.5, iou_threshold=0.5):
        raw = []
        for res in results:
            # OBB 모델 대응 (obb가 있으면 쓰고, 없으면 boxes 씀)
            boxes_data = res.obb if res.obb is not None else res.boxes
            
            if boxes_data is not None:
                # 데이터를 CPU로 이동 후 Numpy 변환
                if res.obb is not None:
                    # OBB의 경우 xywhr 형식이지만, 여기선 xyxy 바운딩 박스로 변환해서 처리
                    # (단순 위치 잡기용이므로)
                    import torch
                    # xywhr -> xyxy 변환 (간이 계산)
                    h_boxes = res.obb.xyxy.cpu().numpy()
                else:
                    h_boxes = res.boxes.xyxy.cpu().numpy()

                scores = boxes_data.conf.cpu().numpy()
                labels = boxes_data.cls.cpu().numpy()

                for box, score, label in zip(h_boxes, scores, labels):
                    if score >= confidence_threshold:
                        raw.append({"box": box.tolist(), "score": score, "label": int(label)})

        if not raw:
            return []

        # 중복 박스 제거 로직 (기존 유지)
        final = []
        used = [False] * len(raw)

        for i, det in enumerate(raw):
            if used[i]:
                continue
            group = [det]
            used[i] = True
            for j, other in enumerate(raw):
                if not used[j] and other["label"] == det["label"]:
                    if self._iou(det["box"], other["box"]) >= iou_threshold:
                        group.append(other)
                        used[j] = True

            boxes = np.array([g["box"] for g in group])
            scores = np.array([g["score"] for g in group])
            labels = [g["label"] for g in group]

            final.append(
                {
                    "box": boxes.mean(axis=0).tolist(),
                    "score": float(scores.mean()),
                    "label": Counter(labels).most_common(1)[0][0],
                }
            )

        return final

    def _iou(self, box1, box2):
        x1, y1 = max(box1[0], box2[0]), max(box1[1], box2[1])
        x2, y2 = min(box1[2], box2[2]), min(box1[3], box2[3])
        inter = max(0.0, x2 - x1) * max(0.0, y2 - y1)
        area1 = (box1[2] - box1[0]) * (box1[3] - box1[1])
        area2 = (box2[2] - box2[0]) * (box2[3] - box2[1])
        union = area1 + area2 - inter
        return inter / union if union > 0 else 0.0