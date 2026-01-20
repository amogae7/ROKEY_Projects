#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from std_msgs.msg import Header
from cv_bridge import CvBridge
import tf2_ros
import open3d as o3d
import numpy as np
import cv2
import threading
import sys
import struct
import os
from scipy.spatial.transform import Rotation
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class TSDFScanner(Node):
    def __init__(self):
        super().__init__('tsdf_scanner_node')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.sub_color = self.create_subscription(Image, '/camera/camera/color/image_raw', self.color_callback, qos_profile)
        self.sub_depth = self.create_subscription(Image, '/camera/camera/aligned_depth_to_color/image_raw', self.depth_callback, qos_profile)
        self.sub_info = self.create_subscription(CameraInfo, '/camera/camera/aligned_depth_to_color/camera_info', self.info_callback, qos_profile)

        # [추가됨] RViz로 3D 데이터를 쏘는 퍼블리셔
        self.pcd_pub = self.create_publisher(PointCloud2, '/tsdf_map', 10)

        self.bridge = CvBridge()
        self.color_img = None
        self.depth_img = None
        self.intrinsics = None
        
        self.volume = o3d.pipelines.integration.ScalableTSDFVolume(
            voxel_length=4.0 / 512.0,
            sdf_trunc=0.04,
            color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8
        )

        self.is_scanning = False
        self.integration_count = 0
        self.get_logger().info("✅ TSDF Real-time Scanner Ready!")
        print("⌨️  [Enter]: 스캔 시작/정지 | [s]: 저장 | [q]: 종료")

        self.key_thread = threading.Thread(target=self.keyboard_listener)
        self.key_thread.daemon = True
        self.key_thread.start()

    def info_callback(self, msg):
        if self.intrinsics is None:
            self.intrinsics = o3d.camera.PinholeCameraIntrinsic(
                msg.width, msg.height, msg.k[0], msg.k[4], msg.k[2], msg.k[5]
            )

    def color_callback(self, msg):
        try:
            self.color_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            cv2.imshow("Scanner View", self.color_img)
            cv2.waitKey(1)
        except: pass

    def depth_callback(self, msg):
        try: self.depth_img = self.bridge.imgmsg_to_cv2(msg, "passthrough")
        except: pass
        
        if self.is_scanning and self.color_img is not None and self.depth_img is not None and self.intrinsics is not None:
            self.integrate_frame()

    def integrate_frame(self):
        try:
            trans = self.tf_buffer.lookup_transform(
                'base_link', 'camera_color_optical_frame', rclpy.time.Time())
            
            t = trans.transform.translation
            r = trans.transform.rotation
            r_mat = Rotation.from_quat([r.x, r.y, r.z, r.w]).as_matrix()
            
            extrinsic = np.eye(4)
            extrinsic[:3, :3] = r_mat
            extrinsic[:3, 3] = [t.x, t.y, t.z]
            camera_pose = np.linalg.inv(extrinsic)

            o3d_color = o3d.geometry.Image(cv2.cvtColor(self.color_img, cv2.COLOR_BGR2RGB))
            o3d_depth = o3d.geometry.Image(self.depth_img)
            
            rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
                o3d_color, o3d_depth, 
                depth_scale=1000.0, depth_trunc=2.0, convert_rgb_to_intensity=False
            )

            self.volume.integrate(rgbd, self.intrinsics, camera_pose)
            self.integration_count += 1
            
            # [핵심] 10프레임마다 RViz로 3D 데이터 전송 (너무 자주는 렉걸림)
            if self.integration_count % 10 == 0:
                print(f"\r📦 데이터 누적 중... ({self.integration_count})", end="")
                self.publish_pointcloud()

        except Exception as e:
            pass

    # [추가됨] Open3D 데이터를 ROS2 메시지로 변환하여 발행하는 함수
    def publish_pointcloud(self):
        # 현재까지 쌓인 TSDF에서 포인트 클라우드 추출
        pcd = self.volume.extract_point_cloud()
        points = np.asarray(pcd.points)
        colors = np.asarray(pcd.colors)

        if len(points) == 0: return

        # ROS2 PointCloud2 메시지 생성
        msg = PointCloud2()
        msg.header = Header()
        msg.header.frame_id = "base_link" # 로봇 바닥 기준
        msg.header.stamp = self.get_clock().now().to_msg()
        
        msg.height = 1
        msg.width = len(points)
        
        # 필드 정의 (XYZ + RGB)
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = msg.point_step * len(points)
        msg.is_dense = True

        # 데이터 바이너리 패킹 (XYZRGB)
        buffer = []
        for i in range(len(points)):
            x, y, z = points[i]
            r, g, b = colors[i]
            # RGB를 하나의 float32로 압축
            rgb = struct.unpack('f', struct.pack('I', (int(r*255) << 16) | (int(g*255) << 8) | int(b*255)))[0]
            buffer.append(struct.pack('ffff', x, y, z, rgb))

        msg.data = b''.join(buffer)
        self.pcd_pub.publish(msg)

    def keyboard_listener(self):
        while rclpy.ok():
            cmd = input()
            if cmd == '':
                self.is_scanning = not self.is_scanning
                status = "🟢 스캔 시작 (RViz 확인!)" if self.is_scanning else "🔴 일시 정지"
                print(f"\n{status}")
            elif cmd == 's':
                self.save_mesh()
            elif cmd == 'q':
                print("종료합니다.")
                rclpy.shutdown()
                sys.exit(0)

    def save_mesh(self):
        if self.integration_count == 0:
            print("\n데이터가 없습니다.")
            return
        print("\n💾 메쉬 저장 중...")
        mesh = self.volume.extract_triangle_mesh()
        mesh.compute_vertex_normals()
        o3d.io.write_triangle_mesh(os.path.expanduser("~/scanned_model.ply"), mesh)
        print(f"✅ 저장 완료: ~/scanned_model.ply")

def main(args=None):
    rclpy.init(args=args)
    node = TSDFScanner()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()