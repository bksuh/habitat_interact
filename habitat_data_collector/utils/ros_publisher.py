import numpy as np
import struct
import time
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from utils.geometry import rotation_matrix_to_quaternion

class ROSPublisher:
    def __init__(self, node, cfg):
        self.cfg = cfg
        self.node = node
        self.bridge = CvBridge()

        # 创建所有需要的发布器
        self.global_path_publisher = node.create_publisher(Path, '/global_path', 10)
        self.local_path_publisher = node.create_publisher(Path, '/local_path', 10)
        self.action_path_publisher = node.create_publisher(Path, '/action_path', 10)

        self.image_publisher = node.create_publisher(Image, '/annotated_image', 10)
        self.fs_image_publisher = node.create_publisher(Image, '/fastsam_image', 10)
        self.fs_image_after_publisher = node.create_publisher(Image, '/fastsam_image_after', 10)

        self.pose_publisher = node.create_publisher(Odometry, '/odom', 10)

        self.local_rgb_publisher = node.create_publisher(PointCloud2, '/local_map/rgb', 10)
        self.local_sem_publisher = node.create_publisher(PointCloud2, '/local_map/semantic', 10)
        
        self.global_rgb_publisher = node.create_publisher(PointCloud2, '/global_map/rgb', 10)
        self.global_sem_publisher = node.create_publisher(PointCloud2, '/global_map/semantic', 10)

    def publish_all(self, hiemap):
        """
        统一发布所有消息：路径、图像、位姿和点云。
        """
        # 1 发布路径
        self._publish_path(hiemap.curr_global_path, 'global')
        self._publish_path(hiemap.curr_local_path, 'local')
        self._publish_path(hiemap.action_path, 'action')

        if self.cfg.use_rviz:

            # 2️ 发布图像
            self._publish_image(hiemap.detector.annotated_image, 'annotated')
            self._publish_image(hiemap.detector.annotated_image_fs, 'fastsam')
            self._publish_image(hiemap.detector.annotated_image_fs_after, 'fastsam_after')

            # 3️ 发布位姿
            self._publish_pose(hiemap.curr_pose)

            # 4️ 发布局部地图（RGB 点云 + 语义点云）

            if len(hiemap.local_map_manager.local_map):
                start_time = time.time()
                self._publish_local_map(hiemap.local_map_manager, hiemap.visualizer, publish_rgb=False)
                print(f"Publishing local map took {time.time() - start_time:.2f} seconds.")
            
            if len(hiemap.global_map_manager.global_map):
                self._publish_global_map(hiemap.global_map_manager, hiemap.visualizer, publish_rgb=False)

    def _publish_path(self, path, path_type):
        if path is None:
            return

        path_msg = Path()
        path_msg.header.stamp = self.node.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'

        for pos in path:
            pose_stamped = PoseStamped()
            pose_stamped.header = path_msg.header
            pose_stamped.pose.position.x = pos[0]
            pose_stamped.pose.position.y = pos[1]
            pose_stamped.pose.position.z = pos[2]
            pose_stamped.pose.orientation.w = 1.0
            path_msg.poses.append(pose_stamped)

        publisher = {
            'global': self.global_path_publisher,
            'local': self.local_path_publisher,
            'action': self.action_path_publisher
        }.get(path_type, None)

        if publisher:
            publisher.publish(path_msg)
            # self.node.get_logger().info(f"Published {path_type} path.")

    def _publish_image(self, image, image_type):
        if image is None:
            return

        ros_image = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
        publisher = {
            'annotated': self.image_publisher,
            'fastsam': self.fs_image_publisher,
            'fastsam_after': self.fs_image_after_publisher
        }.get(image_type, None)

        if publisher:
            publisher.publish(ros_image)

    def _publish_pose(self, pose_matrix):
        if pose_matrix is None:
            return

        # 提取平移和旋转
        translation = pose_matrix[:3, 3]
        quaternion = rotation_matrix_to_quaternion(pose_matrix[:3, :3])

        # 创建 Odometry 消息
        odom_msg = Odometry()
        odom_msg.header.stamp = self.node.get_clock().now().to_msg()  # 时间戳
        odom_msg.header.frame_id = "map"  # 父坐标系（全局坐标系）
        odom_msg.child_frame_id = ""  # 子坐标系（机器人自身坐标系）

        # 设置位置和姿态
        odom_msg.pose.pose.position.x = translation[0]
        odom_msg.pose.pose.position.y = translation[1]
        odom_msg.pose.pose.position.z = translation[2]
        odom_msg.pose.pose.orientation.x = float(quaternion[0])
        odom_msg.pose.pose.orientation.y = float(quaternion[1])
        odom_msg.pose.pose.orientation.z = float(quaternion[2])
        odom_msg.pose.pose.orientation.w = float(quaternion[3])

        # 设置速度信息（此处默认设置为 0）
        odom_msg.twist.twist.linear.x = 0.0
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = 0.0

        # 发布 Odometry 消息
        self.pose_publisher.publish(odom_msg)

    def _publish_local_map(self, local_map_manager, visualizer, publish_rgb=True):
        all_positions = []
        all_rgb_colors = []
        all_semantic_colors = []

        for local_obj in local_map_manager.local_map:
            obj_name = visualizer.obj_classes.get_classes_arr()[local_obj.class_id]
            positions = np.asarray(local_obj.pcd.points)
            colors = (np.asarray(local_obj.pcd.colors) * 255).astype(np.uint8)
            curr_obj_color = np.array(visualizer.obj_classes.get_class_color(obj_name)) * 255
            curr_obj_color = curr_obj_color.astype(np.uint8)
            semantic_colors = np.tile(curr_obj_color, (positions.shape[0], 1))

            all_positions.append(positions)
            all_rgb_colors.append(colors)
            all_semantic_colors.append(semantic_colors)

        if not all_positions:
            return

        all_positions = np.vstack(all_positions)
        all_rgb_colors = np.vstack(all_rgb_colors)
        all_semantic_colors = np.vstack(all_semantic_colors)

        if publish_rgb:
            self.publish_pointcloud(all_positions, all_rgb_colors, self.local_rgb_publisher, "map")
        
        self.publish_pointcloud(all_positions, all_semantic_colors, self.local_sem_publisher, "map")


    def _publish_global_map(self, global_map_manager, visualizer, publish_rgb=True):
        all_positions = []
        all_rgb_colors = []
        all_semantic_colors = []

        for global_obj in global_map_manager.global_map:
            obj_name = visualizer.obj_classes.get_classes_arr()[global_obj.class_id]
            positions = np.asarray(global_obj.pcd_2d.points)
            colors = (np.asarray(global_obj.pcd_2d.colors) * 255).astype(np.uint8)
            curr_obj_color = np.array(visualizer.obj_classes.get_class_color(obj_name)) * 255
            curr_obj_color = curr_obj_color.astype(np.uint8)
            semantic_colors = np.tile(curr_obj_color, (positions.shape[0], 1))

            all_positions.append(positions)
            all_rgb_colors.append(colors)
            all_semantic_colors.append(semantic_colors)

        if not all_positions:
            return

        all_positions = np.vstack(all_positions)
        all_rgb_colors = np.vstack(all_rgb_colors)
        all_semantic_colors = np.vstack(all_semantic_colors)

        if publish_rgb:
            self.publish_pointcloud(all_positions, all_rgb_colors, self.global_rgb_publisher, "map")
        
        self.publish_pointcloud(all_positions, all_semantic_colors, self.global_sem_publisher, "map")



    def publish_pointcloud(self, points, colors, publisher, frame_id):
        """
        更高效的点云发布：用 NumPy 批量打包 RGB 数据
        """
        num_points = points.shape[0]

        # ⚡ 高效 RGB 打包：避免 Python 循环
        r = colors[:, 0].astype(np.uint32)
        g = colors[:, 1].astype(np.uint32)
        b = colors[:, 2].astype(np.uint32)
        rgb_packed = (r << 16) | (g << 8) | b  # RGB → UINT32

        # ⚡ 优化数据拼接：一次性构建 PointCloud 数据
        cloud_data = np.zeros((num_points, 4), dtype=np.float32)
        cloud_data[:, :3] = points
        cloud_data[:, 3] = rgb_packed.view(np.float32)  # 直接转换

        # PointCloud2 字段定义
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1)
        ]

        # 消息头
        header = Header()
        header.stamp = self.node.get_clock().now().to_msg()
        header.frame_id = frame_id

        # ⚡ 构建 PointCloud2 消息
        pointcloud_msg = PointCloud2()
        pointcloud_msg.header = header
        pointcloud_msg.height = 1
        pointcloud_msg.width = num_points
        pointcloud_msg.fields = fields
        pointcloud_msg.is_bigendian = False
        pointcloud_msg.point_step = 16
        pointcloud_msg.row_step = 16 * num_points
        pointcloud_msg.is_dense = True
        pointcloud_msg.data = cloud_data.tobytes()

        # 🚀 发布点云
        publisher.publish(pointcloud_msg)
