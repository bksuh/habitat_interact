# applications/utils/ros_data_collector.py

try:
    import rclpy
    import subprocess
    from rclpy.node import Node
    from nav_msgs.msg import Path
    from sensor_msgs.msg import Image, CameraInfo
    from nav_msgs.msg import Odometry
    from cv_bridge import CvBridge
    import numpy as np

    class ROSDataCollector(Node):
        def __init__(self, ros_enabled=False):
            super().__init__('data_collector')
            self.ros_enabled = ros_enabled
            self.bridge = CvBridge()

            if self.ros_enabled:
                # Initialize ROS publishers
                self.rgb_pub = self.create_publisher(Image, '/camera/rgb/image_raw', 10)
                self.depth_pub = self.create_publisher(Image, '/camera/depth/image_raw', 10)
                self.pose_pub = self.create_publisher(Odometry, '/camera/pose', 10)
                self.camera_info_pub = self.create_publisher(CameraInfo, 'camera_info', 10)

        def publish_rgb(self, rgb_img):
            if self.ros_enabled:
                # Process RGB image (flip channels from RGB to BGR)
                rgb_img_processed = rgb_img[:, :, [2, 1, 0]]  # Convert RGB to BGR
                ros_img = self.bridge.cv2_to_imgmsg(rgb_img_processed, encoding="bgr8")
                ros_img.header.stamp = self.get_clock().now().to_msg()
                self.rgb_pub.publish(ros_img)

        def publish_depth(self, depth_img):
            if self.ros_enabled:
                # Process Depth image (convert depth to mm)
                depth_img_processed = (depth_img * 1000).astype(np.uint16)  # Convert meters to millimeters
                ros_depth = self.bridge.cv2_to_imgmsg(depth_img_processed, encoding="16UC1")
                ros_depth.header.stamp = self.get_clock().now().to_msg()
                self.depth_pub.publish(ros_depth)

        def publish_pose(self, pose):
            if self.ros_enabled:
                # 创建 Odometry 消息
                odom_msg = Odometry()
                
                # 设置时间戳和坐标系
                odom_msg.header.stamp = self.get_clock().now().to_msg()
                odom_msg.header.frame_id = 'map'          # 父坐标系（全局坐标系）
                odom_msg.child_frame_id = ''     # 子坐标系（机器人自身）

                # 设置位置（pose[:3]）和姿态（pose[3:7]）
                odom_msg.pose.pose.position.x = pose[0]
                odom_msg.pose.pose.position.y = pose[1]
                odom_msg.pose.pose.position.z = pose[2]
                odom_msg.pose.pose.orientation.x = pose[3]
                odom_msg.pose.pose.orientation.y = pose[4]
                odom_msg.pose.pose.orientation.z = pose[5]
                odom_msg.pose.pose.orientation.w = pose[6]

                # 设置速度信息（默认为 0）
                odom_msg.twist.twist.linear.x = 0.0
                odom_msg.twist.twist.linear.y = 0.0
                odom_msg.twist.twist.linear.z = 0.0
                odom_msg.twist.twist.angular.x = 0.0
                odom_msg.twist.twist.angular.y = 0.0
                odom_msg.twist.twist.angular.z = 0.0

                # 发布 Odometry 消息
                self.pose_pub.publish(odom_msg)
        
        def publish_camera_info(self, fx, fy, cx, cy, width, height):
            camera_info_msg = CameraInfo()
            
            # intrinsic
            camera_info_msg.width = width
            camera_info_msg.height = height
            camera_info_msg.k = [float(fx), 0.0, float(cx), 0.0, float(fy), float(cy), 0.0, 0.0, 1.0]  # 3x3 内参矩阵
            camera_info_msg.p = [float(fx), 0.0, float(cx), 0.0, 0.0, float(fy), float(cy), 0.0, 0.0, 0.0, 1.0, 0.0]  # 投影矩阵，假设无畸变

            
            # timestamp
            current_time = self.get_clock().now().to_msg()
            camera_info_msg.header.stamp = current_time
            camera_info_msg.header.frame_id = 'camera_frame'

            # pub
            self.camera_info_pub.publish(camera_info_msg)

    class ROSDataListener(Node):
        def __init__(self, ros_enabled=True):
            super().__init__('listener_node')
            self.ros_enabled = ros_enabled

            self.latest_path = None  # 用于存储最近接收到的路径

            if self.ros_enabled:
                # Subscriber for action_path
                self.action_path_subscriber = self.create_subscription(
                    Path,  # 修改为 nav_msgs/Path
                    '/action_path',  # 监听的路径话题
                    self.action_path_callback,
                    10
                )

                self.get_logger().info("Listener initialized and ready to subscribe to topics.")

        def action_path_callback(self, msg):
            """
            Callback for the /action_path topic.

            Args:
                msg (Path): The Path message containing the path data.
            """
            # 将接收到的路径转换为简单列表表示 (x, y, z)
            current_path = [
                (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z) for pose in msg.poses
            ]
            current_path = self.transform_path_to_habitat(current_path)  # 转换路径到 habitat 世界坐标系

            # 如果新路径与上次路径一致，则跳过处理
            if self.latest_path == current_path:
                # self.get_logger().info("Received path is identical to the latest path. Skipping processing.")
                return

            # 更新最新路径
            self.latest_path = current_path
            self.get_logger().info(f"Received global_path with {len(current_path)} poses.")

        def get_latest_path(self):
            """
            Get the latest path received by the listener.

            Returns:
                list or None: The latest path as a list of (x, y, z) tuples, or None if no path is available.
            """
            return self.latest_path
        
        def transform_path_to_habitat(self, path_sys):

            pose_from_ros = [np.array(pose) for pose in path_sys]

            pose_in_habitat = []

            for point in pose_from_ros:
                # 扩展为齐次坐标
                pose_sys = np.eye(4)
                pose_sys[:3, 3] = point  # 只设置位移部分
                # 调用 get_habitat_pose 函数
                transformed_pose = self.get_habitat_pose(pose_sys)
                # 提取转换后的 3D 坐标
                transformed_point = transformed_pose[:3, 3]
                # 将 transformed_point 的第二项替换为 current_position 的第二项
                # transformed_point[1] = current_position[1]
                pose_in_habitat.append(tuple(transformed_point))
            
            return pose_in_habitat

        def get_habitat_pose(self, pose_sys):
            """
            Convert a pose from the system coordinate frame to the Habitat coordinate frame.

            This function transforms a given pose from the system coordinate frame to the
            Habitat coordinate frame using predefined transformation matrices.

            Args:
                pose_sys (numpy.ndarray): A 4x4 transformation matrix representing the
                                          pose in the system coordinate frame.

            Returns:
                numpy.ndarray: A 4x4 transformation matrix representing the pose in the
                               Habitat coordinate frame.
            """
            world_habi_to_world_sys = np.array(
                [[1, 0, 0, 0], [0, 0, -1, 0], [0, 1, 0, 0], [0, 0, 0, 1]]
            )
            cam_sys_to_cam_habi = np.array(
                [[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]]
            )

            world_sys_to_world_habi = np.linalg.inv(world_habi_to_world_sys)
            cam_habi_to_cam_sys = np.linalg.inv(cam_sys_to_cam_habi)

            # world_sys_to_world_habi @ cam_sys_to_world_sys @ cam_habi_to_cam_sys
            cam_habi_to_world_habi = world_sys_to_world_habi @ pose_sys @ cam_habi_to_cam_sys

            return cam_habi_to_world_habi
    
    def start_rosbag_recording(output_path):
        """
        Starts recording rosbag for given topics.

        :param topics: List of ROS topics to record.
        :param output_path: Path to save the rosbag file.
        """

        topics_to_record = ['/camera/rgb/image_raw', '/camera/depth/image_raw', '/camera/pose', '/camera_info']

        # Prepare the command for recording
        command = ['ros2', 'bag', 'record', '-o', output_path] + topics_to_record
        # Start recording in a subprocess
        rosbag_process = subprocess.Popen(command)
        return rosbag_process

    def stop_rosbag_recording(rosbag_process):
        """
        Stops the rosbag recording process.

        :param rosbag_process: The subprocess handle for rosbag recording.
        """
        rosbag_process.terminate()


except ImportError:
    print("ROS 2 environment not found. Skipping ROS 2 integration.")
    ROSDataCollector = None