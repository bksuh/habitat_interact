import os
import time
import hydra
import threading
import habitat_sim

from omegaconf import DictConfig
from .utils.habitat_utils import *
from datetime import datetime
from scipy.spatial.transform import Rotation as R
import numpy as np
import cv2
import open3d as o3d

from habitat_data_collector.utils.gps_compass import HeadingSensor, EpisodicCompassSensor, EpisodicGPSSensor
from habitat_data_collector.mapping.object_point_cloud_map import ObjectPointCloudMap
from habitat_data_collector.mapping.obstacle_map import ObstacleMap
from habitat_data_collector.obs_transformers.utils import image_resize
from habitat_data_collector.util.geometry_utils import get_fov, rho_theta, xyz_yaw_to_tf_matrix


def save_rgb_depth(save_dir, rgb_img, depth_img):
    os.makedirs(save_dir, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    # --- RGB 저장 ---
    rgb_img_bgr = rgb_img[:, :, [2, 1, 0]]  # RGB -> BGR
    rgb_path = os.path.join(save_dir, f"{timestamp}_rgb.png")
    cv2.imwrite(rgb_path, rgb_img_bgr)

    # --- Depth 저장 (m → mm, float32 → uint16) ---
    depth_mm = (depth_img * 1000).astype(np.uint16)  # Convert to mm and cast
    depth_path = os.path.join(save_dir, f"{timestamp}_depth.png")
    cv2.imwrite(depth_path, depth_mm)

    # Depth 시각화 저장 (디버깅용)
    save_depth_visual(depth_img, os.path.join(save_dir, f"{timestamp}_depth_vis.png"))

    # 원본도 저장 (선택)
    np.save(os.path.join(save_dir, f"{timestamp}_depth.npy"), depth_img)
    
    print(f"Saved RGB: {rgb_path}")
    print(f"Saved Depth: {depth_path}")

def save_depth_visual(depth_img, save_path):
    depth_img_norm = (depth_img - depth_img.min()) / (depth_img.max() - depth_img.min() + 1e-6)
    depth_img_8bit = (depth_img_norm * 255).astype(np.uint8)
    cv2.imwrite(save_path, depth_img_8bit)

@hydra.main(version_base=None,
            config_path=str(Path(__file__).parent.parent / "config"),
            config_name='habitat_data_collector.yaml'
            )
def main(cfg: DictConfig) -> None:
    os.environ["MAGNUM_LOG"] = "quiet"
    os.environ["HABITAT_SIM_LOG"] = "quiet"

    os.makedirs(cfg.output_path, exist_ok=True)
    dataset_dir = Path(cfg.output_path) / cfg.dataset_name

    id = 1
    while True:
        scene_dir = dataset_dir / f"{cfg.scene_name}_{id}"
        if not scene_dir.exists():
            break
        id += 1 

    print(f"Collecting data for scene: {cfg.scene_name}, dataset: {cfg.dataset_name}, id: {id}")
    print(f"Dataset will be saved to: {scene_dir}")
    scene_dir.mkdir(parents=True, exist_ok=True)

    config = make_cfg(cfg)
    # print(config)

    sim = habitat_sim.Simulator(config)
    print("Simulator initialized.")
    scene = sim.semantic_scene
    print(f"House has {len(scene.levels)} levels, {len(scene.regions)} rooms, and {len(scene.objects)} semantic objects.")
    print('===============================================================================================================')

    #save scene info into the output directory
    save_scene_object_info(scene, scene_dir)

    agent = sim.initialize_agent(cfg.default_agent)
    #print(f"Agent initialized: {agent}")

    agent_state = habitat_sim.AgentState()
    sim.pathfinder.seed(cfg.data_cfg.seed)
    agent_state.position= np.array([0.0, 0.0, 0.0])
    agent.set_state(agent_state)
    epi_start_position = agent_state.position
    epi_start_rotation = agent_state.rotation

    print(f"Initial poistion {epi_start_position} with rotation {epi_start_rotation}")
    print('===============================================================================================================')

    target_fps = cfg.frame_rate
    frame_interval = 1 / target_fps


    recording = False
    help_count = 0
    all_actions = []

    last_time = time.time()
    # =================================== VLFM 내용 추가하는 부분 ============================================================
    object_map_erosion_size = 5
    min_obstacle_height = 0.61
    max_obstacle_height = 0.88
    obstacle_map_area_threshold = 1.5
    agent_radius = 0.18
    hole_area_thresh = 100000
    __object_map: ObjectPointCloudMap = ObjectPointCloudMap(erosion_size=object_map_erosion_size)
    _obstacle_map = ObstacleMap(min_height=min_obstacle_height,max_height=max_obstacle_height,area_thresh=obstacle_map_area_threshold,agent_radius=agent_radius,hole_area_thresh=hole_area_thresh,)
    min_depth = 0.5
    max_depth = 5.0
    camera_fov = 79
    camera_fov_rad = np.deg2rad(camera_fov)
    _camera_fov = camera_fov_rad
    image_width = 640
    _fx = _fy = image_width / (2 * np.tan(camera_fov_rad / 2))

    _observation_cache = {}
    while True:

        # control the frame rate
        elapsed_time = time.time() - last_time
        sleep_time = frame_interval - elapsed_time
        if sleep_time > 0:
            time.sleep(sleep_time)

        # Update last_time at the start of each loop iteration
        last_time = time.time()

        topdown_map = None
        sim.step_physics(frame_interval)


        obs = sim.get_sensor_observations()
        display_obs(obs, help_count, topdown_map, recording)
        #print(obs.keys())
        k, action = keyboard_control_fast()
        if k != -1:
            if action == "stop":
                cv2.destroyAllWindows()
                break

            if action == 'save_current_view':
                rgb_img = np.array(obs["color_sensor"])
                depth_img = np.array(obs["depth_sensor"])
                save_rgb_depth(scene_dir, rgb_img, depth_img)
                print("Current view saved.")
                continue

            if action == "map":
                map_count += 1
                continue

            action_time = time.time()

            sim.step(action)

            current_state = sim.get_agent(0).get_state()

            camera_yaw = EpisodicCompassSensor(epi_start_rotation, current_state.rotation)
            x, y = EpisodicGPSSensor(epi_start_position, epi_start_rotation, current_state.position, 2)
            print(f"이전 위치 {epi_start_position}, 현재위치 {current_state.position}")
            camera_yaw = camera_yaw[0].astype(np.float32)
            camera_position = np.array([x, -y, cfg.data_cfg.camera_height])
            robot_xy = camera_position[:2]

            tf_camera_to_episodic = xyz_yaw_to_tf_matrix(camera_position, camera_yaw)

            depth = obs["depth_sensor"]
            rgb = obs["color_sensor"]
            _obstacle_map.update_map(depth, tf_camera_to_episodic, min_depth, max_depth, fx=_fx, fy=_fy, topdown_fov=_camera_fov)
            result = _obstacle_map.visualize()
            
            cv2.imshow("Map Visualization", result)
            cv2.waitKey(0)
            cv2.destroyAllWindows()


    

    for action in all_actions:
        if action["action"] is not None:
            print(action)

    if len(all_actions) > 0:
        # restart the sim
        sim = habitat_sim.Simulator(config)
        # Get all the objects
        obj_attr_mgr = sim.get_object_template_manager()

        obj_attr_mgr.load_configs(cfg.objects_path)

        register_templates_from_handles(obj_attr_mgr, cfg)


    sim.close()


if __name__ == "__main__":

    main()