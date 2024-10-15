import carb
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})
import omni.timeline
import asyncio
from omni.isaac.core.world import World
from pegasus.simulator.params import ROBOTS, SIMULATION_ENVIRONMENTS
from pegasus.simulator.logic.graphical_sensors.monocular_camera import MonocularCamera
from pegasus.simulator.logic.backends.mavlink_backend import MavlinkBackend, MavlinkBackendConfig
from pegasus.simulator.logic.backends.ros2_backend import ROS2Backend
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from scipy.spatial.transform import Rotation
import requests
import cv2
import numpy as np
import time
import json

# For LiDAR imports
import omni
from omni.isaac.range_sensor import _range_sensor
from pxr import Gf
import logging

class PegasusApp:
    """
    A Template class that serves as an example on how to build a simple Isaac Sim standalone App.
    """

    def __init__(self):
        # Initialize logging
        logging.basicConfig(level=logging.INFO)
        logging.info("Initializing the Pegasus App...")

        # Initialize the timeline
        logging.info("Initializing the timeline...")
        self.timeline = omni.timeline.get_timeline_interface()

        # Start the Pegasus Interface
        self.pg = PegasusInterface()

        # Attempt to initialize the world
        logging.info("Initializing the world...")
        try:
            self.pg._world = World(**self.pg._world_settings)
            self.world = self.pg.world
        except Exception as e:
            logging.error(f"Error initializing the world: {e}")
            self.world = None

        if self.world is None:
            logging.error("World initialization failed!")
        else:
            logging.info("World initialized successfully!")

        # Load the environment
        logging.info("Loading the environment...")
        try:
            self.pg.load_environment(SIMULATION_ENVIRONMENTS["Hospital"])
        except Exception as e:
            logging.error(f"Error loading environment: {e}")

        if self.pg.world is None:
            logging.error("Failed to load environment!")
        else:
            logging.info("Environment loaded successfully!")

        # Create the vehicle and configure the camera
        if self.world:
            self.initialize_vehicle_and_camera()

        # Initialize LiDAR sensor and attach to drone
        if self.world:
            self.initialize_lidar()

        # Acquire the LiDAR interface to retrieve data
        if self.world:
            self.lidar_interface = _range_sensor.acquire_lidar_sensor_interface()

        # Reset the simulation environment
        if self.world:
            logging.info("Resetting the world...")
            self.world.reset()

        self.stop_sim = False

    def initialize_vehicle_and_camera(self):
        """
        Initializes the drone (multirotor) and attaches a camera.
        """
        logging.info("Initializing vehicle and camera...")
        try:
            config_multirotor = MultirotorConfig()
            mavlink_config = MavlinkBackendConfig({
                "vehicle_id": 0,
                "px4_autolaunch": True,
                "px4_dir": self.pg.px4_path,
                "px4_vehicle_model": self.pg.px4_default_airframe
            })
            config_multirotor.backends = [
                MavlinkBackend(mavlink_config),
                ROS2Backend(vehicle_id=1,
                            config={
                                "namespace": 'drone',
                                "pub_sensors": False,
                                "pub_graphical_sensors": True,
                                "pub_state": True,
                                "sub_control": False
                            })
            ]

            # Create a camera sensor
            self.camera = MonocularCamera("camera", config={"update_rate": 60.0})
            config_multirotor.graphical_sensors = [self.camera]

            # Initialize the multirotor (drone)
            Multirotor(
                "/World/quadrotor",
                ROBOTS['Iris'],
                0,
                [0.0, 0.0, 0.07],
                Rotation.from_euler("XYZ", [0.0, 0.0, 0.0], degrees=True).as_quat(),
                config=config_multirotor,
            )
            logging.info("Vehicle and camera initialized successfully.")
        except Exception as e:
            logging.error(f"Error initializing vehicle and camera: {e}")

    def initialize_lidar(self):
        """
        Method to initialize and attach the LiDAR sensor to the drone.
        """
        logging.info("Initializing LiDAR...")
        lidar_config = "Example_Rotary"  # LiDAR configuration

        try:
            # Create the lidar sensor and attach it to the drone
            result, self.lidar_sensor = omni.kit.commands.execute(
                "RangeSensorCreateLidar",
                path="/World/quadrotor/lidar",  # Attach LiDAR to the drone
                parent="/World/quadrotor",
                min_range=0.4,
                max_range=100.0,
                draw_points=True,
                draw_lines=False,
                horizontal_fov=360.0,
                vertical_fov=30.0,
                horizontal_resolution=0.4,
                vertical_resolution=4.0,
                rotation_rate=0.0,
                high_lod=False,
                yaw_offset=0.0,
                enable_semantics=False,
            )

            if not result:
                logging.error(f"Failed to create LiDAR at path '/World/quadrotor/lidar'")
            else:
                logging.info("LiDAR initialized successfully.")
        except Exception as e:
            logging.error(f"Error initializing LiDAR: {e}")

    async def get_lidar_data(self):
        """
        Asynchronous function to retrieve LiDAR data after each frame.
        """
        await omni.kit.app.get_app().next_update_async()  # Wait one frame for LiDAR data
        self.timeline.pause()  # Optionally pause the simulation to gather data

        # Fetch LiDAR data from the interface
        try:
            depth = self.lidar_interface.get_linear_depth_data("/World/quadrotor/lidar")
            zenith = self.lidar_interface.get_zenith_data("/World/quadrotor/lidar")
            azimuth = self.lidar_interface.get_azimuth_data("/World/quadrotor/lidar")

            # Print LiDAR data
            print("LiDAR Depth Data:", depth)
            print("LiDAR Zenith Data:", zenith)
            print("LiDAR Azimuth Data:", azimuth)

        except Exception as e:
            logging.error(f"Error fetching LiDAR data: {e}")

        self.timeline.play()  # Resume the simulation after gathering data

    def run(self):
        """
        Method that implements the application main loop, where the physics steps are executed.
        """
        if not self.world:
            logging.error("Cannot start simulation. World is not initialized.")
            return

        # Start the simulation
        self.timeline.play()

        # The "infinite" loop
        while simulation_app.is_running() and not self.stop_sim:
            # Update the UI of the app and perform the physics step
            self.world.step(render=True)

            # Process camera and LiDAR data
            self.process_camera_data()

            # Run the asynchronous LiDAR data fetching
            asyncio.ensure_future(self.get_lidar_data())

        # Cleanup and stop
        logging.warning("PegasusApp Simulation App is closing.")
        self.timeline.stop()
        simulation_app.close()

    def process_camera_data(self):
        """
        Method to process camera data and interact with the YOLO server.
        """
        # YOLO server URL for detection
        yolo_detect_url = "http://localhost:5000/detect"

        # Fetch image data from the camera
        state = self.camera.state
        if state and "camera" in state:
            img_data = state["camera"].get_rgba()

            # Convert RGBA image data to BGR format expected by OpenCV
            img_bgr = cv2.cvtColor(img_data, cv2.COLOR_RGBA2BGR)

            # Encode image as JPEG
            success, img_encoded = cv2.imencode('.jpg', img_bgr)
            if not success:
                print("Failed to encode image")
                return

            img_bytes = img_encoded.tobytes()

            # Send the image to the YOLO server for detection
            response = requests.post(yolo_detect_url, files={'image': img_bytes})

            # Process the detection results
            if response.status_code == 200:
                detection_results = response.json()
                if detection_results:
                    print("Immediate Detections:", detection_results)
            else:
                print(f"Failed to get response from YOLO server: {response.status_code}")
                return

        time.sleep(1.0 / 60)  # Adjust this delay according to your desired frequency

def main():
    # Instantiate the PegasusApp
    pg_app = PegasusApp()

    # Run the application loop
    pg_app.run()

if __name__ == "__main__":
    main()

