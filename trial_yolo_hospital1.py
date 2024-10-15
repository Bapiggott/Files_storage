import carb
from isaacsim import SimulationApp

# Ensure necessary extensions are enabled
simulation_app = SimulationApp({
    "headless": False,
    "load_extensions": {
        "omni.isaac.range_sensor": {},
        "omni.isaac.sensor": {}
    }
})

import omni.timeline
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

# Include necessary imports for the Lidar class
import omni.kit.commands
from pxr import Gf, UsdGeom
from omni.isaac.range_sensor import _range_sensor
from omni.usd import get_stage_next_free_path
import omni.replicator.core as rep


class Lidar:
    def __init__(self, lidar_name, config={}):
        """
        Initialize the Lidar class
        """
        # Sensor type and update rate
        self.sensor_type = "Lidar"
        self.update_rate = config.get("frequency", 10.0)
        self._update_period = 1.0 / self.update_rate
        self._time_since_last_update = 0.0

        # Setup the name of the lidar primitive path
        self._lidar_name = lidar_name
        self._stage_prim_path = ""

        # Configurations of the Lidar
        self._position = config.get("position", np.array([0.0, 0.0, 0.10]))
        self._orientation = Rotation.from_euler(
            "ZYX",
            config.get("orientation", np.array([0.0, 0.0, 0.0])),
            degrees=True
        ).as_quat()
        self._sensor_configuration = config.get("sensor_configuration", "Velodyne_VLS128")
        self._show_render = config.get("show_render", False)

        # Create the lidar interface
        self.lidarInterface = _range_sensor.acquire_lidar_sensor_interface()  # Used to interact with the LIDAR

        self._state = {}
        self.initialized = False

    def initialize(self, vehicle):
        """
        Initialize the Lidar sensor
        """
        self._vehicle = vehicle

        # Set the parent path to the vehicle prim path
        parent_path = self._vehicle.prim_path
        print(f"Vehicle prim path: {parent_path}")
        parent_prim = PegasusInterface().world.stage.GetPrimAtPath(parent_path)
        print(f"Parent prim valid: {parent_prim.IsValid()}")

        # Ensure the parent prim is valid
        if not parent_prim.IsValid():
            print(f"Parent prim {parent_path} does not exist.")
            return  # Or handle the error as needed

        # Get the complete stage prefix for the lidar
        self._stage_prim_path = get_stage_next_free_path(
            PegasusInterface().world.stage,
            parent_path + "/" + self._lidar_name,
            False
        )

        # Prepare sensor configuration parameters without 'translation' and 'orientation'
        sensor_params = {
            "path": self._stage_prim_path,
            "parent": parent_path,
            "min_range": 0.1,
            "max_range": 100.0,
            "draw_points": False,
            "draw_lines": False,
            "horizontal_fov": 360.0,
            "vertical_fov": 30.0,
            "horizontal_resolution": 0.2,
            "vertical_resolution": 1.0,
            "rotation_rate": 10.0,  # Rotations per second, adjust as needed
            "high_lod": False,
            "yaw_offset": 0.0,
            "enable_semantics": False
        }

        # Create the Lidar sensor using the correct command
        success, sensor = omni.kit.commands.execute(
            "RangeSensorCreateLidar",
            **sensor_params
        )

        if not success:
            print("Failed to create Lidar sensor.")
        else:
            print("Lidar sensor created successfully.")

            # Get the prim of the Lidar sensor
            prim = PegasusInterface().world.stage.GetPrimAtPath(self._stage_prim_path)
            print(f"Prim at path {self._stage_prim_path} is valid: {prim.IsValid()}")
            if not prim.IsValid():
                print(f"Prim at path {self._stage_prim_path} is invalid.")
            else:
                xform_api = UsdGeom.Xformable(prim)

                # Set the translation
                translate_op = xform_api.AddTranslateOp()
                translate_op.Set(Gf.Vec3d(*self._position))

                # Set the rotation (convert quaternion to Gf.Quatf)
                orient_op = xform_api.AddOrientOp()
                orient_op.Set(Gf.Quatf(
                    self._orientation[3],
                    self._orientation[0],
                    self._orientation[1],
                    self._orientation[2]
                ))

                # Set initialized to True only if the prim is valid
                self.initialized = True

    def start(self):
        # If show_render is True, then create a render product for the lidar in the Isaac Sim environment
        if self._show_render:
            render_prod_path = rep.create.render_product(
                self._stage_prim_path, [1, 1], name=self._lidar_name + "_isaac_render"
            )
            writer = rep.writers.get("RtxLidarDebugDrawPointCloudBuffer")
            writer.initialize()
            writer.attach([render_prod_path])

    @property
    def state(self):
        """
        (dict) The 'state' of the sensor, i.e., the data produced by the sensor at any given point in time
        """
        return self._state

    def update(self, state, dt):
        """
        Method that gets the data from the lidar and returns it as a dictionary.

        Args:
            state: The current state of the vehicle.
            dt (float): The time elapsed between the previous and current function calls (s).

        Returns:
            (dict) A dictionary containing the current state of the sensor (the data produced by the sensor)
        """
        if not self.initialized:
            return

        self._time_since_last_update += dt
        if self._time_since_last_update >= self._update_period:
            self._time_since_last_update = 0.0

            # Just return the prim path of the lidar
            self._state = {"stage_prim_path": self._stage_prim_path}


class PegasusApp:
    """
    A Template class that serves as an example on how to build a simple Isaac Sim standalone App.
    """

    def __init__(self):
        """
        Method that initializes the PegasusApp and is used to setup the simulation environment.
        """
        # Acquire the timeline that will be used to start/stop the simulation
        self.timeline = omni.timeline.get_timeline_interface()

        # Start the Pegasus Interface
        self.pg = PegasusInterface()

        # Acquire the World
        self.pg._world = World(**self.pg._world_settings)
        self.world = self.pg.world

        # Launch one of the worlds provided by NVIDIA
        self.pg.load_environment(SIMULATION_ENVIRONMENTS["Hospital"])

        # Create the vehicle and configure the camera and Lidar
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

        # Initialize Lidar sensor
        print("Initializing Lidar sensor...")
        self.lidar = Lidar("lidar", config={
            "frequency": 10.0,
            # Ensure this configuration is valid or use default parameters
            "sensor_configuration": "Velodyne_VLS128",
            "position": [0.0, 0.0, 0.1],
            "orientation": [0.0, 0.0, 0.0],
            "show_render": False
        })

        # Attach sensors to the multirotor configuration
        config_multirotor.graphical_sensors = [self.camera]
        # Note: Exclude self.lidar from graphical_sensors for now

        # Attach the multirotor to the world
        self.vehicle = Multirotor(
            "/World/quadrotor",
            ROBOTS['Iris'],
            0,
            [0.0, 0.0, 0.07],
            Rotation.from_euler("XYZ", [0.0, 0.0, 0.0], degrees=True).as_quat(),
            config=config_multirotor,
        )

        # Now initialize the Lidar sensor after the vehicle is created
        self.lidar.initialize(self.vehicle)
        self.lidar.start()

        # Reset the simulation environment
        self.world.reset()

        self.stop_sim = False

    def process_sensor_data(self):
        """
        Method to process sensor data (camera and Lidar) and interact with external services.
        """
        # Process camera data (similar to your existing camera logic)
        self.process_camera_data()

        # Fetch and process Lidar data
        self.process_lidar_data()

    def process_lidar_data(self):
        """
        Method to process Lidar data and print depth information.
        """
        # Get the current Lidar state
        self.lidar.update(None, self.world.get_physics_dt())
        state = self.lidar.state
        if state and "stage_prim_path" in state:
            # Use the Lidar interface to get the actual depth/range data
            lidar_interface = self.lidar.lidarInterface

            # Use get_point_cloud_data to fetch the point cloud data using the full prim path
            point_cloud_data = lidar_interface.get_point_cloud_data(state['stage_prim_path'])

            if point_cloud_data is not None and len(point_cloud_data) > 0:
                # The point cloud data is usually a numpy array with shape (N, 3)
                depths = np.linalg.norm(point_cloud_data, axis=1)

                # Print out the depth values
                print("Lidar Depths (in meters):", depths)

                # Optionally, limit the output for clarity
                print("First 10 depth values:", depths[:10])
            else:
                print("No Lidar data available.")
        else:
            print("Lidar state not available.")

        time.sleep(1.0 / 10)  # Adjust this delay according to the Lidar update rate

    def run(self):
        """
        Method that implements the application main loop, where the physics steps are executed.
        """
        # Start the simulation
        self.timeline.play()

        # Wait for a few frames to ensure sensors are producing data
        for _ in range(100):
            self.world.step(render=True)

        # The "infinite" loop
        while simulation_app.is_running() and not self.stop_sim:
            # Update the UI of the app and perform the physics step
            self.world.step(render=True)

            # Process sensor data and interact with YOLO server
            self.process_sensor_data()

        # Cleanup and stop
        carb.log_warn("PegasusApp Simulation App is closing.")
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
                # Handle the immediate detection results (e.g., draw bounding boxes, etc.)
                if detection_results != []:
                    print("Immediate Detections:", detection_results)
                    # Process the immediate detections as needed
            else:
                print(f"Failed to get response from YOLO server: {response.status_code}")
                print(f"Error content: {response.text}")
                return

            # Optionally, retrieve the last 5 detections from the YOLO server
            recent_detections = self.get_recent_detections()

        time.sleep(1.0 / 60)  # Adjust this delay according to your desired frequency

    def get_recent_detections(self):
        """
        Retrieves the last 5 detections from the YOLO server.
        """
        yolo_recent_detections_url = "http://localhost:5000/detections/recent"
        try:
            response = requests.get(yolo_recent_detections_url)
            if response.status_code == 200:
                recent_detections = response.json()
                return recent_detections
            elif response.status_code == 404:
                print("No recent detection results available.")
                return []
            else:
                print(f"Error: Received status code {response.status_code}")
                return []
        except requests.exceptions.RequestException as e:
            print(f"Request failed: {e}")
            return []


def main():
    # Instantiate the template app
    pg_app = PegasusApp()

    # Run the application loop
    pg_app.run()


if __name__ == "__main__":
    main()
