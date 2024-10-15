import carb
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})
import omni.timeline
from omni.isaac.core.world import World
from pegasus.simulator.params import ROBOTS, SIMULATION_ENVIRONMENTS
from pegasus.simulator.logic.graphical_sensors.monocular_camera import MonocularCamera
from pegasus.simulator.logic.backends.mavlink_backend import MavlinkBackend, MavlinkBackendConfig
from pegasus.simulator.logic.backends.ros2_backend import ROS2Backend
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from scipy.spatial.transform import Rotation
from pegasus.simulator.logic.graphical_sensors.lidar import Lidar
import requests
import cv2
import numpy as np
import time
import json

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

        # Initialize Lidar sensor with debug prints
        print("Initializing Lidar sensor...")
        self.lidar = Lidar("lidar", config={
            "frequency": 10.0,
            "sensor_configuration": "Velodyne_VLS128",  # Ensure this configuration is valid
            "position": [0.0, 0.0, 0.1],
            "orientation": [0.0, 0.0, 0.0],
            "show_render": False
        })

        # Check if Lidar is initialized properly
        if self.lidar:
            print("Lidar sensor initialized successfully.")
        else:
            print("Failed to initialize Lidar sensor.")

        # Attach sensors to the multirotor configuration
        config_multirotor.graphical_sensors = [self.camera, self.lidar]

        # Attach the multirotor to the world
        Multirotor(
            "/World/quadrotor",
            ROBOTS['Iris'],
            0,
            [0.0, 0.0, 0.07],
            Rotation.from_euler("XYZ", [0.0, 0.0, 0.0], degrees=True).as_quat(),
            config=config_multirotor,
        )

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
        state = self.lidar._state
        if state and "lidar_name" in state:
            # Use the Lidar interface to get the actual depth/range data
            lidar_interface = self.lidar.lidarInterface

            # Use get_point_cloud_data to fetch the point cloud data
            point_cloud_data = lidar_interface.get_point_cloud_data(self.lidar._lidar_name)

            if point_cloud_data:
                # The point cloud data is usually a numpy array with shape (N, 3) where N is the number of points
                # Each point consists of (x, y, z) coordinates.
                # You can compute the depth (distance from the origin) as the norm of each point's (x, y, z)
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
