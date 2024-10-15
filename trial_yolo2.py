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
        #self.pg.load_environment(SIMULATION_ENVIRONMENTS["Hospital"]) #"/home/liu/Downloads/USD/test1.usd") # SIMULATION_ENVIRONMENTS["Office"]) # Curved Gridroom"])
        self.pg.load_environment("/home/liu/Downloads/USD/test7.usd")

        # Create the vehicle and configure the camera
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

            # Process camera data and interact with YOLO server
            self.process_camera_data()

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
            """if recent_detections:
                # Process the list of recent detections as needed
                print(f"Last {len(recent_detections)} Detections:", recent_detections)"""

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
