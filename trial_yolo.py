import carb
from isaacsim import SimulationApp

# Start Isaac Sim's simulation environment
simulation_app = SimulationApp({"headless": False})

# -----------------------------------
# The actual script should start here
# -----------------------------------
import omni.timeline
from omni.isaac.core.world import World

# Import the Pegasus API for simulating drones
from pegasus.simulator.params import ROBOTS, SIMULATION_ENVIRONMENTS
from pegasus.simulator.logic.graphical_sensors.monocular_camera import MonocularCamera
from pegasus.simulator.logic.backends.mavlink_backend import MavlinkBackend, MavlinkBackendConfig
from pegasus.simulator.logic.backends.ros2_backend import ROS2Backend
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface

# Auxiliary scipy and numpy modules
from scipy.spatial.transform import Rotation

# Additional imports
import cv2
import requests
import numpy as np
import time
import threading  # Import threading module


def publish_rgb_to_yolo(camera: MonocularCamera, freq, yolo_server_url):
    """
    Function to capture RGB images from the camera and send them to a YOLO server for object detection.

    Args:
        camera (MonocularCamera): The camera object from which to capture images.
        freq (int): Frequency of image capture.
        yolo_server_url (str): The URL of the YOLO server to which the images will be sent.
    """
    while True:
        # Get the camera state, which contains the image
        state = camera.state

        if state and "image" in state:
            # Convert the image to JPEG format
            image = state["image"]
            _, img_encoded = cv2.imencode('.jpg', image)
            img_bytes = img_encoded.tobytes()

            # Send the image to the YOLO server
            response = requests.post(yolo_server_url, files={'image': img_bytes})

            # Process the detection results
            if response.status_code == 200:
                detection_results = response.json()
                print("Detections:", detection_results)
            else:
                print(f"Failed to get response from YOLO server: {response.status_code}")

        # Wait for the next capture
        time.sleep(1.0 / freq)


class PegasusApp:
    """
    A Template class that serves as an example of how to build a simple Isaac Sim standalone App.
    """

    def __init__(self):
        """
        Method that initializes the PegasusApp and is used to set up the simulation environment.
        """

        # Acquire the timeline that will be used to start/stop the simulation
        self.timeline = omni.timeline.get_timeline_interface()

        # Start the Pegasus Interface
        self.pg = PegasusInterface()

        # Acquire the World, i.e., the singleton that controls the physics,
        # spawning asset primitives, etc.
        self.pg._world = World(**self.pg._world_settings)
        self.world = self.pg.world

        # Launch one of the worlds provided by NVIDIA
        self.pg.load_environment(SIMULATION_ENVIRONMENTS["Curved Gridroom"])

        from omni.isaac.core.objects import DynamicCuboid
        cube_2 = self.world.scene.add(
            DynamicCuboid(
                prim_path="/new_cube_2",
                name="cube_1",
                position=np.array([-3.0, 0, 2.0]),
                scale=np.array([1.0, 1.0, 1.0]),
                size=1.0,
                color=np.array([255, 0, 0]),
            )
        )

        # Create the vehicle
        config_multirotor = MultirotorConfig()
        mavlink_config = MavlinkBackendConfig({
            "vehicle_id": 0,
            "px4_autolaunch": True,
            "px4_dir": self.pg.px4_path,
            "px4_vehicle_model": self.pg.px4_default_airframe
        })
        config_multirotor.backends = [
            MavlinkBackend(mavlink_config),
            ROS2Backend(vehicle_id=1, config={
                "namespace": 'drone',
                "pub_sensors": False,
                "pub_graphical_sensors": True,
                "pub_state": True,
                "sub_control": False,
            })
        ]

        # Create a camera and lidar sensors
        config_multirotor.graphical_sensors = [MonocularCamera("camera", config={"update_rate": 60.0})]
        self.camera = config_multirotor.graphical_sensors[0]

        Multirotor(
            "/World/quadrotor",
            ROBOTS['Iris'],
            0,
            [0.0, 0.0, 0.07],
            Rotation.from_euler("XYZ", [0.0, 0.0, 0.0], degrees=True).as_quat(),
            config=config_multirotor,
        )

        # Reset the simulation environment so that all articulations (aka robots) are initialized
        self.world.reset()

        # Start publishing images to YOLO server
        yolo_server_url = "http://your-yolo-server-address"  # Replace with your YOLO server URL
        freq = 30  # Frequency of image capture
        self.publish_thread = threading.Thread(target=publish_rgb_to_yolo, args=(self.camera, freq, yolo_server_url))
        self.publish_thread.start()

    def run(self):
        """
        Method that implements the application main loop, where the physics steps are executed.
        """

        # Start the simulation
        self.timeline.play()

        # The "infinite" loop
        while simulation_app.is_running() and self.publish_thread.is_alive():
            # Update the UI of the app and perform the physics step
            self.world.step(render=True)

        # Cleanup and stop
        carb.log_warn("PegasusApp Simulation App is closing.")
        self.timeline.stop()
        simulation_app.close()


def main():
    # Instantiate the template app
    pg_app = PegasusApp()

    # Run the application loop
    pg_app.run()


if __name__ == "__main__":
    main()

