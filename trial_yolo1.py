#!/usr/bin/env python

import carb
from isaacsim import SimulationApp

# Start Isaac Sim's simulation environment
simulation_app = SimulationApp({"headless": False})

# The actual script should start here
import omni.timeline
from omni.isaac.core.world import World

# Import the Pegasus API
from pegasus.simulator.params import ROBOTS, SIMULATION_ENVIRONMENTS
from pegasus.simulator.logic.graphical_sensors.monocular_camera import MonocularCamera
from pegasus.simulator.logic.backends.mavlink_backend import MavlinkBackend, MavlinkBackendConfig
from pegasus.simulator.logic.backends.ros2_backend import ROS2Backend
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface

from flask import Flask, request, jsonify

# YOLO server
app = Flask(__name__)

@app.route('/yolo', methods=['POST'])
def yolo_inference():
    # Dummy YOLO inference logic, replace with actual implementation
    data = request.get_json()
    print(f"Received data for YOLO inference: {data}")
    # Respond with dummy data
    return jsonify({"inference_result": "dummy_result"})


class PegasusApp:
    def __init__(self):
        self.timeline = omni.timeline.get_timeline_interface()
        self.pg = PegasusInterface()
        self.pg._world = World(**self.pg._world_settings)
        self.world = self.pg.world
        self.pg.load_environment(SIMULATION_ENVIRONMENTS["Curved Gridroom"])

        from omni.isaac.core.objects import DynamicCuboid
        import numpy as np
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
                            "sub_control": False,})]

        config_multirotor.graphical_sensors = [MonocularCamera("camera", config={"update_rate": 60.0})]

        Multirotor(
            "/World/quadrotor",
            ROBOTS['Iris'],
            0,
            [0.0, 0.0, 0.07],
            Rotation.from_euler("XYZ", [0.0, 0.0, 0.0], degrees=True).as_quat(),
            config=config_multirotor,
        )

        self.world.reset()
        self.stop_sim = False

    def run(self):
        self.timeline.play()
        while simulation_app.is_running() and not self.stop_sim:
            self.world.step(render=True)
        carb.log_warn("PegasusApp Simulation App is closing.")
        self.timeline.stop()
        simulation_app.close()

def main():
    # Start the YOLO server in a separate thread
    import threading
    server_thread = threading.Thread(target=app.run, kwargs={"port": 5000})
    server_thread.start()

    # Instantiate and run PegasusApp
    pg_app = PegasusApp()
    pg_app.run()

if __name__ == "__main__":
    main()
