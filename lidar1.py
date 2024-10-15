# Import necessary modules
import omni
import asyncio
import carb  # Import the carb module to access carb.getCachedInterface
from omni.isaac.range_sensor import _range_sensor
from pxr import UsdGeom, Gf, UsdPhysics

# Get the current stage from the USD context
stage = omni.usd.get_context().get_stage()

# Cache the material reader factory interface using carb.getCachedInterface()
material_reader_factory = carb.getCachedInterface("omni.sensors.nv.materials.core_material", "IMaterialReaderFactory")

# Acquire the Lidar sensor interface
lidarInterface = _range_sensor.acquire_lidar_sensor_interface()

# Get the timeline interface
timeline = omni.timeline.get_timeline_interface()

# Set the Lidar path to the correct one
lidarPath = "/sensor_01"  # Use the correct path where the sensor was created

# Create the Lidar sensor
result, sensor = omni.kit.commands.execute(
    "IsaacSensorCreateRtxLidar",
    path=lidarPath,
    parent=None,
    config="Example_Rotary",  # Or try "Example_Solid_State"
    translation=(0, 0, 1.0),
    orientation=Gf.Quatd(1.0, 0.0, 0.0, 0.0),  # Gf.Quatd is w,i,j,k
)

if not result:
    print("Failed to create Lidar sensor!")
else:
    print(f"Lidar sensor created at {sensor.GetPath()}")

# Function to print all available prims (objects) in the stage
def print_all_prims():
    for prim in stage.Traverse():
        path = prim.GetPath().pathString
        if not path.startswith("/background/"):  # Skip background paths
            print(path)

# Call this function to print all prims in the USD stage
print_all_prims()

# Async function to retrieve Lidar parameters
async def getLidarParams():
    # Wait for the next update and ensure Lidar is fully initialized
    await omni.kit.app.get_app().next_update_async()
    await asyncio.sleep(1)  # Adding a delay to allow Lidar to initialize
    timeline.pause()

    try:
        # Retrieve Lidar data from the correct path
        depth = lidarInterface.get_linear_depth_data("/World" + lidarPath)
        zenith = lidarInterface.get_zenith_data("/World" + lidarPath)
        azimuth = lidarInterface.get_azimuth_data("/World" + lidarPath)

        # Print Lidar data
        print("Depth Data:", depth)
        print("Zenith Data:", zenith)
        print("Azimuth Data:", azimuth)

    except Exception as e:
        print(f"Error retrieving Lidar data: {e}")

# Start playing the timeline and get the Lidar data
timeline.play()
asyncio.ensure_future(getLidarParams())
