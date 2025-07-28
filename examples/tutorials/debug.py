import os
import mujoco

# Check current setting
print(f"MUJOCO_GL environment variable: {os.environ.get('MUJOCO_GL', 'Not set (will use default)')}")

# Try to get MuJoCo's actual backend choice
try:
    # This will show what backend MuJoCo actually initializes with
    model = mujoco.MjModel.from_xml_string("<mujoco><worldbody><body><geom size='0.1'/></body></worldbody></mujoco>")
    data = mujoco.MjData(model)
    
    # Try to create a renderer to see which backend is used
    renderer = mujoco.Renderer(model, height=100, width=100)
    print("MuJoCo successfully initialized renderer")
    print(f"Renderer backend: {type(renderer).__name__}")
    
except Exception as e:
    print(f"MuJoCo initialization error: {e}")
    print("This might indicate which backends are NOT available")