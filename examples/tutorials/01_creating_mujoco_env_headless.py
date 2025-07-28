import numpy as np
import os
from loco_mujoco import ImitationFactory

# Set environment variable for headless rendering
# os.environ['MUJOCO_GL'] = 'egl'  # Use EGL for headless rendering
# Alternative: os.environ['MUJOCO_GL'] = 'osmesa'  # Software rendering (slower but more compatible)
# !! actually, no need to set this for headless. To implement headless mode, you can simply avoid calling env.render().

# create the environment and task
env = ImitationFactory.make("FourierGR1T2", default_dataset_conf=dict(task="stepinplace1"))

# get the dataset for the chosen environment and task -- can be used for GAIL-like algorithms
#expert_data = env.create_dataset()

action_dim = env.info.action_space.shape[0]
print(f"Action dimension: {action_dim}")

env.reset()

# Don't call env.render() for headless mode
absorbing = False
i = 0

print("Running headless simulation...")
while i < 5000:  # Run for 5000 steps instead of infinite loop
    if i == 1000 or absorbing:
        env.reset()
        print(f"Episode reset at step {i}")
        i = 0
    
    action = np.random.randn(action_dim)
    nstate, reward, absorbing, done, info = env.step(action)
    
    # Optional: print progress every 500 steps
    if i % 500 == 0:
        print(f"Step {i}: reward = {reward:.3f}, absorbing = {absorbing}")
    
    i += 1

print("Simulation completed!")
