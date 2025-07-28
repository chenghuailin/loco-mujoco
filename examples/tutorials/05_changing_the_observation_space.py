from loco_mujoco.core import ObservationType
from loco_mujoco import ImitationFactory


observation_spec = [
    ObservationType.FreeJointPosNoXY(obs_name="free_joint", xml_name="root"),
    ObservationType.FreeJointVel(obs_name="free_joint_vel", xml_name="root"),
    ObservationType.JointPos(obs_name="joint_pos", xml_name="left_hip_pitch"),
    ObservationType.JointVel(obs_name="joint_vel1", xml_name="right_hip_pitch"),
    ObservationType.JointVel(obs_name="joint_vel2", xml_name="left_knee"),
    ObservationType.BodyPos(obs_name="head_pos", xml_name="head"),
    ObservationType.LastAction(obs_name="last_action"),
    # define many more in the order you want ...
]

env = ImitationFactory.make("ToddlerBot", observation_spec=observation_spec,
                            default_dataset_conf=dict(task="walk"))

# checkout the detailed observation space (opens webbrowser)
env.create_observation_summary(filename="observation_space_summary.html")

# run training ....
# this works accordingly for Mjx and Gymnasium environments!


# ## Added because env.create_observation_summary() does not work

# # Print observation space info instead
# print("Observation space created successfully!")
# print(f"Observation space shape: {env.observation_space.shape}")
# print(f"Action space shape: {env.action_space.shape}")

# # Test the environment
# env.reset()
# print("Environment reset successfully!")

# # Run a few steps to verify everything works
# import numpy as np
# for i in range(10):
#     action = np.random.randn(env.action_space.shape[0])
#     obs, reward, done, truncated, info = env.step(action)
#     print(f"Step {i}: obs_shape={obs.shape}, reward={reward:.3f}")

# print("Custom observation space working correctly!")
