import numpy as np
from loco_mujoco.task_factories import ImitationFactory, LAFAN1DatasetConf, DefaultDatasetConf, AMASSDatasetConf


# # example --> you can add as many datasets as you want in the lists!
env = ImitationFactory.make("UnitreeH1",
                            default_dataset_conf=DefaultDatasetConf(["squat", "walk"]),
                            lafan1_dataset_conf=LAFAN1DatasetConf(["dance2_subject4"]),
                            # if SMPL and AMASS are installed, you can use the following:
                            # amass_dataset_conf=AMASSDatasetConf(["DanceDB/DanceDB/20120911_TheodorosSourmelis/Capoeira_Theodoros_v2_C3D_poses",
                            #                                     "KIT/12/WalkInClockwiseCircle11_poses",
                            #                                     "HUMAN4D/HUMAN4D/Subject3_Medhi/INF_JumpingJack_S3_01_poses",
                            #                                     'KIT/359/walking_fast05_poses']),
                            th_params={'random_start': False, 'fixed_start_conf': (0, 0)},
                            n_substeps=20)

# 查看轨迹信息
print(f"总轨迹数量: {env.th.n_trajectories}")
print(f"轨迹数据总长度: {len(env.th.traj.data.qpos)}")
print(f"分割点: {env.th.traj.data.split_points}")
print(f"控制频率: {1/env.th.control_dt} Hz")
print(f"轨迹频率: {1/env.th.traj_dt} Hz")
print("=" * 50)

for i in range(env.th.n_trajectories):
    traj_len = env.th.len_trajectory(i)
    duration_seconds = traj_len * env.th.control_dt
    print(f"轨迹 {i}:")
    print(f"  - 长度: {traj_len} 步")
    print(f"  - 持续时间: {duration_seconds:.2f} 秒")
    print(f"  - 数据范围: {env.th.traj.data.split_points[i]} 到 {env.th.traj.data.split_points[i+1]-1}")
    print()

print("=" * 50)

# # 设置为顺序循环模式：修改环境的轨迹处理器属性
# env.th.random_start = False
# env.th.use_fixed_start = False  # 不使用固定起始位置

# env.play_trajectory(n_episodes=3, n_steps_per_episode=500, render=True)

# 方法2：手动控制每个轨迹的播放
for traj_idx in range(env.th.n_trajectories):
    print(f"播放轨迹 {traj_idx}")
    env.th.fixed_start_conf = (traj_idx, 0)  # 设置播放指定轨迹
    env.th.use_fixed_start = True
    env.play_trajectory(n_episodes=1, n_steps_per_episode=500, render=True)

# 方法3：创建自定义的顺序播放函数
def play_all_trajectories_in_order(env, n_episodes_per_traj=1, n_steps_per_episode=500, render=True):
    """
    按顺序播放所有轨迹
    
    Args:
        env: 环境对象
        n_episodes_per_traj: 每个轨迹播放的episode数
        n_steps_per_episode: 每个episode的步数
        render: 是否渲染
    """
    for traj_idx in range(env.th.n_trajectories):
        print(f"播放轨迹 {traj_idx} (长度: {env.th.len_trajectory(traj_idx)})")
        
        # 设置播放指定轨迹
        env.th.fixed_start_conf = (traj_idx, 0)
        env.th.use_fixed_start = True
        env.th.random_start = False
        
        # 播放这个轨迹
        env.play_trajectory(n_episodes=n_episodes_per_traj, 
                           n_steps_per_episode=n_steps_per_episode, 
                           render=render)

# 使用自定义函数（取消注释下面这行来使用）
# play_all_trajectories_in_order(env, n_episodes_per_traj=1, n_steps_per_episode=500)

# 方法4：根据轨迹实际长度播放完整轨迹
def play_complete_trajectories(env, render=True):
    """
    播放每个轨迹的完整长度
    """
    for traj_idx in range(env.th.n_trajectories):
        traj_length = env.th.len_trajectory(traj_idx)
        duration = traj_length * env.th.control_dt
        
        print(f"播放轨迹 {traj_idx} - 完整长度: {traj_length} 步 ({duration:.2f} 秒)")
        
        # 设置播放指定轨迹
        env.th.fixed_start_conf = (traj_idx, 0)
        env.th.use_fixed_start = True
        env.th.random_start = False
        
        # 播放完整轨迹（n_steps_per_episode=None 表示播放到轨迹结束）
        env.play_trajectory(n_episodes=1, 
                           n_steps_per_episode=None,  # None表示播放完整轨迹
                           render=render)
        print(f"轨迹 {traj_idx} 播放完成\n")

# 使用方法4播放完整轨迹（取消注释下面这行来使用）
# play_complete_trajectories(env)


