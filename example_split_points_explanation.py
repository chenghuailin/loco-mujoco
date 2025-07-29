#!/usr/bin/env python3
"""
详细解释 Trajectory.concatenate() 的 split_points 机制

这个示例说明了 LocoMuJoCo 如何通过 split_points 来跟踪合并后轨迹中
每个原始轨迹的边界位置。
"""

import numpy as np

def explain_split_points():
    """解释 split_points 如何跟踪轨迹边界"""
    
    print("=" * 60)
    print("Split Points 机制详解")
    print("=" * 60)
    
    # 模拟三个不同长度的轨迹
    print("\n1. 假设我们有3个不同长度的轨迹：")
    traj_lengths = [100, 250, 150]  # 三个轨迹的长度
    
    for i, length in enumerate(traj_lengths):
        print(f"   轨迹 {i}: 长度 = {length} 步")
    
    print(f"\n   总数据长度: {sum(traj_lengths)} 步")
    
    # 演示 concatenate 过程中 split_points 的计算
    print("\n2. Concatenate 过程中 split_points 的计算：")
    
    # 原始轨迹的 split_points（每个轨迹都从0开始）
    original_split_points = [
        [0, 100],      # 轨迹0: 从索引0到99
        [0, 250],      # 轨迹1: 从索引0到249  
        [0, 150]       # 轨迹2: 从索引0到149
    ]
    
    print("   原始轨迹的 split_points:")
    for i, sp in enumerate(original_split_points):
        print(f"     轨迹 {i}: {sp}")
    
    # 合并后的 split_points 计算
    print("\n   合并过程:")
    new_split_points = []
    curr_n_samples = 0
    
    for i, (length, orig_sp) in enumerate(zip(traj_lengths, original_split_points)):
        # 调整 split_points，加上当前累积的样本数
        adjusted_sp = [x + curr_n_samples for x in orig_sp]
        print(f"     轨迹 {i}: 原始 {orig_sp} -> 调整后 {adjusted_sp}")
        
        # 只添加开始点（除了最后一个轨迹，还要添加结束点）
        new_split_points.extend(adjusted_sp[:-1])  # 不包括结束点
        curr_n_samples = adjusted_sp[-1]  # 更新累积样本数
    
    # 添加最后的结束点
    new_split_points.append(curr_n_samples)
    
    print(f"\n   最终合并的 split_points: {new_split_points}")
    
    # 验证边界信息
    print("\n3. 如何使用 split_points 找到每个轨迹的数据：")
    
    for i in range(len(traj_lengths)):
        start_idx = new_split_points[i]
        end_idx = new_split_points[i + 1]
        length = end_idx - start_idx
        
        print(f"   轨迹 {i}:")
        print(f"     - 在合并数据中的索引范围: [{start_idx}:{end_idx})")
        print(f"     - 长度: {length} 步")
        print(f"     - 数据切片: merged_data[{start_idx}:{end_idx}]")

def demonstrate_actual_usage():
    """演示实际的数据访问"""
    
    print("\n\n" + "=" * 60)
    print("实际数据访问示例")
    print("=" * 60)
    
    # 模拟合并后的轨迹数据
    total_length = 500  # 100 + 250 + 150
    merged_qpos = np.random.randn(total_length, 10)  # 假设有10个关节

    split_points = [0, 100, 350, 500]  # 对应三个轨迹的边界
    
    print(f"合并后的数据形状: {merged_qpos.shape}")
    print(f"Split points: {split_points}")
    
    print("\n访问各个轨迹的数据:")
    
    for traj_idx in range(3):
        start = split_points[traj_idx]
        end = split_points[traj_idx + 1]
        
        # 提取特定轨迹的数据
        traj_data = merged_qpos[start:end]
        
        print(f"\n轨迹 {traj_idx}:")
        print(f"  - 索引范围: [{start}:{end})")
        print(f"  - 数据形状: {traj_data.shape}")
        print(f"  - 长度: {end - start} 步")
        
        # 这就是 TrajectoryHandler 中 len_trajectory() 的实现
        print(f"  - len_trajectory({traj_idx}) = {end - start}")

def show_imitation_factory_process():
    """展示 ImitationFactory 中的实际过程"""
    
    print("\n\n" + "=" * 60)
    print("ImitationFactory 中的实际流程")
    print("=" * 60)
    
    print("1. ImitationFactory.make() 调用流程:")
    print("   ├── 加载 default_dataset_conf 轨迹")
    print("   ├── 加载 amass_dataset_conf 轨迹")  
    print("   ├── 加载 lafan1_dataset_conf 轨迹")
    print("   ├── 加载 custom_dataset_conf 轨迹")
    print("   └── Trajectory.concatenate(all_trajs)  # 关键步骤！")
    
    print("\n2. Trajectory.concatenate() 内部过程:")
    print("   ├── 调用 TrajectoryData.concatenate()")
    print("   ├── 重新计算 split_points")
    print("   ├── 合并所有数据数组 (qpos, qvel, xpos, etc.)")
    print("   └── 返回包含所有轨迹的单个 Trajectory 对象")
    
    print("\n3. 最终结果:")
    print("   ├── 一个大的数据数组包含所有轨迹")
    print("   ├── split_points 记录每个轨迹的边界")
    print("   └── TrajectoryHandler 使用 split_points 来:")
    print("       ├── 随机选择轨迹 (self.current_traj_no)")
    print("       ├── 计算轨迹长度 (len_trajectory())")
    print("       └── 提取特定轨迹的数据片段")

def explain_playback_control():
    """解释轨迹播放控制机制"""
    
    print("\n\n" + "=" * 60)
    print("轨迹播放控制机制")
    print("=" * 60)
    
    print("TrajectoryHandler 如何知道当前播放位置：")
    print("")
    print("关键状态变量:")
    print("  - current_traj_no: 当前播放的轨迹编号")
    print("  - current_step_traj: 当前轨迹内的步数")
    print("  - split_points: 轨迹边界信息")
    
    print("\n播放过程:")
    print("1. reset_state() 时:")
    print("   ├── 选择轨迹: current_traj_no = random.choice(range(n_trajectories))")
    print("   └── 设置起始位置: current_step_traj = 0 (或随机位置)")
    
    print("\n2. update_state() 时:")
    print("   ├── 计算全局索引: global_idx = split_points[current_traj_no] + current_step_traj")
    print("   ├── 提取数据: qpos = merged_data[global_idx]")
    print("   └── 更新步数: current_step_traj += 1")
    
    print("\n3. 轨迹结束检测:")
    print("   ├── 当前轨迹长度: traj_len = split_points[current_traj_no+1] - split_points[current_traj_no]")
    print("   ├── 是否结束: current_step_traj >= traj_len")
    print("   └── 重新开始: reset_state() -> 选择新轨迹")

if __name__ == "__main__":
    explain_split_points()
    demonstrate_actual_usage()
    show_imitation_factory_process()
    explain_playback_control()
    
    print("\n\n" + "=" * 60)
    print("总结")
    print("=" * 60)
    print("Split Points 的核心作用:")
    print("1. 🎯 轨迹边界标记: 记录每个轨迹在合并数组中的起始和结束位置")
    print("2. 🔍 数据定位: 通过索引快速找到特定轨迹的数据")
    print("3. 📏 长度计算: len = split_points[i+1] - split_points[i]")
    print("4. 🎮 播放控制: 支持随机选择轨迹和顺序播放")
    print("5. 💾 内存效率: 所有轨迹存储在连续内存中，访问高效")
