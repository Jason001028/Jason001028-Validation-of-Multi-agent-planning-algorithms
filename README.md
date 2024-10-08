# Jason001028-Validation-of-Multi-agent-planning-algorithms
用于验证攻防决策项目所用到的几种集群围捕算法
这个项目实现了一个基于人工势场法（Artificial Potential Field, APF）和模型预测控制（Model Predictive Control, MPC）的多智能体追捕仿真系统。该系统模拟了多个追捕者（pursuers）围捕一个逃避者（evader）的场景。
主要特点
多智能体系统：包含多个追捕者和一个逃避者。

人工势场法（APF）：用于智能体之间的避碰和对墙壁的避让。

模型预测控制（MPC）：优化追捕者的运动轨迹。

虚拟目标点：通过虚拟目标点的分配来协调多个追捕者的行为。

匈牙利算法：用于追捕者与虚拟目标点的最优分配。

动态体力条系统：逃避者的健康值会随着被包围的程度而降低。

自适应行为：逃避者具有躲避追捕者和墙壁的能力。

技术细节
智能体初始化：在正方形区域内随机初始化所有智能体的位置。

追捕者控制：使用 MPC 和 APF 结合的方法来控制追捕者的运动，通过虚拟目标点来协调多个追捕者的行为，使用匈牙利算法进行追捕者与虚拟目标点的最优分配。

逃避者控制：具有目标导向的运动模式，通过 APF 方法实现对墙壁和追捕者的避让。

环境交互：实现了周期性边界条件，考虑了墙壁斥力，防止智能体撞墙。

基础类多目标围捕算法：APF,A_star,Voronoi

![APF](https://github.com/user-attachments/assets/dd2a0854-f8ef-4d15-84f7-9d5ae6e448af)

![image](https://github.com/user-attachments/assets/3316d337-b785-4c1e-9c39-3e2177ab9093)

