# 机器人性能体系数据 (裁判系统串口协议 V1.7.0 0x0201)
uint16 current_hp                       # 机器人当前血量
uint16 maximum_hp                       # 机器人血量上限
# 允许发弹量 (裁判系统串口协议 V1.7.0 0x0208)
uint16 projectile_allowance_17mm        # 17mm 弹丸剩余发射次数
uint16 remaining_gold_coin              # 剩余金币数量

bool is_hp_deduced                      # 血量是否下降（上位机二次处理）

# 是否进入强化姿态
uint8 powerful_offensive_posture
uint8 powerful_defensive_posture
uint8 powerful_movement_posture
