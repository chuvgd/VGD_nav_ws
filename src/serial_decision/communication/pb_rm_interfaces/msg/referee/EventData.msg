# 场地事件数据 (裁判系统串口协议 V1.7.0 0x0101)

# Constants for the occupation and activation states
uint8 UNOCCUPIED = 0                # Not occupied or not activated
uint8 OCCUPIED_FRIEND = 1           # Occupied or activated by friendly side
uint8 OCCUPIED_ENEMY = 2            # Occupied or activated by enemy side
uint8 OCCUPIED_BOTH = 3             # Occupied or activated by both sides

uint8 supply_zone

uint8 small_energy                  # 己方小能量机关的激活状态，1 为已激活
uint8 big_energy                    # 己方大能量机关的激活状态，1 为已激活

uint8 r_outpost
uint8 b_outpost       

uint8 r_outpost_control              
uint8 b_outpost_control