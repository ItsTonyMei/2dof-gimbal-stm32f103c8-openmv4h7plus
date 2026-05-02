# 通讯协议

## 1. 概述

本文档描述二自由度云台系统的通讯协议，包括串口通讯、蓝牙通讯和ROS通讯三种控制方式。

**资料包位置**：`WHEELTEC_二自由度云台资料包（C06主控版-当前版本）`

**相关文档**：
- 蓝牙协议：`2.使用教程与开发手册/C06B主板外设资源教程视频/4. 主板外设讲解/6. 蓝牙/单片机与APP通信协议.pdf`
- Python例程：`9.ROS控制例程与Python控制例程/2.python控制例程/python_demo.py`
- ROS例程：`9.ROS控制例程与Python控制例程/1.ROS控制例程/demo01.zip`
- 例程说明：`9.ROS控制例程与Python控制例程/例程使用说明.pdf`

**注意事项**：
- 360度底舵机版本仅支持PS2遥控控制，不支持串口、蓝牙、ROS和Python控制
- 270度底舵机版本支持所有控制方式

## 2. 串口通讯

串口通讯是云台控制的基础方式，通过USB转串口（CH9102芯片）或蓝牙模块连接主控板。

### 2.1 串口参数

| 参数 | 值 |
|------|-----|
| 波特率 | 115200 |
| 数据位 | 8 |
| 停止位 | 1 |
| 校验位 | 无 |
| 流控制 | 无 |

### 2.2 指令格式

串口指令采用10字节固定长度格式：

| 字节索引 | 内容 | 说明 |
|----------|------|------|
| 0 | 0xFF | 帧头字节1 |
| 1 | 0xFE | 帧头字节2 |
| 2 | angle_bottom | 云台舵机角度（底舵机，水平旋转） |
| 3 | angle_top | 摆臂舵机角度（俯仰控制） |
| 4 | 0x00 | 保留 |
| 5 | 0x00 | 保留 |
| 6 | 0x00 | 保留 |
| 7 | 0x00 | 保留 |
| 8 | 0x00 | 保留 |
| 9 | parity | BCC校验码（angle_bottom XOR angle_top） |

**帧头**：固定为 `0xFF 0xFE`，用于识别指令起始位置。

**角度范围**：
- 270度舵机版本：有效范围约 0-180 度
- 角度值直接发送原始数值，单片机根据源码版本转换为PWM占空比

**校验码**：采用BCC（异或校验），计算公式：
```
parity = angle_bottom XOR angle_top
```

### 2.3 指令集说明

#### 2.3.1 角度控制指令

控制云台两个舵机的角度：

```
0xFF 0xFE [angle_bottom] [angle_top] 0x00 0x00 0x00 0x00 0x00 [parity]
```

**参数说明**：
- `angle_bottom`：底舵机角度（云台水平旋转），范围 0-180
- `angle_top`：摆臂舵机角度（俯仰控制），范围 0-180

**示例**：设置底舵机为90度，摆臂舵机为45度
```
0xFF 0xFE 0x5A 0x2D 0x00 0x00 0x00 0x00 0x00 0x77
```
计算：parity = 0x5A XOR 0x2D = 0x77

#### 2.3.2 预留指令

字节4-8目前保留为0x00，用于将来扩展功能。

### 2.4 串口指令示例

以下是基于Python的串口控制示例：

```python
import serial

# 串口参数
port = 'COM22'        # 根据电脑串口号修改
baudrate = 115200

# 角度设置
angle_bottom = 90     # 云台舵机
angle_top = 45        # 摆臂舵机

# BCC校验码
parity = angle_bottom ^ angle_top

# 构造10字节指令
hex_data = [0xff, 0xfe, angle_bottom, angle_top,
            0x00, 0x00, 0x00, 0x00, 0x00, parity]
byte_data = bytes(hex_data)

# 发送指令
ser = serial.Serial(port, baudrate, timeout=1)
ser.write(byte_data)
```

## 3. 蓝牙通讯

蓝牙通讯基于串口协议，通过蓝牙模块（如HC-05/HC-06）与手机APP连接，实现无线控制。

### 3.1 蓝牙模块连接

蓝牙模块连接至主控板的串口接口（TX/RX），通讯参数与串口通讯相同：
- 波特率：115200
- 数据位：8
- 停止位：1
- 无校验

### 3.2 APP通讯协议

手机APP与单片机之间的通讯协议详见：
`2.使用教程与开发手册/C06B主板外设资源教程视频/4. 主板外设讲解/6. 蓝牙/单片机与APP通信协议.pdf`

APP通过蓝牙发送指令，单片机接收后解析并执行相应的舵机控制动作。

### 3.3 蓝牙指令格式

蓝牙指令格式与串口指令格式完全相同，均为10字节固定长度：

```
[0xFF] [0xFE] [angle_bottom] [angle_top] [0x00x5] [parity]
```

## 4. ROS通讯配置

ROS（机器人操作系统）通讯允许将云台集成到更大的机器人系统中。

### 4.1 ROS例程位置

ROS控制例程位于：`9.ROS控制例程与Python控制例程/1.ROS控制例程/demo01.zip`

详细使用说明请参考：`9.ROS控制例程与Python控制例程/例程使用说明.pdf`

### 4.2 ROS节点配置

ROS例程通常包含以下节点：

| 节点名称 | 功能 |
|----------|------|
| pantilt_control | 云台控制节点，接收ROS话题并转换为串口指令 |
| serial_node | 串口通讯节点，负责与主控板数据交互 |

### 4.3 ROS话题接口

典型的ROS控制接口：

| 话题名称 | 类型 | 说明 |
|----------|------|------|
| /pantilt_angle | geometry_msgs/Point | 云台角度控制（x=底舵机角度，y=摆臂舵机角度） |
| /pantilt_state | sensor_msgs/JointState | 云台状态反馈 |

### 4.4 ROS配置步骤

1. 解压demo01.zip到ROS工作空间
2. 编译工作空间：`catkin_make`
3. 确保串口权限：`sudo chmod 666 /dev/ttyUSB0`
4. 启动ROS核心：`roscore`
5. 启动云台控制节点：`. /devel/setup.bash && rosrun pantilt_control pantilt_node`

### 4.5 ROS控制示例

```bash
# 发布角度控制指令
rostopic pub /pantilt_angle geometry_msgs/Point "x: 90.0 y: 45.0 z: 0.0"
```

## 5. 指令示例

### 5.1 串口控制示例

使用WHEELTEC串口调试助手发送指令：

1. 选择正确的COM端口
2. 设置波特率为115200
3. 发送以下十六进制指令：

| 动作 | 指令（十六进制） |
|------|-----------------|
| 底舵机0度，摆臂0度 | FF FE 00 00 00 00 00 00 00 00 |
| 底舵机90度，摆臂45度 | FF FE 5A 2D 00 00 00 00 00 77 |
| 底舵机180度，摆臂90度 | FF FE B4 5A 00 00 00 00 00 EE |

### 5.2 Python控制完整示例

```python
import serial
import time

class PantiltController:
    def __init__(self, port='COM22', baudrate=115200):
        self.ser = serial.Serial(port, baudrate, timeout=1)
        if not self.ser.is_open:
            raise Exception("串口打开失败")

    def set_angle(self, bottom, top):
        """设置云台角度"""
        parity = bottom ^ top
        hex_data = [0xff, 0xfe, bottom, top,
                    0x00, 0x00, 0x00, 0x00, 0x00, parity]
        self.ser.write(bytes(hex_data))
        time.sleep(0.05)

    def close(self):
        self.ser.close()

# 使用示例
if __name__ == "__main__":
    controller = PantiltController('COM22')
    controller.set_angle(90, 45)   # 设置底舵机90度，摆臂45度
    controller.close()
```

### 5.3 ROS Python节点示例

```python
#!/usr/bin/env python
import rospy
import serial
from geometry_msgs.msg import Point

class PantiltNode:
    def __init__(self):
        port = rospy.get_param('~port', '/dev/ttyUSB0')
        self.ser = serial.Serial(port, 115200, timeout=1)

        rospy.Subscriber('/pantilt_angle', Point, self.callback)

    def callback(self, msg):
        bottom = int(msg.x)
        top = int(msg.y)
        parity = bottom ^ top
        hex_data = [0xff, 0xfe, bottom, top,
                    0x00, 0x00, 0x00, 0x00, 0x00, parity]
        self.ser.write(bytes(hex_data))

    def spin(self):
        rospy.spin()

if __name__ == "__main__":
    rospy.init_node('pantilt_control')
    node = PantiltNode()
    node.spin()
```

## 6. 故障排查

### 6.1 串口通讯问题

| 现象 | 可能原因 | 解决方法 |
|------|----------|----------|
| 舵机无反应 | 串口号错误 | 确认正确的COM端口 |
| 舵机无反应 | 波特率不匹配 | 确认为115200 |
| 数据错误 | 校验码错误 | 重新计算parity |
| 控制延迟 | 发送间隔太短 | 增加延时至50ms以上 |

### 6.2 蓝牙通讯问题

| 现象 | 可能原因 | 解决方法 |
|------|----------|----------|
| 无法连接 | 蓝牙模块未配对 | 先进行蓝牙配对 |
| 连接断开 | 距离太远 | 缩短蓝牙距离 |
| 数据乱码 | 串口参数不匹配 | 确认115200/8N1 |

### 6.3 ROS通讯问题

| 现象 | 可能原因 | 解决方法 |
|------|----------|----------|
| 节点启动失败 | 串口权限不足 | chmod 666 /dev/ttyUSB0 |
| 话题无响应 | 节点未正常启动 | 检查roscore和节点状态 |
| 控制方向相反 | 舵机接口接反 | 检查S1/S2接口接线 |


## OpenMV 视觉追踪协议

通过 USART3（PB11）连接 OpenMV 与 STM32。Mode 2 为 OpenMV 模式。

### 通讯参数

- 波特率：115200
- 数据位：8
- 停止位：1
- 校验位：无
- 流控制：无

### 帧格式（5字节）

```text
0xFF 0xFE hasBlob tx ty
```

| 字节 | 字段 | 说明 |
|---|---|---|
| 0 | `0xFF` | 帧头1（同步） |
| 1 | `0xFE` | 帧头2（同步） |
| 2 | `hasBlob` | `0x01`=检测到目标，`0x00`=未检测 |
| 3 | `tx` | 目标X坐标，归一化 0-255，128=中心 |
| 4 | `ty` | 目标Y坐标，归一化 0-255，128=中心 |

STM32 状态机检测到 `0xFF 0xFE` 帧头后，接收后续 3 字节（hasBlob、tx、ty），存入 `OpenMV_Rxbuf[0..2]` 供主循环处理。

### 坐标说明

QVGA 320×240：
- 图像中心点 (cx=160, cy=120) → (tx=128, ty=128)
- tx = cx / 320 × 255
- ty = cy / 240 × 255

### 丢失目标处理

- OpenMV 端：目标丢失后最多续命 5 帧（保持上一帧坐标），之后回传中心 (128, 128)
- STM32 端：超过约 200ms 无有效帧，Velocity1/2 清零，云台停止

### 工作模式

- Mode 0：PS2 遥控模式
- Mode 1：USART1 连接 PC 上位机
- Mode 2：USART3 连接 OpenMV 视觉
