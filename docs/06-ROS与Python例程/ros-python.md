# ROS与Python例程

## 1. 概述

本文档介绍二自由度云台的ROS、ROS2和Python控制例程。这些例程均通过串口与云台主控板通信，实现对云台舵机（angle_bottom）和摆臂舵机（angle_top）的控制。

所有例程使用相同的通信协议：
- 波特率：115200
- 数据帧格式：10字节 `0xFF 0xFE angle_bottom angle_top 0x00 0x00 0x00 0x00 0x00 parity`
- 校验方式：BCC异或校验

## 2. ROS控制例程

ROS控制例程基于ROS1（Kinetic/Melodic），使用Python编写，通过串口发送控制指令。

### 2.1 例程结构

```
demo01/
├── CMakeLists.txt          # CMake构建配置
├── package.xml              # ROS包描述文件
├── include/demo01/          # C++头文件目录（预留）
├── src/                     # C++源码目录（预留）
└── script/
    └── demo01.py            # Python控制脚本
```

### 2.2 配置步骤

1. **创建ROS工作空间**（如果尚未创建）：
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
```

2. **复制ROS例程包**：
```bash
# 将demo01包复制到工作空间的src目录
cp -r /path/to/demo01 ~/catkin_ws/src/
```

3. **安装串口通信依赖**：
```bash
pip install pyserial
```

4. **修改串口配置**：
编辑 `script/demo01.py`，根据实际串口修改：
```python
port = "/dev/ttyACM0"  # Linux串口
# 或 Windows: port = "COM22"
baudrate = 115200
```

5. **设置串口权限**（Linux）：
```bash
sudo usermod -a -G dialout $USER
sudo chmod 666 /dev/ttyACM0
```

6. **编译工作空间**：
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 2.3 使用方法

```bash
# 运行ROS节点
rosrun demo01 demo01.py
```

节点会以20Hz的频率发送串口指令，循环控制两个舵机从0度到180度转动。

### 2.4 核心代码解析

```python
#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String
import serial
import struct

if __name__ == '__main__':
    try:
        port = "/dev/ttyACM0"  # 串口名称
        baudrate = 115200      # 波特率
        parity = 0
        angle_bottom = 1       # 云台舵机角度
        angle_top = 1          # 摆臂舵机角度

        ser = serial.Serial(port, baudrate)
        angle = 1

        rospy.init_node('demo01', anonymous=True)
        rate = rospy.Rate(20)  # 20Hz循环频率

        while not rospy.is_shutdown():
            angle_bottom = angle
            angle_top = angle

            # BCC校验码计算
            parity = angle_bottom ^ parity
            parity = angle_top ^ parity

            # 打包并发送数据
            pack = struct.pack("10B", 0xff, 0xfe, angle_bottom, angle_top,
                               0x00, 0x00, 0x00, 0x00, 0x00, parity)
            ser.write(pack)

            angle += 1
            if angle > 180:
                angle = 1

            rate.sleep()

    except rospy.ROSInterruptException:
        pass
```

## 3. Python控制例程

独立的Python控制脚本，无需ROS环境，可直接运行。

### 3.1 例程结构

```
2.python控制例程/
└── python_demo.py           # Python控制脚本
```

### 3.2 依赖安装

```bash
pip install pyserial
```

### 3.3 使用方法

1. **修改串口配置**：
编辑 `python_demo.py`，根据实际串口修改：
```python
port = 'COM22'    # Windows串口号
# 或 Linux: port = '/dev/ttyACM0'
baudrate = 115200
```

2. **运行脚本**：
```bash
python python_demo.py
```

### 3.4 核心代码解析

```python
import time
import serial

# 串口参数
port = 'COM22'              # 串口名称
baudrate = 115200           # 波特率
parity = 0

angle_bottom = 1            # 云台舵机
angle_top = 1               # 摆臂舵机

ser = serial.Serial(port, baudrate, timeout=1)

if ser.is_open:
    angle = 150
    while True:
        angle_bottom = angle
        angle_top = angle

        # BCC校验码计算
        parity = angle_bottom ^ parity
        parity = angle_top ^ parity

        # 转换为字节串并发送
        hex_data = [0xff, 0xfe, angle_bottom, angle_top,
                     0x00, 0x00, 0x00, 0x00, 0x00, parity]
        byte_data = bytes(hex_data)
        ser.write(byte_data)

        angle += 1
        if angle > 180:
            angle = 1

        time.sleep(0.05)
```

## 4. ROS2控制例程

ROS2控制例程基于ROS2 Foxy/Jazzy等版本，使用rclpy库实现。

### 4.1 例程结构

```
pantilt_control/
├── package.xml              # ROS2包描述文件
├── setup.py                 # Python包安装配置
├── setup.cfg                # 运行时配置
├── resource/pantilt_control # 资源目录标记
├── pantilt_control/
│   ├── __init__.py
│   └── pantiltcontrol.py    # 主控制脚本
└── test/                    # 测试文件
```

### 4.2 配置步骤

1. **创建ROS2工作空间**（如果尚未创建）：
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

2. **复制ROS2例程包**：
```bash
cp -r /path/to/pantilt_control ~/ros2_ws/src/
```

3. **安装串口通信依赖**：
```bash
pip install pyserial
```

4. **修改串口配置**：
编辑 `pantilt_control/pantiltcontrol.py`，根据实际串口修改：
```python
port = "/dev/ttyACM0"  # Linux串口
# 或 Windows: port = "COM22"
baudrate = 115200
```

5. **编译工作空间**：
```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

### 4.3 使用方法

```bash
# 运行ROS2节点
ros2 run pantilt_control pantiltcontrol
```

### 4.4 核心代码解析

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import struct

class SerialPublisher(Node):
    def __init__(self):
        super().__init__('demo01')
        port = "/dev/ttyACM0"
        baudrate = 115200
        self.ser = serial.Serial(port, baudrate)
        self.angle = 1
        self.parity = 0

        # 20Hz循环频率
        self.timer = self.create_timer(0.05, self.timer_callback)

    def timer_callback(self):
        angle_bottom = self.angle
        angle_top = self.angle

        # BCC校验码计算
        self.parity = angle_bottom ^ self.parity
        self.parity = angle_top ^ self.parity

        # 打包并发送数据
        pack = struct.pack("10B", 0xff, 0xfe, angle_bottom, angle_top,
                          0x00, 0x00, 0x00, 0x00, 0x00, self.parity)
        self.ser.write(pack)
        print("pan-tilt running...", self.angle)

        self.angle += 1
        if self.angle > 180:
            self.angle = 1

def main(args=None):
    rclpy.init(args=args)
    node = SerialPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 5. 注意事项

### 5.1 串口识别

- **Linux系统**：通常为 `/dev/ttyACM0`、`/dev/ttyUSB0` 等
- **Windows系统**：通常为 `COM3`、`COM22` 等
- 使用前请通过串口调试助手确认正确的串口号

### 5.2 权限问题

- Linux系统需要当前用户有串口访问权限
- 可通过 `sudo usermod -a -G dialout $USER` 将用户加入dialout组
- 或使用 `sudo chmod 666 /dev/ttyACM0` 临时修改权限

### 5.3 数据帧格式

| 字节索引 | 内容 | 说明 |
|---------|------|------|
| 0 | 0xFF | 帧头 |
| 1 | 0xFE | 帧头 |
| 2 | angle_bottom | 云台舵机角度值 |
| 3 | angle_top | 摆臂舵机角度值 |
| 4-8 | 0x00 | 保留字节 |
| 9 | parity | BCC校验码 |

### 5.4 舵机角度范围

- 云台舵机(angle_bottom)：0-180度
- 摆臂舵机(angle_top)：0-180度

### 5.5 波特率

所有例程统一使用 **115200** 波特率，请确保主控板串口配置与此一致。
