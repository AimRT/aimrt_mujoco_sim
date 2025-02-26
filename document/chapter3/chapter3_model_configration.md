# 第3章 仿真配置
AimRT_Mujoco_Sim 的设计初衷就是让用户仅通过配置文件来描述整个机器人结构（xml 配置文件）以及通信行为（yaml 配置文件）， 即可启动机器人仿真模块， 实现和其他模块的通信，以及机器人仿真。本章将详细介绍 AimRT_Mujoco_Sim 的模型配置和通信配置。

## 3.1 模型配置（.xml）
模型配置（.xml）：描述机器人模型的结构、关节、传感器、驱动器等信息。在项目根目录下的`./src/examples/XXX/install/linux/bin/cfg/model`文件夹下。
主要用于：
  - 定义机器人的物理结构（连杆、关节）
  - 配置驱动器参数（电机、驱动器）
  - 设置传感器属性（IMU、接触传感器）
  - 指定物理参数（质量、惯性、摩擦系数）
  
具体使用请参考 [mujoco 官方文档](https://mujoco.readthedocs.io/en/stable/XMLreference.html)

## 3.2 通信配置（.yaml）
通信配置（.yaml）：配置机器人仿真模块和其他模块的通信。在项目的`./src/examples/inverted_pendulum/install/linux/bin/cfg`文件夹下。
本项目使用 AimRT 作为中间件进行通信。其主要用于：
  - 用来订阅其他控制节点的控制指令，并将控制指令转化为 Mujoco 仿真场景中的动作指令
  - 用来发布 Mujoco 仿真场景中的状态信息，包括机器人当前的位置、姿态、速度等
  
关于其配置主要分为`aimrt`节点配置和`MujocoSimModule`节点配置。关于`aimrt`节点配详细使用方法可以参考[AimRT官方文档](https://docs.aimrt.org/index.html)，本小节主要介绍`MujocoSimModule`节点配置。

### 3.2.1 MujocoSimModule 节点配置

| 节点                           | 类型   | 是否可选 | 默认值 | 作用                                                            |
| ------------------------------ | ------ | -------- | ------ | --------------------------------------------------------------- |
| simulation_model_path          | string | 必选     | ""     | xml模型配置文件路径                                             |
| sim_executor                   | string | 必选     | ""     | Mujoco仿真引擎的执行器                                          |
| gui_executor                   | string | 必选     | ""     | Mujocox可视化渲染的执行器                                       |
| subscriber_options             | array  | 可选     | []     | 订阅者选项，用于订阅控制指令                                    |
| subscriber_options[i].topic    | string | 必选     | ""     | 订阅的控制指令的话题名称                                        |
| subscriber_options[i].type     | string | 必选     | ""     | 订阅的控制指令的类型                                            |
| subscriber_options[i].options  | map    | 可选     | -      | 订阅的控制指令的配置项，具体请参考[驱动器配置](#322-驱动器配置) |
| publisher_options              | array  | 可选     | []     | 发布机器人状态信息的选项                                        |
| publisher_options[i].topic     | string | 必选     | ""     | 发布者选项，用于发布状态信息                                    |
| publisher_options[i].frequency | number | 必选     | 1000   | 发布状态信息的频率(Hz) ，最大值为1000                           |
| publisher_options[i].executor  | string | 必选     | ""     | 发布者所绑定的执行器类型                                        |
| publisher_options[i].type      | string | 必选     | ""     | 发布的状态信息类型                                              |
| publisher_options[i].options   | map    | 可选     | -      | 发布状态信息的配置项  ，具体请参考[传感器配置](#323-传感器配置) |

使用注意点如下：
- subscriber_options 为数组类型， 每一个元素代表会开启一个订阅者用于订阅控制指令。
- publisher_options 为数组类型， 每一个元素代表会开启一个发布者用于发布状态信息。
- subscriber_options[i].type 为订阅的控制指令的类型，目前可选的有：
  - [`joint_actuator`](#joint-类驱动器选项joint_actuator--joint_actuator_ros2)
  - [`joint_actuator_ros2`](#joint-类驱动器选项joint_actuator--joint_actuator_ros2)

- publisher_options[i].frequency 为发布状态信息的频率，单位为 Hz，最大值为1000。用户在设置时建议设置为 1000 的因数。
- publisher_options[i].type 为发布的状态信息类型，目前可选的有：
  - [`joint_sensor`](#joint-类传感器选项joint_sensor--joint_sensor_ros2)
  - [`joint_sensor_ros2`](#joint-类传感器选项joint_sensor--joint_sensor_ros2)
  - [`imu_sensor`](#imu-类传感器选项imu_sensor--imu_sensor_ros2)
  - [`imu_sensor_ros2`](#imu-类传感器选项imu_sensor--imu_sensor_ros2)
  - [`touch_sensor`](#touch-类传感器选项touch_sensor--touch_sensor_ros2)
  - [`touch_sensor_ros2`](#touch-类传感器选项touch_sensor--touch_sensor_ros2)

### 3.2.2 驱动器配置
#### joint 类驱动器选项（joint_actuator & joint_actuator_ros2）

| 节点                         | 类型   | 是否可选 | 默认值 | 作用                                    |
| ---------------------------- | ------ | -------- | ------ | --------------------------------------- |
| joints                       | array  | 可选     | []     | joint类驱动器组                         |
| joints[i].name               | string | 必选     | ""     | 该关节的名称                            |
| joints[i].bind_joint         | string | 必选     | ""     | 该关节在 xml 中绑定的关节名称           |
| joints[i].bind_actuator_type | string | 可选     | ""     | 该关节驱动器 在xml 中绑定的驱动器的类型 |
| joints[i].bind_actuator_name | string | 可选     | ""     | 该关节驱动器在  xml中绑定的驱动器的名称 |

使用注意点如下：
- joints 是一个 array 类型， 其下的每一个元素会通过一个订阅者进行订阅
- bind_joint 必须与 xml 文件中定义的名称一致， 即和下面代码块中 joint 字段一致
- bind_actuator_type 必须与 xml 文件中定义的名称一致，即和下面代码块中  的 motor 所处字段一致
- bind_actuator_name 必须与 xml 文件中定义的名称一致，即和下面代码块中  的 name 字段一致
```xml
  <!-- xml 示例： -->
  <actuator>
    <position name="shoulder_actuator" joint="shoulder" ctrlrange="-1.5708 1.5708" kp="1000" kv="50"/>
    <position name="elbow_actuator" joint="elbow" ctrlrange="-2.618 0" kp="1000" kv="50"/>
  </actuator>
```
```yaml
  # yaml 示例：
  subscriber_options:
    - topic: /examples_hardware/joint/joint_command
      type: joint_actuator
      options:
        joints:
          - name: shoulder_actuator
            bind_joint: shoulder
            bind_actuator_type: position
            bind_actuator_name: shoulder_actuator
          - name: elbow_actuator
            bind_joint: elbow
            bind_actuator_type: position
            bind_actuator_name: elbow_actuator
```

### 3.2.3 传感器配置
#### joint 类传感器选项（joint_sensor & joint_sensor_ros2）

| 节点                                   | 类型   | 是否可选 | 默认值 | 作用                                  |
| -------------------------------------- | ------ | -------- | ------ | ------------------------------------- |
| joints                                 | array  | 可选     | []     | joint类传感器组                       |
| joints[i].name                         | string | 必选     | ""     | 该关节驱动器的名称                    |
| joints[i].bind_joint                   | string | 必选     | ""     | 该关节驱动器在 xml 中绑定的关节名称   |
| joints[i].bind_jointpos_sensor         | string | 可选     | ""     | 该关节在 xml 中绑定的位置传感器名称   |
| joints[i].bind_jointvel_sensor         | string | 可选     | ""     | 该关节在 xml 中绑定的速度传感器名称   |
| joints[i].bind_jointactuatorfrc_sensor | string | 可选     | ""     | 该关节在 xml 中绑定的执行力传感器名称 |

使用注意点如下：
- joints 是一个 array 类型， 其下的每一个元素会通过一个发布者进行发布
- bind_joint 必须与 xml 文件中定义的名称一致, 即和下面代码块中 joint 字段一致
- bind_jointpos_sensor 必须与 xml 文件中定义的名称一致， 即和下面代码块中 jointpos 的 name 字段一致
- bind_jointvel_sensor 必须与 xml 文件中定义的名称一致， 即和下面代码块中 jointvel 的 name 字段一致
- bind_jointactuatorfrc_sensor 必须与 xml 文件中定义的名称一致， 即和下面代码块中 jointactuatorfrc 的 name 字段一致
```xml
  <!-- xml 示例： -->
  <sensor> 
    <jointpos name="jointpos_shoulder" joint="shoulder" noise="0.01"/>  
    <jointvel name="jointvel_shoulder" joint="shoulder" noise="0.01"/>  
    <jointactuatorfrc name="jointactuatorfrc_shoulder" joint="shoulder" noise="0.01"/> 

    <jointpos name="jointpos_elbow" joint="elbow" noise="0.05"/>  
    <jointactuatorfrc name="jointactuatorfrc_elbow" joint="elbow" noise="0.05"/>  
    <jointvel name="jointvel_elbow" joint="elbow" noise="0.05"/>  
  </sensor> 
```
```yaml
  # yaml 示例：
  publisher_options:
    - topic: /examples_hardware/joint/joint_state
      frequency: 1
      executor: work_thread_pool
      type: joint_sensor
      options:
        joints:
          - name: shoulder_sensor
            bind_joint: shoulder
            bind_jointpos_sensor: jointpos_shoulder
            bind_jointvel_sensor: jointvel_shoulder
            bind_jointactuatorfrc_sensor: jointactuatorfrc_shoulder
          - name: elbow_sensor
            bind_joint: elbow
            bind_jointpos_sensor: jointpos_elbow
            bind_jointvel_sensor: jointvel_elbow
            bind_jointactuatorfrc_sensor: jointactuatorfrc_elbow
```

#### imu 类传感器选项（imu_sensor & imu_sensor_ros2）

| 节点               | 类型   | 是否可选 | 默认值 | 作用                                    |
| ------------------ | ------ | -------- | ------ | --------------------------------------- |
| bind_site          | string | 必选     | ""     | IMU 传感器在 xml 中绑定的 site 名称     |
| bind_framequat     | string | 可选     | ""     | 绑定在 xml 中测量姿态四元数的传感器名称 |
| bind_gyro          | string | 可选     | ""     | 绑定在 xml 中测量角速度的传感器名称     |
| bind_accelerometer | string | 可选     | ""     | 绑定在 xml 中测量线加速度的传感器名称   |

使用注意点如下：
- 每个话题的 publisher 最多只能绑定一个 imu 传感器信息
- bind_site 必须与 xml 文件中定义的 site 名称一致
- bind_framequat 必须与 xml 文件中 framequat 的 name 字段一致
- bind_gyro 必须与 xml 文件中 gyro 的 name 字段一致
- bind_accelerometer 必须与 xml 文件中 accelerometer 的 name 字段一致
```xml
  <!-- xml 示例： -->
  <sensor>
      <gyro name="angular-velocity" site="imu" noise="0"/>
      <accelerometer name="linear-acceleration" site="imu" noise="0"/>
      <framequat name="orientation" objtype="site" objname="imu" noise="0"/>
  </sensor>
```
```yaml
  # yaml 示例：
  publisher_options:
    - topic: /examples_hardware/imu/imu_state
      frequency: 1
      executor: work_thread_pool
      type: imu_sensor
      options:
        bind_site: imu
        bind_framequat: orientation
        bind_gyro: angular-velocity
        bind_accelerometer: linear-acceleration
```

#### touch 类传感器选项（touch_sensor & touch_sensor_ros2）

| 节点                    | 类型    | 是否可选 | 默认值 | 作用                                                                   |
| ----------------------- | ------- | -------- | ------ | ---------------------------------------------------------------------- |
| name                    | array   | 必选     | []     | 某一传感器组的名称，每个name与states中的元素一一对应组                 |
| states                  | array2D | 可选     | []     | 外层数组的每个元素代表一个传感器组，内层数组的元素是该组中的传感器配置 |
| states[i][j]            | map     | 可选     | {}     | 代表组中第j个传感器的具体配置                                          |
| states[i][j].bind_site  | string  | 可选     | ""     | touch传感器在xml 中绑定的site名称                                      |
| states[i][j].bind_touch | string  | 可选     | ""     | 绑定在 xml 中测量 touch 传感器名称                                     |

使用注意点如下：

```xml
  <!-- xml 示例： -->
  <sensor>
      <touch name="touch_sensor_1_top" site="touch_sensor_site_1_top"/>
      <touch name="touch_sensor_1_bottom" site="touch_sensor_site_1_bottom"/>
      <touch name="touch_sensor_2_bottom" site="touch_sensor_site_2_bottom"/>
  </sensor>
```
```yaml
  # yaml 示例：
  publisher_options:
    - topic: /examples_hardware/touch_sensor/touch_sensor_state
      frequency: 1
      executor: work_thread_pool
      type: touch_sensor
      options:
        names: 
          - top_ball
          - bottom_ball
        states:
          - - bind_site: touch_sensor_site_1_top
              bind_touch_sensor: touch_sensor_1_top
          - - bind_site: touch_sensor_site_1_bottom
              bind_touch_sensor: touch_sensor_1_bottom
            - bind_site: touch_sensor_site_2_bottom
              bind_touch_sensor: touch_sensor_2_bottom

```