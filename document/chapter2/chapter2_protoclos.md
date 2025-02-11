# 第2章 消息类型 

本章介绍本项目关于消息协议类型的规范。通信协议采用 [`protobuf`](#31-protobuf-类型)以及 [`ros2 msg`](#31-ros2-类型) 标准消息格式，这些消息将作为系统的状态与控制信息的载体。本章将介绍每种消息类型的详细信息， 用户需要根据实际应用场景选择合适的消息类型以实现和 Aimrt_Mujoco_Sim 模块的通信。

## 3.1 protobuf 类型

| 主要消息类型                  | 功能描述                                | 应用场景                   |
| ----------------------------- | --------------------------------------- | -------------------------- |
| [JointState](#311-jointstate) | 关节状态信息，包含位置、速度、力等参数  | - 机器人关节状态监测与控制 |
| [ImuState](#312-imustate)     | IMU传感器数据，包含姿态、速度、加速度等 | - 机器人姿态估计与导航     |


### 3.1.1 `JointState`
用于描述一个或一组关节的状态信息。

| Field  | Type                                                         | Description  |
| ------ | ------------------------------------------------------------ | ------------ |
| header | [aimrt.protocols.common.Header](#aimrtprotocolscommonheader) | 消息头信息   |
| data   | [repeated SingleJointState ](#singlejointstate)              | 关节状态数组 |


#### `SingleJointState`
用于描述单个关节的状态信息。
| Field    | Type   | Description                          |
| -------- | ------ | ------------------------------------ |
| name     | string | 关节名称                             |
| position | double | 位置 (旋转关节:rad / 移动关节:m)     |
| velocity | double | 速度 (旋转关节:rad/s / 移动关节:m/s) |
| effort   | double | 力/力矩 (旋转关节:N⋅m / 移动关节:N)  |


### 3.1.2 `ImuState`
用于描述一个或多个IMU传感器的状态信息。

| Field                          | Type                                                         | Description                                                |
| ------------------------------ | ------------------------------------------------------------ | ---------------------------------------------------------- |
| header                         | [aimrt.protocols.common.Header](#aimrtprotocolscommonheader) | 消息头信息，包含时间戳、序列号等元数据                     |
| orientation                    | [Quaternion](#quaternion)                                    | 姿态四元数 (x,y,z,w)                                       |
| orientation_covariance         | repeated double                                              | 3x3姿态协方差矩阵（行优先存储，9个元素，单位：rad²）       |
| angular_velocity               | [Vector3](#vector3)                                          | 三轴角速度测量值（单位：rad/s）                            |
| angular_velocity_covariance    | repeated double                                              | 3x3角速度协方差矩阵（行优先存储，9个元素，单位：(rad/s)²） |
| linear_acceleration            | [Vector3](#vector3)                                          | 三轴线性加速度测量值（单位：m/s²）                         |
| linear_acceleration_covariance | repeated double                                              | 3x3加速度协方差矩阵（行优先存储，9个元素，单位：(m/s²)²）  |


#### `Quaternion`
用于描述四元数姿态。
| Field | Type   | Description     |
| ----- | ------ | --------------- |
| w     | double | 四元数实部      |
| x     | double | 四元数虚部x分量 |
| y     | double | 四元数虚部y分量 |
| z     | double | 四元数虚部z分量 |

#### `Vector3`
用于描述三维向量。
| Field | Type   | Description |
| ----- | ------ | ----------- |
| x     | double | x轴分量     |
| y     | double | y轴分量     |
| z     | double | z轴分量     |



### 3.1.3 `Common`
#### `aimrt.protocols.common.Header`
用于描述消息头信息。
| Field      | Type   | Description    |
| ---------- | ------ | -------------- |
| time_stamp | uint64 | 时间戳（纳秒） |
| frame_id   | string | 坐标系ID       |




