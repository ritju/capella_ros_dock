# 回充相关接口错误码表(唯一来源), Python 与 C++ 均引用本文件的常量
# 接口列: d2 = /charger/start_docking2, s2 = /charger/start2
#
# code | message               | 备注                          | 接口
#    0 | success               |                               | d2, s2
#    1 | already running       | 相关接口正在运行之前下发的指令 | d2
#    2 | blocked               | 因其他任务正在执行被阻塞, 需重试 | s2
#    3 | invalid param {param} | {param}参数取值无效            | d2
#    5 | cancelled             | 被手动取消                     | d2, s2
#   10 | {interface} failed    | 调用{interface}失败            | d2, s2
#   11 | exceed runtime        | 回充流程整体耗时过长            | d2
#   12 | timeout change pose   | 反复修正回充姿态过久            | d2
#   13 | timeout response      | 接口响应时间过久                | d2, s2
#   20 | camera not ready      | 相机无数据                     | d2
#   21 | obstacle              | 存在障碍物                     | d2
#   22 | marker not visible    | 标记不可见                     | d2
#   23 | not in position       | 机器人未就位                   | s2, d2(对接后无接触/充电中移动)
#   30 | bluetooth not found   | 蓝牙设备不存在                  | d2, s2
#   31 | bluetooth connect error | 蓝牙连接失败                  | d2, s2
#   32 | bluetooth no data     | 蓝牙无数据                     | d2, s2
#   33 | invalid protocol      | 协议格式错误, 可能是协议版本问题  | d2, s2
#   40 | unknown               | 不确定原因                     | d2, s2

uint16 SUCCESS = 0
uint16 ALREADY_RUNNING = 1
uint16 BLOCKED = 2
uint16 INVALID_PARAM = 3
uint16 CANCELLED = 5
uint16 INTERFACE_FAILED = 10
uint16 EXCEED_RUNTIME = 11
uint16 TIMEOUT_CHANGE_POSE = 12
uint16 TIMEOUT_RESPONSE = 13
uint16 CAMERA_NOT_READY = 20
uint16 OBSTACLE = 21
uint16 MARKER_NOT_VISIBLE = 22
uint16 NOT_IN_POSITION = 23
uint16 BLUETOOTH_NOT_FOUND = 30
uint16 BLUETOOTH_CONNECT_ERROR = 31
uint16 BLUETOOTH_NO_DATA = 32
uint16 INVALID_PROTOCOL = 33
uint16 UNKNOWN = 40