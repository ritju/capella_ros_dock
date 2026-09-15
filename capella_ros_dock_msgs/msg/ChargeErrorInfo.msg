# 回充流程错误信息, 由 charge_action / capella_ros_dock(dock) / charge_manager 发布
# 发布时机: 仅在判定到不可重试的错误时发布, 每个发布端在一次流程内最多发布一条
# 订阅方(/charger/start_docking2)只接收 session_id 与本次调用一致的消息, 收到即作为结果返回
#
# session_id: 由 /charger/start_docking2 生成(毫秒时间戳+pid), 经 Charge.Goal 与 Dock.Goal 透传;
#             不经 start_docking2 触发的流程(如 app 直接触发 /charge action)发布空串
# code:       取值见 ChargeErrorCode.msg (码表唯一来源)
# message:    人可读的原因文本, 订阅方优先使用它作为响应 message
# source:     发布者标识: 'charge_action' / 'dock' / 'charge_manager'
string session_id
uint16 code
string message
string source