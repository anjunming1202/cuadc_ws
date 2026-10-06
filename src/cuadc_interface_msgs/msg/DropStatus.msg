# 投放执行状态反馈：一次投放序列已执行（本口 PWM 持续 run_s→回中立）。
# 见 docs/spec/adr/0007。ACK/执行完成≠载荷一定已脱离（无称重/角度闭环），只表示已按验证时序驱动。
bool actuated
uint8 port    # 投放口号：0 左 / 1 右
