#!/bin/bash
# 目标高速率
HIGH_SPEED=921600
# HIGH_SPEED=115200
# 原始速率
LOW_SPEED=115200
# 串口设备
DEV=$1
FILE=$2

# 1. 改变物理串口波特率
stty -F $DEV $HIGH_SPEED
# 2. 执行发送（注意：此时接收端也必须已经切换到高速）
sb -vv --ymodem "$FILE"
# 3. 恢复原始速率
stty -F $DEV $LOW_SPEED
