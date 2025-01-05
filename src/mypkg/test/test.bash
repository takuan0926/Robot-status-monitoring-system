#!/bin/bash
# SPDX-FileCopyrightText: 2024 suzuki takuma <s23c1076vc@s.chibakoudai.jp>
# SPDX-License-Identifier: BSD-3-Clause

ng () {
    echo "${1}行目が違うよ"
    res=1
}

res=0

source /opt/ros/foxy/setup.bash
source ~/your_ros_workspace/install/setup.bash

colcon build --symlink-install
source install/setup.bash

ros2 run mypkg status_publisher.py &

sleep 3

out=$(ros2 topic echo /robot_status --once)
if [[ "$out" != *"バッテリー:"* ]] || [[ "$out" != *"稼働時間:"* ]] || [[ "$out" != *"モード:"* ]]; then
    ng "$LINENO"
fi

out=$(ros2 topic echo /invalid_status 2>&1)
if [[ "$out" != *"トピックが見つかりません"* ]]; then
    ng "$LINENO"
fi

kill %1

if [ "${res}" = 0 ]; then
    echo OK
else
    exit 1
fi

