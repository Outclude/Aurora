json格式
电脑端发送数据格式
设置配速模式目标
const payload = {
    type : 0,
    cadence: cadence,
    pace_sec: paceSec
};
开始跑步
const payload = {
    type : 1
};
暂停跑步
const payload = {
    type : 2
};
继续跑步
const payload = {
    type : 3
};
结束跑步
const payload = {
    type : 4
};
设置游戏模式目标
const payload = {
    type : 5,
    sumDistance: sumDistance,
    rewardDistance: rewardDistance
};


硬件发送数据格式

统计数据
const payload = {
    distance: distance
};
收到同步的回信
const payload = {
    msg: "Error/OK"
};
状态设置成功
const payload = {
    msg: "Status_OK"
};
