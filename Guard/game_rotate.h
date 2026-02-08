#ifndef GAME_ROTATE_H
#define GAME_ROTATE_H

#include "data.h"

// 业务循环，负责主动获取数据并执行逻辑
// 使用 SystemData::getInstance().getCadence() 和 getPaceSec()
void game_rotate_loop();

#endif
