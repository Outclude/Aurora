#ifndef __ROTATE_H__
#define __ROTATE_H__

// 业务循环，负责主动获取数据并执行逻辑
// 使用 SystemData::getInstance().getCadence() 和 getPaceSec()
void rotate_loop();

#endif
