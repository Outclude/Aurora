#ifndef __DATA_H__
#define __DATA_H__

class SystemData {
public:
    // 获取单例实例
    static SystemData& getInstance();

    // Data Accessors
    int getCadence() const;
    void setCadence(int val);

    double getCurrentSpeed() const;
    void setCurrentSpeed(double val);

    double getGoalSpeed() const;
    void setGoalSpeed(double val);

    double getDistance() const;
    void setDistance(double val);

    double getLastDistance() const;
    void setLastDistance(double val);


    //游戏模式
    double getSumDistance() const;
    void setSumDistance(double val);

    double getRewardDistance() const;
    void setRewardDistance(double val);

    bool getStopRun() const;
    void setStopRun(bool val);

private:
    SystemData(); // Private constructor
    
    // 禁止拷贝
    SystemData(const SystemData&) = delete;
    SystemData& operator=(const SystemData&) = delete;
<<<<<<< HEAD
    //配速模式
    volatile int cadence;
    volatile double current_speed;
    volatile double goal_speed;
    

    //游戏模式
    volatile int sumDistance;
    volatile int rewardDistance;

    //公共部分
    volatile double distance;
    volatile double last_distance;
    volatile bool stopRun;
=======

    //前端输入输出
    volatile int cadence; //步频（步数/min）
    volatile int pace_min; //配速（每公里）
    volatile int pace_sec; //配速（每公里）
    volatile int sum_time;
    volatile int distance;

    //过程计算需要(更底层？)
    volatile float accX; //前进方向加速度
    volatile float accY; //横向加速度（左右）
    volatile float accZ; //垂直方向加速度
    volatile float omega_p; //roll - 横滚（x轴）
    volatile float omega_q; //pitch - 俯仰 (y轴)
    volatile float omega_r; //yaw - 偏航(z轴) ⭐震动主相关
    volatile int timestamp; //时间戳
>>>>>>> 0b01d6c0a944cfe943829dbfc16ac771660c740d
};

#endif
