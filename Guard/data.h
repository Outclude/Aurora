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
    //配速模式
    volatile int cadence;
    volatile double current_speed;
    volatile double goal_speed;
    

    //游戏模式
    volatile double sumDistance;
    volatile double rewardDistance;

    //公共部分
    volatile double distance;
    volatile double last_distance;
    volatile bool stopRun;
};

#endif
