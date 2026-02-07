#ifndef __DATA_H__
#define __DATA_H__

class SystemData {
public:
    // 获取单例实例
    static SystemData& getInstance();

    // Data Accessors
    int getCadence() const;
    void setCadence(int val);

    int getPaceMin() const;
    void setPaceMin(int val);

    int getPaceSec() const;
    void setPaceSec(int val);

    int getSumTime() const;
    void setSumTime(int val);

    int getDistance() const;
    void setDistance(int val);

private:
    SystemData(); // Private constructor
    
    // 禁止拷贝
    SystemData(const SystemData&) = delete;
    SystemData& operator=(const SystemData&) = delete;

    volatile int cadence;
    volatile int pace_min;
    volatile int pace_sec;
    volatile int sum_time;
    volatile int distance;
};

#endif
