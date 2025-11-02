#ifndef SCOUT_H
#define SCOUT_H
#include <stdbool.h>
// ==== 配置参数 ==== //
#ifndef PI
    #define PI              3.141592653589793f
#endif 
#define SAMPLE_PERIOD   0.001f      // 采样周期 (s)，例如 1kHz 调用一次
#define VEL_MAX         25.0f       // 最大速度命令 (deg/s 或 rad/s)
#define ANGLE_LIMIT     10.0f       // 云台活动范围 (deg 或 rad)
#define FREQ_START      0.5f        // 起始频率 (Hz)
#define FREQ_END        0.5f       // 终止频率 (Hz)
#define SWEEP_TIME      30.0f       // 扫频时长 (s)

// ==== 内部状态 ==== //
typedef struct {
    float time;         // 当前时间 (s)
    float phase;        // 正弦相位
    float angle;        // 当前角度 (积分得到)
    bool finished;      // 扫频是否完成
} SweepGen_t;

void SweepGen_Init(SweepGen_t *gen);
float SweepGen_Update(SweepGen_t *gen);


#endif // SCOUT_H