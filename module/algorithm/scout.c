#include <math.h>
#include <stdbool.h>
#include "scout.h"


// ==== 初始化 ==== //
void SweepGen_Init(SweepGen_t *gen) {
    gen->time = 0.0f;
    gen->phase = 0.0f;
    gen->angle = 0.0f;
    gen->finished = false;
}

// ==== 对数扫频公式 ==== //
// f(t) = f1 * (f2/f1)^(t/T)
// 相位 φ(t) = 2π * f1 * T / ln(f2/f1) * [ ( (f2/f1)^(t/T) - 1 ) ]
float SweepGen_Update(SweepGen_t *gen) {
    if (gen->finished) return 0.0f;

    // 扫频比率
    float beta = logf(FREQ_END / FREQ_START);

    // 当前相位 (对数扫频)
    float exp_term = powf(FREQ_END / FREQ_START, gen->time / SWEEP_TIME);
    gen->phase = 2.0f * PI * FREQ_START * SWEEP_TIME / beta * (exp_term - 1.0f);

    // 速度命令
    float vel_cmd;
    static bool flag = 0;
    static bool reverse = 0;
    if(sinf(gen->phase) > 0)
    {
        if(flag == 0)
        {
            flag = 1;
            reverse = (reverse + 1) % 2;
        }

        if(reverse == 0)
        {
            vel_cmd = VEL_MAX;
        }
        else
        {
            vel_cmd = -VEL_MAX;
        }


    }
    else
    {
        if(flag == 1)
        {
            flag = 0;
        }
        vel_cmd = 0;
    }


    // 积分得到角度
    gen->angle += vel_cmd * SAMPLE_PERIOD;

    // 限幅保护
    // if (fabsf(gen->angle) > ANGLE_LIMIT) {
    //     vel_cmd = 0.0f;
    //     gen->angle = (gen->angle > 0.0f) ? ANGLE_LIMIT : -ANGLE_LIMIT;
    // }

    // 更新时间
    gen->time += SAMPLE_PERIOD;
    if (gen->time >= SWEEP_TIME) {
        gen->finished = true;
        vel_cmd = 0.0f;
    }

    return vel_cmd;
}
