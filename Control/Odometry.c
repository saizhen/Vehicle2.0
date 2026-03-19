#include "Odometry.h"

/* ==================================================================
 * 参数校准区 (PLEASE MODIFY THESE)
 * 这一步决定了你的车走直线直不直，转弯准不准
 * ================================================================== */

// 1. 轮子直径 (单位: cm)
// 请测量轮胎的直径。N20 电机配套的轮子通常很小。
#define WHEEL_DIAMETER   6.50f  

// 2. 轮距 (单位: cm) -> 核心参数！
// 两个轮子中心点之间的距离。如果转 360 度转多了，把这个值调大；转少了调小。
#define WHEEL_TRACK      14.0f //

// 3. 编码器与电机参数
#define ENCODER_PPR      13     // 编码器线数 (磁环一圈的脉冲)
#define GEAR_RATIO       90     // 减速比 (N20 常见是 1:30)
#define ENCODER_MULT     4      // 软件倍频 (定时器配置为 TI12 模式即 4 倍频)

// [自动计算] 脉冲转距离系数 (cm/pulse)
// 公式: (PI * 直径) / (线数 * 倍频 * 减速比)
// 结果示例: (3.14 * 4.3) / (13 * 4 * 30) ≈ 0.0086 cm/pulse
const float DIST_PER_PULSE = (3.14159265f * WHEEL_DIAMETER) / (ENCODER_PPR * ENCODER_MULT * GEAR_RATIO);

/* ==================================================================
 * 全局变量定义
 * ================================================================== */
float Global_X = 0.0f;
float Global_Y = 0.0f;
float Global_Yaw = 0.0f; // 0 代表正东，PI/2 代表正北

/* ==================================================================
 * 函数实现
 * ================================================================== */

// 初始化：坐标归零
void Odometry_Init(void)
{
    Global_X = 0.0f;
    Global_Y = 0.0f;
    Global_Yaw = 0.0f;
}

// 里程计更新 (请放在 PID 计算的同一个定时器中断里调用)
void Odometry_Update(int enc_L, int enc_R)
{
    // 1. 将脉冲转换为物理距离 (cm)
    // 左轮和右轮走过的微小距离
    float d_L = enc_L * DIST_PER_PULSE;
    float d_R = enc_R * DIST_PER_PULSE;

    // 2. 计算中心位移 (Center Distance)
    float delta_d = (d_L + d_R) / 2.0f;

    // 3. 计算角度变化 (Delta Theta)
    // 公式: (右轮距离 - 左轮距离) / 轮距
    // 假设逆时针旋转为正 (符合右手定则)
    float delta_theta = (d_R - d_L) / WHEEL_TRACK;

    // 4. 计算平均航向角 (Runge-Kutta 2nd order 近似)
    // 认为这段位移是在“当前角度”和“转动后角度”的中间发生的，精度比直接用当前角度高
    float avg_theta = Global_Yaw + (delta_theta / 2.0f);

    // 5. 累加坐标
    Global_X += delta_d * cosf(avg_theta); // 需包含 math.h
    Global_Y += delta_d * sinf(avg_theta);

    // 6. 累加角度并归一化 (-PI ~ +PI)
    Global_Yaw += delta_theta;
    
    // 限制角度在 -PI 到 +PI 之间
    while (Global_Yaw > 3.14159265f) Global_Yaw -= 6.2831853f;
    while (Global_Yaw < -3.14159265f) Global_Yaw += 6.2831853f;
}
