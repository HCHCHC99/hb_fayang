#include "param_validator.h"
#include "App_Params.h"       /* REG_xxx */
#include <stddef.h>

/*=============================================================================
 * 参数校验规则表
 * 按 regAddr 匹配，依次执行: 最大最小值限幅 → 精度取整
 *============================================================================*/
static const ParamRule_t s_rules[] = {
    /* 过压阈值 (0x2714): 25.0V ~ 27.0V, 步进 0.2V
     *   单位: 0.1V, 250 = 25.0V, 270 = 27.0V, step=2 = 0.2V */
    { REG_VOLTAGE_UPPER_LIMIT,   250,   270,    2 },

    /* 欠压阈值 (0x2715): 21.0V ~ 23.0V, 步进 0.2V
     *   单位: 0.1V, 210 = 21.0V, 230 = 23.0V, step=2 = 0.2V */
    { REG_VOLTAGE_LOWER_LIMIT,   210,   230,    2 },

    /* 过流阈值 (0x2716): 0mA ~ 2500mA (2.5A), 步进 50mA
     *   单位: 1mA, step=50 */
    { REG_CURRENT_UPPER_LIMIT,     0,  2500,   50 },

    /* 过流判定时间 (0x271E): 0ms ~ 2000ms, 步进 20ms
     *   单位: 1ms, step=20 */
    { REG_CURRENT_DETECT_MS,       0,  400,   5 },

    /* 减速比 (0x3712): 1.0 ~ 6553.5, 步进 0.1
     *   单位: 0.1, 寄存器值/10 = 实际减速比, step=0 不做精度取整 */
    { REG_RTURN_REDUCTION_RATIO,  10, 65535,    0 },

    /* 电机极对数 (0x3713): 1 ~ 100, 步进 1 */
    { REG_MOTOR_HALL_POLE_PAIRS,   1,   100,    0 },

    /* 哨兵: 全零表示规则表结束 */
    { 0, 0, 0, 0 }
};

/*=============================================================================
 * 查找规则
 *============================================================================*/
const ParamRule_t* Param_GetRule(uint16_t regAddr)
{
    const ParamRule_t *pRule;
    for (pRule = s_rules; pRule->regAddr != 0; pRule++) {
        if (pRule->regAddr == regAddr) {
            return pRule;
        }
    }
    return NULL;
}

/*=============================================================================
 * 参数校验: 限幅 → 取整
 *============================================================================*/
int32_t Param_Validate(uint16_t regAddr, int32_t rawValue, bool *pbModified)
{
    if (pbModified != NULL) {
        *pbModified = false;
    }

    /* 查找规则，无规则则原值放行 */
    const ParamRule_t *pRule = Param_GetRule(regAddr);
    if (pRule == NULL) {
        return rawValue;
    }

    int32_t result = rawValue;

    /* ---- 第1步: 最大值/最小值限幅 ---- */
    if (result < pRule->minVal) {
        result = pRule->minVal;
        if (pbModified != NULL) { *pbModified = true; }
    } else if (result > pRule->maxVal) {
        result = pRule->maxVal;
        if (pbModified != NULL) { *pbModified = true; }
    }

    /* ---- 第2步: 精度取整（向最近步进取整） ---- */
    if (pRule->step > 0) {
        int32_t step  = (int32_t)pRule->step;
        int32_t lower = (result / step) * step;
        int32_t upper = lower + step;

        /* 向最近步进取整，等距时向上取整 */
        int32_t rounded;
        if ((result - lower) >= (upper - result)) {
            rounded = upper;
        } else {
            rounded = lower;
        }

        if (rounded != result) {
            result = rounded;
            if (pbModified != NULL) { *pbModified = true; }
        }
    }

    return result;
}
