#ifndef __PARAM_VALIDATOR_H__
#define __PARAM_VALIDATOR_H__

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief  单个寄存器的校验规则
 * @note   对匹配 regAddr 的写入值，依次执行: 最大最小值限幅 → 精度取整
 */
typedef struct {
    uint16_t regAddr;         /**< 寄存器地址 (REG_xxx)                          */
    int32_t  minVal;          /**< 允许的最小值 (含), INT32_MIN = 不限制下限    */
    int32_t  maxVal;          /**< 允许的最大值 (含), INT32_MAX = 不限制上限    */
    uint16_t step;            /**< 精度步进, 0 = 不做取整                        */
} ParamRule_t;

/**
 * @brief  对寄存器写入值依次执行: 最大最小值限幅 → 精度取整
 * @param  regAddr     寄存器地址
 * @param  rawValue    原始写入值 (int32_t 以兼容有符号/无符号寄存器)
 * @param  pbModified  [出参] true=值被修改过 (可传 NULL)
 * @return 处理后的值
 */
int32_t Param_Validate(uint16_t regAddr, int32_t rawValue, bool *pbModified);

/**
 * @brief  查询某寄存器的校验规则
 * @param  regAddr  寄存器地址
 * @return 规则指针, 未注册则返回 NULL
 */
const ParamRule_t* Param_GetRule(uint16_t regAddr);

#endif /* __PARAM_VALIDATOR_H__ */
