#ifndef PARAM_MANAGER_H_
#define PARAM_MANAGER_H_

#include "main.h"
#include "hc32f46x_flash.h"
#include <stdint.h>
#include <stdbool.h>
#include "Adapter.h"

#include "rtt_manager.h"

/*=============================================================================
 * 调试宏定义（控制调试输出）
 * 统一在 rtt_manager.h 中统一控制PARAM_DEBUG
 *============================================================================*/
#ifdef PARAM_DEBUG
    #define PARAM_DBG(fmt, ...)    MAIN_D("[PARAM] " fmt, ##__VA_ARGS__)
#else
    #define PARAM_DBG(fmt, ...)    ((void)0)
#endif

/*=============================================================================
 * 魔数定义
 *============================================================================*/
#define PARAM_MAGIC_HEAD    0x55AA55AA  /* 参数存储头部魔数 */
#define PARAM_MAGIC_TAIL    0xAA44AA44  /* 参数存储尾部魔数 */

/*=============================================================================
 * 返回值定义
 *============================================================================*/
#define PARAM_OK            0    /* 操作成功 */
#define PARAM_ERR           -1   /* 操作失败 */
#define PARAM_ERR_INVD_PARAM -3  /* 无效参数 */
#define PARAM_ERR_NOT_RDY   -5   /* 未初始化 */

/*=============================================================================
 * 运行时状态结构体（每个实例独立一份）
 *============================================================================*/
typedef struct {
    uint16_t    curr_sec;      /* 当前写到的扇区号 */
    uint32_t    curr_addr;     /* 当前写到的地址（绝对Flash地址） */
    uint32_t    save_count;    /* 成功保存次数 */
    int32_t     last_res;      /* 最后一次操作结果 (PARAM_OK/PARAM_ERR) */
} Param_Runtime_t;

/*=============================================================================
 * 参数配置结构体（初始化后不可变）
 *============================================================================*/
typedef struct {
    void       *pParamBuf;     /* 参数缓冲区指针（指向全局参数结构体） */
    uint32_t    paramSize;     /* 参数结构体大小（字节） */
    uint32_t    magicHead;     /* 头部魔数 */
    uint32_t    magicTail;     /* 尾部魔数 */
    uint32_t    checksumOffset;/* checksum 字段在结构体中的字节偏移 */
    uint32_t    seqOffset;     /* sequence_id 字段在结构体中的字节偏移 */
    uint32_t    eraseCntOffset;/* erase_count 字段在结构体中的字节偏移 */
    uint8_t     secStart;      /* 起始扇区号（磨损均衡区域上界） */
    uint8_t     secEnd;        /* 结束扇区号（磨损均衡区域下界） */
} Param_Config_t;

/*=============================================================================
 * 外部接口函数
 *============================================================================*/

/**
 * @brief  参数模块初始化
 * @param  pConfig  初始化配置（含缓冲区指针、大小、魔数、扇区范围等）
 * @param  pRuntime 运行时状态（调用者自行分配，持久化当前写入位置）
 * @param  pSetDefaults 设置默认值的回调函数
 * @return PARAM_OK 成功；PARAM_ERR 失败
 * @note   从 Flash 扫描有效参数块并加载到 pConfig->pParamBuf
 *         如果未找到有效块，则调用 pSetDefaults 写默认值并写入 Flash
 */
int32_t Param_Init(const Param_Config_t *pConfig,
                   Param_Runtime_t *pRuntime,
                   void (*pSetDefaults)(void));

/**
 * @brief  保存参数到 Flash
 * @param  pConfig  初始化配置
 * @param  pRuntime 运行时状态（调用后自动更新 curr_sec/curr_addr）
 * @return PARAM_OK 成功；PARAM_ERR 失败
 */
int32_t Param_Save(const Param_Config_t *pConfig, Param_Runtime_t *pRuntime);

/**
 * @brief  调试功能：擦除所有参数扇区
 * @param  pConfig  初始化配置
 * @param  pRuntime 运行时状态
 * @param  pSetDefaults 设置默认值的回调函数
 */
void Param_Debug_EraseAll(const Param_Config_t *pConfig,
                          Param_Runtime_t *pRuntime,
                          void (*pSetDefaults)(void));

/**
 * @brief  公共接口：擦除指定扇区
 */
int32_t Param_EraseSector(uint32_t address);

/*=============================================================================
 * Keil Watch 兼容：主实例运行时指针
 *============================================================================*/
extern Param_Runtime_t g_ParamRuntime;

#endif /* PARAM_MANAGER_H_ */
