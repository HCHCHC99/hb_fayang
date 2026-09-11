#include "dev_adc.h"
#include "TickTimer.h"
#include "rtt_log.h"
#include <string.h>
#include <stdlib.h>

static uint8_t s_bAdpAdcInitialized = 0U;

volatile int32_t g_dbg_current_raw_ma = 0;        // 原始ADC值换算的电流(mA)
volatile int32_t g_dbg_current_filtered_ma = 0;   // EMA滤波后的电流(mA)
volatile uint16_t g_dbg_current_raw_adc = 0;      // 原始ADC值
volatile uint16_t g_dbg_current_filtered_adc = 0; // EMA滤波后的ADC值

/* Callback context for ADC interrupt */
typedef struct {
    ADC_Device_t* pstcDevice;
    uint8_t u8Channel;
} ADC_CallbackContext_t;

static ADC_CallbackContext_t s_astcCallbackCtx[ADC_MAX_INSTANCES];
static uint8_t s_u8CallbackCtxCount = 0;

/* Forward declarations */
static void ADC_Device_OnDataReady(uint16_t u16AdcValue, uint8_t u8Channel);
static void ADC_Device_UpdateFilter(ADC_Device_t* pstcDev);

static uint16_t ADC_ValueToVoltage(uint16_t u16AdcValue)
{
    return (uint16_t)(((uint32_t)u16AdcValue * 3300UL) / 4095UL);
}

/**
 * @brief ADC callback from Adc layer - runs in interrupt context
 */
static void ADC_Device_OnDataReady(uint16_t u16AdcValue, uint8_t u8Channel)
{
    for (uint8_t i = 0; i < s_u8CallbackCtxCount; i++) {
        if (s_astcCallbackCtx[i].u8Channel == u8Channel) {
            ADC_Device_t* pstcDev = s_astcCallbackCtx[i].pstcDevice;
            if (pstcDev && pstcDev->u8Initialized && pstcDev->u8UseRawRing) {
                
                ADC_DevRingBuffer_t* pstcRing = &pstcDev->stcRawRing;
                uint8_t u8Next = (pstcRing->u8WriteIndex + 1) % ADC_DEV_RAW_RING_SIZE;
                
                if (u8Next == pstcRing->u8ReadIndex) {
                    pstcRing->u8Overflow = 1;
                }
                
                pstcRing->au16Buffer[pstcRing->u8WriteIndex] = u16AdcValue;
                pstcRing->u8WriteIndex = u8Next;
                if (pstcRing->u8Count < ADC_DEV_RAW_RING_SIZE) {
                    pstcRing->u8Count++;
                }
            }
            break;
        }
    }
}

static uint16_t ADC_Device_ReadRawFromRing(ADC_Device_t* pstcDev, 
                                            uint16_t* pu16Buffer, 
                                            uint16_t u16MaxSize)
{
    ADC_DevRingBuffer_t* pstcRing = &pstcDev->stcRawRing;
    uint16_t u16Read = 0;
    
    while (pstcRing->u8Count > 0 && u16Read < u16MaxSize) {
        pu16Buffer[u16Read] = pstcRing->au16Buffer[pstcRing->u8ReadIndex];
        pstcRing->u8ReadIndex = (pstcRing->u8ReadIndex + 1) % ADC_DEV_RAW_RING_SIZE;
        pstcRing->u8Count--;
        u16Read++;
    }
    
    if (pstcRing->u8Overflow) {
        pstcRing->u8Overflow = 0;
    }
    
    return u16Read;
}

static uint16_t ADC_Filter_Mean(uint16_t* pu16Data, uint8_t u8Count)
{
    if (u8Count == 0) return 0;
    uint32_t u32Sum = 0;
    for (uint8_t i = 0; i < u8Count; i++) {
        u32Sum += pu16Data[i];
    }
    return (uint16_t)(u32Sum / u8Count);
}

static uint16_t ADC_Filter_Median(uint16_t* pu16Data, uint8_t u8Count)
{
    if (u8Count == 0) return 0;
    
    uint16_t au16Sorted[ADC_FILTER_WINDOW_MAX];
    memcpy(au16Sorted, pu16Data, u8Count * sizeof(uint16_t));
    
    for (uint8_t i = 0; i < u8Count - 1; i++) {
        for (uint8_t j = 0; j < u8Count - i - 1; j++) {
            if (au16Sorted[j] > au16Sorted[j + 1]) {
                uint16_t u16Temp = au16Sorted[j];
                au16Sorted[j] = au16Sorted[j + 1];
                au16Sorted[j + 1] = u16Temp;
            }
        }
    }
    return au16Sorted[u8Count / 2];
}

/**
 * @brief First order low-pass filter (EMA)
 *        filtered = prev + alpha * (new - prev)
 *        alpha = 0.2 means new data contributes 20%
 */
static uint16_t ADC_Filter_FirstOrder(ADC_Device_t* pstcDev, uint16_t u16NewValue)
{
    float fPrev = (float)pstcDev->stcFilter.u16FilteredValue;
    float fNew = (float)u16NewValue;
    float fAlpha = pstcDev->stcFilter.fAlpha;
    
    return (uint16_t)(fPrev + fAlpha * (fNew - fPrev));
}

static void ADC_Device_UpdateFilter(ADC_Device_t* pstcDev)
{
    if (!pstcDev || !pstcDev->u8UseRawRing) return;
    
    uint16_t au16RawData[ADC_DEV_RAW_RING_SIZE];
    uint16_t u16Count = ADC_Device_ReadRawFromRing(pstcDev, au16RawData, ADC_DEV_RAW_RING_SIZE);
    
    if (u16Count == 0) return;
    
    uint16_t u16RawLatest = au16RawData[u16Count - 1];
    
    uint16_t u16Filtered = 0;
    uint8_t u8WindowSize = pstcDev->stcFilter.u8WindowSize;
    
    switch (pstcDev->stcFilter.enType) {
        case ADC_FILTER_NONE:
            u16Filtered = au16RawData[u16Count - 1];
            break;
            
        case ADC_FILTER_MEAN: {
            uint8_t u8UseCount = (u16Count > u8WindowSize) ? u8WindowSize : u16Count;
            uint16_t au16Window[ADC_FILTER_WINDOW_MAX];
            for (uint8_t i = 0; i < u8UseCount; i++) {
                au16Window[i] = au16RawData[u16Count - u8UseCount + i];
            }
            u16Filtered = ADC_Filter_Mean(au16Window, u8UseCount);
            break;
        }
        
        case ADC_FILTER_MEDIAN: {
            uint8_t u8UseCount = (u16Count > u8WindowSize) ? u8WindowSize : u16Count;
            uint16_t au16Window[ADC_FILTER_WINDOW_MAX];
            for (uint8_t i = 0; i < u8UseCount; i++) {
                au16Window[i] = au16RawData[u16Count - u8UseCount + i];
            }
            u16Filtered = ADC_Filter_Median(au16Window, u8UseCount);
            break;
        }
        
        case ADC_FILTER_FIRST_ORDER: {
            u16Filtered = ADC_Filter_FirstOrder(pstcDev, au16RawData[u16Count - 1]);
            break;
        }
        
        default:
            u16Filtered = au16RawData[u16Count - 1];
            break;
    }
    
    pstcDev->stcFilter.u16FilteredValue = u16Filtered;
    pstcDev->u16RawValue = u16Filtered;
    pstcDev->u16VoltageMv = ADC_ValueToVoltage(u16Filtered);
    pstcDev->u16AverageValue = u16Filtered;
    pstcDev->u32LastUpdateTime = tickTimer_GetCount();
    pstcDev->u8ValueUpdated = 1;
    
    // ============================================================
    // ★★★ JScope调试变量更新 - 取绝对值 ★★★
    // ============================================================
    if (pstcDev->stcConfig.u8Channel == 5) {
        uint16_t u16RawVoltageMv = ADC_ValueToVoltage(u16RawLatest);
        uint16_t u16FilteredVoltageMv = ADC_ValueToVoltage(u16Filtered);
        
        g_dbg_current_raw_adc = u16RawLatest;
        g_dbg_current_filtered_adc = u16Filtered;
        
        int32_t s32RawMa = (int32_t)((int32_t)u16RawVoltageMv - 1650) * 1000 / 264;
        int32_t s32FilteredMa = (int32_t)((int32_t)u16FilteredVoltageMv - 1650) * 1000 / 264;
        
        // 取绝对值
        g_dbg_current_raw_ma = (s32RawMa >= 0) ? s32RawMa : -s32RawMa;
        g_dbg_current_filtered_ma = (s32FilteredMa >= 0) ? s32FilteredMa : -s32FilteredMa;
    }
}

static void ADC_AdpLayerInit(ADC_Device_t* pstcDev)
{
    stc_adc_config_t stcAdcConfig;

    stcAdcConfig.u8Channel = pstcDev->stcConfig.u8Channel;
    stcAdcConfig.stcPin.u8Port = pstcDev->stcConfig.u8Port;
    stcAdcConfig.stcPin.u8Pin = pstcDev->stcConfig.u16Pin;
    
    stcAdcConfig.pfnCallback = ADC_Device_OnDataReady;

    if (pstcDev->stcConfig.enAcqMode == ADC_ACQ_MODE_INTERRUPT) {
        stcAdcConfig.enMode = ADC_MODE_INTERRUPT;
        stcAdcConfig.stcDmaConfig.u16BufferSize = 0;
        stcAdcConfig.stcDmaConfig.u8DmaChannel = 0;
    } else {
        stcAdcConfig.enMode = ADC_MODE_DMA;
        stcAdcConfig.stcDmaConfig.u16BufferSize = pstcDev->stcConfig.u16DmaBufferSize;
        stcAdcConfig.stcDmaConfig.u8DmaChannel = pstcDev->stcConfig.u8DmaChannel;
    }

    uint8_t u8AdpId = Adc_Create(&stcAdcConfig);
    if (u8AdpId == 0xFF) {
        ADC_DEBUG("Adp_Create failed for CH%d\r\n", pstcDev->stcConfig.u8Channel);
        return;
    }

    pstcDev->stcConfig.u8AdcId = u8AdpId;

    ADC_DEBUG("Adp layer created: dev_ADC_ID=%d -> adp_ADC_ID=%d, CH=%d\r\n",
              pstcDev->stcConfig.u8AdcId, u8AdpId, pstcDev->stcConfig.u8Channel);

    {
        stc_gpio_init_t stcGpioInit;
        (void)GPIO_StructInit(&stcGpioInit);
        stcGpioInit.u16PinAttr = PIN_ATTR_ANALOG;
        LL_PERIPH_WE(LL_PERIPH_GPIO);
        (void)GPIO_Init(pstcDev->stcConfig.u8Port, pstcDev->stcConfig.u16Pin, &stcGpioInit);
        LL_PERIPH_WP(LL_PERIPH_GPIO);

        ADC_ChCmd(ADC_UNIT, ADC_SEQ_A, pstcDev->stcConfig.u8Channel, ENABLE);
    }

    s_astcCallbackCtx[s_u8CallbackCtxCount].pstcDevice = pstcDev;
    s_astcCallbackCtx[s_u8CallbackCtxCount].u8Channel = pstcDev->stcConfig.u8Channel;
    s_u8CallbackCtxCount++;

    if (!s_bAdpAdcInitialized) {
        ADC_DEBUG("First ADC device: initializing Adp layer hardware...\r\n");
        Adc_Init();
        Adc_Start();
        s_bAdpAdcInitialized = 1U;
        ADC_DEBUG("Adp layer ADC hardware initialized and started\r\n");
    } else {
        if (pstcDev->stcConfig.enAcqMode == ADC_ACQ_MODE_INTERRUPT) {
            ADC_DEBUG("Reconfiguring ADC interrupt for new interrupt-mode CH%d...\r\n",
                      pstcDev->stcConfig.u8Channel);
            Adc_EnableInterrupt();
            ADC_DEBUG("ADC interrupt reconfigured for CH%d\r\n", pstcDev->stcConfig.u8Channel);
        }
    }
}

/* ========== Standard Device Operations ========== */

DeviceResult_t ADC_Device_Init(void* handle)
{
    ADC_Device_t* pstcDev = (ADC_Device_t*)handle;
    if (pstcDev == NULL) return RESULT_PARAM_ERR;

    ADC_DEBUG("Init: ADC_ID=%d, CH=%d, Mode=%s\r\n",
              pstcDev->stcConfig.u8AdcId,
              pstcDev->stcConfig.u8Channel,
              (pstcDev->stcConfig.enAcqMode == ADC_ACQ_MODE_INTERRUPT) ? "Interrupt" : "AOS+DMA");

    ADC_AdpLayerInit(pstcDev);

    pstcDev->stcRawRing.u8WriteIndex = 0;
    pstcDev->stcRawRing.u8ReadIndex = 0;
    pstcDev->stcRawRing.u8Count = 0;
    pstcDev->stcRawRing.u8Overflow = 0;
    pstcDev->u8UseRawRing = 1;

    /* ★★★ 使用宏定义配置滤波 ★★★ */
    pstcDev->stcFilter.enType = ADC_DEFAULT_FILTER_TYPE;
    pstcDev->stcFilter.u8WindowSize = ADC_DEFAULT_FILTER_WINDOW;
    pstcDev->stcFilter.u16FilteredValue = 0;
    pstcDev->stcFilter.u32LastUpdateTime = tickTimer_GetCount();
    pstcDev->stcFilter.u16UpdateIntervalMs = ADC_DEFAULT_FILTER_INTERVAL;
    pstcDev->stcFilter.fAlpha = ADC_DEFAULT_FILTER_ALPHA;

    pstcDev->u16RawValue = 0;
    pstcDev->u16VoltageMv = 0;
    pstcDev->u16AverageValue = 0;
    pstcDev->u8ValueUpdated = 0;

    if (pstcDev->stcConfig.enAcqMode == ADC_ACQ_MODE_AOS_DMA) {
        pstcDev->u16DmaBufferSize = pstcDev->stcConfig.u16DmaBufferSize;
        pstcDev->pu16DmaBuffer = NULL;
    } else {
        pstcDev->u16DmaBufferSize = 0;
        pstcDev->pu16DmaBuffer = NULL;
    }

    pstcDev->u8Initialized = 1;
    pstcDev->u32LastUpdateTime = tickTimer_GetCount();

    ADC_DEBUG("Init success: ADC_ID=%d, Filter=EMA(alpha=%.1f)\r\n", 
              pstcDev->stcConfig.u8AdcId, pstcDev->stcFilter.fAlpha);
    return RESULT_OK;
}

DeviceResult_t ADC_Device_Deinit(void* handle)
{
    ADC_Device_t* pstcDev = (ADC_Device_t*)handle;
    if (pstcDev == NULL) return RESULT_PARAM_ERR;
    if (!pstcDev->u8Initialized) return RESULT_OK;

    ADC_DEBUG("Deinit: ADC_ID=%d\r\n", pstcDev->stcConfig.u8AdcId);

    pstcDev->u8Initialized = 0;
    pstcDev->u16RawValue = 0;
    pstcDev->u16VoltageMv = 0;
    pstcDev->u16AverageValue = 0;

    return RESULT_OK;
}

DeviceResult_t ADC_Device_Read(void* handle, void* data, uint32_t size)
{
    ADC_Device_t* pstcDev = (ADC_Device_t*)handle;
    if (pstcDev == NULL || data == NULL) return RESULT_PARAM_ERR;
    if (!pstcDev->u8Initialized) return RESULT_ERROR;

    if (size == sizeof(uint16_t)) {
        *(uint16_t*)data = pstcDev->u16RawValue;
    } else if (size == sizeof(ADC_ReadResponse_t)) {
        ADC_ReadResponse_t* pstcResp = (ADC_ReadResponse_t*)data;
        pstcResp->u16RawValue = pstcDev->u16RawValue;
        pstcResp->u16VoltageMv = pstcDev->u16VoltageMv;
        pstcResp->u16AverageValue = pstcDev->u16AverageValue;
    } else {
        return RESULT_PARAM_ERR;
    }

    return RESULT_OK;
}

DeviceResult_t ADC_Device_Write(void* handle, const void* data, uint32_t size)
{
    (void)handle;
    (void)data;
    (void)size;
    return RESULT_ERROR;
}

DeviceResult_t ADC_Device_Control(void* handle, DeviceCommandData_t* pstcCmd)
{
    ADC_Device_t* pstcDev = (ADC_Device_t*)handle;
    if (pstcDev == NULL || pstcCmd == NULL) return RESULT_PARAM_ERR;
    if (!pstcDev->u8Initialized) return RESULT_ERROR;

    switch (pstcCmd->cmd) {
        case CMD_ADC_GET_RAW_VALUE:
            if (pstcCmd->response != NULL && pstcCmd->response_size >= sizeof(uint16_t)) {
                *(uint16_t*)pstcCmd->response = pstcDev->u16RawValue;
                return RESULT_OK;
            }
            return RESULT_PARAM_ERR;

        case CMD_ADC_GET_VOLTAGE_MV:
            if (pstcCmd->response != NULL && pstcCmd->response_size >= sizeof(uint16_t)) {
                *(uint16_t*)pstcCmd->response = pstcDev->u16VoltageMv;
                return RESULT_OK;
            }
            return RESULT_PARAM_ERR;

        case CMD_ADC_GET_AVERAGE_VALUE:
            if (pstcCmd->response != NULL && pstcCmd->response_size >= sizeof(uint16_t)) {
                *(uint16_t*)pstcCmd->response = pstcDev->u16AverageValue;
                return RESULT_OK;
            }
            return RESULT_PARAM_ERR;

        default:
            return RESULT_ERROR;
    }
}

DeviceResult_t ADC_Device_Update(void* handle)
{
    ADC_Device_t* pstcDev = (ADC_Device_t*)handle;
    if (pstcDev == NULL) return RESULT_ERROR;
    if (!pstcDev->u8Initialized) return RESULT_ERROR;

    uint32_t u32Now = tickTimer_GetCount();
    uint32_t u32Elapsed = u32Now - pstcDev->stcFilter.u32LastUpdateTime;
    
    if (pstcDev->stcFilter.u16UpdateIntervalMs == 0) {
        ADC_Device_UpdateFilter(pstcDev);
        pstcDev->stcFilter.u32LastUpdateTime = u32Now;
    } else if (u32Elapsed >= pstcDev->stcFilter.u16UpdateIntervalMs) {
        ADC_Device_UpdateFilter(pstcDev);
        pstcDev->stcFilter.u32LastUpdateTime = u32Now;
    }

    pstcDev->u32LastUpdateTime = u32Now;
    return RESULT_OK;
}

/* ========== ADC Specific Interfaces ========== */

uint16_t ADC_Device_GetRawValue(ADC_Device_t* pstcDev)
{
    if (pstcDev == NULL || !pstcDev->u8Initialized) return 0;
    return pstcDev->u16RawValue;
}

uint16_t ADC_Device_GetVoltageMV(ADC_Device_t* pstcDev)
{
    if (pstcDev == NULL || !pstcDev->u8Initialized) return 0;
    return pstcDev->u16VoltageMv;
}

uint16_t ADC_Device_GetAverageValue(ADC_Device_t* pstcDev)
{
    if (pstcDev == NULL || !pstcDev->u8Initialized) return 0;
    return pstcDev->u16AverageValue;
}

ADC_Device_t* ADC_Device_Create(const ADC_Config_t* pstcConfig)
{
    if (pstcConfig == NULL) return NULL;

    ADC_Device_t* pstcDev = (ADC_Device_t*)malloc(sizeof(ADC_Device_t));
    if (pstcDev == NULL) {
        ADC_DEBUG("Failed to allocate memory for ADC device\r\n");
        return NULL;
    }

    memset(pstcDev, 0, sizeof(ADC_Device_t));
    memcpy(&pstcDev->stcConfig, pstcConfig, sizeof(ADC_Config_t));

    ADC_DEBUG("Create: ADC_ID=%d, CH=%d, Mode=%s\r\n",
              pstcConfig->u8AdcId,
              pstcConfig->u8Channel,
              (pstcConfig->enAcqMode == ADC_ACQ_MODE_INTERRUPT) ? "Interrupt" : "AOS+DMA");

    pstcDev->u8Initialized = 0;
    return pstcDev;
}

/* ========== Filter Configuration Interfaces ========== */

void ADC_Device_SetFilter(ADC_Device_t* pstcDev, ADC_FilterType_t enType, uint8_t u8WindowSize)
{
    if (pstcDev == NULL) return;
    pstcDev->stcFilter.enType = enType;
    if (u8WindowSize > 0 && u8WindowSize <= ADC_FILTER_WINDOW_MAX) {
        pstcDev->stcFilter.u8WindowSize = u8WindowSize;
    }
    ADC_DEBUG("Filter set: type=%d, window=%d\r\n", enType, pstcDev->stcFilter.u8WindowSize);
}

void ADC_Device_SetFilterInterval(ADC_Device_t* pstcDev, uint16_t u16IntervalMs)
{
    if (pstcDev == NULL) return;
    pstcDev->stcFilter.u16UpdateIntervalMs = u16IntervalMs;
    ADC_DEBUG("Filter interval set: %d ms\r\n", u16IntervalMs);
}

void ADC_Device_SetFirstOrderAlpha(ADC_Device_t* pstcDev, float fAlpha)
{
    if (pstcDev == NULL) return;
    if (fAlpha > 0.0f && fAlpha <= 1.0f) {
        pstcDev->stcFilter.fAlpha = fAlpha;
        ADC_DEBUG("First order alpha set: %.3f\r\n", fAlpha);
    }
}

/* ========== Ring Buffer Operations ========== */

uint16_t ADC_Device_ReadBatch(uint8_t u8AdcDevId, uint16_t* pu16Buffer, uint16_t u16MaxSize)
{
    DeviceNode_t* pstcNode = DeviceManager_Get(u8AdcDevId);
    if (!pstcNode || !pstcNode->private_data) return 0;
    
    ADC_Device_t* pstcDev = (ADC_Device_t*)pstcNode->private_data;
    if (!pstcDev->u8UseRawRing) return 0;
    
    return ADC_Device_ReadRawFromRing(pstcDev, pu16Buffer, u16MaxSize);
}

uint16_t ADC_Device_GetBufferCount(uint8_t u8AdcDevId)
{
    DeviceNode_t* pstcNode = DeviceManager_Get(u8AdcDevId);
    if (!pstcNode || !pstcNode->private_data) return 0;
    
    ADC_Device_t* pstcDev = (ADC_Device_t*)pstcNode->private_data;
    if (!pstcDev->u8UseRawRing) return 0;
    
    return pstcDev->stcRawRing.u8Count;
}

void ADC_Device_ClearBuffer(uint8_t u8AdcDevId)
{
    DeviceNode_t* pstcNode = DeviceManager_Get(u8AdcDevId);
    if (!pstcNode || !pstcNode->private_data) return;
    
    ADC_Device_t* pstcDev = (ADC_Device_t*)pstcNode->private_data;
    if (!pstcDev->u8UseRawRing) return;
    
    pstcDev->stcRawRing.u8ReadIndex = pstcDev->stcRawRing.u8WriteIndex;
    pstcDev->stcRawRing.u8Count = 0;
    pstcDev->stcRawRing.u8Overflow = 0;
}

/* ========== Global Operations Table ========== */
const DeviceOps_t g_adc_ops = {
    .init    = ADC_Device_Init,
    .deinit  = ADC_Device_Deinit,
    .read    = ADC_Device_Read,
    .write   = ADC_Device_Write,
    .control = ADC_Device_Control,
    .update  = ADC_Device_Update
};
