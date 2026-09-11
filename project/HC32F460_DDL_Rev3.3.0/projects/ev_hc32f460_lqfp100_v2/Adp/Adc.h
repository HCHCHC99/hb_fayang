/**
 *******************************************************************************
 * @file  Adc.h
 * @brief ADC Driver for HC32F460 - Generic framework with multiple instances
 *******************************************************************************
 */

#ifndef __ADC_H__
#define __ADC_H__

#include "main.h"
#include "Hardware.h"
#include "TickTimer.h"

#define ADC_RTT 1

#ifdef DEBUG_ADC_Adp
    #define ADC_Adp_DEBUG(fmt, ...)    MAIN_D("[ADC_DEBUG] " fmt, ##__VA_ARGS__)
#else
    #define ADC_Adp_DEBUG(fmt, ...)    ((void)0)
#endif

#ifdef ADC_OUTPUT_Adp
    #define ADC_Adp_OUT(fmt, ...)      MAIN_D("[ADC_OUT] " fmt, ##__VA_ARGS__)
#else
    #define ADC_Adp_OUT(fmt, ...)      ((void)0)
#endif

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Global pre-processor symbols/macros ('#define')
 ******************************************************************************/

/* DMA buffer size */
#define ADC_DMA_BUFFER_SIZE              (8U)

/* ADC hardware definitions */
#define ADC_UNIT                        (CM_ADC1)
#define ADC_PERIPH_CLK                  (FCG3_PERIPH_ADC1)

/* Sequence A hardware trigger configuration */
#define ADC_SEQA_HARDTRIG               (ADC_HARDTRIG_EVT0)
#define ADC_SEQA_AOS_TRIG_SEL           (AOS_ADC1_0)
#define ADC_SEQA_TRIG_EVT               (EVT_SRC_TMR0_1_CMP_B)

/* DMA configuration */
#define DMA_UNIT                        (CM_DMA1)
#define DMA_PERIPH_CLK                  (FCG0_PERIPH_DMA1)
#define DMA_DATA_WIDTH                  (DMA_DATAWIDTH_16BIT)
#define DMA_TRANS_CNT                   (0U)
#define DMA_INT_PRIO                    (DDL_IRQ_PRIO_03)

/* Timer0 for sequence A */
#define TMR0_UNIT                       (CM_TMR0_1)
#define TMR0_CH                         (TMR0_CH_B)
#define TMR0_PERIPH_CLK                 (FCG2_PERIPH_TMR0_1)
#define TMR0_CLK_DIV                    (TMR0_CLK_DIV256)
#define ADC_SAMPLE_INTERVAL_US          (1000U)

/* ADC interrupt configuration */
#define ADC_SEQA_INT_PRIO               (DDL_IRQ_PRIO_06)
#define ADC_SEQA_INT_SRC                (INT_SRC_ADC1_EOCA)
#define ADC_SEQA_INT_IRQn               (INT116_IRQn)

/* ADC debug toggle pin - comment to disable */
//#define ADC_DEBUG_TOGGLE_ENABLE

#ifdef ADC_DEBUG_TOGGLE_ENABLE
#define ADC_DEBUG_TOGGLE_PORT           (GPIO_PORT_A)
#define ADC_DEBUG_TOGGLE_PIN            (GPIO_PIN_07)
#endif

/* ADC reference voltage */
#define ADC_VREF                        (3.3F)
#define ADC_ACCURACY                    (1UL << 12U)
#define ADC_CAL_VOL(adcVal)             (uint16_t)((((float32_t)(adcVal) * ADC_VREF) / ((float32_t)ADC_ACCURACY)) * 1000.F)

/* Maximum ADC instances */
#define ADC_MAX_INSTANCES               (8U)
#define ADC_MAX_CHANNEL                 (32U)
#define ADC_MAX_DMA_CH                  (8U)

/*******************************************************************************
 * Global type definitions ('typedef')
 ******************************************************************************/

/**
 * @brief ADC channel operation mode
 */
typedef enum {
    ADC_MODE_INTERRUPT = 0,
    ADC_MODE_DMA = 1,
} en_adc_mode_t;

/**
 * @brief ADC pin mapping structure
 */
typedef struct {
    uint8_t u8Port;
    uint8_t u8Pin;
} stc_adc_pin_t;

/**
 * @brief ADC callback function type with channel parameter
 */
typedef void (*AdcCallback_t)(uint16_t u16AdcValue, uint8_t u8Channel);

/**
 * @brief ADC channel configuration structure
 */
typedef struct {
    uint8_t u8Channel;
    en_adc_mode_t enMode;
    stc_adc_pin_t stcPin;
    AdcCallback_t pfnCallback;          /* Callback with channel parameter */
    struct {
        uint16_t u16BufferSize;
        uint8_t u8DmaChannel;
    } stcDmaConfig;
} stc_adc_config_t;

/**
 * @brief ADC instance structure
 */
typedef struct {
    uint8_t u8Id;
    uint8_t u8Channel;
    en_adc_mode_t enMode;
    uint8_t u8Port;
    uint8_t u8Pin;
    AdcCallback_t pfnCallback;          /* Callback with channel parameter */
    uint16_t *pu16DmaBuffer;
    uint16_t u16DmaBufferSize;
    uint8_t u8DmaChannel;
    uint32_t u32SampleCount;
    uint16_t u16LatestValue;
    uint8_t u8ValueUpdated;
} stc_adc_instance_t;

/*******************************************************************************
 * Global function prototypes
 ******************************************************************************/

uint8_t Adc_Create(stc_adc_config_t *pstcConfig);
void Adc_Init(void);
void Adc_DeInit(void);

void Adc_Start(void);
void Adc_Stop(void);
void Adc_ProcessData(uint32_t u32PrintIntervalMs);

uint16_t Adc_GetLatestValue(uint8_t u8AdcId);
uint16_t Adc_GetAverageValue(uint8_t u8AdcId);
uint32_t Adc_GetSampleCount(uint8_t u8AdcId);

int8_t Adc_FindIdByChannel(uint8_t u8Channel);

void Adc_EnableInterrupt(void);

#ifdef DEBUG
void Adc_PrintDebugInfo(void);
#endif

#ifdef __cplusplus
}
#endif

#endif /* __ADC_H__ */
