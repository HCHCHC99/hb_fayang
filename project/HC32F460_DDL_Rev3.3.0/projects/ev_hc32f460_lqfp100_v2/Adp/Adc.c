/**
 *******************************************************************************
 * @file  Adc.c
 * @brief ADC Driver for HC32F460 - Generic framework with multiple instances
 *******************************************************************************
 */

#include "Adc.h"
#include "Dma.h"
#include "Gpio_io.h"
#include "rtt_log.h"
#include <stdlib.h>
#include <string.h>

/*******************************************************************************
 * Local variables ('static')
 ******************************************************************************/

static stc_adc_instance_t s_astcAdcInstances[ADC_MAX_INSTANCES];
static uint8_t s_u8AdcInstanceCount = 0;
static bool s_bAdcInitialized = false;
static bool s_bHasDmaInstance = false;
static int8_t s_a8AdcIdToDmaId[ADC_MAX_INSTANCES];

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/

static void Adc_InitConfig(void);
static void Adc_SetPinAnalogMode(uint8_t u8Port, uint8_t u8Pin);
static void Adc_HardTriggerConfig(void);
static void Adc_IrqConfig(void);
static void Adc_HardTriggerStart(void);
static void Timer0_Config(uint32_t u32IntervalUs);
static void ADC1_SeqA_IrqCallback(void);
static uint16_t Adc_CalcVoltage(uint16_t u16AdcValue);
static void Adc_ProcessInterruptChannels(void);
static char Adc_GetPortLetter(uint8_t u8Port);
static uint8_t Adc_GetPinNumber(uint16_t u16Pin);

/*******************************************************************************
 * Implementation
 ******************************************************************************/

static uint8_t Adc_GetPinNumber(uint16_t u16Pin)
{
    switch (u16Pin) {
        case GPIO_PIN_00: return 0;
        case GPIO_PIN_01: return 1;
        case GPIO_PIN_02: return 2;
        case GPIO_PIN_03: return 3;
        case GPIO_PIN_04: return 4;
        case GPIO_PIN_05: return 5;
        case GPIO_PIN_06: return 6;
        case GPIO_PIN_07: return 7;
        case GPIO_PIN_08: return 8;
        case GPIO_PIN_09: return 9;
        case GPIO_PIN_10: return 10;
        case GPIO_PIN_11: return 11;
        case GPIO_PIN_12: return 12;
        case GPIO_PIN_13: return 13;
        case GPIO_PIN_14: return 14;
        case GPIO_PIN_15: return 15;
        default: return 0;
    }
}

static char Adc_GetPortLetter(uint8_t u8Port)
{
    switch (u8Port) {
        case GPIO_PORT_A: return 'A';
        case GPIO_PORT_B: return 'B';
        case GPIO_PORT_C: return 'C';
        case GPIO_PORT_D: return 'D';
        case GPIO_PORT_E: return 'E';
        case GPIO_PORT_H: return 'H';
        default: return '?';
    }
}

static void Adc_SetPinAnalogMode(uint8_t u8Port, uint8_t u8Pin)
{
    stc_gpio_init_t stcGpioInit;
    (void)GPIO_StructInit(&stcGpioInit);
    stcGpioInit.u16PinAttr = PIN_ATTR_ANALOG;
    LL_PERIPH_WE(LL_PERIPH_GPIO);
    (void)GPIO_Init(u8Port, u8Pin, &stcGpioInit);
    LL_PERIPH_WP(LL_PERIPH_GPIO);
}

static void Adc_InitConfig(void)
{
    stc_adc_init_t stcAdcInit;

#ifdef ADC_DEBUG_TOGGLE_ENABLE
    LL_PERIPH_WE(LL_PERIPH_GPIO);
    Output_GPIO_Init(ADC_DEBUG_TOGGLE_PORT, ADC_DEBUG_TOGGLE_PIN, GPIO_INIT_LOW);
    LL_PERIPH_WP(LL_PERIPH_GPIO);
#endif

    LL_PERIPH_WE(LL_PERIPH_FCG);
    FCG_Fcg3PeriphClockCmd(ADC_PERIPH_CLK, ENABLE);
    LL_PERIPH_WP(LL_PERIPH_FCG);

    (void)ADC_StructInit(&stcAdcInit);
    stcAdcInit.u16ScanMode = ADC_MD_SEQA_SINGLESHOT;

    (void)ADC_Init(ADC_UNIT, &stcAdcInit);

    for (uint8_t i = 0; i < s_u8AdcInstanceCount; i++) {
        stc_adc_instance_t *pstcInst = &s_astcAdcInstances[i];
        
        Adc_SetPinAnalogMode(pstcInst->u8Port, pstcInst->u8Pin);
        ADC_Adp_DEBUG("ADC CH%d pin initialized (P%c%d)\r\n", 
               pstcInst->u8Channel,
               Adc_GetPortLetter(pstcInst->u8Port),
               Adc_GetPinNumber(pstcInst->u8Pin));
        
        ADC_ChCmd(ADC_UNIT, ADC_SEQ_A, pstcInst->u8Channel, ENABLE);
        ADC_Adp_DEBUG("ADC SEQ_A CH%d enabled (%s mode)\r\n", 
               pstcInst->u8Channel,
               (pstcInst->enMode == ADC_MODE_INTERRUPT) ? "Interrupt" : "DMA");
    }
    
    ADC_Adp_DEBUG("ADC initialized: %d channels in SEQ_A\r\n", s_u8AdcInstanceCount);
}

static void Timer0_Config(uint32_t u32IntervalUs)
{
    stc_tmr0_init_t stcTmr0Init;
    
    LL_PERIPH_WE(LL_PERIPH_FCG);
    FCG_Fcg2PeriphClockCmd(TMR0_PERIPH_CLK, ENABLE);
    LL_PERIPH_WP(LL_PERIPH_FCG);
    
    uint32_t u32TimerClk = CLK_GetBusClockFreq(CLK_BUS_PCLK1) / (1UL << (TMR0_CLK_DIV >> TMR0_BCONR_CKDIVA_POS));
    uint32_t u32Freq = 1000000UL / u32IntervalUs;
    uint16_t u16CompareValue = (uint16_t)(u32TimerClk / u32Freq) - 1;
    
    (void)TMR0_StructInit(&stcTmr0Init);
    stcTmr0Init.u32ClockDiv = TMR0_CLK_DIV;
    stcTmr0Init.u32Func = TMR0_FUNC_CMP;
    stcTmr0Init.u16CompareValue = u16CompareValue;
    stcTmr0Init.u32ClockSrc = TMR0_CLK_SRC_INTERN_CLK;
    
    (void)TMR0_Init(TMR0_UNIT, TMR0_CH, &stcTmr0Init);
    
    ADC_Adp_DEBUG("Timer0 configured for ADC trigger, interval=%lu us, compare=%u\r\n", 
           u32IntervalUs, u16CompareValue);
}

static void Adc_HardTriggerConfig(void)
{
    ADC_TriggerConfig(ADC_UNIT, ADC_SEQ_A, ADC_SEQA_HARDTRIG);
    ADC_TriggerCmd(ADC_UNIT, ADC_SEQ_A, ENABLE);
    ADC_Adp_DEBUG("SEQ_A: Timer0 -> ADC (mixed interrupt/DMA modes)\r\n");
}

static void Adc_IrqConfig(void)
{
    stc_irq_signin_config_t stcIrq;
    uint8_t u8AdcIntEn = 0U;

    uint8_t u8HasInterruptMode = 0U;
    for (uint8_t i = 0; i < s_u8AdcInstanceCount; i++) {
        if (s_astcAdcInstances[i].enMode == ADC_MODE_INTERRUPT) {
            u8HasInterruptMode = 1U;
            break;
        }
    }

    if (u8HasInterruptMode) {
        stcIrq.enIntSrc    = ADC_SEQA_INT_SRC;
        stcIrq.enIRQn      = ADC_SEQA_INT_IRQn;
        stcIrq.pfnCallback = &ADC1_SeqA_IrqCallback;
        LL_PERIPH_WE(LL_PERIPH_INTC);
        (void)INTC_IrqSignIn(&stcIrq);
        LL_PERIPH_WP(LL_PERIPH_INTC);
        NVIC_ClearPendingIRQ(stcIrq.enIRQn);
        NVIC_SetPriority(stcIrq.enIRQn, ADC_SEQA_INT_PRIO);
        NVIC_EnableIRQ(stcIrq.enIRQn);
        u8AdcIntEn |= ADC_INT_EOCA;
        ADC_Adp_DEBUG("SEQ_A interrupt enabled for interrupt-mode channels\r\n");
    } else {
        ADC_IntCmd(ADC_UNIT, ADC_INT_EOCA, DISABLE);
        ADC_Adp_DEBUG("SEQ_A interrupt disabled (no interrupt-mode channels)\r\n");
    }
    
    if (u8AdcIntEn != 0U) {
        ADC_IntCmd(ADC_UNIT, u8AdcIntEn, ENABLE);
    }
}

/**
 * @brief Process all interrupt mode channels in ADC interrupt
 * @note  Now calls callback with channel parameter for dev_adc layer
 */
static void Adc_ProcessInterruptChannels(void)
{
    uint16_t u16AdcValue;
    
    for (uint8_t i = 0; i < s_u8AdcInstanceCount; i++) {
        stc_adc_instance_t *pstcInst = &s_astcAdcInstances[i];
        
        if (pstcInst->enMode == ADC_MODE_INTERRUPT) {
            u16AdcValue = ADC_GetValue(ADC_UNIT, pstcInst->u8Channel);
            
            pstcInst->u16LatestValue = u16AdcValue;
            pstcInst->u8ValueUpdated = 1U;
            pstcInst->u32SampleCount++;
            
            /* Call callback with channel parameter if registered */
            if (pstcInst->pfnCallback != NULL) {
                pstcInst->pfnCallback(u16AdcValue, pstcInst->u8Channel);
            }
        }
    }
}

static void ADC1_SeqA_IrqCallback(void)
{
#ifdef ADC_DEBUG_TOGGLE_ENABLE
    LL_PERIPH_WE(LL_PERIPH_GPIO);
    GPIO_TOGGLE(ADC_DEBUG_TOGGLE_PORT, ADC_DEBUG_TOGGLE_PIN);
    LL_PERIPH_WP(LL_PERIPH_GPIO);
#endif

    ADC_ClearStatus(ADC_UNIT, ADC_FLAG_EOCA);
    Adc_ProcessInterruptChannels();
}

static void Adc_HardTriggerStart(void)
{
    TMR0_Start(TMR0_UNIT, TMR0_CH);
    ADC_Adp_DEBUG("Timer0 started for SEQ_A\r\n");
}

static uint16_t Adc_CalcVoltage(uint16_t u16AdcValue)
{
    return (uint16_t)((((float32_t)(u16AdcValue) * ADC_VREF) / ((float32_t)ADC_ACCURACY)) * 1000.F);
}

/*******************************************************************************
 * API Functions
 ******************************************************************************/

uint8_t Adc_Create(stc_adc_config_t *pstcConfig)
{
    if (s_u8AdcInstanceCount >= ADC_MAX_INSTANCES) {
        ADC_Adp_DEBUG("ADC instance full! Max %d\r\n", ADC_MAX_INSTANCES);
        return 0xFF;
    }
    
    for (uint8_t i = 0; i < s_u8AdcInstanceCount; i++) {
        if (s_astcAdcInstances[i].u8Channel == pstcConfig->u8Channel) {
            ADC_Adp_DEBUG("ADC CH%d already exists! Skip\r\n", pstcConfig->u8Channel);
            return 0xFF;
        }
    }
    
    uint8_t u8Id = s_u8AdcInstanceCount;
    stc_adc_instance_t *pstcInst = &s_astcAdcInstances[u8Id];
    
    memset(pstcInst, 0, sizeof(stc_adc_instance_t));
    pstcInst->u8Id = u8Id;
    pstcInst->u8Channel = pstcConfig->u8Channel;
    pstcInst->enMode = pstcConfig->enMode;
    pstcInst->u8Port = pstcConfig->stcPin.u8Port;
    pstcInst->u8Pin = pstcConfig->stcPin.u8Pin;
    pstcInst->pfnCallback = pstcConfig->pfnCallback;
    
    s_a8AdcIdToDmaId[u8Id] = -1;
    
    if (pstcConfig->enMode == ADC_MODE_DMA) {
        pstcInst->u16DmaBufferSize = pstcConfig->stcDmaConfig.u16BufferSize;
        pstcInst->u8DmaChannel = pstcConfig->stcDmaConfig.u8DmaChannel;
        s_bHasDmaInstance = true;
    }
    
    s_u8AdcInstanceCount++;
    
    ADC_Adp_DEBUG("ADC instance created: ID=%d, CH=%d, Mode=%s, Pin=P%c%d\r\n",
           u8Id, pstcInst->u8Channel,
           (pstcInst->enMode == ADC_MODE_INTERRUPT) ? "Interrupt" : "DMA",
           Adc_GetPortLetter(pstcInst->u8Port),
           Adc_GetPinNumber(pstcInst->u8Pin));
    
    return u8Id;
}

void Adc_Init(void)
{
    if (s_bAdcInitialized) {
        ADC_Adp_DEBUG("ADC driver already initialized\r\n");
        return;
    }
    
    if (s_u8AdcInstanceCount == 0) {
        ADC_Adp_DEBUG("No ADC instance created! Call Adc_Create first\r\n");
        return;
    }
    
    Adc_InitConfig();
    
    if (s_bHasDmaInstance) {
        for (uint8_t i = 0; i < s_u8AdcInstanceCount; i++) {
            stc_adc_instance_t *pstcInst = &s_astcAdcInstances[i];
            
            if (pstcInst->enMode == ADC_MODE_DMA) {
                stc_dma_config_t stcDmaConfig;
                memset(&stcDmaConfig, 0, sizeof(stc_dma_config_t));
                
                stcDmaConfig.u8DmaUnit      = 1;
                stcDmaConfig.u8Channel      = pstcInst->u8DmaChannel;
                stcDmaConfig.enDir          = DMA_DIR_PERIPH_TO_MEM;
                stcDmaConfig.enTransMode    = DMA_TRANS_MODE_REPEAT;
                stcDmaConfig.u32SrcAddr     = (uint32_t)((uint32_t)&ADC_UNIT->DR0 + (pstcInst->u8Channel * 2U));
                stcDmaConfig.u32DestAddr    = 0;
                stcDmaConfig.u32DataWidth   = DMA_DATAWIDTH_16BIT;
                stcDmaConfig.u16BlockSize   = pstcInst->u16DmaBufferSize;
                stcDmaConfig.u16TransCount  = 0;
                stcDmaConfig.u32SrcAddrInc  = DMA_SRC_ADDR_FIX;
                stcDmaConfig.u32DestAddrInc = DMA_DEST_ADDR_INC;
                stcDmaConfig.u8EnableInt    = 1;
                stcDmaConfig.u8IntPriority  = DMA_DEFAULT_INT_PRIO;
                stcDmaConfig.pfnCallback    = NULL;
                
                uint8_t u8DmaId = Dma_Create(&stcDmaConfig);
                if (u8DmaId != 0xFF) {
                    s_a8AdcIdToDmaId[i] = (int8_t)u8DmaId;
                    ADC_Adp_DEBUG("ADC ID%d -> DMA ID%d (DMA1 CH%d)\r\n",
                           i, u8DmaId, pstcInst->u8DmaChannel);
                } else {
                    ADC_Adp_DEBUG("Failed to create DMA for ADC ID%d (CH%d)\r\n",
                           i, pstcInst->u8Channel);
                }
            }
        }
        
        Dma_Init();
        Dma_StartAll();
    }
    
    Timer0_Config(ADC_SAMPLE_INTERVAL_US);
    Adc_HardTriggerConfig();
    Adc_IrqConfig();
    
    s_bAdcInitialized = true;
    ADC_Adp_DEBUG("ADC driver initialized with %d instance(s)\r\n", s_u8AdcInstanceCount);
}

void Adc_DeInit(void)
{
    Adc_Stop();
    
    if (s_bHasDmaInstance) {
        Dma_DeInit();
    }
    
    s_u8AdcInstanceCount = 0;
    s_bHasDmaInstance = false;
    s_bAdcInitialized = false;
    ADC_Adp_DEBUG("ADC driver deinitialized\r\n");
}

void Adc_Start(void)
{
    Adc_HardTriggerStart();
}

void Adc_Stop(void)
{
    TMR0_Stop(TMR0_UNIT, TMR0_CH);
    ADC_Adp_DEBUG("ADC stopped\r\n");
}

void Adc_ProcessData(uint32_t u32PrintIntervalMs)
{
    static uint32_t s_u32LastPrintTick = 0;
    uint32_t u32CurrentTick = (uint32_t)tickTimer_GetCount();
    bool bPrintNow = false;
    
    if ((u32PrintIntervalMs > 0) && ((u32CurrentTick - s_u32LastPrintTick) >= u32PrintIntervalMs)) {
        bPrintNow = true;
        s_u32LastPrintTick = u32CurrentTick;
    }
    
    for (uint8_t i = 0; i < s_u8AdcInstanceCount; i++) {
        if (s_astcAdcInstances[i].u8ValueUpdated != 0U) {
            s_astcAdcInstances[i].u8ValueUpdated = 0U;
        }
    }
    
    if (bPrintNow) {
        for (uint8_t i = 0; i < s_u8AdcInstanceCount; i++) {
            stc_adc_instance_t *pstcInst = &s_astcAdcInstances[i];
            
            if (pstcInst->enMode == ADC_MODE_INTERRUPT) {
                if(ADC_RTT == 1) {
                    ADC_Adp_DEBUG("ADC ID%d CH%d [P%c%d] samples=%lu, Latest=%4u ADC (%3u mV)\r\n",
                           pstcInst->u8Id, pstcInst->u8Channel,
                           Adc_GetPortLetter(pstcInst->u8Port), 
                           Adc_GetPinNumber(pstcInst->u8Pin),
                           pstcInst->u32SampleCount,
                           pstcInst->u16LatestValue, 
                           Adc_CalcVoltage(pstcInst->u16LatestValue));
                }
            }
            else if (pstcInst->enMode == ADC_MODE_DMA) {
                int8_t s8DmaId = s_a8AdcIdToDmaId[i];
                if (s8DmaId >= 0) {
                    uint16_t u16Latest = Dma_GetLatestValue((uint8_t)s8DmaId);
                    uint16_t u16Avg = Dma_GetAverageValue((uint8_t)s8DmaId);
                    uint32_t u32Count = Dma_GetTransferCount((uint8_t)s8DmaId);
                    
                    if(ADC_RTT == 1) {
                        ADC_Adp_DEBUG("ADC ID%d CH%d [P%c%d] samples=%lu, Latest=%4u ADC (%3u mV), Avg=%4u ADC (%3u mV)\r\n",
                               pstcInst->u8Id, pstcInst->u8Channel,
                               Adc_GetPortLetter(pstcInst->u8Port),
                               Adc_GetPinNumber(pstcInst->u8Pin),
                               u32Count,
                               u16Latest, Adc_CalcVoltage(u16Latest),
                               u16Avg, Adc_CalcVoltage(u16Avg));
                    }
                }
            }
        }
    }
}

uint16_t Adc_GetLatestValue(uint8_t u8AdcId)
{
    if (u8AdcId >= s_u8AdcInstanceCount) {
        return 0;
    }
    
    stc_adc_instance_t *pstcInst = &s_astcAdcInstances[u8AdcId];
    
    if (pstcInst->enMode == ADC_MODE_INTERRUPT) {
        return pstcInst->u16LatestValue;
    } else if (pstcInst->enMode == ADC_MODE_DMA) {
        int8_t s8DmaId = s_a8AdcIdToDmaId[u8AdcId];
        if (s8DmaId >= 0) {
            return Dma_GetLatestValue((uint8_t)s8DmaId);
        }
    }
    return 0;
}

uint16_t Adc_GetAverageValue(uint8_t u8AdcId)
{
    if (u8AdcId >= s_u8AdcInstanceCount) {
        return 0;
    }
    
    stc_adc_instance_t *pstcInst = &s_astcAdcInstances[u8AdcId];
    
    if (pstcInst->enMode == ADC_MODE_INTERRUPT) {
        return pstcInst->u16LatestValue;
    } else if (pstcInst->enMode == ADC_MODE_DMA) {
        int8_t s8DmaId = s_a8AdcIdToDmaId[u8AdcId];
        if (s8DmaId >= 0) {
            return Dma_GetAverageValue((uint8_t)s8DmaId);
        }
    }
    return 0;
}

uint32_t Adc_GetSampleCount(uint8_t u8AdcId)
{
    if (u8AdcId >= s_u8AdcInstanceCount) {
        return 0;
    }
    
    stc_adc_instance_t *pstcInst = &s_astcAdcInstances[u8AdcId];
    
    if (pstcInst->enMode == ADC_MODE_INTERRUPT) {
        return pstcInst->u32SampleCount;
    } else if (pstcInst->enMode == ADC_MODE_DMA) {
        int8_t s8DmaId = s_a8AdcIdToDmaId[u8AdcId];
        if (s8DmaId >= 0) {
            return Dma_GetTransferCount((uint8_t)s8DmaId);
        }
    }
    return 0;
}

int8_t Adc_FindIdByChannel(uint8_t u8Channel)
{
    for (uint8_t i = 0; i < s_u8AdcInstanceCount; i++) {
        if (s_astcAdcInstances[i].u8Channel == u8Channel) {
            return (int8_t)i;
        }
    }
    return -1;
}

void Adc_EnableInterrupt(void)
{
    stc_irq_signin_config_t stcIrq;

    stcIrq.enIntSrc    = ADC_SEQA_INT_SRC;
    stcIrq.enIRQn      = ADC_SEQA_INT_IRQn;
    stcIrq.pfnCallback = &ADC1_SeqA_IrqCallback;
    LL_PERIPH_WE(LL_PERIPH_INTC);
    (void)INTC_IrqSignIn(&stcIrq);
    LL_PERIPH_WP(LL_PERIPH_INTC);

    NVIC_ClearPendingIRQ(stcIrq.enIRQn);
    NVIC_SetPriority(stcIrq.enIRQn, ADC_SEQA_INT_PRIO);
    NVIC_EnableIRQ(stcIrq.enIRQn);

    ADC_IntCmd(ADC_UNIT, ADC_INT_EOCA, ENABLE);

    ADC_Adp_DEBUG("ADC interrupt re-enabled (EOCA)\r\n");
}

#ifdef DEBUG
void Adc_PrintDebugInfo(void)
{
    ADC_Adp_DEBUG("=== ADC Driver Debug Info ===\r\n");
    ADC_Adp_DEBUG("Initialized: %s\r\n", s_bAdcInitialized ? "Yes" : "No");
    ADC_Adp_DEBUG("Instance count: %d\r\n", s_u8AdcInstanceCount);
    
    for (uint8_t i = 0; i < s_u8AdcInstanceCount; i++) {
        stc_adc_instance_t *pstcInst = &s_astcAdcInstances[i];
        if (pstcInst->enMode == ADC_MODE_INTERRUPT) {
            ADC_Adp_DEBUG("  ID%d: CH%d, Interrupt mode, Pin=P%c%d, samples=%lu\r\n",
                   pstcInst->u8Id, pstcInst->u8Channel,
                   Adc_GetPortLetter(pstcInst->u8Port),
                   Adc_GetPinNumber(pstcInst->u8Pin),
                   pstcInst->u32SampleCount);
        } else {
            int8_t s8DmaId = s_a8AdcIdToDmaId[i];
            ADC_Adp_DEBUG("  ID%d: CH%d, DMA mode (DMA ID=%d), Pin=P%c%d, samples=%lu\r\n",
                   pstcInst->u8Id, pstcInst->u8Channel,
                   s8DmaId,
                   Adc_GetPortLetter(pstcInst->u8Port),
                   Adc_GetPinNumber(pstcInst->u8Pin),
                   pstcInst->u32SampleCount);
        }
    }
}
#endif

/*******************************************************************************
 * EOF
 ******************************************************************************/
