#include "rs485.h"
#include "Hardware.h"
#include "TickTimer.h"
#include "Gpio_io.h"
#include "lock.h"
#include "ring_buf.h"
#include <string.h>

/*=============================================================================
 * USART4 ���Ŷ���
 *============================================================================*/
#define USART_RX_PORT       (GPIO_PORT_B)
#define USART_RX_PIN        (GPIO_PIN_12)
#define USART_RX_GPIO_FUNC  (GPIO_FUNC_37)
#define USART_TX_PORT       (GPIO_PORT_B)
#define USART_TX_PIN        (GPIO_PIN_13)
#define USART_TX_GPIO_FUNC  (GPIO_FUNC_36)
#define USART_UNIT          (CM_USART4)
#define USART_FCG_ENABLE()  FCG_Fcg1PeriphClockCmd(FCG1_PERIPH_USART4, ENABLE)

#include "main.h"

/* 硬件板本：RS485 方向控制引脚 */
#if BOARD_VERSION == 0
    // 原HB_chuchai板
    #define RS485_DIR_PORT      (GPIO_PORT_B)
    #define RS485_DIR_PIN       (GPIO_PIN_14)
#else
    // 整合板
    #define RS485_DIR_PORT      (GPIO_PORT_A)
    #define RS485_DIR_PIN       (GPIO_PIN_03)
#endif
/* RS485 �������ģʽѡ�� */
#define RS485_DIR_MODE_1    0
#define RS485_DIR_MODE_2    1
#define RS485_DIR_MODE      RS485_DIR_MODE_1

#if (RS485_DIR_MODE == RS485_DIR_MODE_1)
    #define RS485_TX_MODE()     GPIO_SetPins(RS485_DIR_PORT, RS485_DIR_PIN)
    #define RS485_RX_MODE()     GPIO_ResetPins(RS485_DIR_PORT, RS485_DIR_PIN)
#elif (RS485_DIR_MODE == RS485_DIR_MODE_2)
    #define RS485_TX_MODE()     GPIO_ResetPins(RS485_DIR_PORT, RS485_DIR_PIN)
    #define RS485_RX_MODE()     GPIO_SetPins(RS485_DIR_PORT, RS485_DIR_PIN)
#else
    #error "Invalid RS485_DIR_MODE selection!"
#endif

/* USART �ж϶��� */
#define USART_RX_ERR_IRQn     (INT000_IRQn)
#define USART_RX_ERR_INT_SRC  (INT_SRC_USART4_EI)
#define USART_RX_FULL_IRQn    (INT001_IRQn)
#define USART_RX_FULL_INT_SRC (INT_SRC_USART4_RI)
#define USART_TX_EMPTY_IRQn   (INT002_IRQn)
#define USART_TX_EMPTY_INT_SRC (INT_SRC_USART4_TI)
#define USART_TX_CPLT_IRQn    (INT003_IRQn)
#define USART_TX_CPLT_INT_SRC (INT_SRC_USART4_TCI)

/*=============================================================================
 * �����ʷ�Ƶ�Զ�ѡ��
 *============================================================================*/
#if (RS485_BAUDRATE <= 4800UL)
    #define RS485_CLK_DIV       USART_CLK_DIV64
#elif (RS485_BAUDRATE <= 38400UL)
    #define RS485_CLK_DIV       USART_CLK_DIV16
#elif (RS485_BAUDRATE <= 115200UL)
    #define RS485_CLK_DIV       USART_CLK_DIV4
#else
    #define RS485_CLK_DIV       USART_CLK_DIV1
#endif

/*=============================================================================
 * ֡�����ʱʱ�䣨3.5�ַ�ʱ�䣩
 *============================================================================*/
#if (RS485_BAUDRATE <= 1200UL)
    #define MODBUS_FRAME_TIMEOUT_MS (30U)
#elif (RS485_BAUDRATE <= 2400UL)
    #define MODBUS_FRAME_TIMEOUT_MS (15U)
#elif (RS485_BAUDRATE <= 4800UL)
    #define MODBUS_FRAME_TIMEOUT_MS (8U)
#elif (RS485_BAUDRATE <= 9600UL)
    #define MODBUS_FRAME_TIMEOUT_MS (4U)
#else
    #define MODBUS_FRAME_TIMEOUT_MS (3U)
#endif

/*=============================================================================
 * ��̬����
 *============================================================================*/
static uint8_t m_au8RxRawBuf[RX_RING_BUF_SIZE];
static uint8_t m_au8TxBuf[TX_RING_BUF_SIZE];
static stc_ring_buf_t m_stcRxRingBuf;
static stc_ring_buf_t m_stcTxRingBuf;

static __IO en_flag_status_t m_enTxCompleteFlag = SET;
static uint32_t m_u32LastRxTime = 0;

static mutex_t m_stcRs485Mutex;
static msg_t m_au8TxQueueBuffer[TX_QUEUE_SIZE];
static msg_queue_t m_stcTxQueue;
static msg_t m_au8RxFrameQueueBuffer[RX_FRAME_QUEUE_SIZE];
static msg_queue_t m_stcRxFrameQueue;

static volatile uint8_t m_u8IsSending = 0;
static volatile uint32_t m_u32SendStartTime = 0;

/*=============================================================================
 * ��̬��������
 *============================================================================*/
static void INTC_IrqInstalHandler(const stc_irq_signin_config_t *cfg, uint32_t prio);
static void USART_RxFull_IrqCallback(void);
static void USART_RxError_IrqCallback(void);
static void USART_TxEmpty_IrqCallback(void);
static void USART_TxComplete_IrqCallback(void);
static void RS485_DIR_Init(void);
static void RS485_FrameParser(void);
static void SendData_RS485_Internal(uint8_t *pData, uint16_t len);
static void RS485_OnSendComplete(void);

/*=============================================================================
 * @brief  USART ��������жϻص�
 *============================================================================*/
static void USART_RxFull_IrqCallback(void)
{
    uint8_t ch = (uint8_t)USART_ReadData(USART_UNIT);
    BUF_Write(&m_stcRxRingBuf, &ch, 1U);
    m_u32LastRxTime = tickTimer_GetCount();
}

/*=============================================================================
 * @brief  USART �����жϻص�
 *============================================================================*/
static void USART_RxError_IrqCallback(void)
{
    uint32_t errFlag = USART_FLAG_OVERRUN | USART_FLAG_FRAME_ERR | USART_FLAG_PARITY_ERR;
    USART_ClearStatus(USART_UNIT, errFlag);
    (void)USART_ReadData(USART_UNIT);
    RS485_DEBUG("USART err");
}

/*=============================================================================
 * @brief  USART ���Ϳ��жϻص�
 *============================================================================*/
static void USART_TxEmpty_IrqCallback(void)
{
    uint8_t ch;
    if (!BUF_Empty(&m_stcTxRingBuf))
    {
        BUF_Read(&m_stcTxRingBuf, &ch, 1U);
        USART_WriteData(USART_UNIT, ch);
    }
    else
    {
        USART_FuncCmd(USART_UNIT, USART_INT_TX_CPLT, ENABLE);
        ERR_DEBUG("[RS485_TXE] buf empty, enable TX_CPLT");
    }
}

/*=============================================================================
 * @brief  USART ��������жϻص�
 *============================================================================*/
static void USART_TxComplete_IrqCallback(void)
{
    ERR_DEBUG("[RS485_TXC] TX complete IRQ fired!");
    m_enTxCompleteFlag = SET;
    USART_FuncCmd(USART_UNIT, USART_TX | USART_INT_TX_EMPTY | USART_INT_TX_CPLT, DISABLE);
    RS485_OnSendComplete();
    RS485_DEBUG("TX cplt");
}

/*=============================================================================
 * @brief  �жϰ�װ����
 *============================================================================*/
static void INTC_IrqInstalHandler(const stc_irq_signin_config_t *cfg, uint32_t prio)
{
    if (cfg != NULL)
    {
        INTC_IrqSignIn(cfg);
        NVIC_ClearPendingIRQ(cfg->enIRQn);
        NVIC_SetPriority(cfg->enIRQn, prio);
        NVIC_EnableIRQ(cfg->enIRQn);
    }
}

/*=============================================================================
 * @brief  RS485����������ų�ʼ��
 *============================================================================*/
static void RS485_DIR_Init(void)
{
    LL_PERIPH_WE(LL_PERIPH_GPIO);
    Output_GPIO_Init(RS485_DIR_PORT, RS485_DIR_PIN, GPIO_INIT_LOW);
    GPIO_SetFunc(RS485_DIR_PORT, RS485_DIR_PIN, GPIO_FUNC_0);
    LL_PERIPH_WP(LL_PERIPH_GPIO);
    RS485_DEBUG("DIR init");
}

/*=============================================================================
 * @brief  ֡�������������֡�����ʱ����װ����֡��
 *============================================================================*/
static void RS485_FrameParser(void)
{
    uint32_t currentTime = tickTimer_GetCount();
    uint16_t available = BUF_UsedSize(&m_stcRxRingBuf);
    uint16_t freeSpace = BUF_FreeSize(&m_stcRxRingBuf);

    if (available == 0) {
        return;
    }

    /* �����ջ�����ʣ��ռ䣬������ֵʱ���������ݷ�ֹ��� */
    if (freeSpace < 64U) {
        WARN_DEBUG("RxBuf nearly full! free=%d, discard old", freeSpace);
        uint16_t discardLen = available / 2U;
        uint8_t discardBuf[32];
        while (discardLen > 0) {
            uint16_t chunk = (discardLen > sizeof(discardBuf)) ? sizeof(discardBuf) : discardLen;
            // __disable_irq();
            BUF_Read(&m_stcRxRingBuf, discardBuf, chunk);
            // __enable_irq();
            discardLen -= chunk;
        }
        available = BUF_UsedSize(&m_stcRxRingBuf);
        WARN_DEBUG("After discard, avail=%d", available);
        if (available == 0) {
            return;
        }
    }

    /* ���֡�����ʱ��3.5�ַ�ʱ�䣩*/
    uint32_t elapsed = currentTime - m_u32LastRxTime;

    if (elapsed >= MODBUS_FRAME_TIMEOUT_MS)
    {
        uint8_t frameBuf[256];
        uint16_t frameLen = available;
        if (frameLen > 256) {
            frameLen = 256;
        }

        // __disable_irq();
        BUF_Read(&m_stcRxRingBuf, frameBuf, frameLen);
        // __enable_irq();

        FRAME_DEBUG("Frame done, size=%d", frameLen);

        /* ������֡�������֡���� */
        msg_t rxMsg;
        rxMsg.data = rxMsg.buffer;
        rxMsg.len = frameLen;
        rxMsg.type = 0;
        rxMsg.priority = MSG_PRIO_NORMAL;
        memcpy(rxMsg.buffer, frameBuf, frameLen);
        MsgQueue_Send(&m_stcRxFrameQueue, &rxMsg, false, "RS485_FrameParser");
    }
}

/*=============================================================================
 * @brief  �ڲ����ͺ�����ʵ��ִ�з��ͣ���������
 *============================================================================*/
static void SendData_RS485_Internal(uint8_t *pData, uint16_t len)
{
    uint16_t written;

    RS485_TX_MODE();
    RS485_DEBUG("TX mode");

    m_u32SendStartTime = tickTimer_GetCount();
    written = BUF_Write(&m_stcTxRingBuf, pData, len);
    if (written != len)
    {
        ERR_DEBUG("[RS485_ERR] BUF_Write failed! written=%d, expected=%d", written, len);
    }
    RS485_DEBUG("send %d bytes (written=%d)", len, written);

    if (m_enTxCompleteFlag == SET)
    {
        m_enTxCompleteFlag = RESET;
        USART_FuncCmd(USART_UNIT, USART_TX | USART_INT_TX_EMPTY, ENABLE);
    }
}

/*=============================================================================
 * @brief  ������ɴ���
 *============================================================================*/
static void RS485_OnSendComplete(void)
{
    uint32_t elapsed = tickTimer_GetCount() - m_u32SendStartTime;
    if (elapsed > 500) {
        RS485_DEBUG("send timeout! elapsed=%dms", elapsed);
    }

    RS485_RX_MODE();
    m_u8IsSending = 0;
    Lock_Unlock(&m_stcRs485Mutex, "RS485_OnSendComplete");

    RS485_DEBUG("TX done, queued=%d", MsgQueue_GetCount(&m_stcTxQueue));
}

/*=============================================================================
 * @brief  RS485 ��ʼ��
 *============================================================================*/
void RS485_Init(void)
{
    stc_usart_uart_init_t uartCfg;
    stc_irq_signin_config_t irqCfg;

    /* ��ʼ�������� */
    Lock_Init(&m_stcRs485Mutex, LOCK_TYPE_MUTEX, "RS485_Mutex");

    /* ��ʼ�����Ͷ��� */
    queue_config_t txCfg;
    txCfg.max_size = TX_QUEUE_SIZE;
    txCfg.overwrite = false;
    txCfg.priority_enabled = false;
    txCfg.timeout_ms = 0;
    MsgQueue_Init(&m_stcTxQueue, m_au8TxQueueBuffer, TX_QUEUE_SIZE, &txCfg, "RS485_TxQ");

    /* ��ʼ������֡���� */
    queue_config_t rxCfg;
    rxCfg.max_size = RX_FRAME_QUEUE_SIZE;
    rxCfg.overwrite = false;
    rxCfg.priority_enabled = false;
    rxCfg.timeout_ms = 0;
    MsgQueue_Init(&m_stcRxFrameQueue, m_au8RxFrameQueueBuffer, RX_FRAME_QUEUE_SIZE, &rxCfg, "RS485_RxQ");

    /* ��ʼ�����λ����� */
    BUF_Init(&m_stcRxRingBuf, m_au8RxRawBuf, RX_RING_BUF_SIZE);
    BUF_Init(&m_stcTxRingBuf, m_au8TxBuf, TX_RING_BUF_SIZE);

    m_u8IsSending = 0;
    m_u32LastRxTime = tickTimer_GetCount();

    /* GPIO�������� */
    GPIO_SetFunc(USART_RX_PORT, USART_RX_PIN, USART_RX_GPIO_FUNC);
    GPIO_SetFunc(USART_TX_PORT, USART_TX_PIN, USART_TX_GPIO_FUNC);

    /* USART��ʼ�� */
    USART_FCG_ENABLE();
    USART_UART_StructInit(&uartCfg);
    uartCfg.u32Baudrate      = RS485_BAUDRATE;
    uartCfg.u32ClockDiv      = RS485_CLK_DIV;
    uartCfg.u32OverSampleBit = USART_OVER_SAMPLE_16BIT;
    USART_UART_Init(USART_UNIT, &uartCfg, NULL);

    /* �ж����� */
    irqCfg.enIRQn = USART_RX_ERR_IRQn;
    irqCfg.enIntSrc = USART_RX_ERR_INT_SRC;
    irqCfg.pfnCallback = USART_RxError_IrqCallback;
    INTC_IrqInstalHandler(&irqCfg, DDL_IRQ_PRIO_12);

    irqCfg.enIRQn = USART_RX_FULL_IRQn;
    irqCfg.enIntSrc = USART_RX_FULL_INT_SRC;
    irqCfg.pfnCallback = USART_RxFull_IrqCallback;
    INTC_IrqInstalHandler(&irqCfg, DDL_IRQ_PRIO_12);

    irqCfg.enIRQn = USART_TX_EMPTY_IRQn;
    irqCfg.enIntSrc = USART_TX_EMPTY_INT_SRC;
    irqCfg.pfnCallback = USART_TxEmpty_IrqCallback;
    INTC_IrqInstalHandler(&irqCfg, DDL_IRQ_PRIO_12);

    irqCfg.enIRQn = USART_TX_CPLT_IRQn;
    irqCfg.enIntSrc = USART_TX_CPLT_INT_SRC;
    irqCfg.pfnCallback = USART_TxComplete_IrqCallback;
    INTC_IrqInstalHandler(&irqCfg, DDL_IRQ_PRIO_12);

    /* RS485����������ų�ʼ�� */
    RS485_DIR_Init();

    /* ʹ��USART���� */
    USART_FuncCmd(USART_UNIT, USART_RX | USART_INT_RX, ENABLE);

    RS485_DEBUG("Init done");
}

/*=============================================================================
 * @brief  RS485 ���ͣ������Ͷ��У�
 *============================================================================*/
bool RS485_Send(uint8_t *pData, uint16_t len)
{
    if (pData == NULL || len == 0 || len > 256) {
        return false;
    }

    if (Lock_TryLock(&m_stcRs485Mutex, "RS485_Send")) {
        m_u8IsSending = 1;
        SendData_RS485_Internal(pData, len);
        return true;
    } else {
        RS485_DEBUG("busy, queue msg (qsz=%d)",
               MsgQueue_GetCount(&m_stcTxQueue) + 1);

        msg_t txMsg;
        txMsg.data = txMsg.buffer;
        txMsg.len = len;
        txMsg.type = MSG_TYPE_RS485_TX;
        txMsg.priority = MSG_PRIO_NORMAL;
        memcpy(txMsg.buffer, pData, len);
        return MsgQueue_Send(&m_stcTxQueue, &txMsg, false, "RS485_Send");
    }
}

/*=============================================================================
 * @brief  RS485 ��ѭ����ѯ
 *============================================================================*/
void RS485_Poll(void)
{
    /* 1. ֡���� */
    RS485_FrameParser();

    /* 2. ���Ͷ��д�����������ɺ󣬴Ӷ���ȡ����һ����Ϣ���� */
    if ((m_enTxCompleteFlag == SET) && (MsgQueue_GetCount(&m_stcTxQueue) > 0))
    {
        msg_t next_msg;
        if (MsgQueue_Receive(&m_stcTxQueue, &next_msg, 0, "RS485_Poll")) {
            RS485_DEBUG("dequeue send, remain=%d",
                   MsgQueue_GetCount(&m_stcTxQueue));
            if (Lock_TryLock(&m_stcRs485Mutex, "RS485_Poll")) {
                m_u8IsSending = 1;
                SendData_RS485_Internal(next_msg.data, next_msg.len);
            }
        }
    }
}

/*=============================================================================
 * @brief  ��ȡ����֡����
 *============================================================================*/
msg_queue_t* RS485_GetRxFrameQueue(void)
{
    return &m_stcRxFrameQueue;
}
