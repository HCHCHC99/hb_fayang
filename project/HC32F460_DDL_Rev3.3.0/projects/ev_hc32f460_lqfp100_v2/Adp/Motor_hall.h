#ifndef __MOTOR_HALL_H__
#define __MOTOR_HALL_H__

#include "hc32_ll.h"
#include "Adapter.h"

/* ========== ������ò�������ת�١�ת����أ� ========== */

/**
 * @brief ��������������
 */
typedef struct {
    /* GPIO���� */
    uint8_t hall_a_port;        /* GPIO_PORT_A �� */
    uint16_t hall_a_pin;        /* GPIO_PIN_xx */
    uint8_t hall_b_port;
    uint16_t hall_b_pin;

    /* �ж����� */
    uint32_t eirq_ch_a;         /* EXTINT_CHxx */
    uint32_t eirq_ch_b;
    uint8_t irqn_a;             /* INTxxx_IRQn */
    uint8_t irqn_b;
    uint32_t irq_src_a;         /* INT_PORT_EIRQx */
    uint32_t irq_src_b;
    uint8_t irq_priority;

    /* ���������ת��ת����أ� */
    uint8_t pole_pairs;
    uint8_t hall_count;
    uint16_t custom_pulses_per_rev;

} motor_hall_config_t;

/* ========== Ĭ������ʾ����ԭ�������? - PA9, PA10�� ========== */
#define DEFAULT_HALL_A_PORT      GPIO_PORT_A
#define DEFAULT_HALL_A_PIN       GPIO_PIN_09
#define DEFAULT_HALL_B_PORT      GPIO_PORT_A
#define DEFAULT_HALL_B_PIN       GPIO_PIN_10

#define DEFAULT_HALL_A_EIRQ_CH   EXTINT_CH09
#define DEFAULT_HALL_B_EIRQ_CH   EXTINT_CH10
#define DEFAULT_HALL_A_IRQN      INT009_IRQn
#define DEFAULT_HALL_B_IRQN      INT010_IRQn
#define DEFAULT_HALL_A_IRQ_SRC   INT_PORT_EIRQ9
#define DEFAULT_HALL_B_IRQ_SRC   INT_PORT_EIRQ10

#define DEFAULT_HALL_IRQ_PRIORITY DDL_IRQ_PRIORITY_02

/* Ĭ�ϵ������? */
#define DEFAULT_POLE_PAIRS       (3)
#define DEFAULT_HALL_COUNT       (2)

/* �Զ�����ÿת�������������� �� ������ �� 2��˫���أ� */
#define CALC_PULSES_PER_REV(pole_pairs, hall_count) ((pole_pairs) * (hall_count) * 2)
/* 霍尔方向反转（当A/B接线反时启用�?*/
// #define HALL_DIRECTION_INVERT

/* ========== ����״̬ö�� ========== */
typedef enum {
    MOTOR_DIRECTION_NONE = 0,
    MOTOR_DIRECTION_FORWARD,
    MOTOR_DIRECTION_REVERSE,
    MOTOR_DIRECTION_STOP,
} motor_direction_t;

/* ========== ��������״̬ö�� ========== */
typedef enum {
    HALL_STATUS_NONE = 0,
    HALL_STATUS_A_ONLY,
    HALL_STATUS_B_ONLY,
    HALL_STATUS_BOTH,
    HALL_STATUS_ERROR
} hall_working_status_t;

/* ========== �����������͸��ָ��? ========== */
typedef struct motor_hall_handle_t* motor_hall_handle_t;

/* ========== ����/���ٽӿ� ========== */

motor_hall_handle_t motor_hall_create(const motor_hall_config_t* config);
void motor_hall_destroy(motor_hall_handle_t handle);

/* ========== ��ʼ��/���½ӿ� ========== */

void motor_hall_system_init(void);
void motor_hall_start(motor_hall_handle_t handle);
void motor_hall_stop(motor_hall_handle_t handle);
void motor_hall_update(motor_hall_handle_t handle);

/* ========== ת����ؽӿ�? ========== */

float motor_hall_get_rpm(motor_hall_handle_t handle);
float motor_hall_get_rpm_raw(motor_hall_handle_t handle);
uint32_t motor_hall_get_pulse_interval_us(motor_hall_handle_t handle);
uint8_t motor_hall_is_running(motor_hall_handle_t handle);
uint8_t motor_hall_is_stalled(motor_hall_handle_t handle);

/* ========== ������ؽӿ�? ========== */

motor_direction_t motor_hall_get_direction(motor_hall_handle_t handle);
uint8_t motor_hall_get_direction_confidence(motor_hall_handle_t handle);
uint8_t motor_hall_is_direction_changed(motor_hall_handle_t handle);

/* ========== ���������ӿ� ========== */

uint32_t motor_hall_get_hall_a_count(motor_hall_handle_t handle);
uint32_t motor_hall_get_hall_b_count(motor_hall_handle_t handle);
uint32_t motor_hall_get_total_pulse_count(motor_hall_handle_t handle);
void motor_hall_reset_counts(motor_hall_handle_t handle);
hall_working_status_t motor_hall_get_status(motor_hall_handle_t handle);
uint8_t motor_hall_get_active_hall_count(motor_hall_handle_t handle);
uint16_t motor_hall_get_pulses_per_rev(motor_hall_handle_t handle);
uint8_t motor_hall_get_pole_pairs(motor_hall_handle_t handle);
void motor_hall_set_pole_pairs(motor_hall_handle_t handle, uint8_t pole_pairs);

#endif /* MOTOR_HALL_H */
