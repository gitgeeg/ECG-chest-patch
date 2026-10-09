#ifndef BATTERY_ADC_H_
#define BATTERY_ADC_H_

#include <stdint.h>
#include <stdbool.h>

/*
 * 电池电压采集（ADC1）
 *
 * 分压网络：13k 上拉 → 电池正极，15k 下拉 → 地，ADC 接中间节点。
 *   分压比 = 15k / (13k + 15k) = 15/28
 *   电池电压 Vbat = Vadc × 28 / 15
 */

#define BATTERY_ADC_GPIO        2                 // G2 = ADC1_CH1
#define BATTERY_DIV_NUMERATOR   28
#define BATTERY_DIV_DENOMINATOR 15

/* 初始化 ADC oneshot + 曲线拟合校准，返回是否成功 */
bool battery_adc_init(void);
/* Call only after the battery worker has parked. */
void battery_adc_deinit(void);

/* 多次采样平均后换算为电池电压（mV），返回是否成功 */
bool battery_adc_read_mv(uint32_t *battery_mv);

#endif /* BATTERY_ADC_H_ */
