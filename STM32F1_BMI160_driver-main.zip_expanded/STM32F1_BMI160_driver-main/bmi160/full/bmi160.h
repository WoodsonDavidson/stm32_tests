
#ifndef BMI160_H_
#define BMI160_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "bmi160_defs.h"
#ifdef __KERNEL__
#include <bmi160_math.h>
#else
#include <math.h>
#include <string.h>
#include <stdlib.h>
#endif

int8_t bmi160_init(struct bmi160_dev *dev);

int8_t bmi160_get_regs(uint16_t reg_addr, uint8_t *data, uint16_t len);
int8_t bmi160_set_regs(uint16_t reg_addr, uint8_t *data, uint16_t len);
int8_t bmi160_soft_reset(void);
int8_t bmi160_set_sens_conf(void);
int8_t bmi160_get_sens_conf(void);
int8_t bmi160_set_power_mode(void);
int8_t bmi160_get_power_mode(void);
int8_t bmi160_get_sensor_data(uint8_t select_sensor, struct bmi160_sensor_data *accel, struct bmi160_sensor_data *gyro);
											
int8_t bmi160_get_fifo_data(void);
int8_t bmi160_set_fifo_flush(void);
int8_t bmi160_set_fifo_config(uint8_t config, uint8_t enable);
int8_t bmi160_set_fifo_down(uint8_t fifo_down);
int8_t bmi160_set_fifo_wm(uint8_t fifo_wm);
															
int8_t bmi160_extract_accel(struct bmi160_sensor_data *accel_data, uint8_t *accel_length);
int8_t bmi160_extract_gyro(struct bmi160_sensor_data *gyro_data, uint8_t *gyro_length);

int8_t bmi160_start_foc(const struct bmi160_foc_conf *foc_conf, struct bmi160_offsets *offset);
int8_t bmi160_get_offsets(struct bmi160_offsets *offset);
int8_t bmi160_set_offsets(const struct bmi160_foc_conf *foc_conf, const struct bmi160_offsets *offset);
int8_t bmi160_update_nvm(void);
			
#ifdef BMI_STEP_COUNTER
int8_t bmi160_set_step_counter(uint8_t step_cnt_enable);
int8_t bmi160_read_step_counter(uint16_t *step_val);
#endif
														
#ifdef BMI_INT_TYPE
int8_t bmi160_set_int_config(struct bmi160_int_settg *int_config);
int8_t bmi160_get_int_status(enum bmi160_int_status_sel int_status_sel, union bmi160_int_status *int_status);
#endif									 
#ifdef BMI_AUX_TYPE
int8_t bmi160_aux_read(uint8_t reg_addr, uint8_t *aux_data, uint16_t len);
int8_t bmi160_aux_write(uint8_t reg_addr, uint8_t *aux_data, uint16_t len);
int8_t bmi160_aux_init(void);
int8_t bmi160_set_aux_auto_mode(uint8_t *data_addr);
int8_t bmi160_config_aux_mode(void);
int8_t bmi160_read_aux_data_auto_mode(uint8_t *aux_data);
int8_t bmi160_extract_aux(struct bmi160_aux_data *aux_data, uint8_t *aux_len);
#endif
#ifdef BMI_TEST_TYPE
int8_t bmi160_perform_self_test(uint8_t select_sensor);
#endif
														 
#ifdef __cplusplus
}
#endif
#endif /* BMI160_H_ */
														 
