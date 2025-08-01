#ifndef MOTOR_DRIVER_MOTOR_DRIVER_H
#define MOTOR_DRIVER_MOTOR_DRIVER_H

#include "motor_driver_config.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    motor_driver_state_t state;
    motor_driver_config_t config;
    motor_driver_interface_t interface;
} motor_driver_t;

motor_driver_err_t motor_driver_initialize(
    motor_driver_t* driver,
    motor_driver_config_t const* config,
    motor_driver_interface_t const* interface);
motor_driver_err_t motor_driver_deinitialize(motor_driver_t* driver);

motor_driver_err_t motor_driver_set_position(motor_driver_t* driver,
                                             float32_t position,
                                             float32_t delta_time);

#ifdef __cplusplus
}
#endif

#endif // MOTOR_DRIVER_MOTOR_DRIVER_H
