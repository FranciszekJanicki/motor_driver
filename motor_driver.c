#include "motor_driver.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static motor_driver_err_t motor_driver_motor_initialize(
    motor_driver_t const* driver)
{
    return (driver->interface.motor_initialize != NULL)
               ? driver->interface.motor_initialize(
                     driver->interface.motor_user)
               : MOTOR_DRIVER_ERR_NULL;
}

static motor_driver_err_t motor_driver_motor_deinitialize(
    motor_driver_t const* driver)
{
    return (driver->interface.motor_deinitialize != NULL)
               ? driver->interface.motor_deinitialize(
                     driver->interface.motor_user)
               : MOTOR_DRIVER_ERR_NULL;
}

static motor_driver_err_t motor_driver_motor_set_speed(
    motor_driver_t const* driver,
    float32_t speed)
{
    return (driver->interface.motor_set_speed != NULL)
               ? driver->interface.motor_set_speed(driver->interface.motor_user,
                                                   speed)
               : MOTOR_DRIVER_ERR_NULL;
}

static motor_driver_err_t motor_driver_encoder_initialize(
    motor_driver_t const* driver)
{
    return (driver->interface.encoder_initialize != NULL)
               ? driver->interface.encoder_initialize(
                     driver->interface.encoder_user)
               : MOTOR_DRIVER_ERR_NULL;
}

static motor_driver_err_t motor_driver_encoder_deinitialize(
    motor_driver_t const* driver)
{
    return (driver->interface.encoder_deinitialize != NULL)
               ? driver->interface.encoder_deinitialize(
                     driver->interface.encoder_user)
               : MOTOR_DRIVER_ERR_NULL;
}

static motor_driver_err_t motor_driver_encoder_get_position(
    motor_driver_t const* driver,
    float32_t* position)
{
    return (driver->interface.encoder_get_position != NULL)
               ? driver->interface.encoder_get_position(
                     driver->interface.encoder_user,
                     position)
               : MOTOR_DRIVER_ERR_NULL;
}

static motor_driver_err_t motor_driver_regulator_initialize(
    motor_driver_t const* driver)
{
    return (driver->interface.regulator_initialize != NULL)
               ? driver->interface.regulator_initialize(
                     driver->interface.regulator_user)
               : MOTOR_DRIVER_ERR_NULL;
}

static motor_driver_err_t motor_driver_regulator_deinitialize(
    motor_driver_t const* driver)
{
    return (driver->interface.regulator_deinitialize != NULL)
               ? driver->interface.regulator_deinitialize(
                     driver->interface.regulator_user)
               : MOTOR_DRIVER_ERR_NULL;
}

static motor_driver_err_t motor_driver_regulator_get_control(
    motor_driver_t const* driver,
    float32_t error,
    float32_t* control,
    float32_t delta_time)
{
    return (driver->interface.regulator_get_control != NULL)
               ? driver->interface.regulator_get_control(
                     driver->interface.regulator_user,
                     error,
                     control,
                     delta_time)
               : MOTOR_DRIVER_ERR_NULL;
}

static motor_driver_err_t motor_driver_fault_initialize(
    motor_driver_t const* driver)
{
    return (driver->interface.fault_initialize != NULL)
               ? driver->interface.fault_initialize(
                     driver->interface.fault_user)
               : MOTOR_DRIVER_ERR_NULL;
}

static motor_driver_err_t motor_driver_fault_deinitialize(
    motor_driver_t const* driver)
{
    return (driver->interface.fault_deinitialize != NULL)
               ? driver->interface.fault_deinitialize(
                     driver->interface.fault_user)
               : MOTOR_DRIVER_ERR_NULL;
}

static motor_driver_err_t motor_driver_fault_get_current(
    motor_driver_t const* driver,
    float32_t* current)
{
    return (driver->interface.fault_get_current != NULL)
               ? driver->interface.fault_get_current(
                     driver->interface.fault_user,
                     current)
               : MOTOR_DRIVER_ERR_NULL;
}

static inline float32_t motor_driver_clamp_position(
    motor_driver_t const* driver,
    float32_t position)
{
    if (position < driver->config.min_position) {
        return driver->config.min_position;
    } else if (position > driver->config.max_position) {
        return driver->config.max_position;
    }

    return position;
}

static inline float32_t motor_driver_clamp_speed(motor_driver_t const* driver,
                                                 float32_t speed)
{
    if (speed != 0.0F) {
        if (fabsf(speed) < driver->config.min_speed) {
            return copysignf(driver->config.min_speed, speed);
        } else if (fabsf(speed) > driver->config.max_speed) {
            return copysignf(driver->config.max_speed, speed);
        }
    }

    return speed;
}

static inline float32_t motor_driver_clamp_acceleration(
    motor_driver_t const* driver,
    float32_t acceleration)
{
    if (acceleration != 0.0F) {
        if (fabsf(acceleration) < driver->config.min_acceleration) {
            return copysignf(driver->config.min_acceleration, acceleration);
        } else if (fabsf(acceleration) > driver->config.max_acceleration) {
            return copysignf(driver->config.max_acceleration, acceleration);
        }
    }

    return acceleration;
}

static inline float32_t motor_driver_wrap_position_error(
    motor_driver_t const* driver,
    float32_t error_position,
    float32_t measured_position)
{
    if (measured_position + error_position < driver->config.min_position) {
        return driver->config.min_position - measured_position;
    } else if (measured_position + error_position >
               driver->config.max_position) {
        return driver->config.max_position - measured_position;
    }

    return error_position;
}

static inline bool motor_driver_has_fault(motor_driver_t const* driver,
                                          float32_t current)
{
    return current > driver->config.max_current;
}

motor_driver_err_t motor_driver_initialize(
    motor_driver_t* driver,
    motor_driver_config_t const* config,
    motor_driver_interface_t const* interface)
{
    if (driver == NULL || config == NULL || interface == NULL) {
        return MOTOR_DRIVER_ERR_NULL;
    }

    memset(driver, 0, sizeof(*driver));
    memcpy(&driver->config, config, sizeof(*config));
    memcpy(&driver->interface, interface, sizeof(*interface));

    motor_driver_err_t err = motor_driver_motor_initialize(driver);
    if (err != MOTOR_DRIVER_ERR_OK) {
        return err;
    }

    err = motor_driver_encoder_initialize(driver);
    if (err != MOTOR_DRIVER_ERR_OK) {
        return err;
    }

    err = motor_driver_regulator_initialize(driver);
    if (err != MOTOR_DRIVER_ERR_OK) {
        return err;
    }

    err = motor_driver_fault_initialize(driver);
    if (err != MOTOR_DRIVER_ERR_OK) {
        return err;
    }

    return MOTOR_DRIVER_ERR_OK;
}

motor_driver_err_t motor_driver_deinitialize(motor_driver_t* driver)
{
    if (driver == NULL) {
        return MOTOR_DRIVER_ERR_NULL;
    }

    motor_driver_err_t err = motor_driver_motor_deinitialize(driver);
    if (err != MOTOR_DRIVER_ERR_OK) {
        return err;
    }

    err = motor_driver_encoder_deinitialize(driver);
    if (err != MOTOR_DRIVER_ERR_OK) {
        return err;
    }

    err = motor_driver_regulator_deinitialize(driver);
    if (err != MOTOR_DRIVER_ERR_OK) {
        return err;
    }

    err = motor_driver_fault_deinitialize(driver);
    if (err != MOTOR_DRIVER_ERR_OK) {
        return err;
    }

    memset(driver, 0, sizeof(*driver));

    return MOTOR_DRIVER_ERR_OK;
}

motor_driver_err_t motor_driver_set_position(motor_driver_t* driver,
                                             float32_t reference_position,
                                             float32_t delta_time)
{
    if (driver == NULL) {
        return MOTOR_DRIVER_ERR_NULL;
    }

    if (delta_time <= 0.0F) {
        return MOTOR_DRIVER_ERR_FAIL;
    }

    float32_t fault_current;
    motor_driver_err_t err =
        motor_driver_fault_get_current(driver, &fault_current);
    if (err != MOTOR_DRIVER_ERR_OK) {
        return err;
    }

    if (motor_driver_has_fault(driver, fault_current)) {
        return MOTOR_DRIVER_ERR_FAULT;
    }

    float32_t measure_position;
    err = motor_driver_encoder_get_position(driver, &measure_position);
    if (err != MOTOR_DRIVER_ERR_OK) {
        return err;
    }

    reference_position =
        motor_driver_clamp_position(driver, reference_position);

    float32_t error_position = reference_position - measure_position;
    // if (driver->config.should_wrap_position) {
    //     error_position = motor_driver_wrap_position_error(driver,
    //                                                       error_position,
    //                                                       measure_position);
    // }

    float32_t control_speed;
    err = motor_driver_regulator_get_control(driver,
                                             error_position,
                                             &control_speed,
                                             delta_time);
    if (err != MOTOR_DRIVER_ERR_OK) {
        return err;
    }

    // float32_t control_acceleration =
    //     (control_speed - driver->state.control_speed) / delta_time;
    // control_acceleration =
    //     motor_driver_clamp_acceleration(driver, control_acceleration);

    // control_speed =
    //     driver->state.control_speed + control_acceleration * delta_time;
    control_speed = motor_driver_clamp_speed(driver, control_speed);

    err = motor_driver_motor_set_speed(driver, control_speed);
    if (err != MOTOR_DRIVER_ERR_OK) {
        return err;
    }

    driver->state.measure_position = measure_position;
    driver->state.control_speed = control_speed;
    driver->state.fault_current = fault_current;

    return MOTOR_DRIVER_ERR_OK;
}

motor_driver_err_t motor_driver_set_speed(motor_driver_t* driver,
                                          float32_t reference_speed,
                                          float32_t delta_time)
{
    if (driver == NULL) {
        return MOTOR_DRIVER_ERR_NULL;
    }

    if (delta_time <= 0.0F) {
        return MOTOR_DRIVER_ERR_FAIL;
    }

    float32_t fault_current;
    motor_driver_err_t err =
        motor_driver_fault_get_current(driver, &fault_current);
    if (err != MOTOR_DRIVER_ERR_OK) {
        return err;
    }

    if (motor_driver_has_fault(driver, fault_current)) {
        return MOTOR_DRIVER_ERR_FAULT;
    }

    reference_speed = motor_driver_clamp_speed(driver, reference_speed);

    err = motor_driver_motor_set_speed(driver, reference_speed);
    if (err != MOTOR_DRIVER_ERR_OK) {
        return err;
    }

    driver->state.control_speed = reference_speed;
    driver->state.fault_current = fault_current;

    return MOTOR_DRIVER_ERR_OK;
}

motor_driver_err_t motor_driver_set_acceleration(
    motor_driver_t* driver,
    float32_t reference_acceleration,
    float32_t delta_time)
{
    if (driver == NULL) {
        return MOTOR_DRIVER_ERR_NULL;
    }

    if (delta_time <= 0.0F) {
        return MOTOR_DRIVER_ERR_FAIL;
    }

    float32_t fault_current;
    motor_driver_err_t err =
        motor_driver_fault_get_current(driver, &fault_current);
    if (err != MOTOR_DRIVER_ERR_OK) {
        return err;
    }

    if (motor_driver_has_fault(driver, fault_current)) {
        return MOTOR_DRIVER_ERR_FAULT;
    }

    reference_acceleration =
        motor_driver_clamp_acceleration(driver, reference_acceleration);

    float32_t control_speed =
        driver->state.control_speed + reference_acceleration * delta_time;
    control_speed = motor_driver_clamp_speed(driver, control_speed);

    err = motor_driver_motor_set_speed(driver, control_speed);
    if (err != MOTOR_DRIVER_ERR_OK) {
        return err;
    }

    driver->state.control_speed = control_speed;
    driver->state.fault_current = fault_current;

    return MOTOR_DRIVER_ERR_OK;
}

motor_driver_err_t motor_driver_get_state(motor_driver_t const* driver,
                                          motor_driver_state_t* state)
{
    if (driver == NULL || state == NULL) {
        return MOTOR_DRIVER_ERR_NULL;
    }

    memcpy(state, &driver->state, sizeof(*state));

    return MOTOR_DRIVER_ERR_OK;
}