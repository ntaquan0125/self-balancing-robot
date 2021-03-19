/*
 * robot.c
 *
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "driverlib/flash.h"

#include "robot.h"

params_t robot_params;

// Private variables
static uint32_t address;

// Private functions
static void read(uint32_t* ptr);
static void encoder_handle(void *param);
static void tilt_handle(void *param);
static void tilt_report_handle(void *param);
static void velocity_handle(void *param);
static void velocity_report_handle(void *param);

void robot_init(void)
{
    led_init();
    uart_init(UART_BAUD);

//    MPU6050_init();
    robot_params.imu.gyro_X = -1;
    robot_params.imu.gyro_Y = 0;
    robot_params.imu.gyro_Z = 1;
    motor_init();
    QEI_init();
    PID_init(&robot_params.pid_tilt, TILT_PERIOD * 0.001, 1000, 1000);
    PID_init(&robot_params.pid_vel, VEL_PERIOD * 0.001, 1000, 1000);

    register_thread(encoder_handle, ENC_PERIOD, NULL, THREAD_REPEAT);
    register_thread(imu_handle, IMU_PERIOD, (void *)&robot_params.imu, THREAD_REPEAT);
    register_thread(tilt_handle, TILT_PERIOD, NULL, THREAD_REPEAT);
    register_thread(tilt_report_handle, TILT_REPORT_PERIOD, NULL, THREAD_REPEAT);
    register_thread(velocity_handle, VEL_PERIOD, NULL, THREAD_REPEAT);
    register_thread(velocity_report_handle, VEL_REPORT_PERIOD, NULL, THREAD_REPEAT);
}

void robot_test(void)
{

}

void save_params(params_t *params)
{
    FlashProtectSet(SAVE_ADDR, FlashReadWrite);
    FlashErase(SAVE_ADDR);
    params->is_saved = true;
    FlashProgram((uint32_t *)params, SAVE_ADDR, sizeof(params_t));
}

static void read(uint32_t* ptr)
{
    *ptr = *((volatile uint32_t*)address);
    address += 4;
}

bool load_params(params_t *params)
{
    address = SAVE_ADDR;
    read((uint32_t *)&params->is_saved);
    if (!params->is_saved)
    {
        return false;
    }
    read((uint32_t *)&params->imu.accel_X);
    read((uint32_t *)&params->imu.accel_Y);
    read((uint32_t *)&params->imu.accel_Z);
    read((uint32_t *)&params->imu.gyro_X);
    read((uint32_t *)&params->imu.gyro_Y);
    read((uint32_t *)&params->imu.gyro_Z);
    read((uint32_t *)&params->imu.roll);
    read((uint32_t *)&params->imu.pitch);
    read((uint32_t *)&params->imu.yaw);
    read((uint32_t *)&params->imu.gyro_bias[0]);
    read((uint32_t *)&params->imu.gyro_bias[1]);
    read((uint32_t *)&params->imu.gyro_bias[2]);
    read((uint32_t *)&params->imu.accel_bias[0]);
    read((uint32_t *)&params->imu.accel_bias[1]);
    read((uint32_t *)&params->imu.accel_bias[2]);
    read((uint32_t *)&params->position[0]);
    read((uint32_t *)&params->position[1]);
    read((uint32_t *)&params->velocity[0]);
    read((uint32_t *)&params->velocity[1]);
    read((uint32_t *)&params->pid_tilt.Kp);
    read((uint32_t *)&params->pid_tilt.Ki);
    read((uint32_t *)&params->pid_tilt.Kd);
    read((uint32_t *)&params->pid_vel.Kp);
    read((uint32_t *)&params->pid_vel.Ki);
    read((uint32_t *)&params->pid_vel.Kd);

    return true;
}

bool message_check(char *msg)
{
    return ((msg[0] == START_FLAG) && (msg[msg[1] - 1] == END_FLAG));
}

bool message_decode(char *msg, params_t* params)
{
    if (!message_check(msg))
    {
        return false;
    }
    switch ((msg_id_t)msg[2])
    {
    char Tx_data[20];
    uint8_t len;
    case IMU_GYRO:
        params->imu.gyro_X = *(float *)&msg[3];
        params->imu.gyro_Y = *(float *)&msg[7];
        params->imu.gyro_Z = *(float *)&msg[11];
        break;

    case IMU_GYRO_BIAS:
        params->imu.gyro_bias[0] = *(float *)&msg[3];
        params->imu.gyro_bias[1] = *(float *)&msg[7];
        params->imu.gyro_bias[2] = *(float *)&msg[11];
        break;

    case IMU_ACCEL:
        params->imu.accel_X = *(float *)&msg[3];
        params->imu.accel_Y = *(float *)&msg[7];
        params->imu.accel_Z = *(float *)&msg[11];
        break;

    case IMU_ACCEL_BIAS:
        params->imu.accel_bias[0] = *(float *)&msg[3];
        params->imu.accel_bias[1] = *(float *)&msg[7];
        params->imu.accel_bias[2] = *(float *)&msg[11];
        break;

    case IMU_RPY:
        params->imu.roll = *(float *)&msg[3];
        params->imu.pitch = *(float *)&msg[7];
        params->imu.yaw = *(float *)&msg[11];
        break;

    case MOTOR_POS:
        params->position[0] = *(int32_t *)&msg[3];
        params->position[1] = *(int32_t *)&msg[7];
        break;

    case MOTOR_VEL:
        params->velocity[0] = *(float *)&msg[3];
        params->velocity[1] = *(float *)&msg[7];
        break;

    case PID_TILT:
        params->pid_tilt.Kp = *(float *)&msg[3];
        params->pid_tilt.Ki = *(float *)&msg[7];
        params->pid_tilt.Kd = *(float *)&msg[11];
        break;

    case PID_VEL:
        params->pid_vel.Kp = *(float *)&msg[3];
        params->pid_vel.Ki = *(float *)&msg[7];
        params->pid_vel.Kd = *(float *)&msg[11];
        break;

    case SAVE_LOAD_PARAMS:
        if (msg[3])
        {
            save_params(params);
        }
        else
        {
            load_params(params);
            len = message_pack(Tx_data, PID_TILT, *params);
            uart_send(Tx_data, len);
            len = message_pack(Tx_data, PID_VEL, *params);
            uart_send(Tx_data, len);
        }
        break;

    default:
        return false;
    }
    return true;
}

uint8_t message_pack(char *msg, msg_id_t msg_id, params_t params)
{
    uint8_t msg_len;
    switch (msg_id)
    {
    case IMU_GYRO:
        msg_len = 12 + 4;
        memset(msg, 0, msg_len);
        memcpy(&msg[3], &params.imu.gyro_X, sizeof(float));
        memcpy(&msg[7], &params.imu.gyro_Y, sizeof(float));
        memcpy(&msg[11], &params.imu.gyro_Z, sizeof(float));
        break;

    case IMU_GYRO_BIAS:
        msg_len = 12 + 4;
        memset(msg, 0, msg_len);
        memcpy(&msg[3], &params.imu.gyro_bias[0], sizeof(float));
        memcpy(&msg[7], &params.imu.gyro_bias[1], sizeof(float));
        memcpy(&msg[11], &params.imu.gyro_bias[2], sizeof(float));
        break;

    case IMU_ACCEL:
        msg_len = 12 + 4;
        memset(msg, 0, msg_len);
        memcpy(&msg[3], &params.imu.accel_X, sizeof(float));
        memcpy(&msg[7], &params.imu.accel_Y, sizeof(float));
        memcpy(&msg[11], &params.imu.accel_Z, sizeof(float));
        break;

    case IMU_ACCEL_BIAS:
        msg_len = 12 + 4;
        memset(msg, 0, msg_len);
        memcpy(&msg[3], &params.imu.accel_bias[0], sizeof(float));
        memcpy(&msg[7], &params.imu.accel_bias[1], sizeof(float));
        memcpy(&msg[11], &params.imu.accel_bias[2], sizeof(float));
        break;

    case IMU_RPY:
        msg_len = 12 + 4;
        memset(msg, 0, msg_len);
        memcpy(&msg[3], &params.imu.roll, sizeof(float));
        memcpy(&msg[7], &params.imu.pitch, sizeof(float));
        memcpy(&msg[11], &params.imu.yaw, sizeof(float));
        break;

    case MOTOR_POS:
        msg_len = 8 + 4;
        memset(msg, 0, msg_len);
        memcpy(&msg[3], &params.position[0], sizeof(int64_t));
        memcpy(&msg[7], &params.position[1], sizeof(int64_t));
        break;

    case MOTOR_VEL:
        msg_len = 8 + 4;
        memset(msg, 0, msg_len);
        memcpy(&msg[3], &params.velocity[0], sizeof(float));
        memcpy(&msg[7], &params.velocity[1], sizeof(float));
        break;

    case PID_TILT:
        msg_len = 12 + 4;
        memset(msg, 0, msg_len);
        memcpy(&msg[3], &params.pid_tilt.Kp, sizeof(float));
        memcpy(&msg[7], &params.pid_tilt.Ki, sizeof(float));
        memcpy(&msg[11], &params.pid_tilt.Kd, sizeof(float));
        break;

    case PID_VEL:
        msg_len = 12 + 4;
        memset(msg, 0, msg_len);
        memcpy(&msg[3], &params.pid_vel.Kp, sizeof(float));
        memcpy(&msg[7], &params.pid_vel.Ki, sizeof(float));
        memcpy(&msg[11], &params.pid_vel.Kd, sizeof(float));
        break;

    default:
        return 0;
    }

    msg[0] = START_FLAG;
    msg[1] = msg_len;
    msg[2] = msg_id;
    msg[msg_len - 1] = END_FLAG;

    return msg_len;
}

static void encoder_handle(void *param)
{
    robot_params.position[0] = QEI_get_count(MOTOR_L);
    robot_params.position[1] = QEI_get_count(MOTOR_R);
}

static void tilt_handle(void *param)
{
    if (fabs(robot_params.imu.pitch) > 80)
    {
        PID_reset(&robot_params.pid_tilt);
    }
    float speed = PID_update(&robot_params.pid_tilt, 0, robot_params.imu.pitch);
    motor_set_speed(MOTOR_L, speed);
    motor_set_speed(MOTOR_R, speed);
}

static void tilt_report_handle(void *param)
{
    char Tx_data[20];
    uint16_t len = message_pack(Tx_data, PID_TILT, robot_params);
    uart_send(Tx_data, len);
}

static void velocity_handle(void *param)
{

}

static void velocity_report_handle(void *param)
{
    char Tx_data[20];
    uint16_t len = message_pack(Tx_data, PID_VEL, robot_params);
    uart_send(Tx_data, len);
}
