/*
 * robot.c
 *
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "driverlib/flash.h"

#include "robot.h"

params_t robot_params;

// Private variables
static uint32_t address;
static float tilt_offset = 0;

// Private functions
static void read(uint32_t* ptr);
static void write(uint32_t *ptr);
static void led_handle(void *param);
static void encoder_handle(void *param);
static void tilt_handle(void *param);
static void tilt_report_handle(void *param);
static void velocity_handle(void *param);
static void velocity_report_handle(void *param);

void robot_init(void)
{
    led_init();
    uart_init(UART_BAUD);

    thread_init();
    MPU6050_init();
    MPU6050_calib(&robot_params.imu);
    QEI_init();
    PID_init(&robot_params.pid_tilt, TILT_PERIOD * 0.001, 800, 500);
    PID_init(&robot_params.pid_vel, VEL_PERIOD * 0.001, 800, 500);
    motor_init();

    PID_set_params(&robot_params.pid_tilt, 45.8, 70, 1.5);
    PID_set_params(&robot_params.pid_vel, 0, 0, 0);
    register_thread(led_handle, 1000, NULL, THREAD_REPEAT);
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

static void read(uint32_t* ptr)
{
    *ptr = *((volatile uint32_t*)address);
    address += 4;
}

static void write(uint32_t *ptr)
{
    FlashProtectSet(address, FlashReadWrite);
    FlashProgram(ptr, address, sizeof(ptr));
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
    read((uint32_t *)&params->imu.gyro_bias[0]);
    read((uint32_t *)&params->imu.gyro_bias[1]);
    read((uint32_t *)&params->imu.gyro_bias[2]);
    read((uint32_t *)&params->imu.accel_bias[0]);
    read((uint32_t *)&params->imu.accel_bias[1]);
    read((uint32_t *)&params->imu.accel_bias[2]);
    read((uint32_t *)&params->pid_tilt.Kp);
    read((uint32_t *)&params->pid_tilt.Ki);
    read((uint32_t *)&params->pid_tilt.Kd);
    read((uint32_t *)&params->pid_vel.Kp);
    read((uint32_t *)&params->pid_vel.Ki);
    read((uint32_t *)&params->pid_vel.Kd);

    return true;
}

void save_params(params_t *params)
{
    FlashErase(address);
    address = SAVE_ADDR;
    params->is_saved = true;
    write((uint32_t *)params->is_saved);
    write((uint32_t *)&params->imu.gyro_bias[0]);
    write((uint32_t *)&params->imu.gyro_bias[1]);
    write((uint32_t *)&params->imu.gyro_bias[2]);
    write((uint32_t *)&params->imu.accel_bias[0]);
    write((uint32_t *)&params->imu.accel_bias[1]);
    write((uint32_t *)&params->imu.accel_bias[2]);
    write((uint32_t *)&params->pid_tilt.Kp);
    write((uint32_t *)&params->pid_tilt.Ki);
    write((uint32_t *)&params->pid_tilt.Kd);
    write((uint32_t *)&params->pid_vel.Kp);
    write((uint32_t *)&params->pid_vel.Ki);
    write((uint32_t *)&params->pid_vel.Kd);
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
        len = message_pack(Tx_data, PID_TILT, *params);
        uart_send(Tx_data, len);
        len = message_pack(Tx_data, PID_VEL, *params);
        uart_send(Tx_data, len);
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

static void led_handle(void *param)
{
    if (!robot_params.is_saved)
    {
        toggle_led(LED_RED);
    }
}

static void encoder_handle(void *param)
{
    robot_params.position[0] = QEI_get_count(MOTOR_L);
    robot_params.position[1] = QEI_get_count(MOTOR_R);
}

static void tilt_handle(void *param)
{
    if (fabs(robot_params.imu.pitch) > 60)
    {
        PID_reset(&robot_params.pid_tilt);
    }
    float speed = PID_update(&robot_params.pid_tilt, tilt_offset, robot_params.imu.pitch);
    motor_set_speed(MOTOR_L, (int)(speed));
    motor_set_speed(MOTOR_R, (int)(speed));
}

static void tilt_report_handle(void *param)
{
    char Tx_data[20];
    uint16_t len;
    len = message_pack(Tx_data, IMU_GYRO, robot_params);
    uart_send(Tx_data, len);
    len = message_pack(Tx_data, IMU_RPY, robot_params);
    uart_send(Tx_data, len);
}

static void velocity_handle(void *param)
{
    robot_params.velocity[0] = QEI_get_velocity(MOTOR_L);
    robot_params.velocity[1] = QEI_get_velocity(MOTOR_R);
    float speed = (robot_params.velocity[0] + robot_params.velocity[1]) / 2;
    tilt_offset = PID_update(&robot_params.pid_vel, 0, speed);
}

static void velocity_report_handle(void *param)
{
    char Tx_data[20];
    uint16_t len;
    len = message_pack(Tx_data, MOTOR_POS, robot_params);
    uart_send(Tx_data, len);
    len = message_pack(Tx_data, MOTOR_VEL, robot_params);
    uart_send(Tx_data, len);
}
