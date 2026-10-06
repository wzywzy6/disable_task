#include "cmsis_os.h"
#include "io/bmi088/bmi088.hpp"
#include "io/can/can.hpp"
#include "io/dbus/dbus.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "tools/mahony/mahony.hpp"

sp::CAN can1(&hcan1);
sp::DBus remote(&huart3);
extern sp::DBus DBus;
extern sp::BMI088 bmi088;
sp::RM_Motor motor6020(1, sp::RM_Motors::GM6020);  // 一个电机ID为1, 电流控制模式的6020

extern "C" void can_task()
{
  typedef enum
  {
    MOTOR_DISABLED = 0,
    MOTOR_RELATED = 1,
    MOTOR_RESET = 2,
  } MotorState_t;
  MotorState_t state = MOTOR_DISABLED;

  can1.config();
  can1.start();

  while (true) {
    osDelay(10);

    switch (remote.sw_r) {
      case sp::DBusSwitchMode::UP:
        state = MOTOR_RESET;
        break;
      case sp::DBusSwitchMode::MID:
        state = MOTOR_RELATED;
        break;
      case sp::DBusSwitchMode::DOWN:
        state = MOTOR_DISABLED;
        break;
      default:
        break;
    }
    switch (state) {
      case MOTOR_DISABLED:
        motor6020.cmd(0.0f);
        break;
      case MOTOR_RELATED:
        osDelay(10);
        motor6020.cmd(
          bmi088.gyro[2] * 1.0f);  // 以陀螺仪z轴角速度为目标电流, 乘以10是为了放大电流值
        break;
      case MOTOR_RESET:
        motor6020.cmd(0.1f);
        break;
      default:
        break;
    }
    motor6020.write(can1.tx_data);
    can1.send(motor6020.tx_id);
  }
}

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef * hcan)
{
  auto stamp_ms = osKernelSysTick();

  while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0) {
    if (hcan == &hcan1) {
      can1.recv();

      if (can1.rx_id == motor6020.rx_id) motor6020.read(can1.rx_data, stamp_ms);
    }
  }
}