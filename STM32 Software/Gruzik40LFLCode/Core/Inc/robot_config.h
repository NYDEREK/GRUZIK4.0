/*
 * robot_config.h
 *
 * Hardware constants for GRUZIK4.0.
 */

#ifndef INC_ROBOT_CONFIG_H_
#define INC_ROBOT_CONFIG_H_

#define ROBOT_WHEEL_BASE_M                 0.195f
#define ROBOT_SENSOR_OFFSET_M              0.220f
#define ROBOT_WHEEL_DIAMETER_M             0.023f
#define ROBOT_WHEEL_CIRCUMFERENCE_M        0.07225663f

#define ROBOT_MOTOR_GEAR_TEETH             8.0f
#define ROBOT_WHEEL_GEAR_TEETH             40.0f
#define ROBOT_GEAR_RATIO                   (ROBOT_WHEEL_GEAR_TEETH / ROBOT_MOTOR_GEAR_TEETH)

#define ROBOT_ENCODER_CPR_MOTOR            128.0f
#define ROBOT_ENCODER_QUADRATURE_FACTOR    4.0f
#define ROBOT_ENCODER_COUNTS_PER_WHEEL_REV (ROBOT_ENCODER_CPR_MOTOR * ROBOT_ENCODER_QUADRATURE_FACTOR * ROBOT_GEAR_RATIO)
#define ROBOT_ENCODER_COUNTER_CENTER       32768u

#define ROBOT_CONTROL_DT_S                  0.001f
#define ROBOT_PWM_MAX                       286.0f

#define ROBOT_LEFT_ENCODER_DIRECTION        1
#define ROBOT_RIGHT_ENCODER_DIRECTION       1
#define ROBOT_ENCODER_YAW_DIRECTION         1.0f
#define ROBOT_IMU_Z_DIRECTION               1.0f

#define ROBOT_ODOMETRY_GYRO_WEIGHT          0.92f
#define ROBOT_GYRO_STATIONARY_DPS           2.0f
#define ROBOT_ENCODER_STATIONARY_M          0.00002f
#define ROBOT_GYRO_BIAS_LEARN_RATE          0.0005f

#define ROBOT_MAP_RECORD_EVERY_TICKS        40u
#define ROBOT_MAP_TARGET_RADIUS_M           0.050f
#define ROBOT_MAP_OUTPUT_FILE               "GRUZIK.txt"
#define ROBOT_MAP_INPUT_FILE                "map.txt"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define DEG_TO_RAD                          0.017453292519943295f
#define RAD_TO_DEG                          57.29577951308232f

#endif /* INC_ROBOT_CONFIG_H_ */
