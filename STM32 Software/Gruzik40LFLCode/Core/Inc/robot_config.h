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

#define ROBOT_GAP_BRIDGE_TIMEOUT_MS         4000u
#define ROBOT_GAP_BRIDGE_CENTER_ERROR       2000.0f
#define ROBOT_GAP_BRIDGE_EDGE_GUARD_COUNT   2u

#define ROBOT_LEFT_ENCODER_DIRECTION        -1
#define ROBOT_RIGHT_ENCODER_DIRECTION       1
#define ROBOT_ENCODER_YAW_DIRECTION         1.0f
#define ROBOT_IMU_Z_DIRECTION               1.0f

/* Heading comes from the gyro only (1.0). Encoder yaw is still integrated for telemetry,
   mixing it in only leaks wheel slip into the heading. */
#define ROBOT_ODOMETRY_GYRO_WEIGHT          1.00f
/* Gyro scale correction, 1.0 = datasheet 16.4 LSB/dps. Calibrated from a closed lap
   (GYRO_SCALE reply after mapping) and sent by the app with GyroScale=. */
#define ROBOT_GYRO_SCALE_DEFAULT            1.0f
#define ROBOT_GYRO_STATIONARY_DPS           2.0f
#define ROBOT_ENCODER_STATIONARY_M          0.00002f
#define ROBOT_GYRO_BIAS_LEARN_RATE          0.0005f
/* Odometry tick dt is measured with DWT, these bounds reject garbage */
#define ROBOT_ODOM_DT_MIN_S                 0.0002f
#define ROBOT_ODOM_DT_MAX_S                 0.0200f

#define ROBOT_MAP_RECORD_EVERY_TICKS        40u
#define ROBOT_MAP_TARGET_RADIUS_M           0.075f
#define ROBOT_MAP_ADVANCE_LATERAL_M         0.120f
#define ROBOT_MAP_CLOSE_RADIUS_M            0.030f
#define ROBOT_MAP_CLOSE_MIN_DISTANCE_M      0.450f
#define ROBOT_MAP_CLOSE_MIN_POINTS          8u
/* Lap closure with drift: inside this radius and with the heading back near the start
   heading, the lap closes at the point closest to the start. */
#define ROBOT_MAP_CLOSE_SEARCH_RADIUS_M     0.250f
#define ROBOT_MAP_CLOSE_SEARCH_MIN_M        1.500f
#define ROBOT_MAP_CLOSE_HEADING_RAD         0.52f
#define ROBOT_MAP_CLOSE_LEAVE_M             0.020f
#define ROBOT_MAP_SD_QUEUE_LEN              64u
#define ROBOT_MAP_MAX_POINTS                2000u
#define ROBOT_GYRO_SCALE_MAX_ERR_RAD        0.35f

/* Speed map: track heading sampled every ROBOT_SPEEDMAP_DS_M of driven distance */
#define ROBOT_SPEEDMAP_DS_M                 0.010f
#define ROBOT_SPEEDMAP_MAX_SAMPLES          5000u
#define ROBOT_SPEEDMAP_FILE                 "speedmap.bin"
#define ROBOT_SPEEDMAP_MAX_LANDMARKS        96u
/* Corner landmarks used to re-sync the driven distance with the map */
#define ROBOT_SPEEDMAP_CORNER_ON_KAPPA      3.0f
#define ROBOT_SPEEDMAP_CORNER_OFF_KAPPA     1.8f
#define ROBOT_SPEEDMAP_CORNER_MIN_RAD       0.44f
#define ROBOT_SPEEDMAP_SYNC_WINDOW_M        0.35f
#define ROBOT_SPEEDMAP_SYNC_WINDOW_RATIO    0.08f
#define ROBOT_SPEEDMAP_MAX_REVERSE_PWM      110.0f
#define ROBOT_SPEEDMAP_DUMP_STEP            2u
#define ROBOT_SPEEDMAP_DUMP_PER_LINE        24u

/* Telemetry periods. 9600 baud carries about 960 bytes/s. */
#define ROBOT_TELEMETRY_ODOM_PERIOD_MS      100u
#define ROBOT_TELEMETRY_SENSOR_PERIOD_MS    45u
#define ROBOT_TELEMETRY_LIVE_PERIOD_MS      100u
#define ROBOT_FIRMWARE_VERSION              2u
#define ROBOT_PLAYBACK_LINE_WEIGHT          0.18f
#define ROBOT_PLAYBACK_LINE_CORR_LIMIT_PWM  35.0f
#define ROBOT_PLAYBACK_ODOM_CORR_LIMIT_PWM  115.0f
#define ROBOT_PLAYBACK_CORR_LIMIT_PWM       140.0f
#define ROBOT_PLAYBACK_LOOKAHEAD_BASE_M     0.120f
#define ROBOT_PLAYBACK_LOOKAHEAD_SPEED_M    0.140f
#define ROBOT_PLAYBACK_CROSSTRACK_LIMIT_M   0.180f
#define ROBOT_PLAYBACK_D_FILTER_ALPHA       0.28f
#define ROBOT_PLAYBACK_TURN_SPEED_REDUCTION 0.35f
#define ROBOT_PLAYBACK_LOST_LINE_SPEED_MUL  0.85f
#define ROBOT_MAP_OUTPUT_FILE               "GRUZIK.txt"
#define ROBOT_MAP_INPUT_FILE                "map.txt"

#define ROBOT_BT_DATA_BAUD                  9600u
#define ROBOT_BT_AT_BAUD_PRIMARY            9600u
#define ROBOT_BT_AT_BAUD_SECONDARY          38400u
#define ROBOT_BT_AT_TIMEOUT_MS              220u
#define ROBOT_BT_NAME_FILE                  "btname.txt"
#define ROBOT_BT_NAME_MAX_LEN               20u

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define DEG_TO_RAD                          0.017453292519943295f
#define RAD_TO_DEG                          57.29577951308232f

#endif /* INC_ROBOT_CONFIG_H_ */
