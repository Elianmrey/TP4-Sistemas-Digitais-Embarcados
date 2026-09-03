// Shared constants for TP4 modular ARM64 project.

.equ AT_FDCWD,                  -100
.equ O_RDWR,                    2
.equ O_WRONLY,                  1
.equ O_NONBLOCK,                2048
.equ F_GETFL,                   3
.equ F_SETFL,                   4

.equ SYS_write,                 64
.equ SYS_close,                 57
.equ SYS_exit,                  93
.equ SYS_openat,                56
.equ SYS_ioctl,                 29
.equ SYS_nanosleep,             101
.equ SYS_read,                  63
.equ SYS_fcntl,                 25
.equ SYS_clock_gettime,         113

.equ CLOCK_MONOTONIC,            1

.equ SPI_IOC_WR_MODE,           0x40016b01
.equ SPI_IOC_WR_BITS_PER_WORD,  0x40016b03
.equ SPI_IOC_WR_MAX_SPEED_HZ,   0x40046b04
.equ SPI_IOC_MESSAGE_1,         0x40206b00

.equ SPI_SPEED_HZ,              10000
.equ FRAME_SIZE,                14

.equ GPIO17,                    17
.equ GPIO27,                    27
.equ GPIO5,                      5
.equ GPIO6,                      6
.equ GPIO18,                    18

.equ GPIO_V2_GET_LINE_IOCTL,     0xC250B407
.equ GPIO_V2_LINE_GET_VALUES_IOCTL, 0xC010B40E
.equ GPIO_V2_LINE_SET_VALUES_IOCTL, 0xC010B40F
.equ GPIO_V2_LINE_FLAG_INPUT,    0x4
.equ GPIO_V2_LINE_FLAG_OUTPUT,   0x8
.equ GPIO_V2_LINE_FLAG_EDGE_RISING, 0x10
.equ GPIO_V2_LINE_FLAG_EDGE_FALLING, 0x20
.equ GPIO_V2_LINE_FLAG_BIAS_PULL_UP, 0x100
.equ GPIO_V2_LINE_ATTR_ID_OUTPUT_VALUES, 2
.equ GPIO_V2_REQ_SIZE,            592
.equ GPIO_V2_VALUES_SIZE,         16
.equ GPIO_V2_EVENT_SIZE,          48
.equ GPIO_V2_EVENT_BUF_SIZE,      64

.equ PWM_PERIOD_NS,              1000000
.equ MOTOR_STAGE_MS,             10000
.equ MOTOR_STAGE_COUNT,          4

// 823.13 PPR = 82313 / 100.
.equ ENCODER_PPR_NUM,            82313
.equ ENCODER_PPR_DEN,            100

.equ MOTOR_NOMINAL_RPM,          100
.equ MOTOR_NOMINAL_RPM_X100,    10000

// Safety limits used by the simulated states.
.equ SAFE_TEMP_MIN_X10,        0
.equ SAFE_TEMP_MAX_X10,        1200
.equ SAFE_PRESSURE_MIN_PA,     70000
.equ SAFE_PRESSURE_MAX_PA,     120000
.equ SAFE_POT_MAX_PERCENT,     100

// Simulated automotive sensor limits. These are didactic test limits.
.equ CKP_MIN_RPM,              100
.equ CKP_MAX_RPM,              6000
.equ MAF_MIN_GPS,              10
.equ MAF_MAX_GPS,              300

.equ STATE_1,                    1
.equ STATE_2,                    2
.equ STATE_3,                    3
.equ STATE_4,                    4

.equ CMD_STATUS,                 0
.equ CMD_GPIO,                   1
.equ CMD_HELP,                   2
.equ CMD_QUIT,                   3
.equ CMD_INVALID,                4
.equ CMD_MOTOR,                  5
.equ CMD_STATE1,                 6
.equ CMD_STATE2,                 7
.equ CMD_STATE3,                 8
.equ CMD_STATE4,                 9
