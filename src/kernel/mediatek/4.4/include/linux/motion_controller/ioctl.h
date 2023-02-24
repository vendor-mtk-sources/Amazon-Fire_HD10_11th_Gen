#ifndef __MOTION_CONTROL_IOCTL_H
#define __MOTION_CONTROL_IOCTL_H

typedef enum {
    CMD_READ  = 0x01,
    CMD_WRITE,
} motion_protocol_cmd_type;

#define GET_FW_VERSION  0x0402
#define GET_VEL         0x0C00
#define GET_HARD_STOPS  0x0C01
#define GET_OUTPUT_LIM  0x0C03
#define GET_ACCEL       0x0C04
#define GET_TRAJ        0x0C05
#define GET_TRAJ_PV     0x0C06
#define GET_POS_VEL     0x0C07
#define GET_VEL_LIM     0x0C08
#define GET_ACCEL_LIM   0x0C09

#define TIME_SYNC 0x0001
#define GET_STATUS 0x0002
#define TEST_READ 0xF0F0
#define TEST_WRITE 0x00FF

#define MAX_TRAJ_LENGTH 20

struct __attribute__((__packed__)) abc123_position {
	struct timeval tv;
	int32_t position;
};

struct __attribute__((__packed__)) abc123_velocity {
	struct timeval tv;
	int32_t velocity;
	int32_t omega;
	int16_t divider;
};

struct __attribute__((__packed__)) abc123_accel {
	struct timeval tv;
	int32_t accel;
	int32_t deccel;
};

struct __attribute__((__packed__)) abc123_traj_values {
	int32_t traj[MAX_TRAJ_LENGTH];
	uint8_t length;
};

struct __attribute__((__packed__)) abc123_traj_data {
   struct timeval tv;
   struct abc123_traj_values traj;
};

struct __attribute__((__packed__)) abc123_fw_version {
   char version[16];
};

#define MOTOR_PASSTHROUGH_MAX_LENGTH            256

struct __attribute__((__packed__)) motor_passthrough_packet
{
    uint16_t length;
    uint16_t command;
    uint8_t  data[MOTOR_PASSTHROUGH_MAX_LENGTH];
};

struct __attribute__((__packed__)) sensor_readwrite_data
{
    uint32_t buff_size;
    char *buff;
#ifndef __KERNEL__
    char *pad;
#endif
};

#define MOTION_IOCTL_READ_DEVICE		 _IO('r', 0x28)
#define MOTION_IOCTL_WRITE_DEVICE		 _IO('r', 0x29)

#endif /* __MOTION_CONTROL_IOCTL_H */
