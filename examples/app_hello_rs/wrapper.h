#pragma once

#include <stdint.h>
#include <stdbool.h>

// Axis3f and related IMU axis vector types
#include "../../src/hal/interface/imu_types.h"


/* Data structure used by the stabilizer subsystem.
 * All have a timestamp to be set when the data is calculated.
 */

// stabilizerStep_t represents the number of times the stabilizer loop has run (at 1000 Hz)
typedef uint32_t stabilizerStep_t;

/** Attitude in euler angle form */
typedef struct attitude_s {
  uint32_t timestamp;  // Timestamp when the data was computed

  float roll;
  float pitch;
  float yaw;
} attitude_t;

/* vector */
#define vec3d_size 3
typedef float vec3d[vec3d_size];
typedef float mat3d[vec3d_size][vec3d_size];

/* x,y,z vector */
struct vec3_s {
  uint32_t timestamp; // Timestamp when the data was computed

  float x;
  float y;
  float z;
};

typedef struct vec3_s vector_t;
typedef struct vec3_s point_t;
typedef struct vec3_s velocity_t;
typedef struct vec3_s acc_t;
typedef struct vec3_s jerk_t;

/* Orientation as a quaternion */
typedef struct quaternion_s {
  union {
    struct {
      float q0;
      float q1;
      float q2;
      float q3;
    };
    struct {
      float x;
      float y;
      float z;
      float w;
    };
  };
} quaternion_t;

typedef enum {
  MeasurementSourceLocationService  = 0,
  MeasurementSourceLighthouse       = 1,
} measurementSource_t;

typedef struct tdoaMeasurement_s {
  union {
    point_t anchorPositions[2];
    struct {
      point_t anchorPositionA;
      point_t anchorPositionB;
    };
  };
  union {
    uint8_t anchorIds[2];
    struct {
      uint8_t anchorIdA;
      uint8_t anchorIdB;
    };
  };

  float distanceDiff;
  float stdDev;
} tdoaMeasurement_t;

typedef struct baro_s {
  float pressure;           // mbar
  float temperature;        // degree Celcius
  float asl;                // m (ASL = altitude above sea level)
} baro_t;

typedef struct positionMeasurement_s {
  union {
    struct {
      float x;
      float y;
      float z;
    };
    float pos[3];
  };
  float stdDev;
  measurementSource_t source;
} positionMeasurement_t;

typedef struct poseMeasurement_s {
  union {
    struct {
      float x;
      float y;
      float z;
    };
    float pos[3];
  };
  quaternion_t quat;
  float stdDevPos;
  float stdDevQuat;
} poseMeasurement_t;

typedef struct distanceMeasurement_s {
  union {
    struct {
      float x;
      float y;
      float z;
    };
    float pos[3];
  };
  uint8_t anchorId;
  float distance;
  float stdDev;
} distanceMeasurement_t;

typedef struct zDistance_s {
  uint32_t timestamp;
  float distance;           // m
} zDistance_t;

typedef struct sensorData_s {
  Axis3f acc;               // Gs
  Axis3f gyro;              // deg/s
  Axis3f mag;               // gauss
  baro_t baro;
#ifdef LOG_SEC_IMU
  Axis3f accSec;            // Gs
  Axis3f gyroSec;           // deg/s
#endif
  uint64_t interruptTimestamp;
} sensorData_t;

typedef struct state_s {
  attitude_t attitude;      // deg (legacy CF2 body coordinate system, where pitch is inverted)
  quaternion_t attitudeQuaternion;
  point_t position;         // m
  velocity_t velocity;      // m/s
  acc_t acc;                // Gs (but acc.z without considering gravity)
} state_t;

#define STABILIZER_NR_OF_MOTORS 4

typedef enum control_mode_e {
  controlModeLegacy      = 0, // legacy mode with int16_t roll, pitch, yaw and float thrust
  controlModeForceTorque = 1,
  controlModeForce       = 2,
} control_mode_t;

typedef struct control_s {
  union {
    // controlModeLegacy
    struct {
      int16_t roll;
      int16_t pitch;
      int16_t yaw;
      float thrust;
    };

    // controlModeForceTorque
    // Note: Using SI units for a controller makes it hard to tune it for different platforms. The normalized force API
    // is probably a better option.
    struct {
      float thrustSi;  // N
      union { // Nm
        float torque[3];
        struct {
          float torqueX;
          float torqueY;
          float torqueZ;
        };
      };
    };

    // controlModeForce
    float normalizedForces[STABILIZER_NR_OF_MOTORS]; // 0.0 ... 1.0
  };

  control_mode_t controlMode;
} control_t;

typedef union {
  int32_t list[STABILIZER_NR_OF_MOTORS];
  struct {
    int32_t m1;
    int32_t m2;
    int32_t m3;
    int32_t m4;
  } motors;
} motors_thrust_uncapped_t;

typedef union {
  uint16_t list[STABILIZER_NR_OF_MOTORS];
  struct {
    uint16_t m1;  // PWM ratio
    uint16_t m2;  // PWM ratio
    uint16_t m3;  // PWM ratio
    uint16_t m4;  // PWM ratio
  } motors;
} motors_thrust_pwm_t;

typedef enum mode_e {
  modeDisable = 0,
  modeAbs,
  modeVelocity
} stab_mode_t;

typedef struct setpoint_s {
  uint32_t timestamp;

  attitude_t attitude;      // deg
  attitude_t attitudeRate;  // deg/s
  quaternion_t attitudeQuaternion;
  float thrust;
  point_t position;         // m
  velocity_t velocity;      // m/s
  acc_t acceleration;       // m/s^2
  jerk_t jerk;              // m/s^3
  bool velocity_body;       // true if velocity is given in body frame; false if velocity is given in world frame

  struct {
    stab_mode_t x;
    stab_mode_t y;
    stab_mode_t z;
    stab_mode_t roll;
    stab_mode_t pitch;
    stab_mode_t yaw;
    stab_mode_t quat;
  } mode;
} setpoint_t;

