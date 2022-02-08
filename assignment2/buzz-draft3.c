#include <stdio.h>
#include <stdint.h>
#include <math.h>

#include "contiki.h"
#include "sys/etimer.h"
#include "sys/rtimer.h"
#include "buzzer.h"

#include "board-peripherals.h"

PROCESS(process_buzz, "Process Buzz");
AUTOSTART_PROCESSES(&process_buzz);

// static int counter_rtimer;
static struct rtimer rt;
static rtimer_clock_t imu_timeout_rtimer = RTIMER_SECOND * 0.05;
static rtimer_clock_t light_timeout_rtimer = RTIMER_SECOND * 0.25;

int buzzerFrequency = 2000;

// flags
int motion_flag = 0;
int light_flag = 0;
int light_prev = 0;

// one process, timer driven
// have a flag, infinite loop until flag returns true. timer
// to poll every 100ms for motion

static void print_mpu_reading(long reading) {
  if(reading < 0) {
    printf("-");
    reading = -reading;
  }

  printf("%ld.%02ld", reading / 100, reading % 100);
}

static void init_mpu_reading(void) {
  mpu_9250_sensor.configure(SENSORS_ACTIVE, MPU_9250_SENSOR_TYPE_ALL);
}

static void init_opt_reading(void) {
  SENSORS_ACTIVATE(opt_3001_sensor);
}

// int has_significant_motion(motion_manager *prev_motion, motion_manager *curr_motion) {
int has_significant_motion() {
  // TODO: compare previous value with current value
  init_mpu_reading();
  int acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z;
  gyro_x = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_GYRO_X);
  gyro_y = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_GYRO_Y);
  gyro_z = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_GYRO_Z);
  acc_x = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_ACC_X);
  acc_y = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_ACC_Y);
  acc_z = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_ACC_Z);

  long acc_squared = (long) (acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z);
  double acc = sqrt((double) acc_squared);
  // printf("Acceleration: ");
  // print_mpu_reading((long) acc);
  // printf("\n");

  long gyro_squared = (long) (gyro_x * gyro_x) + (gyro_y * gyro_y) + (gyro_z * gyro_z);
  double gyro = sqrt((double) gyro_squared);
  // printf("Gyro: ");
  // print_mpu_reading((long) gyro);
  // printf("\n");
  
  //TODO: Calibrate this
  return acc > 250.0 || gyro > 35000.0;
}

int has_significant_light() {
  int value;
  value = opt_3001_sensor.value(0);
  init_opt_reading();
  if (value != CC26XX_SENSOR_READING_ERROR) {
    printf("OPT: Light=%d.%02d lux\n", value / 100, value % 100);
    int difference = abs(value - light_prev);
    printf("Light difference: %d\n", difference);
    if (difference >= 3000) {
      light_prev = 0;
      return 1;
    } 
    light_prev = value;
    return 0;
  } else {
    printf("OPT: Light Sensor's Warming Up\n\n");
    return 0;
  }
}

void do_imu_rtimer_timeout(struct rtimer *timer, void *ptr) {
  if (has_significant_motion()) {
    motion_flag = 1;
  } else {
    rtimer_set(&rt, RTIMER_NOW() + imu_timeout_rtimer, 0, do_imu_rtimer_timeout, NULL);
  }
}

void do_light_rtimer_timeout(struct rtimer* timer, void *ptr) {
  if (has_significant_light()) {
    light_flag = 1;
  }
}

PROCESS_THREAD(process_buzz, ev, data) {
  PROCESS_BEGIN();
  buzzer_init();

  while(1) {
    printf("Buzzer in idle state\n");
    init_mpu_reading();
    while(!motion_flag) {
      rtimer_set(&rt, RTIMER_NOW() + imu_timeout_rtimer, 0,  do_imu_rtimer_timeout, NULL);
      PROCESS_YIELD();
    }
    SENSORS_DEACTIVATE(mpu_9250_sensor);
    motion_flag = 0;

    init_opt_reading();
    printf("Buzzer in buzzing state\n");
    static int should_buzz = 1;
    light_flag = 0;
    while (!light_flag) {
      if (should_buzz) {
        buzzer_start(buzzerFrequency);
      }
      static int i = 0;
      for (; i < 12; i++) {
        if (light_flag) {
          break;
        }
        rtimer_set(&rt, RTIMER_NOW() + light_timeout_rtimer, 0,  do_light_rtimer_timeout, NULL);
        PROCESS_YIELD();
      }
      should_buzz = should_buzz == 1 ? 0 : 1;
      if (!should_buzz) {
        buzzer_stop();
      }
    }

    light_flag = 0;
    light_prev = 0;
  }
  PROCESS_END();
}
// while(1) {
//   while(1) {
//     //poll for motion
//     if (is_significant_motion) {
//       break;
//     }
//   }

//   int should_buzz = 1;
//   static struct et3;
//   while (!is_light_changed) {
//     etimer_set(&et3, CLOCK_SECOND / 10);
//     for (int i = 0; i < 30; i++) {
//       etimer_reset(&et3);

//       if (should_buzz) {
//         buzzer_start(buzzerFrequency);
//       }

//       PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et3));

//       if (should_buzz) {
//         buzzer_stop();
//       }

//       if (is_light_changed) {
//         break;
//       }
//     }

//     should_buzz = should_buzz == 1 ? 0 : 1;
//   }
// }