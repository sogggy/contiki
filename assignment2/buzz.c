#include <stdio.h>
#include <stdint.h>

#include "contiki.h"
#include "sys/rtimer.h"
#include "buzzer.h"

#include "board-peripherals.h"

PROCESS(process_buzz, "Process Buzz");
AUTOSTART_PROCESSES(&process_buzz);

#define MAX_BUZZ_COUNT 12
#define MIN_ACC 200
#define MIN_GYRO 18000
#define MIN_ACC_SQUARED MIN_ACC * MIN_ACC
#define MIN_GYRO_SQUARED MIN_GYRO * MIN_GYRO

// static int counter_rtimer;
static struct rtimer rt;
static rtimer_clock_t imu_timeout_rtimer = RTIMER_SECOND * 0.05;
static rtimer_clock_t light_timeout_rtimer = RTIMER_SECOND * 0.25;

static struct etimer et;
static int buzzerFrequency = 2000;
static int buzz_count = 0;

// flags
static int motion_flag = 0;
static int light_flag = 0;
static int light_prev = -1;
static int should_buzz = 1;

static void init_opt_reading(void);
static int has_significant_light(void);

static void print_mpu_reading(int reading) {
  if(reading < 0) {
    printf("-");
    reading = -reading;
  }

  printf("%d.%02d", reading / 100, reading % 100);
}

static void init_mpu_reading(void) {
  mpu_9250_sensor.configure(SENSORS_ACTIVE, MPU_9250_SENSOR_TYPE_ALL);
}

static void init_opt_reading(void) {
  SENSORS_ACTIVATE(opt_3001_sensor);
}

static int has_significant_motion() {
  init_mpu_reading();
  int acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z;
  gyro_x = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_GYRO_X);
  gyro_y = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_GYRO_Y);
  gyro_z = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_GYRO_Z);
  acc_x = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_ACC_X);
  acc_y = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_ACC_Y);
  acc_z = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_ACC_Z);

  int acc_squared = (acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z);
  int gyro_squared = (gyro_x * gyro_x) + (gyro_y * gyro_y) + (gyro_z * gyro_z);

  // printf("Acc squared: ");
  // print_mpu_reading(acc_squared);
  // printf("\n");

  // printf("Gyro squared: ");
  // print_mpu_reading(gyro_squared);
  // printf("\n");
  
  return acc_squared > MIN_ACC_SQUARED || gyro_squared > MIN_GYRO_SQUARED;
}

static int has_significant_light() {
  int value;
  value = opt_3001_sensor.value(0);
  int res = 0;
  if (value != CC26XX_SENSOR_READING_ERROR) {
    // printf("OPT: Light=%d.%02d lux\n", value / 100, value % 100);
    int lux = value / 100;
    if (light_prev == -1) {
      light_prev = lux;
      return 0;
    }
    
    int difference = abs(lux - light_prev);
    // printf("light_prev: %d\n", light_prev);
    // printf("Light difference: %d\n", difference);
    if (difference >= 300) {
      light_prev = -1;
      res = 1;
    } else {
      light_prev = lux;
      res = 0;
    }
  } else {
    printf("OPT: Light Sensor's Warming Up\n\n");
    res = 0;
  }
  init_opt_reading();
  return res;
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
  } else {
    rtimer_set(&rt, RTIMER_NOW() + light_timeout_rtimer, 0, do_light_rtimer_timeout, NULL);
  }
}

void do_etimer_timeout() {
  if (should_buzz && buzz_count == 0) {
    buzzer_start(buzzerFrequency);
  } else if (should_buzz && buzz_count == MAX_BUZZ_COUNT) {
    buzzer_stop();
  }

  if (buzz_count < MAX_BUZZ_COUNT) {
    buzz_count++;
  } else {
    buzz_count = 0;
  }

  if (buzz_count == 0) {
    should_buzz = should_buzz == 1 ? 0 : 1;
  }
}

void reset_flags() {
  buzz_count = 0;
  motion_flag = 0;
  light_flag = 0;
  should_buzz = 0;
  light_prev = -1;
}

PROCESS_THREAD(process_buzz, ev, data) {
  PROCESS_BEGIN();
  reset_flags();
  buzzer_init();

  while(1) {
    printf("Buzzer in idle state\n");
    init_mpu_reading();
    rtimer_set(&rt, RTIMER_NOW() + imu_timeout_rtimer, 0,  do_imu_rtimer_timeout, NULL);
    while(!motion_flag) {
      PROCESS_YIELD();
    }
    SENSORS_DEACTIVATE(mpu_9250_sensor);
    motion_flag = 0;

    init_opt_reading();
    printf("Buzzer in active state\n");
    should_buzz = 1;
    rtimer_set(&rt, RTIMER_NOW() + light_timeout_rtimer, 0,  do_light_rtimer_timeout, NULL);
    while (!light_flag) {
      do_etimer_timeout();
      etimer_set(&et, 0.25 * CLOCK_SECOND);
      PROCESS_WAIT_UNTIL(ev == PROCESS_EVENT_TIMER);
      PROCESS_YIELD();
    }
    if (should_buzz) {
      buzzer_stop();
    }
    reset_flags();
  }
  PROCESS_END();
}
