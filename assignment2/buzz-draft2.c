#include <stdio.h>
#include <stdint.h>

#include "contiki.h"
#include "sys/etimer.h"
#include "sys/rtimer.h"
#include "buzzer.h"

#include "board-peripherals.h"

PROCESS(process_idle, "Idle");
PROCESS(process_buzz, "Active - Buzz");
PROCESS(process_wait, "Active - Wait");
AUTOSTART_PROCESSES(&process_idle);

int buzzerFrequency = 2000;

typedef struct motion_manager {
  int gyro_x;
  int gyro_y;
  int gyro_z;
  int acc_x;
  int acc_y;
  int acc_z;
} motion_manager;

static void init_mpu_reading(void) {
  mpu_9250_sensor.configure(SENSORS_ACTIVE, MPU_9250_SENSOR_TYPE_ALL);
}

void init_motion_values(motion_manager *motion) {
  int value;
  value = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_GYRO_X);
  motion->gyro_x = value;

  value = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_GYRO_Y);
  motion->gyro_y = value;

  value = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_GYRO_Z);
  motion->gyro_z = value;
  
  value = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_ACC_X);
  motion->acc_x = value;
  
  value = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_ACC_Y);
  motion->acc_y = value;
  
  value = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_ACC_Z);
  motion->acc_z = value;
}

int is_significant_motion(motion_manager *prev_motion, motion_manager *curr_motion) {
  // TODO: compare previous value with current value
  return 1;
}

PROCESS_THREAD(process_idle, ev, data) {
  PROCESS_BEGIN();
  init_mpu_reading();
  printf("Buzzer is currently in idle state\n");
  motion_manager *prev_motion;
  init_motion_values(prev_motion);
  motion_manager *curr_motion;

  while (1) {    
    init_motion_values(curr_motion);
    if (is_significant_motion(prev_motion, curr_motion)) {
      process_start(&process_buzz, NULL);
      //do we need to free shit
      // free(prev_motion);
      // free(curr_motion);
      PROCESS_EXIT();
    }
    curr_motion = prev_motion;
  }

  PROCESS_END();
}

PROCESS_THREAD(process_buzz, ev, data) {
  PROCESS_BEGIN();

  printf("Buzzer is currently in buzz state\n");
  static struct etimer et;
  buzzer_init();
  etimer_set(&et, 3 * CLOCK_SECOND);
  // process_poll(&process_wait);
  // process_post(&process_wait, PROCESS_EVENT_INIT, NULL);
  // etimer_reset(&et);
  while(1) {
    buzzer_start(buzzerFrequency);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
    buzzer_stop();
    process_post(&process_wait, PROCESS_EVENT_INIT, NULL);
    PROCESS_YIELD();
    etimer_reset(&et);
  }
  
  PROCESS_END();
}

PROCESS_THREAD(process_wait, ev, data) {
  PROCESS_BEGIN();

  printf("Buzzer is currently in wait state\n");
  static struct etimer et2;
  etimer_set(&et2, 3 * CLOCK_SECOND);
  // process_post(&process_buzz, PROCESS_EVENT_INIT, NULL);
  // process_poll(&process_buzz);
  while(1) {
    printf("test\n");
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et2));
    process_post(&process_buzz, PROCESS_EVENT_INIT, NULL);
    PROCESS_YIELD();
    etimer_reset(&et2);
  }

  PROCESS_END();
}