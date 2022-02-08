#include <stdio.h>
#include <stdint.h>

#include "contiki.h"
#include "sys/etimer.h"
#include "sys/rtimer.h"
#include "buzzer.h"

#include "board-peripherals.h"

PROCESS(process_buzzer, "Buzzer");
AUTOSTART_PROCESSES(&process_buzzer);

#define IDLE "1"
#define ACTIVE "2"
#define BUZZ "2a"
#define WAIT "2b"

// static struct rtimer timer_rtimer;
// static rtimer_clock_t timeout_rtimer = RTIMER_SECOND / 20;  

int buzzerFrequency = 2794;

typedef struct motion_manager {
  int gyro_x;
  int gyro_y;
  int gyro_z;
  int acc_x;
  int acc_y;
  int acc_z;
} motion_manager;

typedef struct state_manager {
  char *current_state;
  char *current_active_state;
  struct motion_manager *curr_motion;
  struct motion_manager *prev_motion;
} state_manager;

static void
init_mpu_reading(void) {
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

int is_significant_motion(state_manager *state_mgr) {
  // if (state_mgr->current_state == ACTIVE) {
  //   return 0;
  // }
  motion_manager *curr_motion = (motion_manager *)malloc(sizeof(motion_manager));
  init_motion_values(curr_motion);

  // TODO: compare previous value with current value
  
  state_mgr->prev_motion = state_mgr->curr_motion;
  state_mgr->curr_motion = curr_motion; 
  return 1;
}

void init_active(state_manager *state_mgr, motion_manager *motion_mgr) {
  struct etimer et;
  buzzer_init();
  while (1) {
    state_mgr->current_state = ACTIVE;
    state_mgr->current_active_state = BUZZ;
    buzzer_start(buzzerFrequency);
    etimer_set(&et, 3 * CLOCK_SECOND);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

    state_mgr->current_active_state = WAIT;
    buzzer_stop();
    etimer_reset(&et);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
  }
}

void init_idle(void *ptr_state_mgr, void *ptr_motion_mgr) {
  state_manager *state_mgr = (state_manager *)ptr_state_mgr;
  motion_manager *motion_mgr = (motion_manager*) ptr_motion_mgr;
  // rtimer_set(&timer_rtimer, RTIMER_NOW() + timeout_rtimer, 0, init_idle, ptr);

  while(1) {
    if (is_significant_motion(state_mgr)) {
      puts("Buzzer is currently in active state");
      init_active(state_mgr, motion_mgr);
    }
  }
}

PROCESS_THREAD(process_buzzer, ev, data) {
  PROCESS_BEGIN();
  init_mpu_reading();
  puts("Buzzer is currently in idle state");
  while (1) {
    state_manager *state_mgr = (state_manager*)malloc(sizeof(state_manager));
    //TODO: init state_mgr values

    motion_manager *curr_motion = (motion_manager *)malloc(sizeof(motion_manager));
    init_motion_values(curr_motion);
    // rtimer_set(&timer_rtimer, RTIMER_NOW() + timeout_rtimer, 0, do_rtimer_timeout, (void *)state_mgr);
    init_idle((void *) state_mgr, (void *) curr_motion);
    free(state_mgr);
    free(curr_motion);
    // PROCESS_YIELD();
  }

  PROCESS_END();
}


