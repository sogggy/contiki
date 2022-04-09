#include "contiki.h"
#include "dev/leds.h"
#include <stdio.h>
#include <stdbool.h>
#include "core/net/rime/rime.h"
#include "dev/serial-line.h"
#include "dev/uart1.h"
#include "node-id.h"
#include "defs_and_types.h"
#include "net/netstack.h"
#include "random.h"
#ifdef TMOTE_SKY
#include "powertrace.h"
#endif
/*---------------------------------------------------------------------------*/

#define SLOT_TIME RTIMER_SECOND/30    // 80 HZ, 0.0125s
#define SLOT_DIM 10
#define CYCLE_DURATION (SLOT_TIME * 1.0 / RTIMER_SECOND * SLOT_DIM * SLOT_DIM)
#define PROXIMITY_DURATION 15
#define AWAY_DURATION 30
#define ARRAY_SIZE 10
#define RSSI -65
#define P1 17
#define P2 11
/*---------------------------------------------------------------------------*/
// sender timer
static struct rtimer rt;
static struct pt pt;
/*---------------------------------------------------------------------------*/
static data_packet_struct received_packet;
static data_packet_struct data_packet;
unsigned long curr_timestamp;

static int row;
static int col;
static discovery_struct hash_table[ARRAY_SIZE];
static discovery_struct discovery;
static int node_count;
/*---------------------------------------------------------------------------*/
PROCESS(tracetogether, "tracetogether");
AUTOSTART_PROCESSES(&tracetogether);
/*---------------------------------------------------------------------------*/

static int hash_code(unsigned long node_id) {
  return node_id % ARRAY_SIZE;
}

static int search(unsigned long node_id) {
  int index = hash_code(node_id);
  int count = 0;

  while (count < ARRAY_SIZE) { 
    if (hash_table[index].node_id == node_id) {
      return index;
    }

    index = (index + 1) % ARRAY_SIZE;
    count++;
  }

  return -1;

}

static void insert() {
  if (node_count >= ARRAY_SIZE) {
    return;
  }

  int index = hash_code(discovery.node_id);

  while (hash_table[index].node_id != -1) {
    index = (index + 1) % ARRAY_SIZE;
  }
  memcpy(&hash_table[index], &discovery, sizeof(discovery_struct));
  node_count++;
  printf("%3lu: New node %lu inserted\n", curr_timestamp / CLOCK_SECOND, discovery.node_id);

}

static void delete(int index) {
  printf("%3lu: Node %lu deleted.\n", curr_timestamp / CLOCK_SECOND, hash_table[index].node_id);
  hash_table[index].node_id = -1;
  node_count--;
}

// static void delete(unsigned long node_id) {
//   int index = hash_code(node_id);
//   int count = 0;
//   while (count < ARRAY_SIZE) {
//     if (hash_table[index].node_id == node_id) {
//       hash_table[index].node_id = -1;
//       node_count--;
//       print("%3lu: Node %lu deleted.", curr_timestamp / CLOCK_SECOND, node_id);
//       return;
//     }
//     index = (index + 1) % ARRAY_SIZE;
//     count++;
//   }

// }

static bool is_active(int slot) {
  return slot / SLOT_DIM == row || slot % SLOT_DIM == col;
  // return slot % P1 == 0 || slot % P2 == 0;
}

static void update_nodes() {

  // float cycle_duration = SLOT_TIME * 1.0 / RTIMER_SECOND * SLOT_DIM * SLOT_DIM;
  curr_timestamp = clock_time();
  int i = 0;
  
  while (i < ARRAY_SIZE) {
    if (hash_table[i].node_id == -1) {
      i++;
      continue;
    }
    float detection_diff = (curr_timestamp - hash_table[i].detection_timestamp) * 1.0 / CLOCK_SECOND;
    float update_diff = (curr_timestamp - hash_table[i].last_seen) * 1.0 / CLOCK_SECOND;
    // printf("%d, %d, %d, %d\n", detection_diff, update_diff, CYCLE_DURATION, hash_table[i].status);
    
    if (hash_table[i].status == Moving_Near && update_diff > CYCLE_DURATION) {
      // if node not in proximity for 15s, remove
      delete(i);
    } else if (hash_table[i].status == In_Proximity && update_diff > CYCLE_DURATION) {
      // if node was in proximity for 15s, start timer for moving away
      hash_table[i].detection_timestamp = curr_timestamp;
      hash_table[i].status = Moving_Away;
    } else if (hash_table[i].status == Moving_Away && detection_diff >= AWAY_DURATION) {
      // if node was moving away for 30s, print and remove
      printf("%lu ABSENT %lu\n", hash_table[i].detection_timestamp / CLOCK_SECOND, hash_table[i].node_id);
      delete(i);
    }
    i++;
  }

}

static void update_received(signed short rssi) {
  bool is_in_proximity = true;
  if (rssi < RSSI) {
    is_in_proximity = false;
  }
  curr_timestamp = clock_time();
  int target = search(received_packet.src_id);
  if (target == -1 && is_in_proximity) {
    discovery.node_id = received_packet.src_id;
    discovery.detection_timestamp = curr_timestamp;
    discovery.last_seen = curr_timestamp;
    discovery.status = Moving_Near;

    insert();
  } else if (target != -1) {
    float detection_diff = (curr_timestamp - hash_table[target].detection_timestamp) * 1.0 / CLOCK_SECOND;

    if (hash_table[target].status == Moving_Near && detection_diff >= PROXIMITY_DURATION && is_in_proximity) {
      // a new node has been in proximity for 15s
      printf("%lu DETECT %lu\n", hash_table[target].detection_timestamp / CLOCK_SECOND, hash_table[target].node_id);
      hash_table[target].status = In_Proximity;
      hash_table[target].last_seen = curr_timestamp;
    } else if (is_in_proximity && hash_table[target].status == Moving_Near) {
      hash_table[target].last_seen = curr_timestamp;
    } else if (is_in_proximity && hash_table[target].status == Moving_Away) {
      // the node thought to be moving away is back.
      hash_table[target].status = In_Proximity;
      hash_table[target].last_seen = curr_timestamp;
    } else if (!is_in_proximity && hash_table[target].status == In_Proximity) {
      // the node in proximity is moving away
      hash_table[target].detection_timestamp = curr_timestamp;
      hash_table[target].status = Moving_Away;
    } else if (!is_in_proximity && hash_table[target].status == Moving_Near) {
      // the node moving near is moving away
      delete(target);
    } else if (!is_in_proximity && hash_table[target].status == Moving_Away && detection_diff >= AWAY_DURATION) {
      // the node has been away for 30s
      printf("%lu ABSENT %lu\n", hash_table[target].detection_timestamp / CLOCK_SECOND, hash_table[target].node_id);
      delete(target);
    }
  }
}

static void
broadcast_recv(struct broadcast_conn *c, const linkaddr_t *from)
{
  leds_on(LEDS_GREEN);
  memcpy(&received_packet, packetbuf_dataptr(), sizeof(data_packet_struct));
  signed short rssi = (signed short)packetbuf_attr(PACKETBUF_ATTR_RSSI);

  printf("Received packet from node %lu with strength %d\n", received_packet.src_id, rssi);
  update_received(rssi);

  leds_off(LEDS_GREEN);
}
static const struct broadcast_callbacks broadcast_call = {broadcast_recv};
static struct broadcast_conn broadcast;
/*---------------------------------------------------------------------------*/
char sender_scheduler(struct rtimer *t, void *ptr) {
  static uint16_t i = 0;
  static int slot=0;
  PT_BEGIN(&pt);

  curr_timestamp = clock_time(); 
  printf("Start clock %lu ticks, timestamp %3lu.%03lu\n", curr_timestamp, curr_timestamp / CLOCK_SECOND, ((curr_timestamp % CLOCK_SECOND)*1000) / CLOCK_SECOND);

  while(1) {
    

    if (is_active(slot)) {
      // radio on
      NETSTACK_RADIO.on();

      for (i = 0; i < NUM_SEND; i++) {
        leds_on(LEDS_RED);
        
        data_packet.seq++;
        curr_timestamp = clock_time();
        data_packet.timestamp = curr_timestamp;

        // printf("Slot %d: Send seq# %lu  @ %8lu ticks   %3lu.%03lu\n", slot, data_packet.seq, curr_timestamp, curr_timestamp / CLOCK_SECOND, ((curr_timestamp % CLOCK_SECOND)*1000) / CLOCK_SECOND);

        packetbuf_copyfrom(&data_packet, (int)sizeof(data_packet_struct));
        broadcast_send(&broadcast);
        leds_off(LEDS_RED);

        if (i == (NUM_SEND - 1)) {
          // At the end of current slot and next slot not active
          NETSTACK_RADIO.off();
        } 
        
        else if (is_active((slot + 1) % (SLOT_DIM * SLOT_DIM))) {
          // Next slot is active
          rtimer_set(t, RTIMER_TIME(t) + SLOT_TIME, 1, (rtimer_callback_t)sender_scheduler, ptr);
          PT_YIELD(&pt);
          break;
        } 
        
        else {
          // Next slot is not active
          rtimer_set(t, RTIMER_TIME(t) + SLOT_TIME, 1, (rtimer_callback_t)sender_scheduler, ptr);
          PT_YIELD(&pt);
        }
      }
    } else {
      rtimer_set(t, RTIMER_TIME(t) + SLOT_TIME, 1, (rtimer_callback_t)sender_scheduler, ptr);
      PT_YIELD(&pt);
    }
    // Do a sweep for all the nodes once per cycle
    if (slot == SLOT_DIM * SLOT_DIM - 1) {
      printf("Find inactive nodes\n");
      update_nodes();
    }
    slot = (slot + 1) % (SLOT_DIM * SLOT_DIM);
    // slot++;
  }
  
  PT_END(&pt);
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(tracetogether, ev, data)
{
  PROCESS_EXITHANDLER(broadcast_close(&broadcast);)

  PROCESS_BEGIN();

  random_init(54222);

  #ifdef TMOTE_SKY
  powertrace_start(CLOCK_SECOND * 5);
  #endif

  broadcast_open(&broadcast, 129, &broadcast_call);

  // for serial port
  #if !WITH_UIP && !WITH_UIP6
  uart1_set_input(serial_line_input_byte);
  serial_line_init();
  #endif

  printf("Tracetogether\n");
  printf("Node %d will be sending packet of size %d Bytes\n", node_id, (int)sizeof(data_packet_struct));

  // radio off
  NETSTACK_RADIO.off();

  // initialize row and col
  row = random_rand() % SLOT_DIM;
  col = random_rand() % SLOT_DIM;
  printf("row: %d, col: %d\n", row, col);

  // initialize data packet
  data_packet.src_id = node_id;
  data_packet.seq = 0;
  // initialize hashtable
  printf("Initializing hash table.\n");
  int i = 0;
  while (i < ARRAY_SIZE) {
    hash_table[i].node_id = -1;
    i++;
  }

  // Start sender in one millisecond.
  rtimer_set(&rt, RTIMER_NOW() + (RTIMER_SECOND / 1000), 1, (rtimer_callback_t)sender_scheduler, NULL);

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
