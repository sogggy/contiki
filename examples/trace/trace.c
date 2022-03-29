#include "contiki.h"
#include "dev/leds.h"
#include <assert.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
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

typedef struct ht ht;
static void* ht_get(ht* table, const unsigned long key);
static const unsigned long ht_set(ht* table, const unsigned long key, void* value);
static size_t ht_length(ht* table);

typedef struct {
  unsigned long key;  // key is NULL if this slot is empty
  void* value;
} ht_entry;

struct ht {
  ht_entry* entries;  // hash slots
  size_t capacity;    // size of _entries array
  size_t length;      // number of items in hash table
};

#define INITIAL_CAPACITY 16

static ht* ht_create(void) {
  // Allocate space for hash table struct.
  ht* table = malloc(sizeof(ht));
  if (table == NULL) {
    return NULL;
  }
  table->length = 0;
  table->capacity = INITIAL_CAPACITY;

  // Allocate (zero'd) space for entry buckets.
  table->entries = calloc(table->capacity, sizeof(ht_entry));
  for (int i = 0; i < table->capacity; i++) {
    table->entries[i].key = -1;
  }

  if (table->entries == NULL) {
    free(table); // error, free table before we return!
    return NULL;
  }
  return table;
}

static void* ht_get(ht* table, const unsigned long key) {
  // AND hash with capacity-1 to ensure it's within entries array.
  uint64_t hash = (uint64_t) key;
  size_t index = (size_t)(hash & (uint64_t)(table->capacity - 1));

  // Loop till we find an empty entry.
  while (table->entries[index].key != 0) { //change to -1
    if (table->entries[index].key == key) {
      // Found key, return value.
      return table->entries[index].value;
    }
    // Key wasn't in this slot, move to next (linear probing).
    index++;
    if (index >= table->capacity) {
      // At end of entries array, wrap around.
      index = 0;
    }
  }
  return NULL;
}

static const unsigned long ht_set_entry(ht_entry* entries, size_t capacity,
        const unsigned long key, void* value, size_t* plength) {
  // AND hash with capacity-1 to ensure it's within entries array.
  uint64_t hash = (uint64_t) key;
  size_t index = (size_t)(hash & (uint64_t)(capacity - 1));

  // Loop till we find an empty entry.
  while (entries[index].key != -1) {
    if (entries[index].key == key) {
      // Found key (it already exists), update value.
      entries[index].value = value;
      return entries[index].key;
    }
    // Key wasn't in this slot, move to next (linear probing).
    index++;
    if (index >= capacity) {
      // At end of entries array, wrap around.
      index = 0;
    }
  }

  // Didn't find key, allocate+copy if needed, then insert it.
  if (plength != NULL) {
    if (key == -1) {
      return -1;
    }
    (*plength)++;
  }
  entries[index].key = (unsigned long)key;
  entries[index].value = value;
  return key;
}

// Expand hash table to twice its current size. Return true on success,
// false if out of memory.
static bool ht_expand(ht* table) {
  // Allocate new entries array.
  size_t new_capacity = table->capacity * 2;
  if (new_capacity < table->capacity) {
    return false;  // overflow (capacity would be too big)
  }
  ht_entry* new_entries = calloc(new_capacity, sizeof(ht_entry));
  if (new_entries == NULL) {
    return false;
  }

  // Iterate entries, move all non-empty ones to new table's entries.
  for (size_t i = 0; i < table->capacity; i++) {
    ht_entry entry = table->entries[i];
    if (entry.key != -1) {
      ht_set_entry(new_entries, new_capacity, entry.key,
                    entry.value, NULL);
    }
  }

  // Free old entries array and update this table's details.
  free(table->entries);
  table->entries = new_entries;
  table->capacity = new_capacity;
  return true;
}

static const unsigned long ht_set(ht* table, const unsigned long key, void* value) {
  assert(value != NULL);
  if (value == NULL) {
    return -1;
  }

  // If length will exceed half of current capacity, expand it.
  if (table->length >= table->capacity / 2) {
    if (!ht_expand(table)) {
      return -1;
    }
  }

  // Set entry and update length.
  return ht_set_entry(table->entries, table->capacity, key, value,
                      &table->length);
}

static void ht_delete(ht *table, unsigned long key) {
  for (size_t i = 0; i < table->capacity; i++) {
    if (table->entries[i].key == key) {
      free((void *) table->entries[i].value);
      return ;
    }
  }
}

static size_t ht_length(ht* table) {
  return table->length;
}

/*---------------------------------------------------------------------------*/
// Try lowering period?
#define SLOT_TIME RTIMER_SECOND/80    // 10 HZ, 0.1s
#define P1 11
#define P2 13
/*---------------------------------------------------------------------------*/
// #define SLEEP_CYCLE  9        	      // 0 for never sleep
// #define SLEEP_SLOT RTIMER_SECOND/10   // sleep slot should not be too large to prevent overflow
/*---------------------------------------------------------------------------*/
// duty cycle = WAKE_TIME / (WAKE_TIME + SLEEP_SLOT * SLEEP_CYCLE)
/*---------------------------------------------------------------------------*/
// sender timer
static struct rtimer rt;
static struct pt pt;

static struct rtimer rt2;
/*---------------------------------------------------------------------------*/
static data_packet_struct received_packet;
static data_packet_struct data_packet;
unsigned long curr_timestamp;

/*---------------------------------------------------------------------------*/
PROCESS(cc2650_nbr_discovery_process, "cc2650 neighbour discovery process");
AUTOSTART_PROCESSES(&cc2650_nbr_discovery_process);
/*---------------------------------------------------------------------------*/

typedef struct {
  bool is_absent;
  unsigned long new_state_first_timing;
  unsigned long last_seen_timing;
} node;

#define DETECT_SECONDS 15
#define ABSENT_SECONDS 30
#define RSSI_THRESHOLD -65 //less than means present, more than is absent

static ht* id_table;
static ht* time_table;

// state
// bool isAbsent = true;
// unsigned long new_state_first_timing = -1;
static unsigned long current_time;
static unsigned long sender_id;

static bool is_active(int slot) {
  return slot % P1 == 0 || slot % P2 == 0;
}

static void insert_into_timetable(ht *time_table, long key, void *value) {
  node *n = (node *)value;
  if (ht_get(time_table, key) == NULL) {
    node *nodes[INITIAL_CAPACITY];
    for (int i = 0; i < INITIAL_CAPACITY; i++) {
      nodes[i] = NULL;
    }
    node **nodes_ptr = nodes;
    ht_set(time_table, key, (void *)nodes_ptr);
    nodes[0] = n;
    return ;
  }

  //pointer to array of pointers - how do i cast properly
  node **nodes = ((node **)ht_get(time_table, key));
  for (int i = 0; i < INITIAL_CAPACITY; i++) {
    if (nodes[i] == NULL) {
      nodes[i] = n;
      return ;
    }
  }
}

static void
broadcast_recv(struct broadcast_conn *c, const linkaddr_t *from)
{
  signed short rssi = (signed short)packetbuf_attr(PACKETBUF_ATTR_RSSI);
  current_time = clock_time();
  leds_on(LEDS_GREEN);
  memcpy(&received_packet, packetbuf_dataptr(), sizeof(data_packet_struct));
  sender_id = received_packet.src_id;

  node *n = (node *) ht_get(id_table, sender_id);
  if (n == NULL) {
    node *new_node = (node *) malloc(sizeof(node));
    new_node->is_absent = false;
    new_node->new_state_first_timing = -1;
    new_node->last_seen_timing = -1;
    ht_set(id_table, sender_id, new_node);
    n = new_node; //check if this is correct
  }

  if (n->is_absent && rssi <= RSSI_THRESHOLD) {
    if (n->new_state_first_timing == -1) {
      n->new_state_first_timing = current_time;
      insert_into_timetable(time_table, (long) (n->new_state_first_timing / CLOCK_SECOND), (void *) n);
    }
    n->last_seen_timing = current_time;

    if ((current_time / CLOCK_SECOND) - (n->new_state_first_timing / CLOCK_SECOND) >= DETECT_SECONDS) {
      n->is_absent = false;
      n->new_state_first_timing = -1;
      printf("%ld DETECT %lu", n->new_state_first_timing / CLOCK_SECOND, received_packet.src_id);
    }
  }
  leds_off(LEDS_GREEN);
  // printf("Received packet from node %lu with sequence number %lu and timestamp %3lu.%03lu, RSSI: %d\n", received_packet.src_id, received_packet.seq, received_packet.timestamp / CLOCK_SECOND, ((received_packet.timestamp % CLOCK_SECOND)*1000) / CLOCK_SECOND, (signed short)packetbuf_attr(PACKETBUF_ATTR_RSSI));
}
static const struct broadcast_callbacks broadcast_call = {broadcast_recv};
static struct broadcast_conn broadcast;
/*---------------------------------------------------------------------------*/
char sender_scheduler(struct rtimer *t, void *ptr) {
  static uint16_t i = 0;
  static unsigned long slot=0;
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

        printf("Slot %lu: Send seq# %lu  @ %8lu ticks   %3lu.%03lu\n", slot, data_packet.seq, curr_timestamp, curr_timestamp / CLOCK_SECOND, ((curr_timestamp % CLOCK_SECOND)*1000) / CLOCK_SECOND);

        packetbuf_copyfrom(&data_packet, (int)sizeof(data_packet_struct));
        broadcast_send(&broadcast);
        leds_off(LEDS_RED);

        if (i == (NUM_SEND - 1)) {
          // At the end of current slot and next slot not active
          NETSTACK_RADIO.off();

        } else {
          rtimer_set(t, RTIMER_TIME(t) + SLOT_TIME, 1, (rtimer_callback_t)sender_scheduler, ptr);
          PT_YIELD(&pt);
        }
      }
    } else {
      rtimer_set(t, RTIMER_TIME(t) + SLOT_TIME, 1, (rtimer_callback_t)sender_scheduler, ptr);
      PT_YIELD(&pt);
    }

    slot = slot + 1;
  }
  
  PT_END(&pt);
}

void check_proximities() {
  // every 1 second, check the time_table and id_table. 
  // need to think about how to check the ids that have been discovered, but 'left' before the 15 second mark. check last_seen_timing
  // check time table, if there are any keys = current_time - 30, output and delete them
  // iterate through id table, if we find any isAbsent but last seen is more than 1 second, then we remove those. 
  rtimer_set(&rt2, RTIMER_NOW() + RTIMER_SECOND, 0, (rtimer_callback_t) check_proximities, NULL);
}

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(cc2650_nbr_discovery_process, ev, data)
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

  printf("CC2650 neighbour discovery\n");
  printf("Node %d will be sending packet of size %d Bytes\n", node_id, (int)sizeof(data_packet_struct));

  id_table = ht_create();
  time_table = ht_create();

  // radio off
  NETSTACK_RADIO.off();

  // initialize data packet
  data_packet.src_id = node_id;
  data_packet.seq = 0;

  // Start sender in one millisecond.
  rtimer_set(&rt, RTIMER_NOW() + (RTIMER_SECOND / 1000), 1, (rtimer_callback_t)sender_scheduler, NULL);
  rtimer_set(&rt2, RTIMER_NOW() + (RTIMER_SECOND / 1000), 1, (rtimer_callback_t) check_proximities, NULL);
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
