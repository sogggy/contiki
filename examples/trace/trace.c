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
#include "lib/memb.h"
#include "node-id.h"
#include "defs_and_types.h"
#include "net/netstack.h"
#include "random.h"
#ifdef TMOTE_SKY
#include "powertrace.h"
#endif

#define INITIAL_CAPACITY 16
#define INVALID_TUPLE	(long)-1
#define MAX_NODES 8

struct hash_item {
  long tuple_id;
  void* value;
};
typedef struct hash_item hash_item_t;
typedef hash_item_t hash_map_t[INITIAL_CAPACITY];

struct hts {
  hash_map_t *id_table;
  hash_map_t *time_table;
};
typedef struct hts hts_t;

struct node {
  long id;
  bool is_absent;
  long new_state_first_timing;
  long last_seen_timing;
};
typedef struct node node_t;
typedef node_t node_arr_t[MAX_NODES]; //map timing of 2,3,4s... to node array in time_table

static void create(hts_t *ht);
static void destroy(hts_t *ht);
static void insert(hash_map_t *hash_map, void *, long);
static void delete(hash_map_t *hash_map, long);

MEMB(ht_memb, hts_t, 1);
MEMB(node_arr_memb, node_arr_t, 1);
MEMB(id_table_memb, hash_map_t, 1);
MEMB(time_table_memb, hash_map_t, 1);

hts_t *ht;
node_arr_t *node_arr;

static unsigned calculate_hash(long value) {
  return (uint16_t) (value % INITIAL_CAPACITY);
}

static void
create(hts_t *ht)
{
  int i;
  hash_map_t *hash_map1;
  hash_map_t *hash_map2;

  printf("Creating hash maps\n");

  hash_map1 = memb_alloc(&id_table_memb);
  hash_map2 = memb_alloc(&time_table_memb);

  if(hash_map1 == NULL || hash_map2 == NULL) {
    printf("Error creating hash maps 1 or 2. \n");
  }

  for(i = 0; i < INITIAL_CAPACITY; i++) {
    hash_map1[i]->tuple_id = INVALID_TUPLE;
    hash_map2[i]->tuple_id = INVALID_TUPLE;
    hash_map1[i]->value = NULL;
    hash_map2[i]->value = NULL;
  }

  ht->id_table = hash_map1;
  ht->time_table = hash_map2;

  return ;
}

static void
destroy(hts_t *ht)
{
  // didnt auto fill here
  memb_free(&id_table_memb, ht->id_table);
  memb_free(&time_table_memb, ht->time_table);

  return ;
}

static void
insert(hash_map_t *hash_map, void *value, long tuple_id)
{
  uint16_t hash_value;
  hash_value = calculate_hash(tuple_id);
  hash_map[hash_value]->tuple_id = tuple_id;
  hash_map[hash_value]->value = value;

  // PRINTF("Inserted value %ld into the hash table\n", value);

  return ;
}

static void * 
get(hash_map_t *hash_map, long tuple_id)
{
  uint16_t hash_value = calculate_hash(tuple_id);
  return hash_map[hash_value]->value;
}

static void
delete(hash_map_t *hash_map, long tuple_id)
{
  uint16_t hash_value = calculate_hash(tuple_id);
  // if(memcmp(&hash_map[hash_value]->tuple_id, tuple_id, sizeof(tuple_id)) != 0) {
  //   printf("Something went wrong deleting tuple\n");
  //   return ;
  // }

  hash_map[hash_value]->tuple_id = INVALID_TUPLE;
  hash_map[hash_value]->value = NULL;
  return ;
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
/*---------------------------------------------------------------------------*/
static data_packet_struct received_packet;
static data_packet_struct data_packet;
unsigned long curr_timestamp;

unsigned long received_time;
unsigned long sender_id;

/*---------------------------------------------------------------------------*/
PROCESS(cc2650_nbr_discovery_process, "cc2650 neighbour discovery process");
AUTOSTART_PROCESSES(&cc2650_nbr_discovery_process);
/*---------------------------------------------------------------------------*/

#define DETECT_SECONDS 15
#define ABSENT_SECONDS 30
#define RSSI_THRESHOLD -65 //less than means present, more than is absent

static bool is_active(int slot) {
  return slot % P1 == 0 || slot % P2 == 0;
}

static void print_node(node_t *n) {
  if (n == NULL) {
    printf("n is NULL\n");
    return;
  }

  printf("n id is %lu, is_absent: %d, new_state_first_timing: %lu, last_seen_timing: %lu\n", n->id, n->is_absent, n->new_state_first_timing, n->last_seen_timing);
}

static void
broadcast_recv(struct broadcast_conn *c, const linkaddr_t *from)
{
  leds_on(LEDS_GREEN);
  memcpy(&received_packet, packetbuf_dataptr(), sizeof(data_packet_struct));
  received_time = received_packet.timestamp;
  sender_id = received_packet.src_id;
  signed short rssi = (signed short)packetbuf_attr(PACKETBUF_ATTR_RSSI);
  // printf("Send seq# %lu  @ %8lu  %3lu.%03lu\n", data_packet.seq, curr_timestamp, curr_timestamp / CLOCK_SECOND, ((curr_timestamp % CLOCK_SECOND)*1000) / CLOCK_SECOND);
  printf("Received packet with RSSI %d, from node %lu with sequence number %lu and timestamp %3lu.%03lu\n", rssi, received_packet.src_id, received_packet.seq, received_packet.timestamp / CLOCK_SECOND, ((received_packet.timestamp % CLOCK_SECOND)*1000) / CLOCK_SECOND);

  // node_t *n = (node_t *) get(ht->id_table, sender_id);
  // print_node(n);
  // if (n == NULL) {
  //   node_t *new_node = (node_t *)malloc(sizeof(node_t));
  //   new_node->id = received_packet.src_id;
  //   new_node->new_state_first_timing = -1;
  //   new_node->last_seen_timing = -1;
  //   new_node->is_absent = true;
  //   n = new_node;
  // }
  // print_node(n);

  leds_off(LEDS_GREEN);
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

  memb_init(&ht_memb);
  memb_init(&id_table_memb);
  memb_init(&time_table_memb);
  memb_init(&node_arr_memb);

  ht = memb_alloc(&ht_memb);
  create(ht);
  node_arr = memb_alloc(&node_arr_memb);

  int i = 0;
  for (i = 0; i < MAX_NODES; i++) {
    node_arr[i]->id = i;
    node_arr[i]->is_absent = true;
    node_arr[i]->last_seen_timing = -1;
    node_arr[i]->new_state_first_timing = -1;
    print_node(node_arr[i]);
  }

  // char* test = "test";
  // insert(ht->id_table, (void *)test, (long) 5);
  // char* val = get(ht->id_table, (long) 5);
  // printf("Value is:  %s\n", val);
  // delete(ht->id_table, (long) 5);
  // printf("Value is %s\n", (char*) get(ht->id_table, (long) 5));

  // radio off
  NETSTACK_RADIO.off();

  // initialize data packet
  data_packet.src_id = node_id;
  data_packet.seq = 0;

  // Start sender in one millisecond.
  rtimer_set(&rt, RTIMER_NOW() + (RTIMER_SECOND / 1000), 1, (rtimer_callback_t)sender_scheduler, NULL);

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
