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

#define INITIAL_CAPACITY 8
#define INVALID_TUPLE	0
#define MAX_NODES 8 

struct hash_item {
  long tuple_id;
  void* value;
};
typedef struct hash_item hash_item_t;
typedef hash_item_t hash_map_t[INITIAL_CAPACITY];

struct node {
  long id;
  bool is_nearby;
  long new_state_first_timing;
  long last_seen_timing;
};
typedef struct node node_t;
typedef node_t node_arr_t[MAX_NODES];

struct hts {
  hash_map_t *id_table;
  node_arr_t *node_arr;
  int next_new_node;
};
typedef struct hts hts_t;

static void create(hts_t *hts);
static void destroy(hts_t *hts);
static void insert(hash_map_t *hash_map, void *, long);

MEMB(hts_memb, hts_t, 1);
MEMB(id_table_memb, hash_map_t, 1);
MEMB(node_arr_memb, node_arr_t, 1);

hts_t *hts;

static void print_node(node_t *n) {
  if (n == NULL) {
    printf("n is NULL\n");
    return;
  }

  printf("n id is %ld, is_nearby: %d, new_state_first_timing: %ld, last_seen_timing: %ld\n", n->id, n->is_nearby, n->new_state_first_timing, n->last_seen_timing);
}

static unsigned calculate_hash(long value) {
  return (uint16_t) (value % INITIAL_CAPACITY);
}

static void
create(hts_t *hts)
{
  int i;
  hash_map_t *hash_map;
  node_arr_t *node_arr;

  printf("Creating hash map and node array\n");

  hash_map = memb_alloc(&id_table_memb);
  node_arr = memb_alloc(&node_arr_memb);

  if(hash_map == NULL || node_arr == NULL) {
    printf("Error creating hash maps or node array. \n");
  }

  for(i = 0; i < INITIAL_CAPACITY; i++) {
    hash_map[i]->value = NULL;
  }

  // create nodes for the hashmap beforehand
  hts->id_table = hash_map;
  hts->node_arr = node_arr;
  hts->next_new_node = 0;

  return ;
}

static void
destroy(hts_t *hts)
{
  // didnt auto fill here
  memb_free(&id_table_memb, hts->id_table);
  memb_free(&node_arr_memb, hts->node_arr);
  memb_free(&hts_memb, hts);
  return ;
}

static void
insert(hash_map_t *hash_map, void *value, long tuple_id)
{
  uint16_t hash_value;
  hash_value = calculate_hash(tuple_id);
  if (hash_map[hash_value]->tuple_id == INVALID_TUPLE || 
                hash_map[hash_value]->tuple_id == tuple_id) {
    hash_map[hash_value]->tuple_id = tuple_id;
    hash_map[hash_value]->value = value;
    return;
  }
  
  //linearly probe for slots.
  int i = 0;
  while (hash_map[hash_value]->tuple_id != INVALID_TUPLE && i < INITIAL_CAPACITY) {
    hash_value = (hash_value + 1) % INITIAL_CAPACITY;
    i++;
  }

  if (hash_map[hash_value]->tuple_id != INVALID_TUPLE) {
    // printf("Hashmap is full. Insertion failed. \n");
    return;
  }

  hash_map[hash_value]->tuple_id = tuple_id;
  hash_map[hash_value]->value = value;

  return ;
}

static void * 
get(hash_map_t *hash_map, long tuple_id)
{
  uint16_t hash_value = calculate_hash(tuple_id);
  if (hash_map[hash_value]->tuple_id == tuple_id) {
    return hash_map[hash_value]->value;
  }

  //linearly probe
  int i = 0;
  while (hash_map[hash_value]->tuple_id != tuple_id && i < INITIAL_CAPACITY) {
    hash_value = (hash_value + 1) % INITIAL_CAPACITY;
    i++;
  }

  if (hash_map[hash_value]->tuple_id != tuple_id) {
    // printf("Key %lu not found in Hashmap. Get failed. \n", tuple_id);
    return NULL;
  }

  return hash_map[hash_value]->value;
}

static node_t *get_new_node(hts_t *hts) {
  int next_new_node = hts->next_new_node;
  int i;
  for (i = 0; i < MAX_NODES; i++) {
    if (hts->node_arr[next_new_node]->id == INVALID_TUPLE) {
      hts->next_new_node = next_new_node + 1;
      return hts->node_arr[next_new_node];
    }
    next_new_node = (next_new_node + 1) % MAX_NODES;
  }

  printf("Cannot hold anymore new nodes, memory full \n");
  return NULL;
}

static void reset_node_state(node_t *node) {
  // printf("Resetting node's state\n");
  node->id = INVALID_TUPLE;
  node->is_nearby = false;
  node->last_seen_timing = INVALID_TUPLE;
  node->new_state_first_timing = INVALID_TUPLE;
  // print_node(node);
}

static void upgrade_node_state(node_t *node, long last_seen_timing) {
  //upgrade to present - is_nearby is false
  node->is_nearby = true;
  node->last_seen_timing = last_seen_timing;
  node->new_state_first_timing = INVALID_TUPLE; 
}

/*---------------------------------------------------------------------------*/
// Try lowering period?
#define SLOT_TIME RTIMER_SECOND/80    // 80 HZ, 0.0125s
#define SLOT_DIM 8
/*---------------------------------------------------------------------------*/
// sender timer
static struct rtimer rt;
static struct pt pt;
/*---------------------------------------------------------------------------*/
static data_packet_struct received_packet;
static data_packet_struct data_packet;
unsigned long curr_timestamp;
unsigned long curr_timestamp_seconds;

static int row;
static int col;

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
  return slot / SLOT_DIM == row || slot % SLOT_DIM == col;
}

static void check_nodes(int slot, int curr_timestamp_seconds) {
  // check every ~1s
  if (slot % (SLOT_DIM * SLOT_DIM) != 10) {
    return ;
  }

  int j;
  for (j = 0; j < MAX_NODES; j++) {
    // is a unassigned node or resetted node
    node_t *node = hts->node_arr[j];
    // print_node(n)
    if (node->id == INVALID_TUPLE || (node->last_seen_timing == INVALID_TUPLE && node->new_state_first_timing == INVALID_TUPLE)) {
      continue;
    }


    if (node->is_nearby == false && curr_timestamp_seconds - node->last_seen_timing >= 3) {
      // for nodes that were seen before and still considered absent, 
      // if the current time is more than the last_seen_timing by more than 3 seconds, 
      // we reset that node's state.
      reset_node_state(node);
    } else if (node->is_nearby && (curr_timestamp_seconds - node->last_seen_timing >= ABSENT_SECONDS)) {
      // for nodes that are already considered present
      // and the current time is more than last_seen_timing by more than 30 seconds,
      // declare that it is absent and upgrade node to absent.
      printf("%ld ABSENT %lu\n", node->last_seen_timing, node->id);
      reset_node_state(node);
    }
  }
}

static void 
update_received(int rssi, long sender_id, long received_time) 
{
  if (rssi < RSSI_THRESHOLD) {
    return ;
  }

  node_t *n = (node_t *) get(hts->id_table, sender_id);

  if (n == NULL) {
    node_t *new_node = get_new_node(hts);
    new_node->id = received_packet.src_id;
    new_node->new_state_first_timing = received_time;
    new_node->last_seen_timing = received_time;
    n = new_node;
    insert(hts->id_table, (void *)n, sender_id);
  }
  // print_node(n);
  if (n->new_state_first_timing == INVALID_TUPLE) {
    n->new_state_first_timing = received_time;
  }
  n->last_seen_timing = received_time;

  //bypass nodes that were already detected and still present
  if (n->is_nearby == false && n->new_state_first_timing != -1 && (received_time - n->new_state_first_timing >= DETECT_SECONDS)) {
    printf("%ld DETECT %ld\n", n->new_state_first_timing, n->id);
    upgrade_node_state(n, received_time);
  }
}

static void
broadcast_recv(struct broadcast_conn *c, const linkaddr_t *from)
{
  leds_on(LEDS_GREEN);
  memcpy(&received_packet, packetbuf_dataptr(), sizeof(data_packet_struct));
  received_time = clock_time() / CLOCK_SECOND;
  sender_id = received_packet.src_id;
  signed short rssi = (signed short)packetbuf_attr(PACKETBUF_ATTR_RSSI);
  // printf("Send seq# %lu  @ %8lu  %3lu.%03lu\n", data_packet.seq, curr_timestamp, curr_timestamp / CLOCK_SECOND, ((curr_timestamp % CLOCK_SECOND)*1000) / CLOCK_SECOND);
  // printf("Received packet with RSSI %d, from node %lu with sequence number %lu and timestamp %3lu.%03lu\n", rssi, received_packet.src_id, received_packet.seq, received_packet.timestamp / CLOCK_SECOND, ((received_packet.timestamp % CLOCK_SECOND)*1000) / CLOCK_SECOND);

  update_received(rssi, sender_id, received_time);

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
  // printf("Start clock %lu ticks, timestamp %3lu.%03lu\n", curr_timestamp, curr_timestamp / CLOCK_SECOND, ((curr_timestamp % CLOCK_SECOND)*1000) / CLOCK_SECOND);

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
        } else if (is_active((slot + 1) % (SLOT_DIM * SLOT_DIM))) {
          // Next slot is active
          rtimer_set(t, RTIMER_TIME(t) + SLOT_TIME, 1, (rtimer_callback_t)sender_scheduler, ptr);
          PT_YIELD(&pt);
          break;
        } else {
          // Next slot is not active
          rtimer_set(t, RTIMER_TIME(t) + SLOT_TIME, 1, (rtimer_callback_t)sender_scheduler, ptr);
          PT_YIELD(&pt);
        }
      }
    } else {
      rtimer_set(t, RTIMER_TIME(t) + SLOT_TIME, 1, (rtimer_callback_t)sender_scheduler, ptr);
      PT_YIELD(&pt);
    }

    check_nodes(slot, clock_time() / CLOCK_SECOND);
    slot = (slot + 1) % (SLOT_DIM * SLOT_DIM);
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

  // radio off
  NETSTACK_RADIO.off();

  // initialize row and col
  row = random_rand() % SLOT_DIM;
  col = random_rand() % SLOT_DIM;
  printf("row: %d, col: %d\n", row, col);

  // initialize data packet
  data_packet.src_id = node_id;
  data_packet.seq = 0;

  memb_init(&hts_memb);
  memb_init(&id_table_memb);
  memb_init(&node_arr_memb);

  hts = memb_alloc(&hts_memb);
  create(hts);

  // Start sender in one millisecond.
  rtimer_set(&rt, RTIMER_NOW() + (RTIMER_SECOND / 1000), 1, (rtimer_callback_t)sender_scheduler, NULL);

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/