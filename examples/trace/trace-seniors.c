#include "contiki.h"
#include "dev/leds.h"
#include <stdio.h>
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
#define WAKE_TIME RTIMER_SECOND / 1000 * 26.5 // 26.5ms
/*---------------------------------------------------------------------------*/                         
#define SLEEP_SLOT RTIMER_SECOND / 1000 * 26.5 // 26.5ms
/*---------------------------------------------------------------------------*/
// sender timer
static struct rtimer rt;
static struct pt pt;
/*---------------------------------------------------------------------------*/
static data_packet_struct received_packet;
static data_packet_struct data_packet;
unsigned long curr_timestamp;
/*---------------------------------------------------------------------------*/
PROCESS(cc2650_nbr_discovery_process, "cc2650 neighbour discovery process");
AUTOSTART_PROCESSES(&cc2650_nbr_discovery_process);
/*---------------------------------------------------------------------------*/
struct nodeInfo
{
  int ID;
  int FIRST;
  long unsigned int INCREMENT;
  int TIME;
  long unsigned int TOTAL_TIME;
  long unsigned int LATENCY;
  int STATE;
  int WITHIN_PROXIMITY;
};

struct nodeInfo nodes[10];

// Process is called when a packet is received.
static void process(int nodeID, int rssi)
{
  // RSSI better than -67dBm means that the device is within 3m.
  if (rssi >= -67)
  {
    int now = clock_time();
    int i = 0;
    int workingNodeID = 0;
    int workingIndex = 0;
    // If such a node ID has already been seen before, we update its values.
    for (i = 0; i < 10; i++)
    {
      if (nodes[i].ID == nodeID)
      {
        workingIndex = i;
        workingNodeID = nodeID;
      }
    }
    // Otherwise, we update an empty struct to represent this new node ID.
    if (workingNodeID == 0)
    {
      int i;
      for (i = 0; i < 10; i++)
      {
        if (nodes[i].ID == 0)
        {
          workingIndex = i;
          nodes[i].ID = nodeID;
          break;
        }
      }
      // We set the initial timestamp of a newly entered node to determine how long it has been in a proximity.
      nodes[workingIndex].FIRST = now;
    }

    // Update the nodes latest seen timestamp.
    nodes[workingIndex].TIME = now;

    // If a node is detected within proximity, print.
    if (!nodes[workingIndex].WITHIN_PROXIMITY)
    {
      long unsigned int t = now / CLOCK_SECOND;
      printf("%lu DETECT %d\n", t, nodeID);
      nodes[workingIndex].WITHIN_PROXIMITY = 1;
    }

    // If a node is in proximity but still waiting on to have its state change, we calculate his total time in proximity.
    if (nodes[workingIndex].STATE == 0)
    {
      nodes[workingIndex].TOTAL_TIME = now - nodes[workingIndex].FIRST;
    }

    // If a state has previously been in proximity fo 30 second, we now reset its time everytime a packet is recieved from it - much like a keep alive connection.
    if (nodes[workingIndex].STATE == 1)
    {
      nodes[workingIndex].FIRST = now;
      nodes[workingIndex].TOTAL_TIME = 0;
    }

    // If the total time for a node in proximity exceeds 30 seconds, we change its state and then reset its values.
    if (nodes[workingIndex].TOTAL_TIME / CLOCK_SECOND > 30 && nodes[workingIndex].STATE == 0)
    {
      long unsigned int t = now / CLOCK_SECOND;
      printf("%lu IN PROXIMITY FOR > 30 SECONDS %d \n", t, nodes[workingIndex].ID);
      nodes[workingIndex].TOTAL_TIME = 0;
      nodes[workingIndex].LATENCY = 0;
      nodes[workingIndex].STATE = 1;
      nodes[workingIndex].FIRST = now;
      nodes[workingIndex].INCREMENT = now;
    }
  }
}

// Check is called before the node sleeps.
static void
check()
{
  int i;
  int now = clock_time();

  // For each node struct in the array that is not empty, we update its latency and determine if we should phase out the node.
  for (i = 0; i < 10; i++)
  {
    if (nodes[i].ID != 0)
    {
      // Latency represents the delta between the current time and the last seen node time.
      nodes[i].LATENCY = now - nodes[i].TIME;

      // If a node's state is 0 but its latency has exceeded the expected deterministic reporting time, we phase the node out and set the struct to 0.
      if (nodes[i].STATE == 0)
      {
        if (nodes[i].LATENCY / CLOCK_SECOND > 4)
        {
          nodes[i].ID = 0;
          nodes[i].TIME = 0;
          nodes[i].FIRST = 0;
          nodes[i].INCREMENT = 0;
          nodes[i].TOTAL_TIME = 0;
          nodes[i].LATENCY = 0;
          nodes[i].STATE = 0;
        }
      }

      // Otherwise, if a node has been in proximity for over 30 seconds...
      if (nodes[i].STATE == 1)
      {
        // We check if a node is still in proximity, if its latency is still high, the node has left the proximity area.
        if (nodes[i].LATENCY / CLOCK_SECOND > 4)
        {
          if (nodes[i].WITHIN_PROXIMITY)
          {
            long unsigned int t = now / CLOCK_SECOND;
            printf("%lu LEAVE %d \n", t, nodes[i].ID);
            nodes[i].WITHIN_PROXIMITY = 0;
          }
        }

        // We also accumulate the total time a node has spend in the area since its first timestamp. Note that this is resetted everytime a new packet arrives from the node i.e. keep alive.
        // The calculation of actual time spent in proximity is kep in INCREMENT instead.
        nodes[i].TOTAL_TIME = now - nodes[i].FIRST;

        // Finally if a node spends over 30 second out of proximity, we print its total spend within it, phase it out and set the struct to 0.
        if (nodes[i].TOTAL_TIME / CLOCK_SECOND > 30)
        {
          long unsigned int t = now / CLOCK_SECOND;
          printf("%lu OUT OF PROXIMITY FOR > 30 SECONDS %d \n", t, nodes[i].ID);
          long unsigned int timeSpent = now - nodes[i].INCREMENT;
          printf("%lu TOTAL TIME SPENT IN PROXIMITY:%3lu.%03lus %d\n", t, timeSpent / CLOCK_SECOND, ((timeSpent % CLOCK_SECOND * 1000) / CLOCK_SECOND), nodes[i].ID);
          nodes[i].ID = 0;
          nodes[i].TIME = 0;
          nodes[i].FIRST = 0;
          nodes[i].INCREMENT = 0;
          nodes[i].TOTAL_TIME = 0;
          nodes[i].LATENCY = 0;
          nodes[i].STATE = 0;
        }
      }
    }
  }
}

static void
broadcast_recv(struct broadcast_conn *c, const linkaddr_t *from)
{
  leds_on(LEDS_GREEN);
  memcpy(&received_packet, packetbuf_dataptr(), sizeof(data_packet_struct));
  int rssi = (signed short)packetbuf_attr(PACKETBUF_ATTR_RSSI);
  process(received_packet.src_id, rssi);
}
static const struct broadcast_callbacks broadcast_call = {broadcast_recv};
static struct broadcast_conn broadcast;
/*---------------------------------------------------------------------------*/
char sender_scheduler(struct rtimer *t, void *ptr)
{
  static int value = 0;
  static uint16_t i = 0;
  PT_BEGIN(&pt);

  curr_timestamp = clock_time();
  while (1)
  {
    // radio on
    NETSTACK_RADIO.on();

    for (i = 0; i < NUM_SEND; i++)
    {
      leds_on(LEDS_RED);

      data_packet.seq++;
      curr_timestamp = clock_time();
      data_packet.timestamp = curr_timestamp;
      packetbuf_copyfrom(&data_packet, (int)sizeof(data_packet_struct));
      broadcast_send(&broadcast);
      leds_off(LEDS_RED);

      if (i != (NUM_SEND - 1))
      {
        rtimer_set(t, RTIMER_TIME(t) + WAKE_TIME, 1, (rtimer_callback_t)sender_scheduler, ptr);
        PT_YIELD(&pt);
      }
    }
    value++;
    check();

    leds_on(LEDS_BLUE);
    // radio off
    NETSTACK_RADIO.off();
    while (value % 11 != 0 && value % 17 != 0)
    {
      value++;
      rtimer_set(t, RTIMER_TIME(t) + SLEEP_SLOT, 1, (rtimer_callback_t)sender_scheduler, ptr);
      PT_YIELD(&pt);
    }
    leds_off(LEDS_BLUE);
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

  // initialize data packet
  data_packet.src_id = node_id;
  data_packet.seq = 0;

  // Start sender in one millisecond.
  rtimer_set(&rt, RTIMER_NOW() + (RTIMER_SECOND / 1000), 1, (rtimer_callback_t)sender_scheduler, NULL);

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
