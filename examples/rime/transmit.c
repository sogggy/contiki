/*---------------------------------------------------------------------------*/
#include "contiki.h"
#include "board-peripherals.h"
#include <stdio.h>
#include "net/rime/rime.h"
#include <stdio.h>
#include "sys/etimer.h"
#include "sys/rtimer.h"
/*---------------------------------------------------------------------------*/


static char message[50];
static int counter;
static struct rtimer transmission_freq_timer;
static rtimer_clock_t timeout_rtimer = RTIMER_SECOND / 4;  
static void send(char message[], int size);

PROCESS(transmit_process, "unicasting...");
AUTOSTART_PROCESSES(&transmit_process);

static const struct unicast_callbacks *3 = {};
static struct unicast_conn uc;

static void send(char message[], int size) {
	linkaddr_t addr;
	printf("%s, %d\n",message, strlen(message));
   packetbuf_copyfrom(message, size);

   // COMPUTE THE ADDRESS OF THE RECEIVER FROM ITS NODE ID, FOR EXAMPLE NODEID 0xBA04 MAPS TO 0xBA AND 0x04 RESPECTIVELY
   // In decimal, if node ID is 47620, this maps to 186 (higher byte) AND 4 (lower byte)
   addr.u8[0] = 0xE3;// HIGH BYTE or 186 in decimal
   addr.u8[1] = 0xC4; // LOW BYTE or 4 in decimal
   if (!linkaddr_cmp(&addr, &linkaddr_node_addr)) {
      unicast_send(&uc, &addr);
      counter++;
   }
}

void do_rtimer_timeout() {
   sprintf(&message, "packets: %d, message: Hi", counter);
   int s, ms1,ms2;
   s = clock_time() / CLOCK_SECOND;
   ms1 = (clock_time()% CLOCK_SECOND)*10/CLOCK_SECOND;
   ms2 = ((clock_time()% CLOCK_SECOND)*100/CLOCK_SECOND)%10;
   printf("%d.%d%d (sec): ", s, ms1, ms2); 
   send(message, strlen(message));
   rtimer_set(&transmission_freq_timer, RTIMER_NOW() + timeout_rtimer, 0,  do_rtimer_timeout, NULL);
}

PROCESS_THREAD(transmit_process, ev, data) {

	PROCESS_EXITHANDLER(unicast_close(&uc);)
	PROCESS_BEGIN();
   static struct etimer transmission_duration_timer;
	unicast_open(&uc, 146, &unicast_callbacks);
   rtimer_set(&transmission_freq_timer, RTIMER_NOW() + timeout_rtimer, 0,  do_rtimer_timeout, NULL);
   // printf("Set 10s timer\n");
   // etimer_set(&transmission_duration_timer, CLOCK_SECOND * 10);
   // PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_TIMER);
   
   


   PROCESS_END();
}

/*---------------------------------------------------------------------------*/
