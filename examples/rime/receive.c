#include "contiki.h"
#include "net/rime/rime.h"
#include <stdio.h>
#include "net/netstack.h"

int counter = 0;

/*---------------------------------------------------------------------------*/
PROCESS(example_unicast_process, "unicast receiver");
AUTOSTART_PROCESSES(&example_unicast_process);
/*---------------------------------------------------------------------------*/
static void recv_uc(struct unicast_conn *c, const linkaddr_t *from){
  char message[50];
  strcpy(message,(char *)packetbuf_dataptr());
  message[packetbuf_datalen()]='\0';
  printf("counter: %d, message: %s, RSSI: %d\n", counter, message, (signed short)packetbuf_attr(PACKETBUF_ATTR_RSSI));
  counter++;
}

static const struct unicast_callbacks unicast_callbacks = {recv_uc};
static struct unicast_conn uc;
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(example_unicast_process, ev, data){
  PROCESS_EXITHANDLER(unicast_close(&uc);)

  PROCESS_BEGIN();

  unicast_open(&uc, 146, &unicast_callbacks);

  while(1) {
    static struct etimer et;
    etimer_set(&et, CLOCK_SECOND / 4);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
