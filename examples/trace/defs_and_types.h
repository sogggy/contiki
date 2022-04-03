/*---------------------------------------------------------------------------*/
#define NUM_SEND 2
/*---------------------------------------------------------------------------*/
enum node_status{Moving_Near, Moving_Away, In_Proximity, To_Remove};
typedef struct {
  unsigned long src_id;
  unsigned long timestamp;
  unsigned long seq;
} data_packet_struct;

typedef struct {
  unsigned long node_id;
  unsigned long detection_timestamp;
  unsigned long last_seen;
  enum node_status status;
} discovery_struct;
/*---------------------------------------------------------------------------*/
