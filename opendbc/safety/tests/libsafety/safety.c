#include <stdbool.h>

#include "../../board/fake_stm.h"
#include "../../board/can.h"

//int safety_tx_hook(CANPacket_t *msg) { return 1; }

#include "../../board/faults.h"
#include "../../safety.h"
#include "../../board/drivers/can_common.h"

// libsafety stuff
#include "safety_helpers.h"
