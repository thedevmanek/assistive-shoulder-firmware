#ifndef NIMBLE_H
#define NIMBLE_H
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "nvs_flash.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "host/ble_uuid.h"
#include "console/console.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "bleprph.h"
#include "nimbleprph.h"

extern TaskHandle_t xHandle;
extern char *notification;
extern bool notify_state;
void gatt_svr_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg);
//static int bleprph_gap_event(struct ble_gap_event *event, void *arg);
void sendNotification();
void vTasksendNotification();
void startBLE();
void stopBLE();
void startNVS();
void init_ble();
#endif


