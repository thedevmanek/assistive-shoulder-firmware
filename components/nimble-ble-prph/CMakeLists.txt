set(srcs "main.c")
idf_component_register(SRCS "nimble-ble-prph.c" "gatt_svr.c"
                       INCLUDE_DIRS "include"
                       REQUIRES  nvs_flash nimble_peripheral_utils bt
                       )
