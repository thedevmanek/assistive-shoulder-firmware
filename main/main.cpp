extern "C" {
#include "nimbleprph.h"
}

#include "imu-sensor.h"


extern "C"
{
void app_main(void) {
    init_imu();
    init_ble();

}
}