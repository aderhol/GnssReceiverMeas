//std library includes
#include <stdint.h>
#include <stdbool.h>

//FreeRTOS includes
#include <FreeRTOS.h>

//driver includes

//user includes
#include <senHubI2c.h>
#include <sen_bmp180.h>
#include <sen_ISL29023.h>
#include <sen_sht21.h>



//configurations


extern void senHubI2cInit(void);

void sensorHubInit(void)
{
    senHubI2cInit(); //I2C for the sensors

    bmp180Init(); //temperature and pressure
    isl29023Init(); //visible and IR light intensity
    sht21Init(); //relative humidity and temperature
}
