# bhi260ab_driver
A driver for configuring and reading the BHI260AB sensor hub.

# Usage
The following initaliation functions must be called during the startup:

* BHI260AB_Init()
* Your SPI initialization function

After the intalization, the following function must be constantly called for keeping the driver functioning:

* BHI260AB_Tasks()
* Your SPI routine

Before starting to read the sensor, the user must call the function BHI260AB_isReady() to be sure that the sensor initialized correctly.

## Configuring Virtual sensors

In the file `bhi260ab.h` you'll find the enumeration called *virtual_sensor_list_t*. There you must include all the virtual sensors that you want to use. 
Once all the sensors have been included in *virtual_sensor_list_t*, you have to go the `bhi260ab.c` file and then to the *BHI260AB_Init()* function, and there be sure that all the virtual sensors that you included in the *virtual_sensor_list_t* have an `.id` and `.data_type` assigned.

In addition, in the file `bhi260ab.c` there is a constant array called *sample_rate_config_list*, that array contains the sample rate to be configured to each virtual sensor included in the *virtual_sensor_list_t*. The order of the sample rates in that list si equivalent to the order present in *virtual_sensor_list_t*.
