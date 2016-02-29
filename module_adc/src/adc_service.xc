#include <xs1.h>
#include <adc_service.h>
#include <adc_ad7265.h>
#include <adc_ad7949.h>

void adc_service(ADCPorts &adc_ports, ADCConfig adc_config, chanend ?c_trigger, interface ADCInterface server i_adc[2]){

    //Set freq to 250MHz (always needed for proper timing)
    write_sswitch_reg(get_local_tile_id(), 8, 1); // (8) = REFDIV_REGNUM // 500MHz / ((1) + 1) = 250MHz

    printstr(">>   SOMANET ADC SERVICE STARTING...\n");

    if(isnull(c_trigger)){ // Check for triggered sampling channel

        switch (adc_config.adc_type)
        {
            case AD7265:
                adc_ad7256(i_adc, adc_ports.ad7265_ports, adc_ports.current_sensor_config);
                break;
            case AD7949:
                adc_ad7949(i_adc, adc_ports.ad7949_ports, adc_ports.current_sensor_config);
                break;
            default:
                printstr("adc_service: ERROR No ADC configured");
                break;
        }
    } else {

        switch (adc_config.adc_type)
        {
            case AD7265:
                adc_ad7256_triggered(i_adc, adc_ports.ad7265_ports, adc_ports.current_sensor_config, c_trigger);
                break;
            case AD7949:
                adc_ad7949_triggered(i_adc, adc_ports.ad7949_ports, adc_ports.current_sensor_config, c_trigger);
                break;
            default:
                printstr("adc_service: ERROR No ADC configured");
                break;
        }
    }
}
