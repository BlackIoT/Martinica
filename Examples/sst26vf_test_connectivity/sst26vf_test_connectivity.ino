/*
 *  Created on: Feb 14, 2023
 *      Author: umberto@blackiot.ch
*/

#include <sst26vf_driver.h>

const uint8_t enable_pin = SS2;

auto chip = sst26vf::flash_driver(enable_pin);

void setup() {
    SerialUSB.begin(115200); while(!SerialUSB);

    SerialUSB.println("Begin chip communication test");
    SerialUSB.println("-----------------------------");

    chip.begin();

    uint8_t m, t, i;
    chip.get_manufacturer_info(&m, &t, &i);

    SerialUSB.println(m, HEX);
    SerialUSB.println(t, HEX);
    SerialUSB.println(i, HEX);
    
    SerialUSB.println("-----------------------------");
}

void loop() {
    asm __volatile__ ("nop");
}
