/**
 *  @file   biped.cpp
 *  @author Simon Yu
 *  @date   01/12/2022
 */

/*
 *  External headers.
 */
#include <EEPROM.h>

#include "soc/timer_group_struct.h"
#include "soc/timer_group_reg.h"
#include "driver/gpio.h"

/*
 *  Project headers.
 */
#include "pin.h"
#include "neopixel.h"
#include "display.h"
#include "serial.h"
#include "MCP23018.h" //...........................

/*
 *  Use biped namespace.
 */
using namespace biped;

unsigned int id;

// TODO create IO expander instance

MCP23018 * io_expander_1;

// TODO declare execution flag(s)

volatile bool io_expander_a_interrupt_flag = false;
bool button_a_pressed = false, button_b_pressed = false, button_c_pressed = false;



void io_expander_a_interrupt_handler() {
    io_expander_a_interrupt_flag = true;
}

void
setup()
{
    //setup the display
    biped::Display::initialize() 
    Wire.setPins(ESP32Pin::i2c_sda, ESP32Pin::i2c_scl);
    biped::Serial::setLogLevelMax(SerialParameter::log_level_max);
    Wire.begin();
    EEPROM.begin(EEPROMParameter::size);
    id = static_cast<unsigned int>(EEPROM.read(AddressParameter::eeprom_serial_number));

    // TODO setup IO expander

    io_expander_1 = new MCP23018(0x00); 
    io_expander_1->begin();
    io_expander_1->SetDirections((1 << IOExpanderAPortAPin::pushbutton_a) || (1 << IOExpanderAPortAPin::pushbutton_b), 1 << IOExpanderAPortBPin::pushbutton_c);//........do i use set bits instead of set byte?
    io_expander_1->writeToRegister(IPOLA,0x00);//.....................not inverted pins
    io_expander_1->writeToRegister(IPOLB,0x00);
    io_expander_1->writeToRegister(INTENA,(1 << IOExpanderAPortAPin::pushbutton_a) || (1 << IOExpanderAPortAPin::pushbutton_b));
    io_expander_1->writeToRegister(INTENB,1 << IOExpanderAPortBPin::pushbutton_c);
    // we did not do stuff with the DEFV registers..........
    io_expander_1->writeToRegister(INTCONA,0x00);
    io_expander_1->writeToRegister(INTCONB,0x00);
    const uint8_t iocon_val = IOCON_MIRROR || IOCON_INTPOL || IOCON_INTCC;  //interrupt pin is active high............we needa read from the INTCAP registers....what if two buttons are pressed though....
    io_expander_1->writeToRegister(IOCON,iocon_val);
    // io_expander_1->SetPullups(uint8_t _a, uint8_t _b)   //setPullups for motors later...............................

    // TODO setup IO expnader interrupts

    attachInterrupt(ESP32Pin::io_expander_a_interrupt, io_expander_a_interrupt_handler, RISING);    //interrupt pin goes high whenever there is a change in input pin values
    readFromRegister(INTCAPA); //clears interrupt flag register
    readFromRegister(INTCAPB);

    Display(3) << "Button A Pressed: false";
    Display(4) << "Button B Pressed: false";
    Display(5) << "Button C Pressed: false";

    // TODO enable motor and set motor speed
}

void
loop()
{
    // TODO read IO expander and update display
    if (io_expander_a_interrupt_flag) {
        const uint8_t intcap_a_val = readFromRegister(INTCAPA);
        const uint8_t intcap_b_val = readFromRegister(INTCAPB);
        const uint8_t intf_a_val = readFromRegister(INTFA);
        const uint8_t intf_b_val = readFromRegister(INTFB);
        if (intf_a_val & (1 << IOExpanderAPortAPin::pushbutton_a) == (1 << IOExpanderAPortAPin::pushbutton_a)) {
            if (port_a_val & (1 << IOExpanderAPortAPin::pushbutton_a) == (1 << IOExpanderAPortAPin::pushbutton_a)) {
                button_a_pressed = false;   //active low
                Display(3) << "Button A Pressed: false";
                
            } else {
                button_a_pressed = true;
                Display(3) << "Button A Pressed: true";
            }
        } else if (intf_a_val & (1 << IOExpanderAPortAPin::pushbutton_b) == (1 << IOExpanderAPortAPin::pushbutton_b)) { // this assumes that INTCAP registers are always read before the next button interrupt happens
            if (port_a_val & (1 << IOExpanderAPortAPin::pushbutton_b) == (1 << IOExpanderAPortAPin::pushbutton_b)) {
                button_b_pressed = false;   //active low
                Display(4) << "Button B Pressed: false";
            } else {
                button_b_pressed = true;
                Display(4) << "Button B Pressed: true";
            }
        } else if (intf_b_val & (1 << IOExpanderAPortAPin::pushbutton_c) == (1 << IOExpanderAPortAPin::pushbutton_c)) {
            if (port_b_val & (1 << IOExpanderAPortAPin::pushbutton_c) == (1 << IOExpanderAPortAPin::pushbutton_c)) {
                button_c_pressed = false;
                Display(5) << "Button C Pressed: false";
            } else {
                button_c_pressed = true;
                Display(5) << "Button C Pressed: true";
            }
        } 
        io_expander_a_interrupt_flag = false;
    } 


    // TODO set motor direction
}
