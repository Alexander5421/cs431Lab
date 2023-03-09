/**
 *  @file   biped.cpp
 *  @author Simon Yu
 *  @date   01/12/2022
 */

/*
 *  External headers.
 */
#include <EEPROM.h>

/*
 *  Project headers.
 */
#include "pin.h"
#include "neopixel.h"
#include "display.h"
#include "serial.h"


#include "soc/timer_group_struct.h" //................
#include "soc/timer_group_reg.h"
#include "driver/gpio.h"
#include "MCP23018.h" 
#include <string>
#define STACK_SIZE 2048
#define LONG_TIME 0xffff
using namespace biped;


/*
 *  Use biped namespace.
 */

unsigned int id;

// TODO create IO expander instance

MCP23018 * io_expander_1;

// TODO declare synchronization primitives

SemaphoreHandle_t button_press_release_Semaphore = NULL, actuating_motors_Semaphore = NULL, button_status_count_display_Semaphore = NULL;
SemaphoreHandle_t mutex = NULL;

bool button_a_pressed = false, button_b_pressed = false, button_c_pressed = false;
int button_a_pressed_num_times = 0, button_b_pressed_num_times = 0, button_c_pressed_num_times = 0;
//..........................................

void io_expander_a_interrupt_handler() {
    // biped::Serial(LogLevel::info) << "here7";
    xSemaphoreGiveFromISR(button_press_release_Semaphore, NULL);
}

// TODO define tasks

void button_press_release_task(void * pvParameters/*...*/) {
    for (;;) {
        if (xSemaphoreTake(button_press_release_Semaphore, LONG_TIME) == pdTRUE) {
            biped::Serial(LogLevel::info) << "here7";
            uint8_t intf_a_val = io_expander_1->readFromRegister(INTFA);
            uint8_t intf_b_val = io_expander_1->readFromRegister(INTFB);
            uint8_t intcap_a_val = io_expander_1->readFromRegister(INTCAPA);
            uint8_t intcap_b_val = io_expander_1->readFromRegister(INTCAPB);
            if(xSemaphoreTake(mutex, LONG_TIME) == pdTRUE){
                biped::Serial(LogLevel::info) << "here8";
                 if ((intf_a_val & (1 << IOExpanderAPortAPin::pushbutton_a)) == (1 << IOExpanderAPortAPin::pushbutton_a)) {
                    if ((intcap_a_val & (1 << IOExpanderAPortAPin::pushbutton_a)) == (1 << IOExpanderAPortAPin::pushbutton_a)) {
                        button_a_pressed = false;   //active low
                    } else {
                        button_a_pressed = true;
                        button_a_pressed_num_times++;
                    }   

                } else if ((intf_a_val & (1 << IOExpanderAPortAPin::pushbutton_b)) == (1 << IOExpanderAPortAPin::pushbutton_b)) { // this assumes that INTCAP registers are always read before the next button interrupt happens
                    if ((intcap_a_val & (1 << IOExpanderAPortAPin::pushbutton_b)) == (1 << IOExpanderAPortAPin::pushbutton_b)) {
                        button_b_pressed = false;   //active low
                    } else {
                        button_b_pressed = true;
                        button_b_pressed_num_times++;
                    }
                } else if ((intf_b_val & (1 << IOExpanderAPortBPin::pushbutton_c)) == (1 << IOExpanderAPortBPin::pushbutton_c)) {
                    if ((intcap_b_val & (1 << IOExpanderAPortBPin::pushbutton_c)) == (1 << IOExpanderAPortBPin::pushbutton_c)) {
                        button_c_pressed = false;
                    } else {
                        button_c_pressed = true;
                        button_c_pressed_num_times++;
                    }
                } 
                xSemaphoreGive(mutex);
            }
            biped::Serial(LogLevel::info) << "here9";
            //put here 
            xSemaphoreGive(actuating_motors_Semaphore);
            xSemaphoreGive(button_status_count_display_Semaphore);
            biped::Serial(LogLevel::info) << "here1";
        } 
    }
}

void actuating_motors_task(void * pvParameters/*...*/) {
    for (;;) { /*...........*/
        if (xSemaphoreTake(actuating_motors_Semaphore, LONG_TIME) == pdTRUE) {
            switch (button_a_pressed_num_times%4){
                case 0:
                // both forward
                io_expander_1->setBitInRegister(GPIOA,IOExpanderAPortAPin::motor_left_direction,true);
                io_expander_1->setBitInRegister(GPIOA,IOExpanderAPortAPin::motor_right_direction,true);
                break;
                case 1: 
                // both backward
                io_expander_1->setBitInRegister(GPIOA,IOExpanderAPortAPin::motor_left_direction,false);
                io_expander_1->setBitInRegister(GPIOA,IOExpanderAPortAPin::motor_right_direction,false);
                break;
                case 2:
                // left forward, right backward
                io_expander_1->setBitInRegister(GPIOA,IOExpanderAPortAPin::motor_left_direction,true);
                io_expander_1->setBitInRegister(GPIOA,IOExpanderAPortAPin::motor_right_direction,false);
                break;
                case 3:
                // left backward, right forward
                io_expander_1->setBitInRegister(GPIOA,IOExpanderAPortAPin::motor_left_direction,false);
                io_expander_1->setBitInRegister(GPIOA,IOExpanderAPortAPin::motor_right_direction,true);
                break;
            }
        }
    }
}

void button_status_count_display_task(void * pvParameters/*...*/) {
    for (;;) {
        if ((xSemaphoreTake(button_status_count_display_Semaphore, LONG_TIME) == pdTRUE) && (xSemaphoreTake(mutex, LONG_TIME) == pdTRUE)) {
            Display(0) << "Button A Pressed: " << (button_a_pressed?"true":"false") ;
            Display(1) << "Count: " << button_a_pressed_num_times;
            Display(2) << "Button B Pressed: " << (button_b_pressed?"true":"false");
            Display(3) << "Count: " << button_b_pressed_num_times;
            Display(4) << "Button C Pressed: " << (button_c_pressed?"true":"false");
            Display(5) << "Count: " << button_c_pressed_num_times;
            Display::display();
            xSemaphoreGive(mutex);
            
        }
    }
}

void
setup()
{
    biped::Serial::initialize(); //......................
    Wire.setPins(ESP32Pin::i2c_sda, ESP32Pin::i2c_scl);
    biped::Serial::setLogLevelMax(SerialParameter::log_level_max);
    Wire.begin();
    EEPROM.begin(EEPROMParameter::size);
    id = static_cast<unsigned int>(EEPROM.read(AddressParameter::eeprom_serial_number));
    Display::initialize(); //...............
    button_press_release_Semaphore = xSemaphoreCreateBinary(), //..................
    actuating_motors_Semaphore = xSemaphoreCreateBinary(), 
    button_status_count_display_Semaphore = xSemaphoreCreateBinary();
    mutex = xSemaphoreCreateMutex();


    // TODO setup IO expander

    io_expander_1 = new MCP23018(0x00); 
    io_expander_1->begin();
    io_expander_1->SetDirections((1 << IOExpanderAPortAPin::pushbutton_a) | (1 << IOExpanderAPortAPin::pushbutton_b), 1 << IOExpanderAPortBPin::pushbutton_c);//........do i use set bits instead of set byte?
    io_expander_1->writeToRegister(IPOLA,0x00);//.....................not inverted pins
    io_expander_1->writeToRegister(IPOLB,0x00);
    // we did not do stuff with the DEFV registers..........
    io_expander_1->writeToRegister(INTCONA,0x00);
    io_expander_1->writeToRegister(INTCONB,0x00);
    const uint8_t iocon_val = IOCON_MIRROR | IOCON_INTPOL | IOCON_INTCC;  //interrupt pin is active high............we needa read from the INTCAP registers....what if two buttons are pressed though....
    io_expander_1->writeToRegister(IOCON,iocon_val);
    io_expander_1->SetPullups((1 << IOExpanderAPortAPin::pushbutton_a) | (1 << IOExpanderAPortAPin::pushbutton_b), (1 << IOExpanderAPortBPin::pushbutton_c) | (1 << IOExpanderAPortBPin::motor_enable));   //setPullups for motors later...............................
    io_expander_1->setBitInRegister(GPPUA,IOExpanderAPortAPin::motor_left_direction,true);
    io_expander_1->setBitInRegister(GPPUA,IOExpanderAPortAPin::motor_right_direction,true);

    // TODO setup IO expnader interrupts

    attachInterrupt(ESP32Pin::io_expander_a_interrupt, io_expander_a_interrupt_handler, RISING);    //interrupt pin goes high whenever there is a change in input pin values
    
    io_expander_1->writeToRegister(INTENA,(1 << IOExpanderAPortAPin::pushbutton_a) | (1 << IOExpanderAPortAPin::pushbutton_b));
    io_expander_1->writeToRegister(INTENB,1 << IOExpanderAPortBPin::pushbutton_c);
    io_expander_1->readFromRegister(INTCAPA); //clears interrupt flag register
    io_expander_1->readFromRegister(INTCAPB);

    Display(0) << "Button A Pressed: false";
    Display(1) << "Count: " << button_a_pressed_num_times;
    Display(2) << "Button B Pressed: false";
    Display(3) << "Count: " << button_b_pressed_num_times;
    Display(4) << "Button C Pressed: false";
    Display(5) << "Count: " << button_c_pressed_num_times;
    Display::display();

    // TODO enable motor and set motor speed

    io_expander_1->setBitInRegister(GPIOB,IOExpanderAPortBPin::motor_enable, true); 

    int level = 20;
    analogWrite(ESP32Pin::motor_left_pwm,level);
    analogWrite(ESP32Pin::motor_right_pwm,level);

    // TODO setup tasks

    BaseType_t button_press_release_task_Returned, actuating_motors_task_Returned, button_status_count_display_task_Returned;

    button_press_release_task_Returned = xTaskCreate(button_press_release_task, "Button Press Release", STACK_SIZE /*.....*/, ( void * ) 1 /*.....*/, (1|portPRIVILEGE_BIT) /*...*/, NULL);
    actuating_motors_task_Returned = xTaskCreate(actuating_motors_task, "Actuating Motors", STACK_SIZE /*.....*/, ( void * ) 1 /*.....*/, (1|portPRIVILEGE_BIT) /*...*/, NULL);
    button_status_count_display_task_Returned = xTaskCreate(button_status_count_display_task, "Button Status Count Display", STACK_SIZE /*.....*/, ( void * ) 1 /*.....*/, (1|portPRIVILEGE_BIT) /*...*/, NULL);
}

void
loop()
{
    delay(1000);
}
