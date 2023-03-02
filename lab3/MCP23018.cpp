/*
  Copyright (C) 2011 James Coliz, Jr. <maniacbug@ymail.com>
 
  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License
  version 2 as published by the Free Software Foundation.
*/

/* #include <Arduino.h> */
#include <Wire.h>
#include "MCP23018.h"

int MCP23018::writeToRegister(uint8_t addr, uint8_t data) {
	Wire.beginTransmission(i2c_address);
	Wire.write(addr);
	Wire.write(data);
	int ret = Wire.endTransmission();
	if (ret != 0) {
		return -ret;
	}

	return 0;
}

int MCP23018::writePairToRegister(uint8_t addr, uint8_t first_data, uint8_t second_data) {
	Wire.beginTransmission(i2c_address);
	Wire.write(addr);
	Wire.write(first_data);
	Wire.write(second_data);
	int ret = Wire.endTransmission();
	if (ret != 0) {
		return -ret;
	}
	
	return 0;
}

int MCP23018::readPairFromRegister(uint8_t addr) {//..................when do we use pairs......
	uint8_t reg_a, reg_b;

	Wire.beginTransmission(i2c_address);
	Wire.write(addr);
	int ret = Wire.endTransmission(false);
	if (ret != 0) {
		return -ret;
	}
	
	// Request one data byte
	Wire.requestFrom(i2c_address, (uint8_t) 2);

    reg_a = Wire.read();
    reg_b = Wire.read();

	return reg_a | (reg_b << 8);
}

int MCP23018::readFromRegister(uint8_t addr) {
	uint8_t received_data = 0;
	
	// Establish connection, select receipt addr
	Wire.beginTransmission(i2c_address);
	Wire.write(addr);
	int ret = Wire.endTransmission(false);
	if (ret != 0) {
		return -ret;
	}
	
	// Request one data byte
	Wire.requestFrom(i2c_address, (uint8_t) 1);
	
	// Fill variables when ready
    received_data = Wire.read();
	
	return received_data;
}

int MCP23018::setMaskInRegister(uint8_t addr, uint8_t mask, bool state) {
	uint8_t temp;
	const int prev = readFromRegister(addr);
    if (prev < 0) {
        return prev;
    }
    const uint8_t prev_port = prev;
	const uint8_t res = state ? (prev_port | mask) : (prev_port & ~mask);
	return writeToRegister(addr, res);
}

int MCP23018::setBitInRegister(uint8_t addr, uint8_t bit, bool state) {
	uint8_t temp;
	const int prev = readFromRegister(addr);
    if (prev < 0) {
        return prev;
    }
    const uint8_t prev_port = prev;
	const uint8_t bit_mask = 1 << bit;
	const uint8_t res = state ? (prev_port | bit_mask) : (prev_port & ~bit_mask);
	return writeToRegister(addr, res);
}

MCP23018::MCP23018(uint8_t _address) {
	i2c_address = ( _address & 0x07 ) | MCP23018_ADDR;
}

int MCP23018::begin() {
	Wire.beginTransmission(i2c_address);
	return -Wire.endTransmission();
}

int MCP23018::SetDirections(uint8_t _a, uint8_t _b) { 
	return writeToRegister(IODIRA,a) & writeToRegister(IODIRB,b);
}

int MCP23018::SetPullups(uint8_t _a, uint8_t _b) { 
	return writeToRegister(GPPUA,a) & writeToRegister(GPPUB,b);
}

int MCP23018::SetPortA(uint8_t _data) { 
	return writeToRegister(GPIOA,data);	//............or is it OLATA 
}

int MCP23018::SetPortB(uint8_t _data) { 
	return writeToRegister(GPIOB,data);
}

int MCP23018::GetPortA() { 
	return readFromRegister(GPIOA);	//..............or is it INTCAP
}

int MCP23018::GetPortB() { 
	return readFromRegister(GPIOB);
}

int MCP23018::GetLatchPortA() { 
	return readFromRegister(OLATA);
}

int MCP23018::GetLatchPortB() { 
	return readFromRegister(OLATB);
}

int MCP23018::SetAPin(uint8_t pin, bool state) { 
	return setBitInRegister(GPIOA,pin,state);	//hopefully pin = bit
}

int MCP23018::SetBPin(uint8_t pin, bool state) { 
	return setBitInRegister(GPIOB,pin,state);
}
