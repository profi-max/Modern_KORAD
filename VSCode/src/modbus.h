/*******************************************************************************
Created by profi-max (Oleg Linnik) 2024
https://profimaxblog.ru
https://github.com/profi-max

*******************************************************************************/
#ifndef _KORAD_MODBUS_H
#define _KORAD_MODBUS_H

#include <Arduino.h>
#include "common.h"

#define PROFILES_COUNT 20


typedef struct {
	uint16_t magicKey;
	uint16_t profileIdx;
	uint16_t param;
	uint16_t maxmem;
	uint16_t color1;
	uint16_t color2;
	uint16_t color3;
	uint16_t reserved;
} model_global_data_t;

typedef struct {
	uint16_t  uset;	
	uint16_t  iset;	
	uint16_t  sopp;	
	uint16_t  prfs;	
	uint16_t  otim;	
	uint16_t  reserved1;	
	uint16_t  reserved2;	
	uint16_t  reserved3;	
} model_profile_t;

void modbus_updateGlobalData(void);
void modbus_updateCurrentProfile(void);
void modbus_initData(void);
void modbus_loop(void);
void modbus_tcp_server_run(bool value);

#endif