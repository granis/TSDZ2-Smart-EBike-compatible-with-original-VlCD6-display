/*
 * TongSheng TSDZ2 motor controller firmware/
 *
 * Copyright (C) Casainho, 2018.
 *
 * Released under the GPL License, Version 3
 */

#ifndef _EEPROM_H_
#define _EEPROM_H_

#include "main.h"

void eeprom_init(void);
void eeprom_init_variables(void);
#if ENABLE_EEPROM_WRITE_IF_CHANGED
void eeprom_write_if_values_changed(void);
#endif

#endif /* _EEPROM_H_ */
