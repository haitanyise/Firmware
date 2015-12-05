#ifndef __SBUS_H_
#define __SBUS_H_

void sbus_data_handle(void);
int sbus_init(const char *device);
bool sbus_input(void);
void sbus1_output(uint16_t *values, uint16_t num_values);
void sbus2_output(uint16_t *values, uint16_t num_values);
/*
 * Debug logging
 */

#ifdef DEBUG
# include <debug.h>
# define debug(fmt, args...)	lowsyslog(fmt "\n", ##args)
#else
# define debug(fmt, args...)	do {} while(0)
#endif
#endif
