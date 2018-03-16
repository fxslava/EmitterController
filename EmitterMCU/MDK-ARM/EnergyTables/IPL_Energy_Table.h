#ifndef __IPL_ENRGY_TABLE_H
#define __IPL_ENRGY_TABLE_H

#include <stdint.h>

#define IPL_VOLTAGES_NUM	14
#define IPL_DURATIONS_NUM	26

#define IPL_VOLTAGES_MAX_NUM	16
#define IPL_DURATIONS_MAX_NUM	32

typedef struct IPL_Table_Struct {
	uint16_t IPL_Energy_Table  [IPL_VOLTAGES_MAX_NUM * IPL_DURATIONS_MAX_NUM];
	uint16_t IPL_Duration_Table[IPL_DURATIONS_MAX_NUM];
	uint16_t IPL_Voltage_Table [IPL_VOLTAGES_MAX_NUM];
} Fract1440_Table_TypeDef;

extern Fract1440_Table_TypeDef global_IPL_Table;

#endif
