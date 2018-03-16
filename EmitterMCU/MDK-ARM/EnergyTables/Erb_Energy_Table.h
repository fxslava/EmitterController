#ifndef __ERB_ENRGY_TABLE_H
#define __ERB_ENRGY_TABLE_H

#include <stdint.h>

#define ERB_MAX_NUM_ENERGY 16
#define ERB_VOLTAGES_NUM	 11

typedef struct Erb_Table_Struct {
	uint16_t Erb_Duration_Table[3 * ERB_MAX_NUM_ENERGY];
  uint16_t Erb_Voltage_Table [3 * ERB_MAX_NUM_ENERGY];
  uint16_t Erb_Energy_Table  [3 * ERB_MAX_NUM_ENERGY];
} Erb_Table_TypeDef;

extern Erb_Table_TypeDef global_Erb_Table;

#endif
