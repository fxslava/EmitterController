#ifndef __FRACT_ENERGY_TABLE_H
#define __FRACT_ENERGY_TABLE_H

#include <stdint.h>

#define FRACT_MAX_NUM_ENERGY		16

#define FRACT1440NM_NUM_ENERGY	10
#define FRACT1340NM_NUM_ENERGY	7

typedef struct Fract1440_Table_Struct {
	uint16_t Fract1440_Duration_Table [3 * FRACT_MAX_NUM_ENERGY];
  uint16_t Fract1440_Voltage_Table  [3 * FRACT_MAX_NUM_ENERGY];
  uint16_t Fract1440_Energy_Table   [3 * FRACT_MAX_NUM_ENERGY];
} Fract1440_Table_TypeDef;

typedef struct Fract1340_Table_Struct {
	uint16_t Fract1340_Duration_Table [3 * FRACT_MAX_NUM_ENERGY];
  uint16_t Fract1340_Voltage_Table  [3 * FRACT_MAX_NUM_ENERGY];
  uint16_t Fract1340_Energy_Table   [3 * FRACT_MAX_NUM_ENERGY];
} Fract1340_Table_TypeDef;

extern Fract1440_Table_TypeDef global_Fract1440_Table;
extern Fract1340_Table_TypeDef global_Fract1340_Table;

#endif
