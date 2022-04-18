#ifndef __MAIN_H__
#define __MAIN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* ------> INCLUDES <------ */
#include <stdio.h>
#include <inttypes.h>

/* ------> MACROS <------ */

/* ------> DATA TYPES <------ */

/* ------> PUBLIC FUNCTION PROTOTYPES <------ */

void
print_bytes(const uint8_t *bytes, int len);

void
print_addr(const void *addr);

void
blehr_tx_hrate(uint8_t * buff, uint8_t buff_size);


#ifdef __cplusplus
}
#endif

#endif // __MAIN_H__
