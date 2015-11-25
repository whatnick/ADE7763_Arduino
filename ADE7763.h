/**********************************************************************
 *
 * Include file for high level ADE7763 routines
 *
 *
 *********************************************************************/

#ifndef __ADE7366_H__
#define __ADE7366_H__

#include "PowerMeter.h"


/* Prototypes */
void          ADE_config(void);
void          ADE_resetPeaks(void);
char          ADE_sampleRMS(unsigned long *vrms, unsigned long *irms);
char          ADE_sample(char type, unsigned long *sample);
void          ADE_setInterrupt(unsigned int interrupt);
void          ADE_dump(void);
void          ADE_reset(void);
void          ADE_data();


#endif

