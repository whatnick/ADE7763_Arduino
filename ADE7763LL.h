/**********************************************************************
 *
 * Inlcude file for low level ADE7763 routines
 *
 *
 *********************************************************************/

#ifndef __ADE7366LL_H__
#define __ADE7366LL_H__



/* Prototypes */

void ADE_read (unsigned char addr,
               unsigned char * data,
               unsigned char count);
               
void ADE_write(unsigned char addr,
               unsigned char * data,
               unsigned char count);

              

#endif

