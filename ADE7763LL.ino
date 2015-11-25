/**********************************************************************
 *
 * Low Level routines for ADE7763
 *
 *
 *********************************************************************/

#include "Pins.h"
#include "ADE7763Reg.h"
#include "ADE7763.h"
#include "ADE7763LL.h"



/**************************************************************************
 *   ADE_read
 *
 *   This routine enables the chip select and read the selected number
 *   of bytes from the ADE.
 *
 *   ADE is big endian and the Arduino is little. Therefore items are 
 *   assembled in revese order.
 *************************************************************************/
void ADE_read (unsigned char addr,
               unsigned char * data,
               unsigned char count)
{
  unsigned char i;
  
  /* If a 3 byte read, zero out the MSB of the 4 byte data */
  if (count == 3) {
    //*((unsigned long *)(data+4-1)) = 0;
  }

  /* Select the chip */
  digitalWrite(PIN_ADE_CS, LOW);
  delayMicroseconds(10);
  
  /* Point to the Arduino MSB */
  data += (count-1);

  /* Write the address to access */
  SPI.transfer(addr); 
                                     
  /* Must wait 4 us for data to become valid */
  delayMicroseconds(4);
  
  // Do for each byte in transfer
  for (i=0; i<count; i++)
  {

    /* Transer the byte */
    *data = SPI.transfer (0x00);
    data--;
  }

  /* Deselect the chip */
  digitalWrite(PIN_ADE_CS, HIGH);
  delayMicroseconds(10);
  

} /* end ADE_read */





/**************************************************************************
 *   ADE_write
 *   This routine enables chip select and reads the selected number of
 *   bytes from the ADE
 *
 *   ADE is big endian and the Arduino is little. Therefore items are 
 *   assembled in revese order.
 *************************************************************************/
void ADE_write(unsigned char addr,
               unsigned char * data,
               unsigned char count)
{
  unsigned char i;

  /* Select the chip */
  digitalWrite(PIN_ADE_CS, LOW);
  delayMicroseconds(10);
  
  /* Point to the Arduino MSB */
  data += (count-1);
  
  /* Write the address to access */
  SPI.transfer(addr | ADE_WRITE_FLAG);
                                 
  // Do for each byte */
  for (i=0; i<count; i++)
  {
    /* Transer the byte */
    SPI.transfer(*data);             // write all the bytes
    data--;
  }

  /* Deselect chip */
  digitalWrite(PIN_ADE_CS, HIGH);
  delayMicroseconds(10);

} /* end ADE_write */

