/**********************************************************************
 *
 * Conrol of the Sensor Node bus
 *
 *
 *********************************************************************/


#include "Pins.h"


static Node_t lastNode = 0xff;

/**************************************************************************
 *   BUS_init
 *   Configures the bus to the ADE sensor nodes
 *************************************************************************/
void BUS_init(void)
{
  
  /* Configure Chip Select */    
  pinMode(PIN_ADE_CS,  OUTPUT);
  digitalWrite(PIN_ADE_CS, HIGH);  // Deselect
  delayMicroseconds(10);
  
  /* Configure Reset Select */    
  pinMode(PIN_ADE_RST,  OUTPUT);
  digitalWrite(PIN_ADE_RST, HIGH);  // Deselect
  delayMicroseconds(10);
  
  /* Interrupt */
  pinMode(PIN_ADE_INT, INPUT);
  digitalWrite(PIN_ADE_INT, HIGH);  // Enable pullup on ~INT line
  
  /* Now that IRQ register is set and cleared, attach the ISR */
  attachInterrupt(0, ADE_isr, FALLING);
  
  /* Enable SPI */  
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE1);   // clk idle low, sample falling edge
  SPI.setClockDivider(SPI_CLOCK_DIV4);
  
  return;                    

} /* end ADE_config */




