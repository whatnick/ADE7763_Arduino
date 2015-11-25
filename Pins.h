/**********************************************************************
 *
 * Pin definitions for Home Energy Monitor
 *
 *
 *********************************************************************/

#ifndef __Pins_H__
#define __Pins_H__


/***************************************
 * Pin assignments
 **************************************/
//const char PIN_RX       = 0;  // UART RX  
//const char PIN_TX       = 1;  // UART TX
const char PIN_ADE_INT  = 2;  // Interrupt from ADE
const char PIN_ADE_RST  = 3;  // ~Reset to ADE
const char PIN_ADE_CS   = 10;  // SPI ~chip select to ADE    
const char PIN_ADE_CLK  = 13;  // SPI clock to ADE
const char PIN_ADE_MISO = 12;  // SPI MISO from ADE 
const char PIN_ADE_MOSI = 11;  // SPI MOSI from ADE  


#endif

