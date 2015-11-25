/**********************************************************************
 *
 * Home energy monitor using ADE7763
 *
 *
 * This program is used to monitor home electrical consumption by
 * instrumenting the electrical panel with current transformers (CT).
 *
 * This monitor is based on an Arduino for basic command and control of
 * the ADE7763 energy monitoring chip.
 * To support multiple CTs, this system has multiple sensor nodes (SN).
 * Each SN has a 4 position mux that routes four CT signals to the ADE.
 * To control the mux, a saml PIC (16F627A) sits on a data/clock bus that
 * the Arduino can drive to select the proper channel.
 *
 *********************************************************************/
 
#include <SPI.h>

#include "PowerMeter.h"
#include "Pins.h"
#include "ADE7763Reg.h"
#include "ADE7763.h"
#include "ADE7763LL.h"


unsigned long target;       /* Target time for next data dump */
char          cmd[32];      /* Command string for user interface */ 
unsigned char iCmd = 0;     /* Index into current command */
unsigned char run = 1;      /* Run data loop flag */

const long deltaT = 1000*2; /* Milli seconds to delay */

/**********************************************************************
 *
 * Set up routine
 *
 *********************************************************************/
void setup(void) {
  
  unsigned char revision;
  unsigned char uc;
  Node_t        node;
  
  /* Initialize the serial port to host */
  Serial.begin(9600);

  /****************************/
  /* Initialize SPI Bus       */
  /****************************/
  BUS_init();
  
  /*****************************/
  /* Initialize the ADE        */
  /*****************************/
  ADE_config(); 

  /* Set first target time for data dump */
  target = millis();
  
  Serial.println("setup complete");
  
  return;
  
  
} /* end setup */



/**********************************************************************
 *
 * Main loop
 *
 *********************************************************************/
void loop(void) {
  int           i, v, p;
  int           bi;
  char          b;
  unsigned char uc;
  unsigned int  ui;
  unsigned long ul;
  
  /* If the target time has come up */
  if ((millis() >= target) && run) {
    
    /* Wait for data to settle */
    delay(1000);
  
    /* Print the data */
    ADE_dump();
    
    /* Compute next target time (five seconds in future) */
    target = millis() + deltaT;

    
  } /* if target time */


  /* If we have a byte waitting */
  if (Serial.available() > 0) {

    /* Get the byte, exit if blank */
    bi = Serial.read();
    if (bi == -1) {
      return;
    }

    /* Truncate long commands */    
    if (iCmd == 31) bi = '\r';
    
    /* If this string has been terminated */
    if (bi != '\r') {
     
      /* Acculuate byte */
      cmd[iCmd] = (char)bi;
      iCmd++;
      
    } else {
      
      /* Null terminate the string */
      cmd[iCmd] = 0;
      
      /* Free up the fring for next command */
      iCmd = 0;
      
      /* Parse the string */
      Serial.print("Got cmd: ");
      Serial.println(cmd);
  
      /* Switch based on first byte */
      switch (cmd[0]) {
        
        case 'g':
          run = 0;
          break;
          
        case 'G':
          target = millis();
          run = 1;
          break;
        
        case 'r':
          /* Reset the ADE */
          Serial.println("Reset ADE");
          ADE_reset();
          break;
         
        case 'D':  
          Serial.println("Dump");
          ADE_dump();
          break;
  
        case 'R':        
          Serial.println("Reset");
          ADE_resetPeaks();
          break;
          
        case 'i':
          ul = 0;
          ADE_read(MR_IRMS,      (unsigned char *)&ul, MR_IRMS_CNT);
          Serial.print("IRMS 0x");
          Serial.println(ul, HEX);
          break;
            
        case 'v':
          ul = 0;
          ADE_read(MR_VRMS,      (unsigned char *)&ul, MR_VRMS_CNT);
          Serial.print("VRMS 0x");
          Serial.println(ul, HEX);
          break;
          
        case 'I':
          ul = 0;
          ADE_read(MR_IPEAK,      (unsigned char *)&ul, MR_IPEAK_CNT);
          Serial.print("IPEAK 0x");
          Serial.println(ul, HEX);
          break;
            
        case 'V':
          ul = 0;
          ADE_read(MR_VPEAK,      (unsigned char *)&ul, MR_VPEAK_CNT);
          Serial.print("VPEAK 0x");
          Serial.println(ul, HEX);
          break;
          
        case '.':
          /* Reset interrupt status */
          ADE_read (MR_IRQ,       (unsigned char *)&ui, MR_IRQ_CNT);
          Serial.print("IRQ: 0x:");
          Serial.println(ui, HEX);
          break;
  
          
        default:
          Serial.println("unknown command");
          break;
      }
      
      
    }
   

  } /* byte available */


} /* end loop */



