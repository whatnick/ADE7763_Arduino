/**********************************************************************
 *
 * High level ADE7763 routines for Home energy monitor
 *
 *
 *********************************************************************/

#include "Pins.h"
#include "ADE7763Reg.h"
#include "ADE7763.h"
#include "ADE7763LL.h"


#define SAMPLE_TIME 100   /* ms to wait for sample data */
//#define VRMS_GAIN (1.0/6525.0)  // BLUE Clip on
//#define IRMS_GAIN (1.0/185230.0)  // BLUE Clip on
//#define VRMS_GAIN (120.0/768000.0)
//#define IRMS_GAIN (350.0/120.0/156400.0)

/* Local variables */
unsigned long sample_;          // Generic sample
unsigned long temp_;            // Temperature
unsigned long vrms_;            // Vrms sampled at ZX
unsigned long irms_;            // Irms sampled at ZX
char          grabData_   = 0;  // Flag used to sample RMS data
char          sampleDone_ = 0;  // Flag used to sample waveform

/* Gains for each voltage sensor. These are all the same right now */
float  vGain[] = {120.0/768000.0,
                  120.0/768000.0,
                  120.0/768000.0,
                  120.0/768000.0,
                  120.0/768000.0,
                  120.0/768000.0,
                  120.0/768000.0,
                  120.0/768000.0};

/* Gains for each CT. */
float iGain[] = {350.0/120.0/156400.0,
                 1.0/185230.0,
                 350.0/120.0/156400.0,
                 350.0/120.0/156400.0,
                 350.0/120.0/156400.0,
                 350.0/120.0/156400.0,
                 350.0/120.0/156400.0,
                 350.0/120.0/156400.0};

/**************************************************************************
 * ADE_config
 * Performs a hardware and software reset of the chip
 *
 * @return void
 *************************************************************************/
void ADE_config(void)
{
  unsigned int    ui;
  unsigned char   uc;
  
  /* Reset the ADEs */
  ADE_reset();
  
  /* Perform a software reset */
  ADE_read  (MR_MODE, (unsigned char *)&ui, MR_MODE_CNT);
  ui |= MODE_SWRST;
  ADE_write (MR_MODE, (unsigned char *)&ui, MR_MODE_CNT);
  delayMicroseconds (200); // Wait for reset to take effect
 
  /* Get die version */
  ADE_read (MR_DIEREV, (unsigned char *)&uc, MR_DIEREV_CNT);
  
  /* Write to Mode register */
  ui = 0x00 |
       //MODE_DISHPF    |   // Disable HPF in Channel 1
       //MODE_DISLPF2   |   // Disable LPF after the multiplier (LPF2)
       MODE_DISCF     |   // Disable frequency output
       MODE_DISSAG    |   // Disable line voltage sag detection
       //MODE_ASUSPEND  |   // Disable A/D converters
       //MODE_TEMPSEL   |   // Start temperature conversion
       //MODE_SWRST     |   // Software Chip Reset.
       //MODE_CYCMODE   |   // Enable line cycle accumulation mode
       //MODE_DISCH1    |   // Short out Chan1 (Current)
       //MODE_DISCH2    |   // Short out Chan2 (Voltage)
       //MODE_SWAP      |   // Swap Chan1 and Chan2
       //MODE_DTRT_3K5  |   // Waverform data rate to 3.5ksps
       MODE_WAV_POWER |   // Sample active power
       //MODE_POAM      |   // Accumulated positive power only
       0x00;                        

  /* Set the mode */
  ADE_write (MR_MODE, (unsigned char *)&ui, MR_MODE_CNT);
 
  /* Write to interrupt enable register */
  ui = IRQ_NONE;
  ADE_write (MR_IRQEN, (unsigned char *)&ui, MR_IRQEN_CNT);

  /* Reset interrupt status (with reset) */
  ADE_read (MR_RSTIRQ, (unsigned char *)&ui, MR_RSTIRQ_CNT);
 
  /* Set up the gain register */
  uc = 0x00;  // ADC1,2 gain = 1 full scale range is +-0.5V
  ADE_write (MR_GAIN, (unsigned char *)&uc, MR_GAIN_CNT);
  
  /* Set up the offset correction for ADC1 */
  uc = 0x00;
  ADE_write (MR_CH1OS, (unsigned char *)&uc, MR_CH1OS_CNT);
  
  /* Set up the offset correction for ADC2 */
  uc = 0x00;
  ADE_write (MR_CH2OS, (unsigned char *)&uc, MR_CH2OS_CNT);

  return;                    

} /* end ADE_init */


/**************************************************************************
 *   ADE_reset
 *************************************************************************/
void ADE_reset(void) {
    
  /* Strobe the ADE reset line */
  digitalWrite(PIN_ADE_RST, 0);
  delay(20);
  digitalWrite(PIN_ADE_RST, 1);
  delay(20);

}

/**************************************************************************
 * ADE_setInterrupt
 * Performs a hardware and software reset of the chip
 *
 * @return void
 *************************************************************************/
void ADE_setInterrupt(unsigned int interrupt)
{
  unsigned int    ui;
 
  /* Write to interrupt enable register */
  ADE_write (MR_IRQEN, (unsigned char *)&interrupt, MR_IRQEN_CNT);

  /* Reset interrupt status (with reset) */
  ADE_read (MR_RSTIRQ, (unsigned char *)&ui, MR_RSTIRQ_CNT);
  
  /* At this we will respond to the interrupt requested */
  
}



/**********************************************************************
 *
 * Reset peak values
 *
 * @return void
 *
 *********************************************************************/
void ADE_resetPeaks(void) {
  unsigned char uc;
  unsigned int  ui;
  unsigned long ul;
  
  ul = 0;
  ADE_read(MR_RAENERGY,  (unsigned char *)&ul, MR_RAENERGY_CNT);
  Serial.println("Reset AENERGY");
  
  ul = 0;
  ADE_read(MR_RVAENERGY, (unsigned char *)&ul, MR_RVAENERGY_CNT);
  Serial.println("Reset VAENERGY");
  
  ui = 0;
  ADE_read(MR_RSTIRQ,    (unsigned char *)&ui, MR_RSTIRQ_CNT);
  Serial.println("Reset IRQ");
  
  ul = 0;
  ADE_read(MR_RSTIPEAK,  (unsigned char *)&ul, MR_RSTIPEAK_CNT);
  Serial.println("Reset IPEAK");
  
  ul = 0;
  ADE_read(MR_RSTVPEAK,  (unsigned char *)&ul, MR_RSTVPEAK_CNT);
  Serial.println("Reset VPEAK");

} /* end ADE_resetPeaks */


/**************************************************************************
 * ADE_Sample
 * 
 * This routine will set the proper sample mode in the Mode register, then
 * set the flag for the ISR to capture the value at the next zero crossing.
 * This routine will block for a period of time should the sample not complete.
 *
 *  @param type   The type of sample to collect. Select from MODE_WAV_POWER, 
                                                             MODE_WAV_RESERVED, 
                                                             MODE_WAV_ADC1, or
                                                             MODE_WAV_ADC2
 *  @param sample Pointer to a location to place the sample value
 *  @return 0 for error, 1 for success
 *************************************************************************/
char ADE_sample(char type, unsigned long *sample) {
  unsigned int ui;
  char         i;
  char         retval;
  
 
  /* Set sample Mode */
  ui = 0;
  ADE_read  (MR_MODE, (unsigned char *)&ui, MR_MODE_CNT);
  ui &= ~(MODE_WAVSEL1 | MODE_WAVSEL0);
  ui |= type;
  ADE_write (MR_MODE, (unsigned char *)&ui, MR_MODE_CNT);
  
  /* Enable sample mode */
  sampleDone_ = 0;
  ui = 0;
  ADE_read  (MR_IRQEN, (unsigned char *)&ui, MR_IRQEN_CNT);
  ui |= IRQ_WSMP;
  ADE_write (MR_IRQEN, (unsigned char *)&ui, MR_IRQEN_CNT);
  
  /* Enable the Sample interrupt */
  ADE_setInterrupt(IRQ_WSMP);
  
  /* Wait for sample to come in */
  for (i=0; i<SAMPLE_TIME; i++) {
    if (sampleDone_ ==  0) break;
    delay(1);
  }

  /* If sample did not come in */
  if (i == SAMPLE_TIME) {
    *sample = 0;
    retval = 0;
    Serial.println("No data");
  } else {
    *sample = sample_;
    retval = 1;
  }
  
  /* Enable the Sample interrupt */
  ADE_setInterrupt(IRQ_NONE);
  
  /* Return the error code */
  return (retval);
  
} /* end ADE_sample */



/**************************************************************************
 * ADE_sampleRMS
 *
 * @param vmrs pointer to returned rms voltage
 * @param imrs pointer to returned rms current
 * @return 0 for fail, 1 for pass
 *
 *************************************************************************/
char ADE_sampleRMS(unsigned long *vrms, unsigned long *irms) {
  unsigned int ui;
  int          i;
  char         retval;
  
  /* Enable the ZX interrupt */
  ADE_setInterrupt(IRQ_ZX);
  delay(18); // Wait for a few zero crossing to go by

  /* Set flag used in ISR to capture RMS data at next zero crossings */
  grabData_ = 1;
  
  /* Wait for sample to come in */
  for (i=0; i<SAMPLE_TIME; i++) {
    delay(1);
    if (grabData_ == 0) break;
  }
  
  /* Turn off interrupts */
  ADE_setInterrupt(IRQ_NONE);
  
  if (i == SAMPLE_TIME) {
    *irms = 0;
    *vrms = 0;
    retval = 0;
    Serial.println("No data2");
  } else {
    *irms = irms_;
    *vrms = vrms_;
    retval = 1;
  }
  
  return(retval);
  
} /* end ADE_sampleRMS */


/**************************************************************************
 *   ADE_isr
 *
 *  This is the interrupt service routine that gets called each time the 
 *  selected ADE generates an interrupt. Note that no correlation between
 * selected node is done here.
 *
 *  It will read the IRQ register and perform the needed operation based
 *  on what caused the interrupt.
 *
 *  @return void
 *************************************************************************/
void ADE_isr(void) {
  unsigned int irqstat;
  unsigned int irqen;
 
  /* Read IRQ Status and reset */
  irqstat = 0;
  ADE_read (MR_RSTIRQ, (unsigned char *)&irqstat, MR_RSTIRQ_CNT);
  
  /* Active energy more than 1/2 full */
  if (irqstat & IRQ_AEHF) {
  }
    
  /* Voltage sag */
  if (irqstat & IRQ_SAG) {
  }
    
  /* Energy integration over n counts done */  
  if (irqstat & IRQ_CYCEND) {
  }
    
  /* Sample available */  
  if (irqstat & IRQ_WSMP) {
    
    /* Read waveform register to get sample */
    sample_ = 0;
    ADE_read (MR_WAVEFORM, (unsigned char *)&sample_, MR_WAVEFORM_CNT);
    
    /* Clear bit in interrupt enable register to stop sampling */
    ADE_read  (MR_IRQEN, (unsigned char *)&irqen, MR_IRQEN_CNT);
    irqen &= ~IRQ_WSMP;
    ADE_write (MR_IRQEN, (unsigned char *)&irqen, MR_IRQEN_CNT);
    
    sampleDone_ = 1;

  }
    
  /* Zero crossing done */  
  if (irqstat & IRQ_ZX) {
    
    /* If we are to sample the RMS data */
    if (grabData_) {
      irms_ = 0;
      ADE_read(MR_IRMS,      (unsigned char *)&irms_, MR_IRMS_CNT);
      vrms_ = 0;
      ADE_read(MR_VRMS,      (unsigned char *)&vrms_, MR_VRMS_CNT);
      grabData_ = 0;
    }
    
  }
    
  /* Temperature available */  
  if (irqstat & IRQ_TEMP) {
    temp_ = 0;
    ADE_read (MR_WAVEFORM, (unsigned char *)&temp_, MR_WAVEFORM_CNT);
  }
    
  /* Reset complete */  
  if (irqstat & IRQ_RESET) {
  }
    
  /* Active energy register overflowed */  
  if (irqstat & IRQ_AEOF) {
  }
    
  /* Channel 2 (V) greater than VPKLVL */  
  if (irqstat & IRQ_PKV) {
  }
    
  /* Channel 1 (I) greater than IPKLVL */  
  if (irqstat & IRQ_PKI) {
  }
    
  /* Apparent energy more than 1/2 full */  
  if (irqstat & IRQ_VAEHF) {
  }
    
  /* Apparent energy register overflowed */  
  if (irqstat & IRQ_VAEOF) {
  }
    
  /* Missing a zero crossing */
  if (irqstat & IRQ_ZXTO) {
  }
    
  /* Power changed to positive */  
  if (irqstat & IRQ_PPOS) {
  }
    
  /* Power changed to negative */
  if (irqstat & IRQ_PNEG) {
  }
    
  /* Reserved */  
  if (irqstat & IRQ_RESERVED) {
  }
  
  
} /* end ADE_isr */


/**************************************************************************
 *  ADE_data
 *
 *  Collect and print data
 *
 *  @return void
 *************************************************************************/
void ADE_data() {
  unsigned long vrms;
  unsigned long irms;
  float v;
  float i;
  float p;

  /* Get RMS values */
  ADE_sampleRMS(&vrms, &irms);

  /* Compute calibrate Vrms and Irms */
  v = (float)vrms ;//* vGain[0];
  i = (float)irms ;// * iGain[0];

  /* Computer Power (rms) */
  p = v * i;

  /* Print data in CSV form for collector */
  Serial.print(0);
  Serial.print(",");
  Serial.print(irms);
  Serial.print(",");
  Serial.print(i);
  Serial.print(",");
  Serial.print(vrms);
  Serial.print(",");
  Serial.print(v);
  Serial.print(",");
  Serial.print(p);
  Serial.println("");
  

} /* end ADE_data */


/**************************************************************************
 * ADE_dump
 * Dump many of the ADE registers
 *
 * @return void
 *************************************************************************/
void ADE_dump(void) {
  unsigned char uc;
  unsigned int  ui;
  unsigned long ul;
  
  ul = 0;
  ADE_read(MR_WAVEFORM,  (unsigned char *)&ul, MR_WAVEFORM_CNT);
  Serial.print("Waveform: 0x");
  Serial.println(ul, HEX);
  
#if 0
  ul = 0;
  ADE_read(MR_AENERGY,   (unsigned char *)&ul, MR_AENERGY_CNT);
  Serial.println((long)ul, HEX);
  
  ul = 0;
  ADE_read(MR_RAENERGY,  (unsigned char *)&ul, MR_RAENERGY_CNT);
  Serial.println((long)ul, HEX);
  
  ul = 0;
  ADE_read(MR_LAENERGY,  (unsigned char *)&ul, MR_LAENERGY_CNT);
  Serial.println((long)ul, HEX);
  
  ADE_read(MR_VAENERGY,  (unsigned char *)&ul, MR_VAENERGY_CNT);
  Serial.println((long)ul, HEX);
  
  ul = 0;
  ADE_read(MR_RVAENERGY, (unsigned char *)&ul, MR_RVAENERGY_CNT);
  Serial.println((long)ul, HEX);
  
  ul = 0;
  ADE_read(MR_LVAENERGY, (unsigned char *)&ul, MR_LVAENERGY_CNT);
  Serial.println((long)ul, HEX);

#endif
  
  ui = 0;
  ADE_read(MR_MODE,      (unsigned char *)&ui, MR_MODE_CNT);
  Serial.print("Mode: 0x");
  Serial.println(ui, HEX);
  
  ui = 0;
  ADE_read(MR_IRQEN,     (unsigned char *)&ui, MR_IRQEN_CNT);
  Serial.print("IRQEN: 0x");
  Serial.println(ui, HEX);
  
  ui = 0;
  ADE_read(MR_IRQ,       (unsigned char *)&ui, MR_IRQ_CNT);
  Serial.print("IRQ: 0x");
  Serial.println(ui, HEX);
  
#if 0
  ui = 0;
  ADE_read(MR_RSTIRQ,    (unsigned char *)&ui, MR_RSTIRQ_CNT);
  Serial.print("RSTIRQ: 0x---");
  Serial.println((long)ui, HEX);
  
  uc = 0;
  ADE_read(MR_CH1OS,     (unsigned char *)&uc, MR_CH1OS_CNT);
  Serial.println((long)uc, HEX);
  
  uc = 0;
  ADE_read(MR_CH2OS,     (unsigned char *)&uc, MR_CH2OS_CNT);
  Serial.println((long)uc, HEX);
  
  uc = 0;
  ADE_read(MR_GAIN,      (unsigned char *)&uc, MR_GAIN_CNT);
  Serial.println((long)uc, HEX);
  
  uc = 0;
  ADE_read(MR_PHCAL,     (unsigned char *)&uc, MR_PHCAL_CNT);
  Serial.println((long)uc, HEX);
  
  ui = 0;
  ADE_read(MR_APOS,      (unsigned char *)&ui, MR_APOS_CNT);
  Serial.println((long)ui, HEX);
  
  ui = 0;
  ADE_read(MR_WGAIN,     (unsigned char *)&ui, MR_WGAIN_CNT);
  Serial.println((long)ui, HEX);
  
  uc = 0;
  ADE_read(MR_WDIV,      (unsigned char *)&uc, MR_WDIV_CNT);
  Serial.println((long)uc, HEX);
  
  ui = 0;
  ADE_read(MR_CFNUM,     (unsigned char *)&ui, MR_CFNUM_CNT);
  Serial.println((long)ui, HEX);
  
  ui =0;
  ADE_read(MR_CFDEN,     (unsigned char *)&ui, MR_CFDEN_CNT);
  Serial.println((long)ui, HEX);
#endif
  
  ul = 0;
  ADE_read(MR_IRMS,      (unsigned char *)&ul, MR_IRMS_CNT);
  Serial.print("IRMS 0x");
  Serial.println(ul, HEX);
  
  ul = 0;
  ADE_read(MR_VRMS,      (unsigned char *)&ul, MR_VRMS_CNT);
  Serial.print("VRMS 0x");
  Serial.println(ul, HEX);
  
#if 0
  ui = 0;
  ADE_read(MR_IRMSOS,    (unsigned char *)&ui, MR_IRMSOS_CNT);
  Serial.println((long)ui, HEX);
  
  ui = 0;
  ADE_read(MR_VRMSOS,    (unsigned char *)&ui, MR_VRMSOS_CNT);
  Serial.println((long)ui, HEX);
  
  ui = 0;
  ADE_read(MR_VAGAIN,    (unsigned char *)&ui, MR_VAGAIN_CNT);
  Serial.println((long)ui, HEX);
  
  uc = 0;
  ADE_read(MR_VADIV,     (unsigned char *)&uc, MR_VADIV_CNT);
  Serial.println((long)uc, HEX);
  
  ui = 0;
  ADE_read(MR_LINECYC,   (unsigned char *)&ui, MR_LINECYC_CNT);
  Serial.println((long)ui, HEX);
  
  ui = 0;
  ADE_read(MR_ZXTOUT,    (unsigned char *)&ui, MR_ZXTOUT_CNT);
  Serial.println((long)ui, HEX);
  
  uc = 0;
  ADE_read(MR_SAGCYC,    (unsigned char *)&uc, MR_SAGCYC_CNT);
  Serial.println((long)uc, HEX);
  
  uc = 0;
  ADE_read(MR_SAGLVL,    (unsigned char *)&uc, MR_SAGLVL_CNT);
  Serial.println((long)uc, HEX);
  
  uc = 0;
  ADE_read(MR_IPKLVL,    (unsigned char *)&uc, MR_IPKLVL_CNT);
  Serial.println((long)uc, HEX);
  
  uc = 0;
  ADE_read(MR_VPKLVL,    (unsigned char *)&uc, MR_VPKLVL_CNT);
  Serial.println((long)uc, HEX);
#endif

  ul = 0;
  ADE_read(MR_IPEAK,     (unsigned char *)&ul, MR_IPEAK_CNT);
  Serial.print("IPEAK 0x");
  Serial.println(ul, HEX);
  
#if 0
  ul = 0;
  ADE_read(MR_RSTIPEAK,  (unsigned char *)&ul, MR_RSTIPEAK_CNT);
  Serial.println(ul, HEX);
#endif
  
  ul = 0;
  ADE_read(MR_VPEAK,     (unsigned char *)&ul, MR_VPEAK_CNT);
  Serial.print("VPEAK 0x");
  Serial.println(ul, HEX);
  
#if 0
  ul = 0;
  ADE_read(MR_RSTVPEAK,  (unsigned char *)&ul, MR_RSTVPEAK_CNT);
  Serial.println((long)ul, HEX);
#endif

#if 0  
  uc = 0;
  ADE_read(MR_TEMP,      (unsigned char *)&uc, MR_TEMP_CNT);
  Serial.println((long)uc, HEX);
  
  ui = 0;
  ADE_read(MR_PERIOD,    (unsigned char *)&ui, MR_PERIOD_CNT);
  Serial.println((long)ui, HEX);
  
  uc = 0;
  ADE_read(MR_TMODE,     (unsigned char *)&uc, MR_TMODE_CNT);
  Serial.println((long)uc, HEX);
  
  uc = 0;
  ADE_read(MR_CHKSUM,    (unsigned char *)&uc, MR_CHKSUM_CNT);
  Serial.println((long)uc, HEX);
#endif
  
  uc = 0;
  ADE_read(MR_DIEREV,    (unsigned char *)&uc, MR_DIEREV_CNT);
  Serial.print("DIEREV 0x");
  Serial.println(uc, HEX);

} /* end ADE_dump */



