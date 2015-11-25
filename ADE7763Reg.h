/**********************************************************************
 *
 * ADE7763 register definitions
 *
 *
 *********************************************************************/

#ifndef __ADE7366Reg_H__
#define __ADE7366Reg_H__



/***************************************
 * ADE register addresses
 **************************************/
const int MR_WAVEFORM  = 0x01; /* Waveform Register. */
const int MR_AENERGY   = 0x02; /* Active Energy Register.*/
const int MR_RAENERGY  = 0x03; /* AENERGY with reset. */
const int MR_LAENERGY  = 0x04; /* Line Accumulation Active Energy Register. */
const int MR_VAENERGY  = 0x05; /* Apparent Energy Register. */
const int MR_RVAENERGY = 0x06; /* VAENERGY with reset. */
const int MR_LVAENERGY = 0x07; /* Line Accumulation Apparent Energy Register. */
const int MR_MODE      = 0x09; /* Mode Register. */
const int MR_IRQEN     = 0x0A; /* Interrupt Enable Register. */
const int MR_IRQ       = 0x0B; /* Interrupt Status Register. */
const int MR_RSTIRQ    = 0x0C; /* IRQ with reset. */
const int MR_CH1OS     = 0x0D; /* Current Channel Offset Adjust. */
const int MR_CH2OS     = 0x0E; /* Voltage Channel Offset Adjust. */
const int MR_GAIN      = 0x0F; /* PGA Gain Adjust. */
const int MR_PHCAL     = 0x10; /* Phase Calibration Register. */
const int MR_APOS      = 0x11; /* Active Power Offset Correction. */
const int MR_WGAIN     = 0x12; /* Power Gain Adjust. */
const int MR_WDIV      = 0x13; /* Active Energy Divider Register. */
const int MR_CFNUM     = 0x14; /* CF Frequency Divider Numerator Register. */
const int MR_CFDEN     = 0x15; /* CF Frequency Divider Denominator Register. */
const int MR_IRMS      = 0x16; /* Current Channel 1 RMS Value. */
const int MR_VRMS      = 0x17; /* Voltage Channel 2 RMS Value. */
const int MR_IRMSOS    = 0x18; /* Current Channel 1 RMS Offset Correction Register. */
const int MR_VRMSOS    = 0x19; /* Voltage Channel 2 RMS Offset Correction Register. */
const int MR_VAGAIN    = 0x1A; /* Apparent Gain Register. */
const int MR_VADIV     = 0x1B; /* Apparent Energy Divider Register. */
const int MR_LINECYC   = 0x1C; /* Line Cycle Energy Accum. Mode Register. */
const int MR_ZXTOUT    = 0x1D; /* Zero-Crossing Timeout. */
const int MR_SAGCYC    = 0x1E; /* Sag Line Cycle Register. */
const int MR_SAGLVL    = 0x1F; /* Sag Voltage Level. */
const int MR_IPKLVL    = 0x20; /* Channel 1 Peak Level Threshold. */
const int MR_VPKLVL    = 0x21; /* Voltage Channel Peak Level Threshold. */
const int MR_IPEAK     = 0x22; /* Current Channel Peak Register. */
const int MR_RSTIPEAK  = 0x23; /* IPEAK with reset */
const int MR_VPEAK     = 0x24; /* Voltage Channel Peak Register. */
const int MR_RSTVPEAK  = 0x25; /* VPEAK with reset */
const int MR_TEMP      = 0x26; /* Temperature Register. */
const int MR_PERIOD    = 0x27; /* Period of the Voltage Channel. */
/* Reserve values 0x28 to 0x3C */
const int MR_TMODE     = 0x3D; /* Test Mode Register. */
const int MR_CHKSUM    = 0x3E; /* Checksum Register. */
const int MR_DIEREV    = 0x3F; /* Die Revision Register. */

/***************************************
 * ADE register size in bytes
 **************************************/
const int MR_WAVEFORM_CNT  = 3;
const int MR_AENERGY_CNT   = 3;
const int MR_RAENERGY_CNT  = 3;
const int MR_LAENERGY_CNT  = 3;
const int MR_VAENERGY_CNT  = 3;
const int MR_RVAENERGY_CNT = 3;
const int MR_LVAENERGY_CNT = 3;
const int MR_MODE_CNT      = 2;
const int MR_IRQEN_CNT     = 2;
const int MR_IRQ_CNT       = 2;
const int MR_RSTIRQ_CNT    = 2;
const int MR_CH1OS_CNT     = 1;
const int MR_CH2OS_CNT     = 1;
const int MR_GAIN_CNT      = 1;
const int MR_PHCAL_CNT     = 1;
const int MR_APOS_CNT      = 2;
const int MR_WGAIN_CNT     = 2;
const int MR_WDIV_CNT      = 1;
const int MR_CFNUM_CNT     = 2;
const int MR_CFDEN_CNT     = 2;
const int MR_IRMS_CNT      = 3;
const int MR_VRMS_CNT      = 3;
const int MR_IRMSOS_CNT    = 2;
const int MR_VRMSOS_CNT    = 2;
const int MR_VAGAIN_CNT    = 2;
const int MR_VADIV_CNT     = 1;
const int MR_LINECYC_CNT   = 2;
const int MR_ZXTOUT_CNT    = 2;
const int MR_SAGCYC_CNT    = 1;
const int MR_SAGLVL_CNT    = 1;
const int MR_IPKLVL_CNT    = 1;
const int MR_VPKLVL_CNT    = 1;
const int MR_IPEAK_CNT     = 3;
const int MR_RSTIPEAK_CNT  = 3;
const int MR_VPEAK_CNT     = 3;
const int MR_RSTVPEAK_CNT  = 3;
const int MR_TEMP_CNT      = 1;
const int MR_PERIOD_CNT    = 2;
const int MR_TMODE_CNT     = 1;
const int MR_CHKSUM_CNT    = 1;
const int MR_DIEREV_CNT    = 1;


const int ADE_WRITE_FLAG = 0x80;



/* Bit definitions for the MODE register */
#define MODE_DISHPF   0x0001   // HPF in Channel 1 is disabled when this 
                               // bit is set.
#define MODE_DISLPF2  0x0002   // LPF after the multiplier (LPF2) is disabled 
                               // when this bit is set.
#define MODE_DISCF    0x0004   // Frequency output CF is disabled when this 
                               // bit is set.
#define MODE_DISSAG   0x0008   // Line voltage sag detection is disabled when 
                               // this bit is set.
#define MODE_ASUSPEND 0x0010   // By setting this bit to Logic 1, both A/D 
                               // converters can be turned off. During normal 
                               // operation, this bit should be left at 
                               // Logic 0. All digital functionality can be 
                               // stopped by suspending the clock signal at 
                               // CLKIN pin.
#define MODE_TEMPSEL  0x0020   // Temperature conversion starts when this bit 
                               // is set to 1. This bit is automatically reset 
                               // to 0 after the temperature conversion.
#define MODE_SWRST    0x0040   // Software Chip Reset. A data transfer should 
                               // not take place to the ADE7763 for at least 
                               // 18 �s after a software reset.
#define MODE_CYCMODE  0x0080   // Setting this bit to Logic 1 places the chip 
                               // in line cycle energy accumulation mode.
#define MODE_DISCH1   0x0100   // ADC 1 (Channel 1) inputs are internally 
                               // shorted together.
#define MODE_DISCH2   0x0200   // ADC 2 (Channel 2) inputs are internally 
                               // shorted together.
#define MODE_SWAP     0x0400   // By setting this bit to Logic 1, the analog 
                               // inputs V2P and V2N are connected to ADC 1 
                               // and the analog inputs V1P and V1N are 
                               // connected to ADC 2.
#define MODE_DTRT0    0x0800   // Use these bits to select the waveform 
                               // register update rate.
#define MODE_DTRT1    0x1000   // Use these bits to select the waveform 
                               // register update rate.
#define MODE_WAVSEL0  0x2000   // Use these bits to select the source of the 
                               // sampled data for the waveform register.
#define MODE_WAVSEL1  0x4000   // Use these bits to select the source of the 
                               // sampled data for the waveform register.
#define MODE_POAM     0x8000   // Writing Logic 1 to this bit allows only 
                               // positive active power to accumulate. The 
                               // default value of this bit is 0.

/* Combinations of DTRT bits */
#define MODE_DTRT_27K9  (0x00)
#define MODE_DTRT_14K   (0x00 | MODE_DTRT0)
#define MODE_DTRT_7K    (0x00 | MODE_DTRT1)
#define MODE_DTRT_3K5   (0x00 | MODE_DTRT1 | MODE_DTRT0)


/* Combinations of WAVSEL bits */
#define MODE_WAV_POWER    (0x00)                           // Active power
#define MODE_WAV_RESERVED (0x00 | MODE_WAVSEL0)            // Reserved 
#define MODE_WAV_ADC1     (0x00 | MODE_WAVSEL1)            // ADC1 (i)
#define MODE_WAV_ADC2     (0x00 | MODE_WAVSEL0 | WAVSEL1)  // ADC2 (v)

/* Bit definitions for the ISR register */
#define IRQ_NONE      0x0000
#define IRQ_AEHF      0x0001   // Indicates that an interrupt occurred because 
                               // the active energy register, AENERGY, is more 
                               // than half full.
#define IRQ_SAG       0x0002   // Indicates that an interrupt was caused by a 
                               // sag on the line voltage.
#define IRQ_CYCEND    0x0004   // Indicates the end of energy accumulation over
                               // an integral number of half line cycles, as 
                               // defined by the content of the LINECYC register
#define IRQ_WSMP      0x0008   // Indicates that new data is present in the 
                               // waveform register.
#define IRQ_ZX        0x0010   // This status bit is set to Logic 0 on the 
                               // rising and falling edge of the voltage 
                               // waveform.
#define IRQ_TEMP      0x0020   // Indicates that a temperature conversion 
                               // result is available in the temperature 
                               // register.
#define IRQ_RESET     0x0040   // Indicates the end of a reset for software and
                               // hardware resets. The corresponding enable bit
                               // has no function in the interrupt enable 
                               // register, i.e., this status bit is set at the
                               // end of a reset, but cannot be enabled to 
                               // cause an interrupt.
#define IRQ_AEOF      0x0080   // Indicates that the active energy register 
                               // has overflowed.
#define IRQ_PKV       0x0100   // Indicates that the waveform sample from 
                               // Channel 2 has exceeded the VPKLVL value.
#define IRQ_PKI       0x0200   // Indicates that the waveform sample from 
                               // Channel 1 has exceeded the IPKLVL value.
#define IRQ_VAEHF     0x0400   // Indicates that the apparent energy register, 
                               // VAENERGY, is more than half full.
#define IRQ_VAEOF     0x0800   // Indicates that the apparent energy register 
                               // has overflowed.
#define IRQ_ZXTO      0x1000   // Indicates that an interrupt was caused by a 
                               // missing zero crossing on the line voltage for
                               // a specified number of line cycles.
#define IRQ_PPOS      0x2000   // Indicates that the power has gone from 
                               // negative to positive.
#define IRQ_PNEG      0x4000   // Indicates that the power has gone from 
                               // positive to negative.
#define IRQ_RESERVED  0x8000   // Reserved.


#define  GAIN_ADC1_0     0x01  // ADC1 bit 0 gain
#define  GAIN_ADC1_1     0x02  // ADC1 bit 1 gain
#define  GAIN_ADC1_2     0x04  // ADC1 bit 2 gain
#define  GAIN_ADC1_FS_0  0x08  // ADC1 fullscale bit 0
#define  GAIN_ADC1_FS_1  0x10  // ADC1 fullscale bit 1
#define  GAIN_ADC2_0     0x20  // ADC2 bit 0 gain
#define  GAIN_ADC3_1     0x40  // ADC2 bit 1 gain
#define  GAIN_ADC4_2     0x80  // ADC2 bit 2 gain


              

#endif

