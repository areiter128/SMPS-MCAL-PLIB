/*LICENSE *****************************************************************************************
 *
 * Software License Agreement
 *
 * Copyright (R) 2012 Microchip Technology Inc.  All rights reserved. Microchip licenses to you the
 * right to use, modify, copy and distribute Software only when embedded on a Microchip 
 * microcontroller or digital signal controller, which is integrated into your product or third 
 * party product (pursuant to the sublicense terms in the accompanying license agreement).
 *
 * You should refer to the license agreement accompanying this Software for additional information 
 * regarding your rights and obligations.
 *
 * SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR 
 * IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE, NON-INFRINGEMENT 
 * AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR
 * OBLIGATED UNDER CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR 
 * OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES INCLUDING BUT NOT  
 * LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR CONSEQUENTIAL DAMAGES, LOST PROFITS  
 * OR LOST DATA, COST OF PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY  
 * THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *
 * ***********************************************************************************************/

#include <xc.h> // Device header file
#include <stdint.h>
#include <stdbool.h>

#include "p33SMPS_hsadc.h"


/*!p33EGS_adc.c
 * ************************************************************************************************
 * Summary:
 * Driver file for the dsPIC33ExxGS-ADC SFRs
 *
 * Description:
 * The SMPS ADC module offers a number of registers and configuration options. This additional
 * driver file contains initialization routines for all majority required settings.
 * ***********************************************************************************************/

volatile uint16_t adcc_usage=0; // Private variable tracking the ADC cores in use while
                                // ADC inputs and the module gets initialized

/*!hsadc_module_power_up()
 * *****************************************************************************************************
 * Summary:
 * Turns on the base power of the ADC module 
 *
 * Parameters: 
 *   (none)
 *
 * Description:
 * 
 * *****************************************************************************************************/

volatile uint16_t hsadc_module_power_up(void)
{
    volatile uint16_t fres=1;

    #if defined (_ABGMD)
    _ABGMD = 0;         // Turn on power to analog bandgap reference
    fres &= (1-_ABGMD);
    #endif
    #if defined (_ADCMD)
    _ADCMD = 0; 		// Turn on power to ADC module
    fres &= (1-_ADCMD);
    #elif defined (_ADC1MD)
    _ADC1MD = 0; 		// Turn on power to ADC module
    fres &= (1-_ADC1MD);
    #endif
    return(fres);

} 

/*!hsadc_module_power_down()
 * *****************************************************************************************************
 * Summary:
 * Turns off the base power of the ADC module 
 *
 * Parameters: 
 *   (none)
 *
 * Description:
 * 
 * *****************************************************************************************************/

volatile uint16_t hsadc_module_power_down(void)
{
    volatile uint16_t fres=1;
    
    Nop();  // No-Operation Instructions required to ensure proper register access
    Nop();  // DO NOT REMOVE
    Nop();  // *******************************************************************

    #if defined (_ABGMD)
    _ABGMD = 1;         // Turn off power to analog bandgap reference
    fres &= (_ABGMD);
    #endif
    #if defined (_ADCMD)
    _ADCMD = 1; 		// Turn off power to PWM channel #1
    fres &= (_ADCMD);
    #elif defined (_ADC1MD)
    _ADC1MD = 1; 		// Turn off power to PWM channel #1
    fres &= (_ADC1MD);
    #else
    fres = 0;
    #endif
    return(fres);

}

/*!hsadc_init_adc_module
 * ************************************************************************************************
 * Summary:
 * Initializes the basic ADC configuration
 *
 * Parameters:
 *	HSADC_MODULE_CONFIG_t adc_cfg = holds the register value for module base registers
 *
 * Description:
 * Basic options like clock source, early interrupts, format options, sampling order and modes
 * are set here.
 * ***********************************************************************************************/
volatile uint16_t hsadc_adc_module_initialize( volatile HSADC_ADMODCFG_t adc_cfg )
{
    volatile uint16_t fres = 1;
    
    // Ensure ADC module is powered up before registers are written
    fres &= hsadc_module_power_up();
    
	// Reset all ADC configuration registers to defaults
    fres &= hsadc_module_reset();
    
	// Setting ADC configuration block #1 according to user settings.
	// Please note:
	//   ADC ENABLE is ALWAYS masked out! 
    //   The ADC has to be enabled later during the configuration process.

    _ADON = ADCON1_ADON_DISABLED; // Always disable the ADC converter peripheral while being configured
    _ADSIDL = adc_cfg.config.adsidl;   // Set ADC Stop in Idle Mode bit

    _REFSEL = adc_cfg.refcfg.refsel; // ADC Reference Voltage Selection bits
    _REFCIE = adc_cfg.refcfg.refcie; // Band Gap and Reference Voltage Ready Common Interrupt Enable bit
    _REFERCIE = adc_cfg.refcfg.refercie; // Band Gap or Reference Voltage Error Common Interrupt Enable bit
    _WARMTIME = adc_cfg.config.warmtime;

    _FORM = adc_cfg.config.form;   // Set Fractional Data Output Format bit
    _EIEN = adc_cfg.config.eien;   // Set Early Interrupts Enable bit
    ADCON2Lbits.PTGEN = adc_cfg.config.ptgen; // Set External Conversion Request Interface bit
    
    _CNVCHSEL = adc_cfg.swtrig.cnvchsel; // Channel Number Selection for Software Individual Channel Conversion Trigger bits
    _SUSPEND = adc_cfg.swtrig.suspend; // All ADC Core Triggers Disable bit
    _SUSPCIE = adc_cfg.swtrig.suspcie; // Suspend All ADC Cores Common Interrupt Enable bit
   
    ADCON3Hbits.CLKSEL = adc_cfg.cores.clksel;  // Set ADC Module Clock Source Selection bits
    ADCON3Hbits.CLKDIV = adc_cfg.cores.clkdiv;  // Set ADC Module Clock Source Divider bits

    _SHRADCS = adc_cfg.cores.shared_core.adcs;  // Shared ADC Core Input Clock Divider bits
    _SHRRES = adc_cfg.cores.shared_core.res;    // Shared ADC Core Resolution Selection bits
    _SHRSAMC = adc_cfg.cores.shared_core.samc;  // Shared ADC Core Sample Time Selection bits
    _SHREISEL = adc_cfg.cores.shared_core.eisel; // Shared Core Early Interrupt Time Selection bits
    
    #if defined (ADCORE0L)
    ADCORE0Hbits.ADCS = adc_cfg.cores.core0.adcs;       // Dedicated ADC Core x Input Clock Divider bits
    ADCORE0Hbits.RES = adc_cfg.cores.core0.res;         // Dedicated ADC Core x Resolution Selection bits
    ADCORE0Hbits.EISEL = adc_cfg.cores.core0.eisel;     // Dedicated ADC Core x Early Interrupt Time Selection bits
    ADCORE0Lbits.SAMC = adc_cfg.cores.core0.samc;       // Dedicated ADC Core x Conversion Delay Selection bits
    ADCON4Lbits.SAMC0EN = adc_cfg.cores.core0.samc_en;  // Dedicated ADC Core x Conversion Delay Enable bit
    #endif
    #if defined (ADCORE1L)
    ADCORE1Hbits.ADCS = adc_cfg.cores.core1.adcs;       // Dedicated ADC Core x Input Clock Divider bits
    ADCORE1Hbits.RES = adc_cfg.cores.core1.res;         // Dedicated ADC Core x Resolution Selection bits
    ADCORE1Hbits.EISEL = adc_cfg.cores.core1.eisel;     // Dedicated ADC Core x Early Interrupt Time Selection bits
    ADCORE1Lbits.SAMC = adc_cfg.cores.core1.samc;       // Dedicated ADC Core x Conversion Delay Selection bits
    ADCON4Lbits.SAMC0EN = adc_cfg.cores.core1.samc_en;  // Dedicated ADC Core x Conversion Delay Enable bit
    #endif
    #if defined (ADCORE2L)
    ADCORE2Hbits.ADCS = adc_cfg.cores.core2.adcs;       // Dedicated ADC Core x Input Clock Divider bits
    ADCORE2Hbits.RES = adc_cfg.cores.core2.res;         // Dedicated ADC Core x Resolution Selection bits
    ADCORE2Hbits.EISEL = adc_cfg.cores.core2.eisel;     // Dedicated ADC Core x Early Interrupt Time Selection bits
    ADCORE2Lbits.SAMC = adc_cfg.cores.core2.samc;       // Dedicated ADC Core x Conversion Delay Selection bits
    ADCON4Lbits.SAMC0EN = adc_cfg.cores.core2.samc_en;  // Dedicated ADC Core x Conversion Delay Enable bit
    #endif

    
    // Return 1=success, 0=failure
	return(fres);

}


/*!hsadc_init_adc_channel
 * ************************************************************************************************
 * Summary:
 * Initializes an individual ADC input configuration
 *
 * Parameters:
 *	HSADC_CHANNEL_CONFIG_t = holds all available settings of one ADC input (e.g. AN3)
 *
 * Description:
 * This routine configures the individual settings of an analog input .
 * ***********************************************************************************************/

volatile uint16_t hsadc_adc_input_initialize( volatile HSADC_ADCANCFG_t adin_cfg ) {
    
    volatile uint16_t fres = 1;
    
    // Check if given analog input number is available
    if(adin_cfg.ad_input > ADC_ANINPUT_COUNT) { return(0); }
    
    // Track ADC cores in use
    adcc_usage |= (0x0001 << adin_cfg.config.core_index);

    // Set AD input mode (differential or single ended) and 
    // Conversion data output format (signed/unsigned)
    fres &= hsadc_set_adc_input_mode(adin_cfg);

    // Set interrupt enable and early interrupt enable
   fres &= hsadc_set_adc_input_interrupt(adin_cfg);
    
    // Set interrupt trigger mode (level/edge)
    fres &= hsadc_set_adc_input_trigger_mode(adin_cfg);

    // Set interrupt trigger source
    fres &= hsadc_set_adc_input_trigger_source(adin_cfg);

    
    return(fres);
    
}


/*!hsadc_init_adc_core
 * ************************************************************************************************
 * Summary:
 * Initializes an individual ADC Core configuration (dedicated or shared)
 *
 * Parameters:
 *	HSADC_ADCOREx_CONFIG_t core_cfg  data structure holding all required ADC core 
 *                          configuration settings
 *
 * Description:
 * Fundamental operating settings of a dedicated or shared core are written to the respective
 * registers, such as like conversion delay, early interrupt period, ADC resolution and 
 * input clock dividers. 
 * 
 * Dedicated ADC cores have their own register set (ADCORExH, ADCORExL) while the shared core 
 * settings are distributed across multiple registers (ADCON1H and ADCON2L). This routine
 * offers one function call for both and treats the settings accordingly.
 * 
 * As different devices may have a different number of dedicated cores, the highest index
 * is always taken as index of the shared ADC core instance.
 * ***********************************************************************************************/

//volatile uint16_t hsadc_adc_core_initialize(volatile HSADC_ADCORECFG_t adcore_cfg)
//{
//
//    volatile uint16_t fres=1;       // Success/Failure result
//    volatile uint16_t *regptr;      // Register address pointer
//    volatile uint16_t reg_buf=0;    // Register value buffer
//    volatile uint16_t reg_mask=0;   // Dummy variable for complex filter bit-masks
//    volatile uint16_t reg_val=0;    // Dummy variable for preparing data before WRITE events
//
//    // Variables for shared ADC core configuration
//    volatile REGBLK_ADCON1H_t regADCON1H;  // ADCON1H register copy for WRITE operation
//    volatile REGBLK_ADCON2L_t regADCON2L;  // ADCON2L register copy for WRITE operation
//    volatile REGBLK_ADCON2H_t regADCON2H;  // ADCON2H register copy for WRITE operation
//    
//    // Variables for dedicated ADC core configuration
//    #if defined (ADCORE0L)
//    volatile REGBLK_ADCON4L_t  regADCON4L;  // ADCON4L register copy for WRITE operating
////    volatile REGBLK_ADCON4H_t  regADCON4H;  // ADCON4H register copy for WRITE operating
//    volatile REGBLK_ADCON5H_t  regADCON5H;  // ADCON5H register copy for WRITE operating
//    volatile REGBLK_ADCORExL_t regADCORExL; // ADCORExL register copy for WRITE operation
//    volatile REGBLK_ADCORExH_t regADCORExH; // ADCORExH register copy for WRITE operation
//    volatile uint16_t reg_offset=0;
//    #endif
//
//    
//    // check if given ADC core instance index is greater than number of available ADC cores
//	if (adcore_cfg.index >= ADC_CORE_COUNT) return(0);
//    
//    // Track ADC cores in use
//    adcc_usage |= (0x0001 << adcore_cfg.index);
//
//
//    // Check which core type needs to be configured
//    if (adcore_cfg.config.type == ADCORE_TYPE_DEDICATED)
//    {
//        // Dedicated cores have their individual configuration 
//        // registers ADCORExL and ADCORExH
//        
//        #if defined (ADCORE0L)
//        // Dual core devices like dsPIC33CH only have shared cores on the master side.
//        // Only slave cores and single core devices like dsPIC33CK have multiple dedicated
//        // cores and one shared core. The shared core always has the highest instance index.
//
//        reg_offset = (adcore_cfg.index-1) * ((volatile uint16_t)&ADCORE1L - (volatile uint16_t)&ADCORE0L); // Get register set offset
//
//        regADCORExL.bits.samc = adcore_cfg.config.samc;      // Capture ADC core configuration ADCORExL
//        
//        regptr = (volatile uint16_t *)&ADCORE0L + reg_offset;   // Get pointer to ADC core configuration register
//        reg_buf = (regADCORExL.value & REG_ADCORExL_VALID_DATA_MSK);  // Get value to be written to register
//        *regptr = reg_buf;                                      // Write configuration to register
//        fres &= (volatile bool)((*regptr & REG_ADCORExL_VALID_DATA_MSK) == reg_buf);  // Check if WRITE operation was successful
//
//        
//        regADCORExH.bits.adcs = adcore_cfg.config.adcs;    // Capture ADC core clock divider configuration
//        regADCORExH.bits.eisel = adcore_cfg.config.eisel;  // Capture ADC core Early Interrupt Period configuration
//        regADCORExH.bits.res = adcore_cfg.config.res;      // Capture ADC core resolution configuration
//
//        regptr = (volatile uint16_t *)&ADCORE0H + reg_offset;   // Get pointer to ADC core configuration register
//        reg_buf = (regADCORExH.value & REG_ADCORExH_VALID_DATA_MSK);  // Get value to be written to register
//        *regptr = reg_buf;                                      // Write configuration to register
//        fres &= (volatile bool)((*regptr & REG_ADCORExH_VALID_DATA_MSK) == reg_buf);  // Check if WRITE operation was successful
//
//        
//        switch (adcore_cfg.index) { 
//            #if (ADC_CORE_COUNT > 1)
//            case 0: regADCON4L.bits.samc0en = adcore_cfg.config.samc_en; break;
//            #endif
//            #if (ADC_CORE_COUNT > 2)
//            case 1: regADCON4L.bits.samc1en = adcore_cfg.config.samc_en; break;
//            #endif
//            #if (ADC_CORE_COUNT > 3)
//            case 2: regADCON4L.bits.samc2en = adcore_cfg.config.samc_en; break;
//            #endif
//            #if (ADC_CORE_COUNT > 4)
//            case 3: regADCON4L.bits.samc3en = adcore_cfg.config.samc_en; break;
//            #endif
//            #if (ADC_CORE_COUNT > 5)
//            case 4: regADCON4L.bits.samc4en = adcore_cfg.config.samc_en; break;
//            #endif
//            #if (ADC_CORE_COUNT > 6)
//            case 5: regADCON4L.bits.samc5en = adcore_cfg.config.samc_en; break;
//            #endif
//            #if (ADC_CORE_COUNT > 7)
//            case 6: regADCON4L.bits.samc6en = adcore_cfg.config.samc_en; break;
//            #endif
//            default:
//                return(0);
//                break;
//        }
//                
//        regptr = (volatile uint16_t *)&ADCON4L;   // Get pointer to ADC configuration register
//        reg_buf = (regADCON4L.value & REG_ADCON4L_VALID_DATA_WRITE_MSK);  // Get value to be written to register
//        *regptr = reg_buf;                                      // Write configuration to register
//        fres &= (volatile bool)((*regptr & REG_ADCON4L_VALID_DATA_WRITE_MSK) == reg_buf);  // Check if WRITE operation was successful
//
//        regADCON5H.bits.warmtime = adcore_cfg.config.warmtime;
//        
//        regptr = (volatile uint16_t *)&ADCON5H;   // Get pointer to ADC configuration register
//        reg_buf = (regADCON5H.value & REG_ADCON5H_VALID_DATA_WRITE_MSK);  // Get value to be written to register
//        *regptr = reg_buf;                                      // Write configuration to register
//        fres &= (volatile bool)((*regptr & REG_ADCON5H_VALID_DATA_WRITE_MSK) == reg_buf);  // Check if WRITE operation was successful
//
///*        
//        switch (adcore_cfg.index) { 
//            #if (ADC_CORE_COUNT > 1)
//            case 0: regADCON4H.bits.C0CHS = adcore_cfg.adin_select.C0CHS; break;
//            #endif
//            #if (ADC_CORE_COUNT > 2)
//            case 1: regADCON4H.bits.C1CHS = adcore_cfg.adin_select.C1CHS; break;
//            #endif
//            #if (ADC_CORE_COUNT > 3)
//            case 2: regADCON4H.bits.C2CHS = adcore_cfg.adin_select.C2CHS; break;
//            #endif
//            #if (ADC_CORE_COUNT > 4)
//            case 3: regADCON4H.bits.C3CHS = adcore_cfg.adin_select.C3CHS; break;
//            #endif
//            #if (ADC_CORE_COUNT > 5)
//            case 4: regADCON4H.bits.C4CHS = adcore_cfg.adin_select.C4CHS; break;
//            #endif
//            #if (ADC_CORE_COUNT > 6)
//            case 5: regADCON4H.bits.C5CHS = adcore_cfg.adin_select.C5CHS; break;
//            #endif
//            #if (ADC_CORE_COUNT > 7)
//            case 6: regADCON4H.bits.C6CHS = adcore_cfg.adin_select.C6CHS; break;
//            #endif
//            default:
//                return(0);
//                break;
//        }
//                
//        regptr = (volatile uint16_t *)&ADCON4H;   // Get pointer to ADC configuration register
//        reg_buf = (regADCON4H.value & REG_ADCON4H_VALID_DATA_WRITE_MSK);  // Get value to be written to register
//        *regptr = reg_buf;                                      // Write configuration to register
//        fres &= (volatile bool)((*regptr & REG_ADCON4H_VALID_DATA_WRITE_MSK) == reg_buf);  // Check if WRITE operation was successful
//  */      
//        #endif
//        
//    }
//    else if(adcore_cfg.config.type == ADCORE_TYPE_SHARED)
//    {
//        Nop();
//        
//        // The configuration of the shared core is incorporated into the generic registers 
//        // ADCON1H, ADCON2L and ADCON2H while dedicated cores have their own configuration 
//        // registers ADCORExH and ADCORExL. The values, however, are identical and therefore
//        // this library API treats them as equal on the surface but differently when accessing
//        // registers. So don't get confused.
//        
//        // Set settings in ADCON1H register
//        reg_mask = (REG_ADCON1H_EXCLUDE_SHRADC_CFG_MSK & REG_ADCON1H_VALID_DATA_MSK);
//        regADCON1H.value = (((volatile uint16_t)ADCON1H) & reg_mask);   // Get pointer to ADC core configuration in ADC configuration #1 HIGH register
//        reg_val = (volatile uint16_t)adcore_cfg.config.res;
//        regADCON1H.bits.shrres = reg_val;  // Overwrite ADC core resolution configuration
//        reg_buf = (regADCON1H.value & REG_ADCON1H_VALID_DATA_MSK);  // Get value to be written to register
//        ADCON1H = reg_buf; // Write configuration to register with bit-filter
//        fres &= (volatile bool)((ADCON1H & REG_ADCON1H_VALID_DATA_MSK) == reg_buf);  // Check if WRITE operation was successful
//
//        // Set settings in ADCON2L register
//        reg_mask = (REG_ADCON2L_EXCLUDE_SHRADC_CFG_MSK & REG_ADCON2L_VALID_DATA_MSK);
//        regADCON2L.value = (((volatile uint16_t)ADCON2L) & reg_mask);   // Get pointer to ADC core configuration in ADC configuration #1 HIGH register
//        reg_val = (volatile uint16_t)adcore_cfg.config.adcs;
//        regADCON2L.bits.shradcs = reg_val;  // Overwrite user defined ADC clock divider setting
//        reg_val = (volatile uint16_t)adcore_cfg.config.eisel; 
//        regADCON2L.bits.shreisel = reg_val; // Overwrite user defined ADC Early Interrupt Period
//        reg_buf = (regADCON2L.value & REG_ADCON2L_VALID_DATA_MSK);  // Get value to be written to register
//        ADCON2L = reg_buf; // Write configuration to register with bit-filter
//        fres &= (volatile bool)((ADCON2L & REG_ADCON2L_VALID_DATA_MSK) == reg_buf);  // Check if WRITE operation was successful
//
//        // Set settings in ADCON2H register
//        reg_mask = (REG_ADCON2H_EXCLUDE_SHRADC_CFG_MSK & REG_ADCON2H_VALID_DATA_MSK);
//        regADCON2H.value = (((volatile uint16_t)ADCON2H) & reg_mask);   // Get pointer to ADC core configuration in ADC configuration #1 HIGH register
//        reg_val = (volatile uint16_t)adcore_cfg.config.samc;
//        regADCON2H.bits.shrsamc = reg_val;  // Overwrite user defined ADC clock divider setting
//        reg_buf = (regADCON2H.value & REG_ADCON2H_VALID_DATA_MSK);  // Get value to be written to register
//        ADCON2H = reg_buf; // Write configuration to register with bit-filter
//        fres &= (volatile bool)((ADCON2H & REG_ADCON2H_VALID_DATA_MSK) == reg_buf);  // Check if WRITE operation was successful
//
//    }
//    else {
//        Nop(); // Place breakpoint here to debug failure
//        Nop();
//        return(0);
//    }
//
//    return(fres);
//    
//}


/*!hsadc_module_enable()
 * ************************************************************************************************
 * Summary:
 * Enables the ADC module
 *
 * Parameters: (none)
 *
 * Description:
 * The ADC module has a start-up time of a couple of micro seconds. Therefore the 
 * enable-instruction is followed by a short delay loop.
 * ***********************************************************************************************/

volatile uint16_t hsadc_module_enable(void)
{

    volatile uint16_t fres=1;
    volatile uint16_t i=0;
    volatile uint16_t adcore_check=0;
    
	_ADON = ADC_ON; // Enable ADC module
    fres &= _ADON;  // Check if ADC module is enabled
    
    // Enable all ADC cores
    for (i=0; i<ADC_CORE_COUNT; i++) {
        adcore_check = (adcc_usage & (0x0001 << i));
        if (adcore_check)
        { fres &= hsadc_adc_core_power_on(i); }
    }
    
	return(fres);
	 
}

/*!hsadc_module_disable()
 * ************************************************************************************************
 * Summary:
 * Disables the ADC module
 *
 * Parameters: (none)
 *
 * Description:
 * Disables the entire ADC module, which will also affect the port registers. Fault states 
 * at certain pins will be lost as every pin will be re-configured as GPIO.
 * ***********************************************************************************************/

volatile uint16_t hsadc_module_disable(void)
{

	ADCON1Lbits.ADON = ADC_OFF;			// Disable ADC module 
	return(1 - ADCON1Lbits.ADON);

}

/*!hsadc_reset()
 * ************************************************************************************************
 * Summary:
 * Resets ADC configuration
 *
 * Parameters:	(none)
 *
 * Description:
 * Resets the entire ADC configuration including the port control registers. All ANx-inputs will
 * become GPIOs.
 * ***********************************************************************************************/

volatile uint16_t hsadc_module_reset(void) {
    
    /* ToDo: Add register contents check after WRITE */
    
	// Reset all ADC configuration registers to defaults

	ADCON1Lbits.ADON = ADC_OFF;         // Disable ADC
  
	ADCON1L	   = REG_ADCON1L_RESET;     // Disable and reset ADC configuration register 1 low
	ADCON1H	   = REG_ADCON1H_RESET;     // Disable and reset ADC configuration register 1 high
	ADCON2L	   = REG_ADCON2L_RESET;     // Disable and reset ADC configuration register 2 low
	ADCON2H	   = REG_ADCON2H_RESET;     // Disable and reset ADC configuration register 2 high
    ADCON3L    = REG_ADCON3L_RESET;     // Disable and reset ADC configuration register 3 low
    ADCON3H    = REG_ADCON3H_RESET;     // Disable and reset ADC configuration register 3 high
    #if defined (ADCON4L)     
    ADCON4L    = REG_ADCON4L_RESET;     // Disable and reset ADC configuration register 4 low
    #endif     
    #if defined (ADCON4H)     
    ADCON4H    = REG_ADCON4H_RESET;     // Disable and reset ADC configuration register 4 high
    #endif     
    ADCON5L    = REG_ADCON5L_RESET;     // Disable and reset ADC configuration register 5 low
    ADCON5H    = REG_ADCON5H_RESET;     // Disable and reset ADC configuration register 5 high

    // Reset registers for ADC input channels and sub-modules
    ADMOD0L    = REG_ADMOD0L_RESET;     // Disable and reset ADC INPUT MODE CONTROL REGISTER 0 LOW
    ADMOD0H    = REG_ADMOD0H_RESET;     // Disable and reset ADC INPUT MODE CONTROL REGISTER 0 HIGH
    ADMOD1L    = REG_ADMOD1L_RESET;     // Disable and reset ADC INPUT MODE CONTROL REGISTER 1 LOW
    ADMOD1H    = REG_ADMOD1H_RESET;     // Disable and reset ADC INPUT MODE CONTROL REGISTER 1 HIGH     
    ADIEL      = REG_ADIEL_RESET;       // Disable and reset ADC INTERRUPT ENABLE REGISTER LOW
    ADIEH      = REG_ADIEH_RESET;       // Disable and reset ADC INTERRUPT ENABLE REGISTER HIGH
    ADSTATL    = REG_ADSTATL_RESET;     // Disable and reset ADC DATA READY STATUS REGISTER LOW 
    ADSTATH    = REG_ADSTATH_RESET;     // Disable and reset ADC DATA READY STATUS REGISTER HIGH 
    ADEIEL     = REG_ADEIEL_RESET;      // Disable and reset ADC EARLY INTERRUPT ENABLE REGISTER LOW 
    ADEIEH     = REG_ADEIEH_RESET;      // Disable and reset ADC EARLY INTERRUPT ENABLE REGISTER HIGH 
    ADEISTATL  = REG_ADEISTATL_RESET;   // Disable and reset ADC EARLY INTERRUPT STATUS REGISTER LOW
    ADEISTATH  = REG_ADEISTATH_RESET;   // Disable and reset ADC EARLY INTERRUPT STATUS REGISTER HIGH

    ADLVLTRGL  = REG_ADLVLTRGL_RESET;   // Disable and reset ADC LEVEL-SENSITIVE TRIGGER CONTROL REGISTER LOW
    ADLVLTRGH  = REG_ADLVLTRGH_RESET;   // Disable and reset ADC LEVEL-SENSITIVE TRIGGER CONTROL REGISTER HIGH
    
    // Reset ADC core usage tracking variable
    adcc_usage = 0;
    
	return(1);
}

/*!hsadc_adc_cores_check_ready()
 * ************************************************************************************************
 * Summary:
 * Checks the 'READY' status of all powered ADC cores
 *
 * Parameters:	(none)
 * 
 * Returns:
 *      0: Failure
 *      1: Success
 *
 * Description:
 * Each ADC core of the dsPIC33E and dsPIC33C family of devices can be powered and enabled
 * individually. The READY bit indicates, if the core is powered and the configured WARMTIME 
 * has passed, so that the converter is ready to be used. 
 * This routine compares if all READY bits for all POWERED ADC cores are set and returns
 * an OK/NOT_READY response.
 * ***********************************************************************************************/

volatile uint16_t hsadc_adc_cores_check_ready(void)
{
    volatile uint16_t fres = 1;
    volatile uint16_t timeout = 0;
    volatile uint16_t rdy_compare = 0;
    volatile uint16_t reg_buf = 0;

    /* ToDo: Add register contents check after WRITE */

    fres &= _REFRDY;     // Check if Band Gap reference is running and ready
    fres &= (1-_REFERR); // Check if Band Gap reference indicating error
    
    reg_buf = (ADCON5L & 0x00FF); // Capture enabled and powered ADC core bits
    rdy_compare = ((reg_buf << 8) | reg_buf); // Set filter mask for ADC CORE READY bits

    while( (ADCON5L != rdy_compare) && (timeout++ < 0xFFFE) );
    if(timeout == 0xFFFF) { fres = 0; }

    return(fres);
}

/*!hsadc_adc_core_power_on()
 * ************************************************************************************************
 * Summary:
 * Turns on a single ADC core
 *
 * Parameters:	
 *      - uint16_t index:   Index of an individual ADC core
 *
 * Returns:
 *      0: Failure
 *      1: Success
 * 
 * Description:
 * The individual ADC cores of the ADC peripheral can be individually turned on and off. 
 * This function will turned on the power to an ADC core determined by the parameter 'index'.
 * The turn-on sequence is 
 *      (1) enabling the power to the ADC core
 *      (2) wait until AdC core is ready
 *      (3) Set Enable bit of the ADC core
 * 
 * ***********************************************************************************************/

volatile uint16_t hsadc_adc_core_power_on(volatile uint16_t index)
{
    volatile uint16_t fres=1;
    volatile uint16_t *regptr;
    volatile uint16_t reg_buf=0;
    volatile uint16_t timeout=0;

    /* ToDo: Add register contents check after WRITE */
    
    if (index >= ADC_CORE_COUNT) return(0);
    
    regptr = (volatile uint16_t *)&ADCON5L;
    
    // Power on ADC core x 
    if(index == ADC_SHARED_CORE_INDEX) { 
      // Shared Core Power Setting is located in Bit #7, while all others are 
      // enumerated on Bits #0 = dedicated core #0, #1 = dedicated core #1, etc
        reg_buf  = (REG_ADCON5L_SHRPWR_ON & REG_ADCON5L_VALID_DATA_WRITE_MSK);
        
    }
    else {
      // Dedicated core power on enable bits are set based on the index/bit position 
        reg_buf  = ((0x0001 << index) & REG_ADCON5L_VALID_DATA_WRITE_MSK);
    }

    if(!(*regptr & reg_buf)) // if bit hasn't been set yet...
    { *regptr |= reg_buf; } // write to register
    fres &= (volatile bool)((*regptr & reg_buf) == reg_buf);

    // Check READY status of powered-up ADC core
    reg_buf <<= 8;   // Shift selected bit up into the high-word of ADCON5L for the status check
    while(!(ADCON5L & reg_buf) && (timeout++ < 0xFFFE) );
    if(timeout == 0xFFFF) return(0);
    
    // Set pointer onto ADC core enable register ADCON3H
    regptr = (volatile uint16_t *)&ADCON3H;
    
    // Power on ADC core x 
    if(index == ADC_SHARED_CORE_INDEX) { 
      // Shared Core Power Setting is located in Bit #7, while all others are 
      // enumerated on Bits #0 = dedicated core #0, #1 = dedicated core #1, etc
        reg_buf  = (REG_ADCON3H_SHREN_ENABLED & REG_ADCON3H_VALID_DATA_WRITE_MSK);
        
    }
    else {
      // Dedicated core power on enable bits are set based on the index/bit position 
        reg_buf  = ((0x0001 << index) & REG_ADCON3H_VALID_DATA_WRITE_MSK);
    }

    if(!(*regptr & reg_buf)) // if bit hasn't been set yet...
    { *regptr |= reg_buf; } // write to register
    fres &= (volatile bool)((*regptr & reg_buf) == reg_buf);
    
	return(fres);
	
}

/*!hsadc_set_adc_input_trigger_source()
 * ************************************************************************************************
 * Summary:
 * Configures interrupt trigger source setting of an individual analog input
 *
 * Parameters:
 *     - HSADC_ADCAN_CONFIG_t adin_cfg  // ADC input channel settings data structure
 *
 * Returns: 
 *      0: failure
 *      1: success
 * 
 * Description:
 * This function configures the trigger source selection for corresponding analog inputs by
 * extracting the user setting from the function parameter adin_cfg.config.trigger_source 
 * stored in the adin_cfg data structure.
 * 
 * ***********************************************************************************************/

volatile uint16_t hsadc_set_adc_input_trigger_source(volatile HSADC_ADCANCFG_t adin_cfg)
{
    volatile uint16_t fres = 1;
    volatile uint8_t *regptr;
    
    // Check if given ADC input index is within available range
    if (adin_cfg.ad_input >= ADC_ANINPUT_COUNT) return(0);

    // Determine register set offset
    regptr = (volatile uint8_t *)&ADTRIG0L; // get register block base address
    regptr += (volatile uint8_t)adin_cfg.ad_input;   // add offset
    *regptr = (volatile uint8_t)adin_cfg.config.trigger_source;
    
    // Check if WRITE operation was successful
    fres &= (volatile bool)((*regptr == (volatile uint8_t)adin_cfg.config.trigger_source));  

    return(1);
}

/*!hsadc_set_adc_input_interrupt()
 * ************************************************************************************************
 * Summary:
 * configures the interrupt generation of a single ADC core
 *
 * Parameters:	
 *     - HSADC_ADCAN_CONFIG_t adin_cfg  // ADC input channel settings data structure
 * 
 * Returns:
 *     0: failure
 *     1: success
 * 
 * Description:
 * The individual ADC cores of the ADC peripheral support the generation of interrupts. Further
 * Those interrupts can be "pulled-in" to compensate the interrupt latency of the controller
 * ***********************************************************************************************/

volatile uint16_t hsadc_set_adc_input_interrupt(volatile HSADC_ADCANCFG_t adin_cfg)
{
    volatile uint16_t fres = 1;
    volatile uint16_t ad_idx = 0;
    volatile uint16_t reg_val = 0;
    
    /* ToDo: Add register contents check after WRITE */
    
    if (adin_cfg.ad_input >= ADC_ANINPUT_COUNT) 
        return(0);
    
    ad_idx = (volatile uint16_t)adin_cfg.ad_input;

    if (ad_idx<16) {   

        // Setting the Early Interrupt Enable Bit
        reg_val = (volatile uint16_t)adin_cfg.config.early_interrupt_enable;
        reg_val <<= ad_idx;
        ADEIEL |= (reg_val & REG_ADEIEL_VALID_DATA_MSK); 
        fres &= (volatile bool)((ADEIEL & reg_val) == reg_val);

        // Setting the Interrupt Enable Bit
        reg_val = (volatile uint16_t)adin_cfg.config.interrupt_enable;
        reg_val <<= ad_idx;
        ADIEL |= (reg_val & REG_ADIEL_VALID_DATA_MSK); 
        fres &= (volatile bool)((ADIEL & reg_val) == reg_val);
        
    }
    else {

        ad_idx -= 16; // Shift analog input channel bit back to position zero

        #ifdef ADEIEH
        // Setting the Early Interrupt Enable Bit
        reg_val = (volatile uint16_t)adin_cfg.config.early_interrupt_enable;
        reg_val <<= ad_idx;
        reg_val &= REG_ADEIEH_VALID_DATA_MSK; 
        ADEIEH |= reg_val; 
        fres &= (volatile bool)((ADEIEH & reg_val) == reg_val);
        
        // Setting the Interrupt Enable Bit
        reg_val = (volatile uint16_t)adin_cfg.config.interrupt_enable;
        reg_val <<= ad_idx;
        ADIEH |= (reg_val & REG_ADIEH_VALID_DATA_MSK); 
        fres &= (volatile bool)((ADIEH & reg_val) == reg_val);
        
        #else
        return(0);
        #endif
    }
    
        
	return(fres);
	
}

/*!hsadc_set_adc_input_trigger_mode()
 * ************************************************************************************************
 * Summary:
 * Configures the trigger mode of an individual analog input
 *
 * Parameters:	
 *     - HSADC_ADCAN_CONFIG_t adin_cfg  // ADC input channel settings data structure
 *
 * Returns:
 *     0: failure
 *     1: success
 * 
 * Description:
 * The individual analog inputs of the dsPIC can be triggered by many different sources
 * specified by the trigger source (see function hsadc_set_adc_input_trigger_source()).
 * This function is used to enable the desired trigger mode by setting the related 
 * register bits.
 * ***********************************************************************************************/

volatile uint16_t hsadc_set_adc_input_trigger_mode(volatile HSADC_ADCANCFG_t adin_cfg)
{
    volatile uint16_t fres = 1;
    volatile uint16_t regval = 0;
    volatile uint16_t ad_idx = 0;
    
    // Check if channel number exists
    if (adin_cfg.ad_input >= ADC_ANINPUT_COUNT) return(0);

    ad_idx = adin_cfg.ad_input; // Copy analog input number
    
    // Map bit on right register (HIGH or LOW)
    if (ad_idx<16) {   
        // Setting the Trigger Mode Bit
        regval = (((volatile uint16_t)adin_cfg.config.trigger_mode) << ad_idx);
        ADLVLTRGL |= (regval & REG_ADSTATL_VALID_DATA_MSK);
        fres &= (volatile bool)((ADLVLTRGL & regval) == regval); // Check if bit has been set
    }
    else {

        #ifdef ADLVLTRGH
        ad_idx -= 16;    // Scale ANx-number down into single register field
        regval = (((volatile uint16_t)adin_cfg.config.trigger_mode) << ad_idx);
        ADLVLTRGH |= (regval & REG_ADSTATH_VALID_DATA_MSK);
        fres &= (volatile bool)((ADLVLTRGH & regval) == regval); // Check if bit has been set
        #else
        return(0);  // Return ERROR if register does not exists
        #endif
    }

    // Return Success/Error bit
	return(fres);
	
}

/*!hsadc_set_adc_input_mode()
 * ************************************************************************************************
 * Summary:
 * Configures the analog input mode of an individual analog input
 *
 * Parameters:	
 *     - index: 
 *         index of the ADC input (e.g. 0 for AN, 1 for AN1, etc)
 *     - ADMOD_INPUT_MODE_e input_mode: 
 *         0 = Single-Ended
 *         1 = Differential
 *     - ADMOD_OUTPUT_DATA_MODE_e data_mode:
 *         0 = unsigned number
 *         1 = signed number
 * 
 * Returns:
 *     0: failure
 *     1: success
 * 
 * Description:
 * The individual analog inputs of the dsPIC can be configured in single-ended or differential mode.
 * Each configuration has to be considered during the hardware design to make sure the signals
 * are captured correctly.
 * This function is used to enable the desired mode by setting the related register bits.
 * ***********************************************************************************************/

volatile uint16_t hsadc_set_adc_input_mode(volatile HSADC_ADCANCFG_t adin_cfg) 
{
    volatile uint16_t fres = 1;
    volatile uint16_t ad_idx=0;
    volatile uint16_t regval = 0;
    
    // Check if channel number exists
    if (adin_cfg.ad_input >= ADC_ANINPUT_COUNT) 
        return(0);

    ad_idx = adin_cfg.ad_input; // Copy analog input channel number
    
    // Build dual-bit value of settings
    regval = (volatile uint16_t)adin_cfg.config.input_mode;
    regval <<= 1;
    regval |= (volatile uint16_t)adin_cfg.config.data_mode;
    
    // Map bit on right register (HIGH or LOW)
    if (adin_cfg.ad_input<8) {   
        // Setting the analog input mode of channel 0 to 7
        ad_idx <<= 1;    // Multiply index by 2
        regval <<= ad_idx; // Shift bits up to channel index
        ADMOD0L |= (regval & REG_ADMOD0L_VALID_DATA_MSK); // write value into register
        fres &= (volatile bool)((ADMOD0L & regval) == regval); // Check if bit has been set
    }
    else if (adin_cfg.ad_input<16) {   
        // Setting the analog input mode of channel 8 to 15
        ad_idx -= 8;     // Scale ANx-number down into single register field
        ad_idx <<= 1;    // Multiply index by 2
        regval <<= ad_idx; // Shift bits up to channel index
        ADMOD0H |= (regval & REG_ADMOD0H_VALID_DATA_MSK); // write value into register
        fres &= (volatile bool)((ADMOD0H & regval) == regval); // Check if bit has been set
    }
    else if (adin_cfg.ad_input<24) {   
        // Setting the analog input mode of channel 16 to 23
        ad_idx -= 16;     // Scale ANx-number down into single register field
        ad_idx <<= 1;    // Multiply index by 2
        regval <<= ad_idx; // Shift bits up to channel index
        ADMOD1L |= (regval & REG_ADMOD1L_VALID_DATA_MSK); // write value into register
        fres &= (volatile bool)((ADMOD1L & regval) == regval); // Check if bit has been set
    }
    else {

        #if defined (ADMOD1H)
        // Setting the analog input mode of channel 24 to 31
        ad_idx -= 24;     // Scale ANx-number down into single register field
        ad_idx <<= 1;    // Multiply index by 2
        regval <<= ad_idx; // Shift bits up to channel index
        ADMOD1H |= (regval & REG_ADMOD1H_VALID_DATA_MSK); // write value into register
        fres &= (volatile bool)((ADMOD1H & regval) == regval); // Check if bit has been set
        
        #else
        fres = 0;  // Return ERROR if register does not exists
        #endif
    }


    // Return Success/Error bit
	return(fres);
	
}



/*!hsadc_init_adc_comp
 * ************************************************************************************************
 * Summary:
 * Initializes an individual ADC digital Comparator configuration
 *
 * Parameters:
 *	HSADC_ADCMP_CONFIG_t adcmp_cfg = holds the ADC digital comparator configuration and values
 *                                   of upper and lower comparison thresholds
 * 
 * Description:
 * The ADC peripheral features a digital comparator, which compares the latest sample of a dedicated
 * ANx input with given upper and lower threshold values. If the values is outside the given range,
 * an interrupt will be triggered (if enabled).
 * This routine allows configuration of the comparator, the input source and its thresholds.
 * ***********************************************************************************************/

volatile uint16_t hsadc_init_adc_comp(volatile uint16_t index, volatile HSADC_ADCMP_CONFIG_t adcmp_cfg)
{

    volatile uint16_t fres = 1;
    volatile uint16_t *regptr16;
    volatile uint16_t reg_offset = 0;
    volatile uint16_t reg_value = 0;


	if (index >= (ADC_ADCMP_COUNT-1)) return(0);    // Check if analog input number is valid
    
    // ADC Comparator Configuration
    reg_offset = (index) * ((volatile uint16_t)&ADCMP1CON - (volatile uint16_t)&ADCMP0CON); // Get register offset
    regptr16 = (volatile uint16_t *)&ADCMP0CON + reg_offset;      // Set digital comparator configuration value
    *regptr16 = (adcmp_cfg.ADCMPxCON.value & REG_ADCMPxCON_VALID_DATA_WR_MSK); // Write digital comparator value
    fres &= ((*regptr16 & REG_ADCMPxCON_VALID_DATA_RD_MSK) == (adcmp_cfg.ADCMPxCON.value & REG_ADCMPxCON_VALID_DATA_WR_MSK)); // Test if written value matches parameter
    
    // Lower Threshold Compare Value
    reg_offset = (index) * ((volatile uint16_t)&ADCMP1LO - (volatile uint16_t)&ADCMP0LO); // Get register offset
    regptr16 = (volatile uint16_t *)&ADCMP0LO + reg_offset;       // Get lower threshold value
    *regptr16 = (adcmp_cfg.ADCMPxLO & REG_ADCMPxLO_VALID_DATA_MSK);     // Write lower threshold value
    fres &= ((*regptr16 & REG_ADCMPxLO_VALID_DATA_MSK) == (adcmp_cfg.ADCMPxLO & REG_ADCMPxLO_VALID_DATA_MSK)); // Test if written value matches parameter

    // Upper Threshold Compare Value
    reg_offset = (index) * ((volatile uint16_t)&ADCMP1HI - (volatile uint16_t)&ADCMP0HI); // Get register offset
    regptr16 = (volatile uint16_t *)&ADCMP0HI + reg_offset;       // Get upper threshold value
    *regptr16 = (adcmp_cfg.ADCMPxHI & REG_ADCMPxHI_VALID_DATA_MSK);     // Write upper threshold value
    fres &= ((*regptr16 & REG_ADCMPxHI_VALID_DATA_MSK) == (adcmp_cfg.ADCMPxHI & REG_ADCMPxHI_VALID_DATA_MSK)); // Test if written value matches parameter
    
    
    // Capture ANx input number as source for comparison (this bit is automatically set and doesn't require to be set by the user)
    reg_value = adcmp_cfg.ADCMPxCON.bits.chnl;  // Read ANx input number as comparison source
    
    if (reg_value < 16) {   

        // Enabling the corresponding analog input comparator input
        reg_offset = (index) * ((volatile uint16_t)&ADCMP1ENL - (volatile uint16_t)&ADCMP0ENL); // Get register offset
        regptr16 = (volatile uint16_t *)&ADCMP0ENL + reg_offset;    // Get upper threshold value
        *regptr16 = ((1 << reg_value) & REG_ADCMPxENH_VALID_DATA_MSK); // Write input channel enable bit

    }
    else if (reg_value < 32) {

        #ifdef ADCMP0ENH
        reg_value -= 16; // "Scale" analog input number 16-31 into register 

        // Enabling the corresponding analog input comparator input
        reg_offset = (index) * ((volatile uint16_t)&ADCMP1ENH - (volatile uint16_t)&ADCMP0ENH); // Get register offset
        regptr16 = (volatile uint16_t *)&ADCMP0ENH + reg_offset;    // Get upper threshold value
        *regptr16 = ((1 << reg_value) & REG_ADCMPxENH_VALID_DATA_MSK); // Write input channel enable bit
        #else
        return(0);
        #endif
    }
    else { return(0); }

    // Check if Analog Input Comparator Enable has been set correctly
    fres &= (volatile bool)(*regptr16 & (1 << reg_value)); // Test if written value matches parameter

    return(fres);
    
}


/*!hsadc_init_adc_filter
 * ************************************************************************************************
 * Summary:
 * Initializes an individual ADC digital Filter configuration
 *
 * Parameters:
 *	HSADC_ADFLT_CONFIG_t adflt_cfg = holds the ADC digital filter configuration and values
 *                                   of upper and lower comparison thresholds
 * 
 * Description:
 * The ADC peripheral features a digital filter, which offers averaging and oversampling 
 * filter capabilities. Depending on the setting, this filter will be averaging the most recent 
 * samples of the assigned analog input ANx until the averaging setting is matched and the data
 * is provided in an independent register FLTxDAT. In oversampling mode, the assigned analog 
 * input ANx is continuously oversampled until the oversampling setting is matched. 
 * 
 * This routine allows configuration of the comparator, the input source and its thresholds.
 * The filter result register FLTxDAT will be reset or pre-charged, but not updated automatically.
 * ***********************************************************************************************/

volatile uint16_t hsadc_init_adc_filter(volatile uint16_t index, volatile HSADC_ADFLT_CONFIG_t adflt_cfg) {

    volatile uint16_t fres = 1;
    volatile uint16_t *regptr16;
    volatile uint16_t reg_offset = 0;


	// Check if analog input number is valid
    if (index >= (ADC_ADFL_COUNT-1)) 
        return(0);    
    
    // ADC Comparator Configuration
    reg_offset = (index) * ((volatile uint16_t)&ADFL1CON - (volatile uint16_t)&ADFL0CON); // Get register offset
    regptr16 = (volatile uint16_t *)&ADFL0CON + reg_offset;      // Set digital comparator configuration value
    *regptr16 = (adflt_cfg.ADFLxCON.value & REG_ADFLxCON_VALID_DATA_WR_MSK); // Write digital comparator value
    fres &= ((*regptr16 & REG_ADFLxCON_VALID_DATA_RD_MSK) == (adflt_cfg.ADFLxCON.value & REG_ADFLxCON_VALID_DATA_WR_MSK)); // Test if written value matches parameter
    
    // Lower Threshold Compare Value
    reg_offset = (index) * ((volatile uint16_t)&ADFL1DAT - (volatile uint16_t)&ADFL0DAT); // Get register offset
    regptr16 = (volatile uint16_t *)&ADFL0DAT + reg_offset;       // Get lower threshold value
    *regptr16 = (adflt_cfg.ADFLxDAT & REG_ADFLxDAT_VALID_DATA_MSK);     // Write lower threshold value
    fres &= ((*regptr16 & REG_ADFLxDAT_VALID_DATA_MSK) == (adflt_cfg.ADFLxDAT & REG_ADFLxDAT_VALID_DATA_MSK)); // Test if written value matches parameter

    
    return(fres);
    
}



// EOF
