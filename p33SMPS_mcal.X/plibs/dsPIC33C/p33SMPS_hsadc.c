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

inline volatile uint16_t hsadc_module_power_up(void)
{
    volatile uint16_t fres=0;
    
        #if defined (_ABGMD)
        _ABGMD = 0;         // Turn on power to analog bandgap reference
        fres = (1-_ABGMD);
        #endif
        #if defined (_ADCMD)
        _ADCMD = 0; 		// Turn on power to ADC module
        fres &= (1-_ADCMD);
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

inline volatile uint16_t hsadc_module_power_down(void)
{
    #if defined (_ADCMD)
    _ADCMD = 1; 		// Turn on power to PWM channel #1
    return(_ADCMD);
    #else
    return(1);
    #endif

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
inline volatile uint16_t hsadc_init_adc_module( HSADC_MODULE_CONFIG_t adc_cfg )
{
    volatile uint16_t fres = 1;
    volatile uint32_t *regptr32;
    volatile uint32_t regbuf32;
    
    // Ensure ADC module is powered up before registers are written
    fres &= hsadc_module_power_up();
    
	// Reset all ADC configuration registers to defaults

	ADCON1L	= REG_ADCON1L_RESET;			// Disable and reset ADC configuration register 1 low
	ADCON1H	= REG_ADCON1H_RESET;			// Disable and reset ADC configuration register 1 high
	ADCON2L	= REG_ADCON2L_RESET;			// Disable and reset ADC configuration register 2 low
	ADCON2H	= REG_ADCON2H_RESET;			// Disable and reset ADC configuration register 2 high
    ADCON3L = REG_ADCON3L_RESET;			// Disable and reset ADC configuration register 3 low
    ADCON3H = REG_ADCON3H_RESET;			// Disable and reset ADC configuration register 3 high
    #if defined (ADCON4L) && defined (ADCON4H)  // Registers ADCON4L/H are only available if ADC has dedicated cores
    ADCON4L = REG_ADCON4L_RESET;			// Disable and reset ADC configuration register 4 low
    ADCON4H = REG_ADCON4H_RESET;			// Disable and reset ADC configuration register 4 high
    #endif
    ADCON5L = REG_ADCON5L_RESET;			// Disable and reset ADC configuration register 5 low
    ADCON5H = REG_ADCON5H_RESET;			// Disable and reset ADC configuration register 5 high
    
	// Setting ADC configuration block #1 according to user settings.
	// Please note:
	//   ADC ENABLE is masked out! The ADC has to be enabled by the user in software.
		
    regptr32 = (volatile uint32_t*) ((volatile uint8_t*)&ADCON1L);
    *regptr32 = (adc_cfg.ADCON1.value & (REG_ADCON1_OFF_STATE_WRITE_MASK));
    regbuf32 = *regptr32;
    if(regbuf32 == (adc_cfg.ADCON1.value & REG_ADCON1_OFF_STATE_WRITE_MASK))
    { fres = 1; }
    else
    { fres = 0; }
    
	// Setting ADC configuration block #2 according to user settings.
    
    regptr32 = (volatile uint32_t*) ((volatile uint8_t*)&ADCON2L);
    *regptr32 = (adc_cfg.ADCON2.value & (REG_ADCON2_VALID_DATA_WRITE_MASK));
    regbuf32 = *regptr32;
    if(regbuf32 == (adc_cfg.ADCON2.value & REG_ADCON2_VALID_DATA_WRITE_MASK))
    { fres &= 1; }
    else
    { fres = 0; }

	// Setting ADC configuration block #3 according to user settings.
    
    regptr32 = (volatile uint32_t*) ((volatile uint8_t*)&ADCON3L);
    *regptr32 = ((adc_cfg.ADCON3.value & REG_ADCON3_DISABLE_ADC_CORES_MASK) & (REG_ADCON3_VALID_DATA_WRITE_MASK));
    regbuf32 = *regptr32;
    if(regbuf32 == ((adc_cfg.ADCON3.value & REG_ADCON3_DISABLE_ADC_CORES_MASK) & REG_ADCON3_VALID_DATA_WRITE_MASK))
    { fres &= 1; }
    else
    { fres = 0; }

	// Setting ADC configuration block #3 according to user settings.
    #if defined(ADCON4L)
    regptr32 = (volatile uint32_t*) ((volatile uint8_t*)&ADCON4L);
    *regptr32 = (adc_cfg.ADCON4.value & (REG_ADCON4_VALID_DATA_WRITE_MASK));
    regbuf32 = *regptr32;
    if(regbuf32 == (adc_cfg.ADCON4.value & REG_ADCON4_VALID_DATA_WRITE_MASK))
    { fres &= 1; }
    else
    { fres = 0; }
    #endif
    
	// Setting ADC configuration block #5 according to user settings.
    
    regptr32 = (volatile uint32_t*) ((volatile uint8_t*)&ADCON5L);
    *regptr32 = ((adc_cfg.ADCON5.value & REG_ADCON5_DISABLE_ADC_CORES_MASK) & (REG_ADCON5_VALID_DATA_WRITE_MASK));
    regbuf32 = *regptr32;
    if(regbuf32 == ((adc_cfg.ADCON5.value & REG_ADCON5_DISABLE_ADC_CORES_MASK) & REG_ADCON5_VALID_DATA_WRITE_MASK))
    { fres &= 1; }
    else
    { fres = 0; }


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

inline volatile uint16_t hsadc_init_adc_channel( HSADC_CHANNEL_CONFIG_t adin_cfg ) {
    
    volatile uint16_t fres = 1;
    
    // Check if given analog input number is available
    if(adin_cfg.ad_input > ADC_ANINPUT_COUNT) { return(0); }
    
    // Set AD input mode (differential or single ended)
    fres &= hsadc_set_adc_input_mode(adin_cfg.ad_input, 
            adin_cfg.config.bits.input_mode, adin_cfg.config.bits.data_mode);

    // Set interrupt enable and early interrupt enable
    fres &= hsadc_set_adc_input_interrupt(adin_cfg.ad_input, 
            adin_cfg.config.bits.interrupt_enable, adin_cfg.config.bits.early_interrupt_enable);
    
    // Set interrupt trigger source
    fres &= hsadc_set_adc_input_trigger_source(adin_cfg.ad_input, adin_cfg.config.bits.trigger_source);

    // Set interrupt trigger mode (level/edge)
    fres &= hsadc_set_adc_input_trigger_mode(adin_cfg.ad_input, adin_cfg.config.bits.trigger_mode);

    return(fres);
    
}


/*!hsadc_init_adc_core
 * ************************************************************************************************
 * Summary:
 * Initializes an individual ADC Core configuration
 *
 * Parameters:
 *	regADCORExL	= holds the register value for the ADCORExL register
 *	regADCORExH	= holds the register value for the ADCORExL register
 *
 * Description:
 * Basic options like conversion delay, early interrupt period, ADC resolution and input clock
 * dividers are set here.
 * ***********************************************************************************************/

inline volatile uint16_t hsadc_init_adc_core(uint16_t index, uint16_t regADCORExL, uint16_t regADCORExH)
{

volatile uint16_t *regptr;
volatile uint16_t reg_buf=0;

#if defined (ADCORE0L)
volatile uint16_t reg_offset=0;
#endif

    /* ToDo: Add register contents check after WRITE */


	if (index >= ADC_CORE_COUNT) return(0);

    if(index != ADC_SHARED_CORE_INDEX)
    {
        // dedicated cores have their individual configuration 
        // registers ADCORExL and ADCORExH
        
        #if defined (ADCORE0L)
        // Dual core devices like dsPIC33CH only have shared cores on the master side.
        // Only slave cores have multiple, dedicated cores

        reg_offset = (index-1) * ((volatile uint16_t)&ADCORE1L - (volatile uint16_t)&ADCORE0L);

        regptr = (volatile uint16_t *)&ADCORE0L + reg_offset;
        reg_buf = (regADCORExL & REG_ADCORExL_VALID_DATA_MSK);
        if((*regptr & REG_ADCORExL_VALID_DATA_MSK) != reg_buf)
        { *regptr = reg_buf; }

        regptr = (volatile uint16_t *)&ADCORE0H + reg_offset;
        reg_buf = (regADCORExH & REG_ADCORExH_VALID_DATA_MSK);
        if((*regptr & REG_ADCORExH_VALID_DATA_MSK) != reg_buf)
        { *regptr = reg_buf; }

        #endif
        
    }
    else
    {
    
        // the configuration of the shared core is incorporated into 
        // the generic registers ADCON2L and ADCON2H
        // Unfortunately register assignments of xxxH and xxxL are reversed
        // for shared and dedicated cores.... So don't get confused !!!
        
        regptr = (volatile uint16_t *)&ADCON2L;
        reg_buf = (*regptr & REG_ADCON2L_REF_CFG_MASK);  // Read bandgap reference register bits
        reg_buf |= (regADCORExH & REG_ADCON2L_SHRADC_CFG_MASK);
        if((*regptr & REG_ADCON2L_SHRADC_CFG_MASK) != (reg_buf & REG_ADCON2L_SHRADC_CFG_MASK))
        { *regptr = reg_buf; }

        
        regptr = (volatile uint16_t *)&ADCON2H;
        reg_buf = (*regptr & REG_ADCON2H_REF_CFG_MASK);  // Read bandgap reference register bits
        reg_buf |= (regADCORExL & REG_ADCON2H_SHRADC_CFG_MASK);
        if((*regptr & REG_ADCON2H_SHRADC_CFG_MASK) != (reg_buf & REG_ADCON2H_SHRADC_CFG_MASK))
        { *regptr = reg_buf; }
    
    }

    return(1);
    
}


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

inline volatile uint16_t hsadc_module_enable(void)
{

	ADCON1Lbits.ADON	= ADC_ON; 	// Enable ADC module
	return(ADCON1Lbits.ADON);
	 
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

inline volatile uint16_t hsadc_module_disable(void)
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

inline volatile uint16_t hsadc_reset(void) {
    
    /* ToDo: Add register contents check after WRITE */
    
	// Reset all ADC configuration registers to defaults

	ADCON1Lbits.ADON = ADC_OFF;				// Disable ADC
  
	ADCON1L	   = REG_ADCON1L_RESET;			// Disable and reset ADC configuration register 1 low
	ADCON1H	   = REG_ADCON1H_RESET;			// Disable and reset ADC configuration register 1 high
	ADCON2L	   = REG_ADCON2L_RESET;			// Disable and reset ADC configuration register 2 low
	ADCON2H	   = REG_ADCON2H_RESET;			// Disable and reset ADC configuration register 2 high
    ADCON3L    = REG_ADCON3L_RESET;			// Disable and reset ADC configuration register 3 low
    ADCON3H    = REG_ADCON3H_RESET;			// Disable and reset ADC configuration register 3 high
    #if defined (ADCON4L)
    ADCON4L    = REG_ADCON4L_RESET;			// Disable and reset ADC configuration register 4 low
    #endif
    #if defined (ADCON4H)
    ADCON4H    = REG_ADCON4H_RESET;			// Disable and reset ADC configuration register 4 high
    #endif
    ADCON5L    = REG_ADCON5L_RESET;			// Disable and reset ADC configuration register 5 low
    ADCON5H    = REG_ADCON5H_RESET;			// Disable and reset ADC configuration register 5 high

    /* ToDo: Rework RESET routine to filter on available registers
    ADMOD0L    = REG_ADMOD0L_RESET;
    ADMOD0H    = REG_ADMOD0H_RESET;
    ADMOD1L    = REG_ADMOD1L_RESET;   
    ADMOD1H    = REG_ADMOD1H_RESET;     
    ADIEL      = REG_ADIEL_RESET;
    ADIEH      = REG_ADIEH_RESET;
    ADSTATL    = REG_ADSTATL_RESET;      
    ADSTATH    = REG_ADSTATH_RESET;  
    ADCMP0ENL  = REG_ADCMPxENL_RESET;
    ADCMP0ENH  = REG_ADCMPxENH_RESET; 
    ADCMP0LO   = REG_ADCMPxLO_RESET;
    ADCMP0HI   = REG_ADCMPxHI_RESET;
    ADCMP1ENL  = REG_ADCMPxENL_RESET;
    ADCMP1ENH  = REG_ADCMPxENH_RESET;
    ADCMP1HI   = REG_ADCMPxHI_RESET;
    ADCMP2ENL  = REG_ADCMPxENL_RESET;
    ADCMP2ENH  = REG_ADCMPxENH_RESET;
    ADCMP2LO   = REG_ADCMPxLO_RESET;
    ADCMP2HI   = REG_ADCMPxHI_RESET;
    ADCMP3ENL  = REG_ADCMPxENL_RESET;
    ADCMP3ENH  = REG_ADCMPxENH_RESET;
    ADCMP3LO   = REG_ADCMPxLO_RESET;
    ADCMP3HI   = REG_ADCMPxHI_RESET;
    ADFL0DAT   = REG_ADFLxDAT_RESET;
    ADFL0CON   = REG_ADFLxCON_RESET;
    ADFL1DAT   = REG_ADFLxDAT_RESET;
    ADFL1CON   = REG_ADFLxCON_RESET;
    ADFL2DAT   = REG_ADFLxDAT_RESET;
    ADFL2CON   = REG_ADFLxCON_RESET;
    ADFL3DAT   = REG_ADFLxDAT_RESET;
    ADFL3CON   = REG_ADFLxCON_RESET;
    ADTRIG0L   = REG_ADTRIGxL_TRGSRC_NONE;
    ADTRIG0H   = REG_ADTRIGxH_TRGSRC_NONE;
    ADTRIG1L   = REG_ADTRIGxL_TRGSRC_NONE;
    ADTRIG1H   = REG_ADTRIGxH_TRGSRC_NONE;
    ADTRIG2L   = REG_ADTRIGxL_TRGSRC_NONE;
    ADTRIG2H   = REG_ADTRIGxL_TRGSRC_NONE;
    ADTRIG3L   = REG_ADTRIGxL_TRGSRC_NONE;
    ADTRIG3H   = REG_ADTRIGxH_TRGSRC_NONE;
    ADTRIG4L   = REG_ADTRIGxL_TRGSRC_NONE;
    ADTRIG4H   = REG_ADTRIGxH_TRGSRC_NONE;
    ADTRIG5L   = REG_ADTRIGxL_TRGSRC_NONE;
    ADTRIG5H   = REG_ADTRIGxH_TRGSRC_NONE;
    ADTRIG6L   = REG_ADTRIGxL_TRGSRC_NONE;
    ADCMP0CON  = REG_ADCMPxCON_RESET;
    ADCMP1CON  = REG_ADCMPxCON_RESET;
    ADCMP2CON  = REG_ADCMPxCON_RESET;
    ADCMP3CON  = REG_ADCMPxCON_RESET;
    ADLVLTRGL  = REG_ADLVLTRGL_RESET;
    ADLVLTRGH  = REG_ADLVLTRGH_RESET;
    ADCORE0L   = REG_ADCORExL_RESET;
    ADCORE0H   = REG_ADCORExH_RESET;
    ADCORE1L   = REG_ADCORExL_RESET;
    ADCORE1H   = REG_ADCORExH_RESET;
    ADEIEL     = REG_ADEIEL_RESET;
    ADEIEH     = REG_ADEIEH_RESET;
    ADEISTATL  = REG_ADEISTATL_RESET;
    ADEISTATH  = REG_ADEISTATH_RESET;
    */
    
	return(1);
}

/*!hsadc_power_on_adc_core()
 * ************************************************************************************************
 * Summary:
 * Turns on a single ADC core considering warm-up time
 *
 * Parameters:	(none)
 *
 * Description:
 * The individual ADC cores of the ADC peripheral require a self calibration procedure 
 * to meet the values given in the data sheet. The following function performs the self
 * calibration of the given ADC core.
 * ***********************************************************************************************/

inline volatile uint16_t hsadc_check_adc_cores_ready(void)
{
    volatile uint16_t timeout = 0, rdy_compare = 0, reg_buf = 0;

    /* ToDo: Add register contents check after WRITE */
    
    reg_buf = (ADCON5L & 0x00FF);
    rdy_compare = (reg_buf << 8);

    while( (((ADCON5L << 8) & 0xFF00) != rdy_compare) && (timeout++ < 0xFFFE) );
    if(timeout == 0xFFFF) return(0);
    else return(1);

}

/*!hsadc_calibrate_adc_core()
 * ************************************************************************************************
 * Summary:
 * Calls the ADC calibration of a single ADC core
 *
 * Parameters:	(none)
 *
 * Description:
 * The individual ADC cores of the ADC peripheral require a self calibration procedure 
 * to meet the values given in the data sheet. The following function performs the self
 * calibration of the given ADC core.
 * ***********************************************************************************************/
#if defined (__P33SMPS_EP__)
inline volatile uint16_t hsadc_calibrate_adc_core(uint16_t index, uint16_t calib_mode)
{
	volatile uint16_t timeout=0;
    volatile uint16_t *regptr;
    volatile uint16_t reg_offset=0;
    
    if (index >= ADC_CORE_COUNT) return(0);
    
    // Determine register set offset
    reg_offset = (index) * ((volatile uint16_t)&ADCAL1L - (volatile uint16_t)&ADCAL0L);
    regptr = (volatile uint16_t *)&ADCAL0L + reg_offset;
    
    // differentiate between odd and even indices due to shared registers
    if (index == ADC_SHARED_CORE_INDEX) {

        regptr = (volatile uint16_t *)&ADCAL1H;

        if ((*regptr & REG_ADCALx_HB_CALxRDY_SET_MSK) == 0)
        {
            *regptr |= REG_ADCALx_HB_CALxEN_SET_MSK;  // Enable calibration
            *regptr &= REG_ADCALx_HB_CALxSKIP_CLR_MSK;  // Initiate calibration
            *regptr &= ((calib_mode << 8) | REG_ADCALx_HB_CALxDIFF_CLR_MSK);  // Set calibration mode (differential or single ended))
            *regptr |= REG_ADCALx_HB_CALxRUN_SET_MSK;  // Initiate calibration

            // Wait until ADC core calibration has completed
            while( ((*regptr & REG_ADCALx_HB_CALxRDY_SET_MSK) == 0) && (timeout++<5000));
            if((*regptr & REG_ADCALx_HB_CALxRDY_SET_MSK) == 0) return(0);  // If core doesn't enter READY state, return failure

            *regptr &= REG_ADCALx_HB_CALxEN_CLR_MSK;  // Disable calibration
        }
        
    }
    else if(index & 0x0001) {
        // Odd index (1, 3, 5, ...) and shared ADC bits are located in the high-byte of the register word
        
        if ((*regptr & REG_ADCALx_HB_CALxRDY_SET_MSK) == 0)
        {
            *regptr |= REG_ADCALx_HB_CALxEN_SET_MSK;  // Enable calibration
            *regptr &= REG_ADCALx_HB_CALxSKIP_CLR_MSK;  // Initiate calibration
            *regptr &= ((calib_mode << 8) | REG_ADCALx_HB_CALxDIFF_CLR_MSK);  // Set calibration mode (differential or single ended))
            *regptr |= REG_ADCALx_HB_CALxRUN_SET_MSK;  // Initiate calibration

            // Wait until ADC core calibration has completed
            while( ((*regptr & REG_ADCALx_HB_CALxRDY_SET_MSK) == 0) && (timeout++<5000));
            if((*regptr & REG_ADCALx_HB_CALxRDY_SET_MSK) == 0) return(0);  // If core doesn't enter READY state, return failure

            *regptr &= REG_ADCALx_HB_CALxEN_CLR_MSK;  // Disable calibration
        }
    }
    else{
        // Even index (0, 2, 4, ...) bits are located in the low-byte of the register word
        
        if ((*regptr & REG_ADCALx_LB_CALxRDY_SET_MSK) == 0)
        {
            *regptr |= REG_ADCALx_LB_CALxEN_SET_MSK;  // Enable calibration
            *regptr &= REG_ADCALx_LB_CALxSKIP_CLR_MSK;  // Initiate calibration
            *regptr &= (calib_mode | REG_ADCALx_LB_CALxDIFF_CLR_MSK);  // Set calibration mode (differential or single ended))
            *regptr |= REG_ADCALx_LB_CALxRUN_SET_MSK;  // Initiate calibration

            // Wait until ADC core calibration has completed
            while( ((*regptr & REG_ADCALx_LB_CALxRDY_SET_MSK) == 0) && (timeout++<5000));
            if((*regptr & REG_ADCALx_LB_CALxRDY_SET_MSK) == 0) return(0);  // If core doesn't enter READY state, return failure

            *regptr &= REG_ADCALx_LB_CALxEN_CLR_MSK;  // Disable calibration
        }
    }
	
	return(1);
	
}
#endif
/*!hsadc_power_on_adc_core()
 * ************************************************************************************************
 * Summary:
 * Turns on a single ADC core considering warm-up time
 *
 * Parameters:	(none)
 *
 * Description:
 * The individual ADC cores of the ADC peripheral require a self calibration procedure 
 * to meet the values given in the data sheet. The following function performs the self
 * calibration of the given ADC core.
 * ***********************************************************************************************/

inline volatile uint16_t hsadc_power_on_adc_core(uint16_t index)
{
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
        reg_buf  = (REG_ADCON5L_SHRPWR_ON & REG_ADCON5L_VALID_DATA_WRITE_MASK);
        
    }
    else {
      // Dedicated core power on enable bits are set based on the index/bit position 
        reg_buf  = ((0x0001 << index) & REG_ADCON5L_VALID_DATA_WRITE_MASK);
    }

    if(!(*regptr & reg_buf))   // if bit hasn't been set yet...
    { *regptr |= reg_buf; } // write to register


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
    
	return(1);
	
}

/*!hsadc_set_adc_core_trigger()
 * ************************************************************************************************
 * Summary:
 * Configures interrupt trigger setting of an individual Analog Input
 *
 * Parameters:
 * Returns: 	1: success, 0: failure
 *
 * Description:
 * 
 * ***********************************************************************************************/

inline volatile uint16_t hsadc_set_adc_input_trigger_source(uint16_t index, ADTRIG_TRGSRC_e trigger_source)
{

    volatile uint16_t *regptr;
    volatile uint16_t reg_offset;
    volatile uint16_t reg_buf=0;

    /* ToDo: Add register contents check after WRITE */
    
    if (index >= ADC_ANINPUT_COUNT) return(0);

    // Determine register set offset
    reg_offset = (index) * ((volatile uint16_t)&ADTRIG1L - (volatile uint16_t)&ADTRIG0L);
    regptr = (volatile uint16_t *)&ADTRIG0L + reg_offset;

    if (index & 0x0001) {
    // Odd analog input numbers are located in the high-byte of the register word
        reg_buf = ((trigger_source << 8) & REG_ADTRIGx_VALID_DATA_MSK);
        *regptr |= reg_buf;
    }
    else{
    // Even analog input numbers are located in the low-byte of the register word
        reg_buf = (trigger_source & REG_ADTRIGx_VALID_DATA_MSK);
        *regptr |= reg_buf;

    }

    return(1);
}

/*!hsadc_set_adc_input_interrupt()
 * ************************************************************************************************
 * Summary:
 * configures the interrupt generation of a single ADC core
 *
 * Parameters:	
 *     - index: 
 *         index of the ADC input (e.g. 0 for AN, 1 for AN1, etc)
 *     - interrupt_enable: 
 *         0 = no interrupt will be generated
 *         1 = interrupt will be generated
 *     - early_interrupt_enable: 
 *         0 = interrupt will be triggered after conversion is complete
 *         1 = interrupt will be triggered n TADs before conversion is complete
 * 
 * Returns:
 *     0: failure
 *     1: success
 * 
 * Description:
 * The individual ADC cores of the ADC peripheral support the generation of interrupts. Further
 * Those interrupts can be "pulled-in" to compensate the interrupt latency of the controller
 * ***********************************************************************************************/

inline volatile uint16_t hsadc_set_adc_input_interrupt(uint16_t index, uint16_t interrupt_enable, uint16_t early_interrupt_enable)
{
    volatile uint16_t *regptr;

    /* ToDo: Add register contents check after WRITE */
    
    if (index >= ADC_ANINPUT_COUNT) return(0);
    
    if (index<16) {   

        // Setting the Early Interrupt Enable Bit
        regptr = (volatile uint16_t *)&ADEIEL;
        *regptr |= (early_interrupt_enable << index);

        // Setting the Interrupt Enable Bit
        regptr = (volatile uint16_t *)&ADIEL;
        *regptr |= (interrupt_enable << index);
    }
    else {

        index -= 16;

        #ifdef ADEIEH
        // Setting the Early Interrupt Enable Bit
        regptr = (volatile uint16_t *)&ADEIEH;
        *regptr |= (early_interrupt_enable << index);

        // Setting the Interrupt Enable Bit
        regptr = (volatile uint16_t *)&ADIEH;
        *regptr |= (interrupt_enable << index);
        #else
        return(0);
        #endif
    }
    
	return(1);
	
}

/*!hsadc_set_adc_input_trigger_mode()
 * ************************************************************************************************
 * Summary:
 * Configures the trigger mode of an individual analog input
 *
 * Parameters:	
 *     - uint16_t index: 
 *         index of the ADC input (e.g. 0 for AN, 1 for AN1, etc)
 *     - ADLVLTRG_e trigger_mode: 
 *         0 = Edge trigger mode
 *         1 = level trigger mode
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

inline volatile uint16_t hsadc_set_adc_input_trigger_mode(uint16_t index, ADLVLTRG_e trigger_mode)
{
    volatile uint16_t fres = 1;
    volatile uint16_t *regptr;
    volatile uint16_t regval = 0;
    
    // Check if channel number exists
    if (index >= ADC_ANINPUT_COUNT) return(0);

    // Map bit on right register (HIGH or LOW)
    if (index<16) {   
        // Setting the Trigger Mode Bit
        regptr = (volatile uint16_t *)&ADLVLTRGL;
        regval = (((volatile uint16_t)trigger_mode << index) & REG_ADSTATL_VALID_DATA_MSK);
    }
    else {

        #ifdef ADLVLTRGH
        index -= 16;    // Scale ANx-number down into single register field
        regptr = (volatile uint16_t *)&ADLVLTRGH; // Set the Trigger Mode Bit
        regval = (((volatile uint16_t)trigger_mode << index) & REG_ADSTATH_VALID_DATA_MSK);
        #else
        return(0);  // Return ERROR if register does not exists
        #endif
    }

    // write value to register
    *regptr |= regval;
    
    // Check if bit has been set
    fres &= (volatile bool)((*regptr & regval) == regval);

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

inline volatile uint16_t hsadc_set_adc_input_mode(uint16_t index, ADMOD_INPUT_MODE_e input_mode, ADMOD_OUTPUT_DATA_MODE_e data_mode) 
{
    volatile uint16_t fres = 1;
    volatile uint16_t *regptr;
    volatile uint16_t regval = 0;
    
    // Check if channel number exists
    if (index >= ADC_ANINPUT_COUNT) return(0);

    // Build dual-bit value of settings
    regval = ((uint16_t)input_mode << 1) | ((uint16_t)data_mode);
    
    // Map bit on right register (HIGH or LOW)
    if (index<8) {   
        // Setting the analog input mode of channel 0 to 7
        regptr = (volatile uint16_t *)&ADMOD0L;
        index <<= 1;    // Multiply index by 2
        regval = (((volatile uint16_t)input_mode << index) & REG_ADMOD0L_VALID_DATA_MSK);
    }
    else if (index<16) {   
        // Setting the analog input mode of channel 8 to 15
        index -= 8;     // Scale ANx-number down into single register field
        regptr = (volatile uint16_t *)&ADMOD0H;
        index <<= 1;    // Multiply index by 2
        regval = (((volatile uint16_t)input_mode << index) & REG_ADMOD0H_VALID_DATA_MSK);
    }
    else if (index<24) {   
        // Setting the analog input mode of channel 16 to 23
        index -= 16;     // Scale ANx-number down into single register field
        regptr = (volatile uint16_t *)&ADMOD1L;
        index <<= 1;    // Multiply index by 2
        regval = (((volatile uint16_t)input_mode << index) & REG_ADMOD1L_VALID_DATA_MSK);
    }
    else {

        #ifdef ADLVLTRGH
        // Setting the analog input mode of channel 24 to 31
        index -= 24;     // Scale ANx-number down into single register field
        regptr = (volatile uint16_t *)&ADMOD1H; // Set the Trigger Mode Bit
        index <<= 1;    // Multiply index by 2
        regval = (((volatile uint16_t)input_mode << index) & REG_ADMOD1H_VALID_DATA_MSK);
        #else
        return(0);  // Return ERROR if register does not exists
        #endif
    }

    // write value to register
    *regptr |= regval;
    
    // Check if bit has been set
    fres &= (volatile bool)((*regptr & regval) == regval);

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

inline volatile uint16_t hsadc_init_adc_comp(uint16_t index, HSADC_ADCMP_CONFIG_t adcmp_cfg) {

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
    *regptr16 = (adcmp_cfg.ADCMPxLO & REG_ADCMPxLO_VALID_DATA_MASK);     // Write lower threshold value
    fres &= ((*regptr16 & REG_ADCMPxLO_VALID_DATA_MASK) == (adcmp_cfg.ADCMPxLO & REG_ADCMPxLO_VALID_DATA_MASK)); // Test if written value matches parameter

    // Upper Threshold Compare Value
    reg_offset = (index) * ((volatile uint16_t)&ADCMP1HI - (volatile uint16_t)&ADCMP0HI); // Get register offset
    regptr16 = (volatile uint16_t *)&ADCMP0HI + reg_offset;       // Get upper threshold value
    *regptr16 = (adcmp_cfg.ADCMPxHI & REG_ADCMPxHI_VALID_DATA_MASK);     // Write upper threshold value
    fres &= ((*regptr16 & REG_ADCMPxHI_VALID_DATA_MASK) == (adcmp_cfg.ADCMPxHI & REG_ADCMPxHI_VALID_DATA_MASK)); // Test if written value matches parameter
    
    // Assigning ANx inputs to the comparison
    reg_value = adcmp_cfg.ADCMPxCON.bits.CHNL;
    reg_value -= 16;
    
    if (reg_value < 16) {   

        // Enabling the corresponding analog input comparator input
        reg_offset = (index) * ((volatile uint16_t)&ADCMP1ENL - (volatile uint16_t)&ADCMP0ENL); // Get register offset
        regptr16 = (volatile uint16_t *)&ADCMP0ENL + reg_offset;    // Get upper threshold value
        *regptr16 |= (1 << reg_value);                              // Write upper threshold value

    }
    else if (reg_value < 32) {

        #ifdef ADCMP0ENH
        // Enabling the corresponding analog input comparator input
        reg_offset = (index) * ((volatile uint16_t)&ADCMP1ENH - (volatile uint16_t)&ADCMP0ENH); // Get register offset
        regptr16 = (volatile uint16_t *)&ADCMP0ENH + reg_offset;    // Get upper threshold value
        *regptr16 = (1 << reg_value);                               // Write upper threshold value
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

inline volatile uint16_t hsadc_init_adc_filter(uint16_t index, HSADC_ADFLT_CONFIG_t adflt_cfg) {

    volatile uint16_t fres = 1;
    volatile uint16_t *regptr16;
    volatile uint16_t reg_offset = 0;


	if (index >= (ADC_ADFL_COUNT-1)) return(0);    // Check if analog input number is valid

    
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
