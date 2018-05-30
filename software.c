/**
*
*
* @file software.c
* @author Rahul Marathe( rmarathe@pdx.edu)
* @copyright Portland State University 2018
*
* This C file implments the Colour Wheel on the two RGB LEDs on the NExys4DDR Board. The implementation is done via Pulse Width Modulation variation on a HSV Scale and then
* converting the values to appropriate RGB values to display on the pmodOLED and the RGB LEDS
*
* The flow of execution is as follows:
* The HSV inputs are varied by turning the Rotary Encoder to change Hue, Pushbuttons BtnR Increases the Saturation, BtnL Decreases Saturation
* and finally BtnU Increases the Value and BtnD Decreases the Value
* The HSV inputs are then fed to the pmodOLED and converted to RGB before being fed to the two on board RGB LEDS
* A function HSV2RGB will convert the HSV inputs into RGB Format
* PWM is extracted from the RGB LEDS and detected via both Hardware and Software Modules 
* The detected PWM for the individual Red,Green and Blue is displayed on the Seven Segment Display
* <pre>
*
*
* REVISION HISTORY:
*     Ver  	  Who 		 Date     		Changes
*    ----- 	 ---- 		-------- 		-----------------------------------------------
*     1.0    RMM        7th April,'18   Basic Functionality of Software Detection
*     2.0    RMM        15th April,'18  Added Hardware Detection Component
*     3.0    RMM        17th April,'18  Changed the OLED Display from Decimal to BCD for Display
*
* </pre>
*
* @note
* The minimal hardware configuration for this test is a Microblaze-based system with at least 32KB of memory,
* an instance of Nexys4IO, an instance of the pmodOLEDrgb AXI slave peripheral, and instance of the pmodENC AXI
* slave peripheral, an instance of AXI GPIO, an instance of AXI timer and an instance of the AXI UARTLite
* (used for xil_printf() console output)
*
* @note
* The driver code and test application(s) for the pmodOLDrgb and pmodENC are based on code provided by Digilent, Inc.
******************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "platform.h"
#include "xparameters.h"
#include "xstatus.h"
#include "microblaze_sleep.h"
#include "nexys4IO.h"
#include "PmodOLEDrgb.h"
#include "PmodENC.h"
#include "xgpio.h"
#include "xintc.h"
#include "xtmrctr.h"

/************************** Constant Definitions ****************************/

// Clock frequencies
#define CPU_CLOCK_FREQ_HZ		XPAR_CPU_CORE_CLOCK_FREQ_HZ
#define AXI_CLOCK_FREQ_HZ		XPAR_CPU_M_AXI_DP_FREQ_HZ

// AXI timer parameters
#define AXI_TIMER_DEVICE_ID		XPAR_AXI_TIMER_0_DEVICE_ID
#define AXI_TIMER_BASEADDR		XPAR_AXI_TIMER_0_BASEADDR
#define AXI_TIMER_HIGHADDR		XPAR_AXI_TIMER_0_HIGHADDR
#define TmrCtrNumber			0


// Definitions for peripheral NEXYS4IO
#define NX4IO_DEVICE_ID		XPAR_NEXYS4IO_0_DEVICE_ID
#define NX4IO_BASEADDR		XPAR_NEXYS4IO_0_S00_AXI_BASEADDR
#define NX4IO_HIGHADDR		XPAR_NEXYS4IO_0_S00_AXI_HIGHADDR

// Definitions for peripheral PMODOLEDRGB
#define RGBDSPLY_DEVICE_ID		XPAR_PMODOLEDRGB_0_DEVICE_ID
#define RGBDSPLY_GPIO_BASEADDR	XPAR_PMODOLEDRGB_0_AXI_LITE_GPIO_BASEADDR
#define RGBDSPLY_GPIO_HIGHADDR	XPAR_PMODOLEDRGB_0_AXI_LITE_GPIO_HIGHADD
#define RGBDSPLY_SPI_BASEADDR	XPAR_PMODOLEDRGB_0_AXI_LITE_SPI_BASEADDR
#define RGBDSPLY_SPI_HIGHADDR	XPAR_PMODOLEDRGB_0_AXI_LITE_SPI_HIGHADDR

// Definitions for peripheral PMODENC
#define PMODENC_DEVICE_ID		XPAR_PMODENC_0_DEVICE_ID
#define PMODENC_BASEADDR		XPAR_PMODENC_0_AXI_LITE_GPIO_BASEADDR
#define PMODENC_HIGHADDR		XPAR_PMODENC_0_AXI_LITE_GPIO_HIGHADDR

// Fixed Interval timer - 100 MHz input clock, 40KHz output clock
// FIT_COUNT_1MSEC = FIT_CLOCK_FREQ_HZ * .001
#define FIT_IN_CLOCK_FREQ_HZ	CPU_CLOCK_FREQ_HZ
#define FIT_CLOCK_FREQ_HZ		40000
#define FIT_COUNT				(FIT_IN_CLOCK_FREQ_HZ / FIT_CLOCK_FREQ_HZ)
#define FIT_COUNT_1MSEC			40

// GPIO parameters
#define GPIO_0_DEVICE_ID			XPAR_AXI_GPIO_0_DEVICE_ID
#define GPIO_0_INPUT_0_CHANNEL		1
#define GPIO_0_OUTPUT_0_CHANNEL		2

// GPIO 1 Used as Dual Input Channel for sending Hardware detected values
#define GPIO_1_DEVICE_ID			XPAR_AXI_GPIO_1_DEVICE_ID
#define GPIO_1_INPUT_0_CHANNEL		1
#define GPIO_1_INPUT_1_CHANNEL		2

// GPIO 2 Used as Dual Input Channel for sending Hardware detected values
#define GPIO_2_DEVICE_ID			XPAR_AXI_GPIO_2_DEVICE_ID
#define GPIO_2_INPUT_0_CHANNEL		1
#define GPIO_2_INPUT_1_CHANNEL		2

// GPIO 3 Used as Dual Input Channel for sending Hardware detected values
#define GPIO_3_DEVICE_ID			XPAR_AXI_GPIO_3_DEVICE_ID
#define GPIO_3_INPUT_0_CHANNEL		1
#define GPIO_3_INPUT_1_CHANNEL		2

// Interrupt Controller parameters
#define INTC_DEVICE_ID			XPAR_INTC_0_DEVICE_ID
#define FIT_INTERRUPT_ID		XPAR_MICROBLAZE_0_AXI_INTC_FIT_TIMER_0_INTERRUPT_INTR

/**************************** Type Definitions ******************************/

/***************** Macros (Inline Functions) Definitions ********************/

/************************** Variable Definitions ****************************/
// Microblaze peripheral instances
uint64_t 	timestamp = 0L;
PmodOLEDrgb	pmodOLEDrgb_inst;
PmodENC 	pmodENC_inst;
XGpio		GPIOInst0;					// GPIO instance
XIntc 		IntrptCtlrInst;				// Interrupt Controller instance
XTmrCtr		AXITimerInst;				// PWM timer instance
XGpio		GPIOInst1;					// Instance declaration for GPIO1
XGpio		GPIOInst2;					// Instance declaration for GPIO2
XGpio		GPIOInst3;					// Instance declaration for GPIO3

// The following variables are shared between non-interrupt processing and
// interrupt processing therefore they must be global(and declared volatile)
// These variables are controlled by the FIT timer interrupt handler



volatile u8						Hue 	        = 0;				  // Variable declaration for Hue
volatile u8						Saturation 	    = 0;				  // Variable declaration for Saturation
volatile u8						Value 	        = 0;				  // Variable declaration for Value

volatile u16					RGB_Value 	    = 0;			      // Variable Declaration to store the 16 bit RGB integer value obtained after conversion.
volatile u16					PrevRGB	        = 0;			      // Variable Declaration to store the Previous 16 bit RGB integer value.
volatile u16                    Red_Duty_Cycle  = 0;                  // Variable declaration for Software detected Duty Cycle for Red
volatile u16                    Green_Duty_Cycle= 0;                  // Variable declaration for Software detected Duty Cycle for Green
volatile u16                    Blue_Duty_Cycle = 0;                  // Variable declaration for Software detected Duty Cycle for Blue

volatile u32			        gpio_in         = 0;			      // GPIO input port
volatile u32			        Prevgpio_in     = 0;			      // GPIO input port
volatile u32					PrevHue 	    = 0;				  // Variable declaration for Previous value of Hue
volatile u32					PrevSaturation 	= 0;				  // Variable declaration for Previous Value of Saturation
volatile u32					PrevValue 	    = 0;				  // Variable declaration for Previous Value of Value
volatile u32                    State           = 0;                  // Variable declaration for the PMODENC CurrentState
volatile u32                    LastState       = 0;                  // Variable declaration for the PMODENC LastState

volatile u32                    Redpwm          = 0;                  // Variable declaration to store the current Red Pwm
volatile u32                    Greenpwm        = 0;                  // Variable declaration to store the current Green Pwm
volatile u32                    Bluepwm         = 0;                  // Variable declaration to store the current Blue Pwm

volatile u32                    PrevRedpwm      = 0;                  // Variable declaration to store the previous Red Pwm
volatile u32                    PrevGreenpwm    = 0;                  // Variable declaration to store the previous Green Pwm
volatile u32                    PrevBluepwm     = 0;                  // Variable declaration to store the previous Blue Pwm

volatile u32					Red_High_SW     = 0;			      // High Count for Red via Software Detection
volatile u32  					Red_Low_SW      = 0;				  // Low Count  for Red via Software Detection

volatile u32					Blue_High_SW 	= 0;				  // High Count for Blue via Software Detection
volatile u32  					Blue_Low_SW  	= 0;				  // Low Count  for Blue via Software Detection

volatile u32					Green_High_SW   = 0;				  // High Count for Green via Software Detection
volatile u32  					Green_Low_SW    = 0;				  // Low Count  for Green via Software Detection

volatile u32 					Temp_HCount_Red = 0;				  // Temporary High Count Variable for Red=0
volatile u32					Temp_LCount_Red = 0;				  // Temporary Low Count Variable for Red=0

volatile u32 					Temp_HCount_Blue = 0;				  // Temporary High Count Variable for Blue=0
volatile u32					Temp_LCount_Blue = 0;				  // Temporary Low Count Variable for Blue=0

volatile u32 					Temp_HCount_Green = 0;				  // Temporary High Count Variable for Green=0
volatile u32					Temp_LCount_Green = 0;				  // Temporary Low Count Variable for Green=0

volatile u32               	    RotaryCount     = 0;                  // Variable declaration for storing the Rotary Count
volatile u32	                PrevRotaryCount = 0;

volatile u32                    SaturationCount = 0;                  // Variable for Storing Saturation Count
volatile u32                    ValueCount      = 0;                  // Variable for Storing Value Count



volatile u32					Red_High_HW      =0;		          // GPIO1 ch1 input port
volatile u32					Red_Low_HW       =0;			      // GPIO1 ch2 input port
volatile u32					Green_High_HW    =0;			      // GPIO2 ch1 input port
volatile u32					Green_Low_HW     =0;			      // GPIO2 ch2 input port
volatile u32					Blue_High_HW     =0;			      // GPIO3 ch1 input port
volatile u32					Blue_Low_HW      =0;			      // GPIO3 ch2 input port


/************************** Function Prototypes *****************************/
void PMDIO_itoa(int32_t value, char *string, int32_t radix);
void PMDIO_puthex(PmodOLEDrgb* InstancePtr, uint32_t num);
void PMDIO_putnum(PmodOLEDrgb* InstancePtr, int32_t num, int32_t radix);
int	 do_init(void);											        // initialize system
void FIT_Handler(void);										        // fixed interval timer interrupt handler
int AXI_Timer_initialize(void);

void HSV2RGB(void);                                                 // Declaration for the HSV to RGB conversion function
void displayHSV2RGB(void);                                          // Declaration for Displaying Initial Value on OLED
void UpdateOLED(void);                                              // Declaration for Updating Values on OLED
void hw_detect(void);                                               // Declaration for Hardware PWM Detection Block
void sw_detect(void);                                               // Declaration for Software PWM Detection Block
void DisplaySevenSegment(uint8_t,uint8_t,uint8_t);                  // Declaration for Displaying value on Seven Segment
/************************** MAIN PROGRAM ************************************/
int main(void)
{
    init_platform();

	uint32_t sts;

	sts = do_init();
	if (XST_SUCCESS != sts)
	{
		exit(1);
	}

	microblaze_enable_interrupts();


	xil_printf("ECE 544 Colour Wheel Display\n\r");
	xil_printf("By Rahul Marathe. 07-April-2018\n\n\r");
	NX4IO_SSEG_setDecPt(SSEGLO, DIGIT7, false);
	NX4IO_SSEG_setDecPt(SSEGLO, DIGIT6, false);
	NX4IO_SSEG_setDecPt(SSEGLO, DIGIT5, false);
	NX4IO_SSEG_setDecPt(SSEGLO, DIGIT4, false);
	NX4IO_SSEG_setDecPt(SSEGHI, DIGIT3, false);
	NX4IO_SSEG_setDecPt(SSEGHI, DIGIT2, false);
	NX4IO_SSEG_setDecPt(SSEGHI, DIGIT1, false);
	NX4IO_SSEG_setDecPt(SSEGHI, DIGIT0, false);

	NX4IO_setLEDs(0x00000000);                         // Clear LEDS

	// Initialize and Clear the OLED Screen,sets font color to Red and Background to Black
	OLEDrgb_begin(&pmodOLEDrgb_inst, RGBDSPLY_GPIO_BASEADDR, RGBDSPLY_SPI_BASEADDR);
	OLEDrgb_SetFontColor(&pmodOLEDrgb_inst,OLEDrgb_BuildRGB(200, 12, 44));
	// Initialize the PMOD ENC and clear rotation count
	ENC_begin(&pmodENC_inst, PMODENC_BASEADDR);

	displayHSV2RGB();                                   //Displays the initial HSV values on the OLED

  	HSV2RGB();                                          // Main Logic for conversion of HSV Signals to Send it to the OLED


	// Turn OFF the RGB LEDS after the implementation
		NX4IO_RGBLED_setChnlEn(RGB1, true, true, true); // Enable all the Channels
		NX4IO_RGBLED_setDutyCycle(RGB1, 0, 0, 0);       // Set Duty Cycles of the RGB LED1 to 0
		NX4IO_RGBLED_setChnlEn(RGB2, true, true, true); // Enable all the Channels
		NX4IO_RGBLED_setDutyCycle(RGB2, 0, 0, 0);       // Set Duty Cycles of the RGB LED1 to 0
		OLEDrgb_end(&pmodOLEDrgb_inst);                 // Switch Off OLED After use.
		timestamp = 0;
	// cleanup and exit
    cleanup_platform();
    exit(0);
}

/******************* Colour Conversion Function *********************/
/**
* HSV2RGB : This function takes 3 inputs namely Hue,Saturation and Value
*            and converts them into appropriate RGB Values for displaying on OLED and OnBoard LEDS
* Takes Input Values from Rotary Encoder for Hue, Pushbuttons R and L for Saturation and Pushbuttons U and D for Value
* Converts them into RGB value using Sub-function Convert2RGB()
*
*
**/
/**************************** HELPER FUNCTIONS ******************************/
 void HSV2RGB(void)
 {

 xil_printf(" Vary the PmodENC shaft to change the Rotary Encoder count \n");
 xil_printf(" Press BNTR to Increase saturation, BTNL to Decrease saturation,BTNU to Increase value and BTND to Decrease value\n");
 xil_printf(" Press Rotary encoder shaft or BTNC to exit from Loop \n");
 LastState = ENC_getState(&pmodENC_inst);
 while(1)
 {
	 State = ENC_getState(&pmodENC_inst);
  // check if the rotary encoder pushbutton or BTNC is pressed
  // exit the loop if either one is pressed.
		if (ENC_buttonPressed(State) && !ENC_buttonPressed(LastState))// Check for BTNC or Rotary Shaft Press
		{
			break;                                                    // If either is pressed exit
		}
		if (NX4IO_isPressed(BTNC))
		{
			break;                                                    // If either is pressed exit
		}
		else
		{
        RotaryCount += ENC_getRotation(State, LastState);            // Get the current rotation count
		}
  // Hue is in between 0 and 360
  if(RotaryCount!=PrevRotaryCount)
  {
	 if(RotaryCount>359)                                            // Check if ROtary COunt exceeds 359
  {

    RotaryCount=0;    											    // Wrap around to 0 degree
  }
  	else if(RotaryCount<0)                                         // Check if Rotary COunt is Below 0
  {
	RotaryCount=359;                                               // If yes,Wrap around to 359
  }
  Hue= RotaryCount*255/360;                                       // Convert Hue to 0 to 255 Range by Scaling Factor 255/360
  }
  LastState = State;                                              // Update LastState for comparison
  PrevRotaryCount = RotaryCount;                                  // Update RotaryCount for comparison


 // Saturation is in between 0 and 100, to convert it to a range of 0-255 RGB we use the following functions
 if (NX4IO_isPressed(BTNR)&& SaturationCount<=99)                 // If Count is Greater than 99 and Increment Button is Pressed
 {
  SaturationCount++;                                             // Increment Saturation Count
 }
 else if(NX4IO_isPressed(BTNR) && SaturationCount>99)            // If Count is Greater than 99 and Increment Button is Pressed
 {
	 SaturationCount=0;                                          // Wrap Around to 0
 }
 else if (NX4IO_isPressed(BTNL) && SaturationCount>0)           // If Count is Greater than 0 and Decrement Button is Pressed
 {
   SaturationCount--;                                           // Decrement Saturation Count
 }
 else if ( NX4IO_isPressed(BTNL) && SaturationCount==0)         // If Count is Equal to 0 and Decrement Button is Pressed
 {
	 SaturationCount=99;                                        // Wrap Around to 99
 }
 else
	 SaturationCount=SaturationCount;                           // No Conditions Satisfied Retain Current Count
   Saturation= SaturationCount*255/99;                          // Scale Saturation to 0-255 range by factor 255/99

  // Value is in between 0 and 100, to convert it to a 0-255 Scale for RGB we use the following functions

   if (NX4IO_isPressed(BTNU)&& ValueCount<=99)                  // If Count is Greater than 99 and Increment Button is Pressed
 {
  ValueCount++;                                                 // Increment Value Count
 }
 else if(NX4IO_isPressed(BTNU) && ValueCount>99)                // If Count is Greater than 99 and Increment Button is Pressed
 {
	 ValueCount=0;                                              // Wrap Around to 0
 }
 else if (NX4IO_isPressed(BTND) && ValueCount>0)                // If Count is Greater than 0 and Decrement Button is Pressed
 {
   ValueCount--;                                                // Decrement Value Count
 }
 else if ( NX4IO_isPressed(BTND) && ValueCount==0)              // If Count is Equal to 0 and Increment Button is Pressed
 {
	 ValueCount=99;                                             // Wrap Around to 99
 }
 else
 ValueCount=ValueCount;                                         // No Conditions Satisfied Retain Current Count
 Value= ValueCount*255/99;                                      // Scale Value to 0-255 range by factor 255/99

 /***************************************************************************/
 /**First Check if the values of Hue, Saturation and Values are same or not.
 * If Values are different , then another function UpdateOLED() will display the values on the OLED which will update/overwrite the Initial Values.
 *
 ****************************************************************************/
if((Hue!= PrevHue) || (Saturation!=PrevSaturation) || ( Value!=PrevValue))
        {
            xil_printf("Hue : %d,Sat : %d, Value : %d \n",Hue,SaturationCount,ValueCount);
	            RGB_Value=OLEDrgb_BuildHSV(Hue,Saturation,Value);
            xil_printf("RGB values are %d \n",RGB_Value);

            NX4IO_RGBLED_setChnlEn(RGB1, true, true, true);
            NX4IO_RGBLED_setChnlEn(RGB2, true, true, true);
            NX4IO_RGBLED_setDutyCycle(RGB1, OLEDrgb_ExtractRFromRGB(RGB_Value)<<3, OLEDrgb_ExtractGFromRGB(RGB_Value)<<2, OLEDrgb_ExtractBFromRGB(RGB_Value)<<3);
            NX4IO_RGBLED_setDutyCycle(RGB2, OLEDrgb_ExtractRFromRGB(RGB_Value)<<3, OLEDrgb_ExtractGFromRGB(RGB_Value)<<2, OLEDrgb_ExtractBFromRGB(RGB_Value)<<3);
            usleep(5000);

			UpdateOLED();

			uint16_t ledvalue;
			ledvalue=NX4IO_getSwitches();     // Get the current value of switches
			NX4IO_setLEDs(ledvalue); // Set the LEDS as per the Switch Values
			if(ledvalue==1){
			xil_printf("----------------In Hardware detect--------------------------\n");
			hw_detect();		// Hardware Pulse Width Detection
			}
		   else
			{
			xil_printf("------------------In Software detect-----------------------\n");
			sw_detect();		// Software Pulse WIdth Detection
            }
         }

    }

  xil_printf(" End of Rotary Function. Exit is requested \n");

 }
/****************************************************************************/
/**
* initialize the system
*
* This function is executed once at start-up and after resets.  It initializes
* the peripherals and registers the interrupt handler(s)
*****************************************************************************/

int	 do_init(void)
{
	uint32_t status;				// status from Xilinx Lib calls

	// initialize the Nexys4 driver and (some of)the devices
	status = (uint32_t) NX4IO_initialize(NX4IO_BASEADDR);
	if (status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}

	// set all of the display digits to blanks and turn off
	// the decimal points using the "raw" set functions.
	// These registers are formatted according to the spec
	// and should remain unchanged when written to Nexys4IO...
	// something else to check w/ the debugger when we bring the
	// drivers up for the first time
	NX4IO_SSEG_setSSEG_DATA(SSEGHI, 0xFFFFFFFF);
	NX4IO_SSEG_setSSEG_DATA(SSEGLO, 0xFFFFFFFF);

	OLEDrgb_begin(&pmodOLEDrgb_inst, RGBDSPLY_GPIO_BASEADDR, RGBDSPLY_SPI_BASEADDR);

	// initialize the pmodENC and hardware
	ENC_begin(&pmodENC_inst, PMODENC_BASEADDR);

	// initialize the GPIO instances
	status = XGpio_Initialize(&GPIOInst0, GPIO_0_DEVICE_ID);
	if (status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}

	status = XGpio_Initialize(&GPIOInst1, GPIO_1_DEVICE_ID);
	if (status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}

	status = XGpio_Initialize(&GPIOInst2, GPIO_2_DEVICE_ID);
	if (status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}

	status = XGpio_Initialize(&GPIOInst3, GPIO_3_DEVICE_ID);
	if (status != XST_SUCCESS)
	{
	return XST_FAILURE;
	}

	XGpio_SetDataDirection(&GPIOInst0, GPIO_0_INPUT_0_CHANNEL, 0xFF); // GPIO0 channel 1 is an 8-bit input port.
	XGpio_SetDataDirection(&GPIOInst0, GPIO_0_OUTPUT_0_CHANNEL, 0x00);// GPIO0 channel 2 is an 8-bit output port.

	XGpio_SetDataDirection(&GPIOInst1, GPIO_1_INPUT_0_CHANNEL, 0xFF); // GPIO1 channel 1 is an 32-bit input port.
	XGpio_SetDataDirection(&GPIOInst1, GPIO_1_INPUT_1_CHANNEL, 0xFF); // GPIO1 channel 2 is an 32-bit input port.

	XGpio_SetDataDirection(&GPIOInst2, GPIO_2_INPUT_0_CHANNEL, 0xFF); // GPIO2 channel 1 is an 32-bit input port.
	XGpio_SetDataDirection(&GPIOInst2, GPIO_2_INPUT_1_CHANNEL, 0xFF); // GPIO2 channel 2 is an 32-bit input port.

	XGpio_SetDataDirection(&GPIOInst3, GPIO_3_INPUT_0_CHANNEL, 0xFF); // GPIO3 channel 1 is an 32-bit input port.
	XGpio_SetDataDirection(&GPIOInst3, GPIO_3_INPUT_1_CHANNEL, 0xFF); // GPIO3 channel 2 is an 32-bit input port.
	status = AXI_Timer_initialize();
	if (status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}

	// initialize the interrupt controller
	status = XIntc_Initialize(&IntrptCtlrInst, INTC_DEVICE_ID);
	if (status != XST_SUCCESS)
	{
	   return XST_FAILURE;
	}
	
	// connect the fixed interval timer (FIT) handler to the interrupt
	status = XIntc_Connect(&IntrptCtlrInst, FIT_INTERRUPT_ID,
						   (XInterruptHandler)FIT_Handler,
						   (void *)0);
	if (status != XST_SUCCESS)
	{
		return XST_FAILURE;

	}

	// start the interrupt controller such that interrupts are enabled for
	// all devices that cause interrupts.
	status = XIntc_Start(&IntrptCtlrInst, XIN_REAL_MODE);
	if (status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}

	// enable the FIT interrupt
	XIntc_Enable(&IntrptCtlrInst, FIT_INTERRUPT_ID);
	return XST_SUCCESS;
}
/*
 * AXI timer initializes it to generate out a 4Khz signal, Which is given to the Nexys4IO module as clock input.
 * DO NOT MODIFY
 */
int AXI_Timer_initialize(void){

	uint32_t status;				// status from Xilinx Lib calls
	u32		ctlsts;		// control/status register or mask

	status = XTmrCtr_Initialize(&AXITimerInst,AXI_TIMER_DEVICE_ID);
		if (status != XST_SUCCESS) {
			return XST_FAILURE;
		}
	status = XTmrCtr_SelfTest(&AXITimerInst, TmrCtrNumber);
		if (status != XST_SUCCESS) {
			return XST_FAILURE;
		}
	ctlsts = XTC_CSR_AUTO_RELOAD_MASK | XTC_CSR_EXT_GENERATE_MASK | XTC_CSR_LOAD_MASK |XTC_CSR_DOWN_COUNT_MASK ;
	XTmrCtr_SetControlStatusReg(AXI_TIMER_BASEADDR, TmrCtrNumber,ctlsts);

	//Set the value that is loaded into the timer counter and cause it to be loaded into the timer counter
	XTmrCtr_SetLoadReg(AXI_TIMER_BASEADDR, TmrCtrNumber, 24998);
	XTmrCtr_LoadTimerCounterReg(AXI_TIMER_BASEADDR, TmrCtrNumber);
	ctlsts = XTmrCtr_GetControlStatusReg(AXI_TIMER_BASEADDR, TmrCtrNumber);
	ctlsts &= (~XTC_CSR_LOAD_MASK);
	XTmrCtr_SetControlStatusReg(AXI_TIMER_BASEADDR, TmrCtrNumber, ctlsts);

	ctlsts = XTmrCtr_GetControlStatusReg(AXI_TIMER_BASEADDR, TmrCtrNumber);
	ctlsts |= XTC_CSR_ENABLE_TMR_MASK;
	XTmrCtr_SetControlStatusReg(AXI_TIMER_BASEADDR, TmrCtrNumber, ctlsts);

	XTmrCtr_Enable(AXI_TIMER_BASEADDR, TmrCtrNumber);
	return XST_SUCCESS;

}

/*********************** DISPLAY-RELATED FUNCTIONS ***********************************/

/****************************************************************************/
/**
* Converts an integer to ASCII characters
*
* algorithm borrowed from ReactOS system libraries
*
* Converts an integer to ASCII in the specified base.  Assumes string[] is
* long enough to hold the result plus the terminating null
*
* @param 	value is the integer to convert
* @param 	*string is a pointer to a buffer large enough to hold the converted number plus
*  			the terminating null
* @param	radix is the base to use in conversion, 
*
* @return  *NONE*
*
* @note
* No size check is done on the return string size.  Make sure you leave room
* for the full string plus the terminating null in string
*****************************************************************************/
void PMDIO_itoa(int32_t value, char *string, int32_t radix)
{
	char tmp[33];
	char *tp = tmp;
	int32_t i;
	uint32_t v;
	int32_t  sign;
	char *sp;

	if (radix > 36 || radix <= 1)
	{
		return;
	}

	sign = ((10 == radix) && (value < 0));
	if (sign)
	{
		v = -value;
	}
	else
	{
		v = (uint32_t) value;
	}
	
  	while (v || tp == tmp)
  	{
		i = v % radix;
		v = v / radix;
		if (i < 10)
		{
			*tp++ = i+'0';
		}
		else
		{
			*tp++ = i + 'a' - 10;
		}
	}
	sp = string;
	
	if (sign)
		*sp++ = '-';

	while (tp > tmp)
		*sp++ = *--tp;
	*sp = 0;
	
  	return;
}


/****************************************************************************/
/**
* Write a 32-bit unsigned hex number to PmodOLEDrgb in Hex
*       
* Writes  32-bit unsigned number to the pmodOLEDrgb display starting at the current
* cursor position.
*
* @param num is the number to display as a hex value
*
* @return  *NONE*
*
* @note
* No size checking is done to make sure the string will fit into a single line,
* or the entire display, for that matter.  Watch your string sizes.
*****************************************************************************/ 
void PMDIO_puthex(PmodOLEDrgb* InstancePtr, uint32_t num)
{
  char  buf[9];
  int32_t   cnt;
  char  *ptr;
  int32_t  digit;
  
  ptr = buf;
  for (cnt = 7; cnt >= 0; cnt--) {
    digit = (num >> (cnt * 4)) & 0xF;
    
    if (digit <= 9)
	{
      *ptr++ = (char) ('0' + digit);
	}
    else
	{
      *ptr++ = (char) ('a' - 10 + digit);
	}
  }

  *ptr = (char) 0;
  OLEDrgb_PutString(InstancePtr,buf);
  
  return;
}


/****************************************************************************/
/**
* Write a 32-bit number in Radix "radix" to LCD display
*
* Writes a 32-bit number to the LCD display starting at the current
* cursor position. "radix" is the base to output the number in.
*
* @param num is the number to display
*
* @param radix is the radix to display number in
*
* @return *NONE*
*
* @note
* No size checking is done to make sure the string will fit into a single line,
* or the entire display, for that matter.  Watch your string sizes.
*****************************************************************************/ 
void PMDIO_putnum(PmodOLEDrgb* InstancePtr, int32_t num, int32_t radix)
{
  char  buf[16];
  
  PMDIO_itoa(num, buf, radix);
  OLEDrgb_PutString(InstancePtr,buf);
  
  return;
}
/********************* Initialize and display default values on OLED *************/

/**
 * Function displayHSV2RGB() will display the initial values of Hue Saturation and Value on the OLED
 *
 * Function will also display a Rectangle having the same RGB colour as the PMODOLED
 *
 * @return *NONE*
*********************************************************************************/
void displayHSV2RGB(void)
{

OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 1, 1);                 // Set Cursor to 1,1 Location
OLEDrgb_PutString(&pmodOLEDrgb_inst,"H :");                 // Print Hue Label On OLED
OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 4, 1);                 // Set Cursor to 4,1 Location
PMDIO_putnum(&pmodOLEDrgb_inst, RotaryCount, 10);           // Print initial value of Hue on OLED(0)

OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 1, 3);                 // Set Cursor to 1,3 Location
OLEDrgb_PutString(&pmodOLEDrgb_inst,"S :");                 // Print Saturation Label On OLED
OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 4, 3);                 // Set Cursor to 4,3 Location
PMDIO_putnum(&pmodOLEDrgb_inst, SaturationCount/10, 10);    // Print MSB of initial value(0) of Saturation on OLED
OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 5, 3);                 // Set Cursor to 5,3 Location
PMDIO_putnum(&pmodOLEDrgb_inst, SaturationCount%10, 10);    // Print LSB of initial value(0) of Saturation on OLED

OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 1, 5);                 // Set Cursor to 1,5 Location
OLEDrgb_PutString(&pmodOLEDrgb_inst,"V :");                 // Print Value Label On OLED
OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 4, 5);                 // Set Cursor to 4,5 Location
PMDIO_putnum(&pmodOLEDrgb_inst, ValueCount/10, 10);         // Print MSB of initial Value(0) on OLED
OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 5, 5);                 // Set Cursor to 5,5 Location
PMDIO_putnum(&pmodOLEDrgb_inst, ValueCount%10, 10);         // Print LSB of initial Value(0) on OLED

OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst, 70, 70, 90, 90, RGB_Value, true, RGB_Value); // Draw Rectangle with Initial RGB Colour(0,0,0)

}

/********************* Initialize and display Updated values on OLED *************/
/**
 * Function displayHSV2RGB() will display the Updated values of Hue Saturation and Value on the OLED
 *
 * Function will also display a Rectangle having the same RGB colour as the PMOD-OLED
 *
 *  @return *NONE*
*********************************************************************************/
void UpdateOLED(void)
{

PrevRGB=RGB_Value;                                       // Update Value of PrevPwm used for comparison
OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 1, 1);              // Set Cursor to 1,1 Location
OLEDrgb_PutString(&pmodOLEDrgb_inst,"H :");              // Print Hue Label On OLED

OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 4, 1);              // Set Cursor to 4,1 Location
PMDIO_putnum(&pmodOLEDrgb_inst, RotaryCount, 10);        // Print current value of Hue on OLED
PrevHue=Hue;                                             // Update previous Hue for comparison

OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 1, 3);              // Set Cursor to 1,3 Location
OLEDrgb_PutString(&pmodOLEDrgb_inst,"S :");              // Print Saturation Label On OLED
OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 4, 3);              // Set Cursor to 4,3 Location
PMDIO_putnum(&pmodOLEDrgb_inst, SaturationCount/10, 10); // Print MSB of current value of Saturation on OLED
OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 5, 3);              // Set Cursor to 5,3 Location
PMDIO_putnum(&pmodOLEDrgb_inst, SaturationCount%10, 10); // Print LSB of current value of Saturation on OLED
PrevSaturation=Saturation;                               // Update previous Saturation for comparison

OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 1, 5);              // Set Cursor to 1,5 Location
OLEDrgb_PutString(&pmodOLEDrgb_inst,"V :");              // Print Value Label On OLED
OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 4, 5);              // Set Cursor to 4,5 Location
PMDIO_putnum(&pmodOLEDrgb_inst, ValueCount/10, 10);      // Print MSB of current Value on OLED
OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 5, 5);              // Set Cursor to 5,5 Location
PMDIO_putnum(&pmodOLEDrgb_inst, ValueCount%10, 10);      // Print LSB of Current Value on OLED
PrevValue=Value;                                         // Update previous Value for comparison

OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst, 70, 70, 90, 90, RGB_Value, true, RGB_Value); // Draw Rectangle with RGB Colour
PrevRGB=RGB_Value;                                       // Update PrevRGB for comparison

}


/********************* Software Detection of Pulse Width *************/
/** The Duty Cycle of Red, Blue and Green is calculated using the Formula :
*   Duty Cycle= (High_Count*100/(High_Count+Low_Count))
*
*   Calls a Display Function ,"DisplaySevenSegment()" which prints values to the Seven Segment Display
*
*  @returns DisplaySevenSegment(uint8_t Red,uint8_t Green,uint8_t Blue)
*********************************************************************************/
void sw_detect(void)
{
	Red_Duty_Cycle=((Red_High_SW*100)/(Red_High_SW+Red_Low_SW));
	Green_Duty_Cycle=((Green_High_SW*100)/(Green_High_SW+Green_Low_SW));
	Blue_Duty_Cycle=((Blue_High_SW*100)/(Blue_High_SW+Blue_Low_SW));

	xil_printf("Red_High_SW : %d, Red_Low_SW : %d Red_Duty_SW is: %d \n", Red_High_SW, Red_Low_SW,Red_Duty_Cycle);
	xil_printf("Green_High_SW : %d, Green_Low_SW : %d Green_Duty_SW is: %d \n", Green_High_SW, Green_Low_SW,Green_Duty_Cycle);
	xil_printf("Blue_High_SW : %d, Blue_Low_SW : %d Blue_Duty_SW is: %d \n", Blue_High_SW, Blue_Low_SW,Blue_Duty_Cycle);


	DisplaySevenSegment(Red_Duty_Cycle,Green_Duty_Cycle,Blue_Duty_Cycle);
}

/********************* Hardware Detection of Pulse Width *************/
/** Function hw_detect will calculate the values from the GPIO Channels sent from the Hardware Detection
*
*  @returns DisplaySevenSegment(uint8_t Red,uint8_t Green,uint8_t Blue)
*********************************************************************************/
void hw_detect(void)
{

	Red_High_HW = XGpio_DiscreteRead(&GPIOInst1, GPIO_1_INPUT_1_CHANNEL);				// Read the GPIO in1_0 port and store the value in gpio1_in1
	Red_Low_HW = XGpio_DiscreteRead(&GPIOInst1, GPIO_1_INPUT_0_CHANNEL);				// Read the GPIO in1_1 port and store the value in gpio1_in2

	Green_High_HW = XGpio_DiscreteRead(&GPIOInst2, GPIO_2_INPUT_1_CHANNEL);				// Read the GPIO in2_0 port and store the value in gpio2_in1
	Green_Low_HW = XGpio_DiscreteRead(&GPIOInst2, GPIO_2_INPUT_0_CHANNEL);				// Read the GPIO in2_1 port and store the value in gpio2_in2

	Blue_High_HW = XGpio_DiscreteRead(&GPIOInst3, GPIO_3_INPUT_1_CHANNEL);				// Read the GPIO in3_0 port and store the value in gpio3_in1
	Blue_Low_HW = XGpio_DiscreteRead(&GPIOInst3, GPIO_3_INPUT_0_CHANNEL);				// Read the GPIO in3_1 port and store the value in gpio3_in2


	u8 Red_Duty_Cycle_HW = (Red_High_HW*100)/(Red_High_HW + Red_Low_HW );				// Calculate the duty cycle for Red color
	u8 Green_Duty_Cycle_HW= (Green_High_HW*100)/(Green_High_HW + Green_Low_HW);			// Calculate the duty cycle for Green color
	u8 Blue_Duty_Cycle_HW=(Blue_High_HW)*100/(Blue_High_HW + Blue_Low_HW );				// Calculate the duty cycle for Blue color

	xil_printf("Red_High_HW : %d, Red_Low_HW : %d Red_Duty_HW is: %d \n", Red_High_HW, Red_Low_HW,Red_Duty_Cycle_HW);
	xil_printf("Green_High_HW : %d, Green_Low_HW : %d Green_Duty_HW is: %d \n", Green_High_HW, Green_Low_HW,Green_Duty_Cycle_HW);
	xil_printf("Blue_High_HW : %d, Blue_Low_HW : %d Blue_Duty_HW is: %d \n", Blue_High_HW, Blue_Low_HW,Blue_Duty_Cycle_HW);

	DisplaySevenSegment(Red_Duty_Cycle_HW,Green_Duty_Cycle_HW,Blue_Duty_Cycle_HW);
	//sleep(5000);
}

/**************************** SEVEN SEGMENT DISPLAY FUNCTION *****************/
/** Function takes 3 8 Bit RGB Values and Displays them on the respective seven segment
 *
 *****************************************************************************/
void DisplaySevenSegment(uint8_t Red,uint8_t Green,uint8_t Blue)
{
NX4IO_SSEG_setDigit(SSEGHI, DIGIT7, (Red/10));    // Red MSB
NX4IO_SSEG_setDigit(SSEGHI, DIGIT6, (Red%10));    // Red LSB

NX4IO_SSEG_setDigit(SSEGHI, DIGIT4, (Green/10));  // Green MSB
NX4IO_SSEG_setDigit(SSEGLO, DIGIT3, (Green%10));  // Green LSB

NX4IO_SSEG_setDigit(SSEGLO, DIGIT1, (Blue/10));   // Blue MSB
NX4IO_SSEG_setDigit(SSEGLO, DIGIT0, (Blue%10));   // Blue LSB
usleep(5000);
}


/*********************************************************************************/

/**************************** INTERRUPT HANDLERS ******************************/

/****************************************************************************/
/**
* Fixed interval timer interrupt handler
*
* Reads the GPIO port which reads back the hardware generated PWM wave for the RGB Leds
*
* @note
* ECE 544 students - When you implement your software solution for pulse width detection in
* Project 1 this could be a reasonable place to do that processing.
 *****************************************************************************/

void FIT_Handler(void)
{
	gpio_in = XGpio_DiscreteRead(&GPIOInst0, GPIO_0_INPUT_0_CHANNEL); // Read the GPIO port to read back the generated PWM signal for RGB led's
    if(gpio_in!=Prevgpio_in)
    {
	Redpwm= (gpio_in & 0x0004)>>2;   // Shift right by two to get to LSB
	Bluepwm= (gpio_in & 0x0002)>>1;  // Shift right by one to get to LSB
	Greenpwm= (gpio_in & 0x0001)>>0; // Bit is at the LSB Position

    }
    Prevgpio_in=gpio_in;

//------------------------------------------------------------------------------//
    if(Redpwm==1 && PrevRedpwm==0)  // If the Previous is low and Current is High, Transition has taken place
    {
    Red_Low_SW= Temp_LCount_Red;    // Store the value from the temporary variable to the final Low Count
    Temp_LCount_Red=0;
    }
    else if (Redpwm ==0 && PrevRedpwm==1) // If the Previous is High and Current is Low, Transition has taken place
    {
    Red_High_SW= Temp_HCount_Red;   // Store the value from the temporary variable to the final High Count
    Temp_HCount_Red=0;
    }
if(Redpwm==1)                       // If Input Bit is High
{
 Temp_HCount_Red++;                 // Increment the Temporary High Count
 }
else                                // else
 {
 Temp_LCount_Red++;                 // Increment the Temporary Low Count
 }


PrevRedpwm=Redpwm;                  // Update the Previous Pwm

//-----------------------------------------------------------------------------//
if( Greenpwm==1 && PrevGreenpwm==0) // If the Previous is low and Current is High, Transition has taken place
{
Green_Low_SW= Temp_LCount_Green;  // Store the value from the temporary variable to the final Low Count
Temp_LCount_Green=0;
}
else if (Greenpwm ==0 && PrevGreenpwm==1) // If the Previous is High and Current is Low, Transition has taken place
{
Green_High_SW= Temp_HCount_Green; // Store the value from the temporary variable to the final High Count
Temp_HCount_Green=0;
}
if(Greenpwm==1)                  // If Input Bit is High
{
Temp_HCount_Green++;             // Increment the Temporary High Count
}
else                             // else
{
Temp_LCount_Green++;             // Increment the Temporary Low Count
}

PrevGreenpwm=Greenpwm;           // Update the Previous Pwm

//----------------------------------------------------------------------------//
if( Bluepwm==1 && PrevBluepwm==0) // If the Previous is low and Current is High, Transition has taken place
{
Blue_Low_SW= Temp_LCount_Blue;    // Store the value from the temporary variable to the final Low Count
Temp_LCount_Blue=0;               // Reset the Temporary variable
}
else if (Bluepwm ==0 && PrevBluepwm==1) // If the Previous is High and Current is Low, Transition has taken place
{
Blue_High_SW= Temp_HCount_Blue;   // Store the value from the temporary variable to the final High Count
Temp_HCount_Blue=0;               // Reset the Temporary variable
}
if(Bluepwm==1)                    // If Input Bit is High
{
Temp_HCount_Blue++;               // Increment the Temporary High Count
}
else                              // else if Input Bit is Low
{
Temp_LCount_Blue++;               // Increment the Temporary Low Count
}

PrevBluepwm=Bluepwm;              // Update the Previous Pwm

}

