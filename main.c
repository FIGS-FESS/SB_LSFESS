//###########################################################################
//
// FILE:   main.c
//
// ASSUMPTIONS:
//
//   This program requires the DSP2833x header files.
//
//   Make sure the CPU clock speed is properly defined in
//   DSP2833x_Examples.h before compiling this example.
//
//
//    As supplied, this project is configured for "boot to SARAM"
//    operation.  The 2833x Boot Mode table is shown below.
//    For information on configuring the boot mode of an eZdsp,
//    please refer to the documentation included with the eZdsp,
//
//       $Boot_Table:
//
//         GPIO87   GPIO86     GPIO85   GPIO84
//          XA15     XA14       XA13     XA12
//           PU       PU         PU       PU
//        ==========================================
//            1        1          1        1    Jump to Flash
//            1        1          1        0    SCI-A boot
//            1        1          0        1    SPI-A boot
//            1        1          0        0    I2C-A boot
//            1        0          1        1    eCAN-A boot
//            1        0          1        0    McBSP-A boot
//            1        0          0        1    Jump to XINTF x16
//            1        0          0        0    Jump to XINTF x32
//            0        1          1        1    Jump to OTP
//            0        1          1        0    Parallel GPIO I/O boot
//            0        1          0        1    Parallel XINTF boot
//            0        1          0        0    Jump to SARAM	    <- "boot to SARAM"
//            0        0          1        1    Branch to check boot mode
//            0        0          1        0    Boot to flash, bypass ADC cal
//            0        0          0        1    Boot to SARAM, bypass ADC cal
//            0        0          0        0    Boot to SCI-A, bypass ADC cal
//                                              Boot_Table_End$
//
//###########################################################################
//
// Brent Kisling
// Kevin Ramus
// University of Idaho
// Flywheel Project
// 2013
//
// $TI Release: 2833x/2823x Header Files and Peripheral Examples V133 $
// $Release Date: June 8, 2012 $
//###########################################################################

#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include <math.h>

// --------- Prototype statements for functions found within this file ---------

// ISR
interrupt void adc_isr(void);
interrupt void EPWM1_isr(void);

// GPIO Initialization
void GPIO_setup(void);

// Controller Functions
void Bang_Bang_Cntrl(float32 *Current_Cmd);
float32 Position_PID_Cntrl_x(void);
float32 Position_PID_Cntrl_y(void);

// Filter Functions
float32 Digital_Filter_2nd_order_x(float32 x_k);
float32 Digital_Filter_2nd_order_y(float32 x_k);

// Coordinate Transformation
//void Coordinate_Transform(void);


//--------- Macro Definitions ---------

// ------ ADC conversion

// Generic ADC conversion
#define ADC_CONVERSION 0.000732421875  	// Generic ADC conversion into analog value
//	#define ADC_CONVERSION 7.324219e-4	// Generic ADC conversion into analog value


// Current(I) measurement scaling and/or shifting conversion


// --------- IN_amp	CURRENT

#define Current_Conv_Constant 3.7214e-3 // Takes into account DivBy2 and ADC_CONVERSION

#define Current_Conv_Offset -15.01433

// Position measurement scaling and/or shifting conversion

// --------- Full Range Distance

// For position sensor SN 193965
#define Position_Conv_Constant_x 1.977102e-7 	// Takes into account DivBy4 and ADC_CONVERSION

//#define Position_Conv_Constant_x 3.954204e-7 	// Takes into account DivBy2 and ADC_CONVERSION

#define Position_Conv_Offset_x 1.33028e-6 // New as of 3/3


// --------- Full Range Distance

// For position sensor SN 193967

#define Position_Conv_Constant_y 1.981154e-7 	// Takes into account DivBy4 and ADC_CONVERSION

//#define Position_Conv_Constant_y 3.962308e-7 	// Takes into account DivBy2 and ADC_CONVERSION

#define Position_Conv_Offset_y -1.2779e-6




// ------ Position Controller

// PID gains

#define KP_pos 14209
#define KI_pos 117475
#define KD_pos 65.16

// Integral saturation limits
#define Integral_max 2
//0.05
#define Integral_min -2
//-0.05


#define mTfplusPt5Ts -2.2873e-04
#define	plusTfplusPt5Ts 3.2873e-04
#define one_div_plusTfplusPt5Ts 3.0420e+03


// Position Controller Output Limits
#define Max_Current 10
#define Min_Current 0.005
//0.005

#define Max_delta_I 8
#define	Min_delta_I 1.9



// Position Controller Bias Settings
#define Cur_bias 2.6
//2.6
#define Pos_bias 0.001


// Setpoint variation to simulate step change in command
#define Setpoint_val1 -1e-4
						//1e-4
#define Setpoint_val2 0
						//-1e-4

// ------ Digital Filter

//1.5kHz cutoff	2nd order	//v3

#define a0	1
#define a1	-0.7478
#define a2	0.2722
#define b0	0.1311
#define b1	0.2622
#define b2	0.1311


// ------ Current Controller
//#define Threshold_Level 0.5    			// Change this to reflect correct value!!!

#define coil_num 4

// ------ Measurement Sampling and timing
#define Pos_sample_freq	10000  	// 	10kHz
#define Pos_sample_time	0.0001  // 	1/10kHz
#define Pos_Sample_x 4 //  x position sensor sampled
#define Pos_Sample_y 8 //  y position sensor sampled
// Pos_Sample = Current_Sample_Frequency/(Pos_Sample_div)
// For example: if Pos_Sample_div = 2 and Current_Sample_Frequency = 40kHz, then 20kHz = 40kHz/(2)
// Will NOT work for Pos_Sample_div < 2 , must be an integer

// Misc
#define PWM1_INT_ENABLE 1
#define ADC_EOS_INT_ENABLE 1
#define	DivBy4 0.25
#define DivBy2 0.5



//--------- Global Variables ---------
Uint16 IdleCount;
Uint16 volatile InterruptCount;
Uint16 volatile BB_cntrl_flag;
Uint16 volatile Position_Sampled_Flag_x,Position_Sampled_Flag_y;
Uint16 volatile Position_PID_cntrl_flag_x, Position_PID_cntrl_flag_y;
Uint16 volatile Position_sampx1, Position_sampx2, Position_sampx3, Position_sampx4;
Uint16 volatile Position_sampy1, Position_sampy2, Position_sampy3, Position_sampy4;
Uint16 volatile Current_samp1, Current_samp2, Current_samp3, Current_samp4;
Uint16 volatile Current_samp5, Current_samp6, Current_samp7, Current_samp8;
float32 volatile Delta_Setpoint = 0;
Uint32 volatile Time_counter = 0;




main()
{
	float32 Current_Cmd[coil_num] = {1.5, 1.5, 1.5, 1.5};
	float32 delta_I_x;
	float32 delta_I_y;

// Step 1. Initialize System Control:
// PLL, WatchDog, enable Peripheral Clocks
// This example function is found in the DSP2833x_SysCtrl.c file.
   InitSysCtrl();

   EALLOW;
   #if (CPU_FRQ_150MHZ)     // Default - 150 MHz SYSCLKOUT
     #define ADC_MODCLK 0x3 // HSPCLK = SYSCLKOUT/2*ADC_MODCLK2 = 150/(2*3)   = 25.0 MHz
   #endif
   #if (CPU_FRQ_100MHZ)
     #define ADC_MODCLK 0x2 // HSPCLK = SYSCLKOUT/2*ADC_MODCLK2 = 100/(2*2)   = 25.0 MHz
   #endif
   EDIS;

   // Define ADCCLK clock frequency ( less than or equal to 25 MHz )
   // Assuming InitSysCtrl() has set SYSCLKOUT to 150 MHz
   EALLOW;
   SysCtrlRegs.HISPCP.all = ADC_MODCLK;
   EDIS;

// Step 2. Initialize GPIO:

   GPIO_setup();
// Disregard following two comment lines...
// This example function is found in the DSP2833x_Gpio.c file and
// illustrates how to set the GPIO to it's default state.


// Step 3. Clear all interrupts and initialize PIE vector table:
// Disable CPU interrupts
   DINT;

// Initialize the PIE control registers to their default state.
// The default state is all PIE interrupts disabled and flags
// are cleared.
// This function is found in the DSP2833x_PieCtrl.c file.
   InitPieCtrl();

// Disable CPU interrupts and clear all CPU interrupt flags:
   IER = 0x0000;
   IFR = 0x0000;

// Initialize the PIE vector table with pointers to the shell Interrupt
// Service Routines (ISR).
// This will populate the entire table, even if the interrupt
// is not used in this example.  This is useful for debug purposes.
// The shell ISR routines are found in DSP2833x_DefaultIsr.c.
// This function is found in DSP2833x_PieVect.c.
   InitPieVectTable();

// Interrupts that are used in this example are re-mapped to
// ISR functions found within this file.
   EALLOW;  // This is needed to write to EALLOW protected register
   PieVectTable.ADCINT = &adc_isr;

   PieVectTable.EPWM1_INT = &EPWM1_isr;

   EDIS;    // This is needed to disable write to EALLOW protected registers

// Step 4. Initialize all the Device Peripherals:
// This function is found in DSP2833x_InitPeripherals.c
// InitPeripherals(); // Not required for this example
   InitAdc();  // For this example, initialize the ADC

// Step 5. Enable interrupts:

// Enable ADCINT in PIE
   PieCtrlRegs.PIEIER1.bit.INTx6 = ADC_EOS_INT_ENABLE;
   IER |= M_INT1; // Enable CPU Interrupt 1

// Enable EPWM1 in PIE
   PieCtrlRegs.PIEIER3.bit.INTx1 = PWM1_INT_ENABLE;
   IER |= M_INT3; // Enable CPU INT3 which is connected to EPWM1-6 INT

   EINT;          // Enable Global interrupt INTM
   ERTM;          // Enable Global realtime interrupt DBGM

// Initialize Global Variables

   IdleCount = 0;
   Position_Sampled_Flag_x = 0;
   Position_Sampled_Flag_y = 0;
   InterruptCount = 0;

//--------- Configure ADC ---------
   AdcRegs.ADCTRL3.bit.SMODE_SEL = 0x1;		// Simultaneous sampling mode
   AdcRegs.ADCTRL1.bit.SEQ_CASC = 0x1;		// Cascade sequencer 1 and 2
   	   	   	   	   	   	   	   	   	   	    // SOC commands for SEQ1 now initiates SOC for the
   	   	   	   	   	   	   	   	   	   	    // cascaded sequencer consisting of SEQ1 and SEQ2


	#define ADC_A_POS_CUR_SAMPLING_x \
		do { \
			AdcRegs.ADCMAXCONV.all = 0x0006; \
			AdcRegs.ADCCHSELSEQ1.bit.CONV00 = 0x1; \
			AdcRegs.ADCCHSELSEQ1.bit.CONV01 = 0x1; \
			AdcRegs.ADCCHSELSEQ1.bit.CONV02 = 0x2; \
			AdcRegs.ADCCHSELSEQ1.bit.CONV03 = 0x3; \
			AdcRegs.ADCCHSELSEQ2.bit.CONV04 = 0x4; \
			AdcRegs.ADCCHSELSEQ2.bit.CONV05 = 0x0; \
			AdcRegs.ADCCHSELSEQ2.bit.CONV06 = 0x0; \
		} while (0)

	#define ADC_A_POS_CUR_SAMPLING_y \
		do { \
			AdcRegs.ADCMAXCONV.all = 0x0006; \
			AdcRegs.ADCCHSELSEQ1.bit.CONV00 = 0x1; \
			AdcRegs.ADCCHSELSEQ1.bit.CONV01 = 0x1; \
			AdcRegs.ADCCHSELSEQ1.bit.CONV02 = 0x2; \
			AdcRegs.ADCCHSELSEQ1.bit.CONV03 = 0x3; \
			AdcRegs.ADCCHSELSEQ2.bit.CONV04 = 0x4; \
			AdcRegs.ADCCHSELSEQ2.bit.CONV05 = 0x5; \
			AdcRegs.ADCCHSELSEQ2.bit.CONV06 = 0x5; \
		} while (0)

	#define ADC_B_CUR_SAMPLING \
		do { \
			AdcRegs.ADCMAXCONV.all = 0x0004; \
			AdcRegs.ADCCHSELSEQ1.bit.CONV00 = 0x1; \
			AdcRegs.ADCCHSELSEQ1.bit.CONV01 = 0x1; \
			AdcRegs.ADCCHSELSEQ1.bit.CONV02 = 0x2; \
			AdcRegs.ADCCHSELSEQ1.bit.CONV03 = 0x3; \
			AdcRegs.ADCCHSELSEQ2.bit.CONV04 = 0x4; \
		} while (0)


   ADC_B_CUR_SAMPLING;

   AdcRegs.ADCTRL2.bit.EPWM_SOCA_SEQ1 = 1;	// Enable SOCA from ePWM to start SEQ1
   AdcRegs.ADCTRL2.bit.INT_ENA_SEQ1 = 1;  	// Enable SEQ1 interrupt (every EOS)

//--------- Configure EPWM1 ---------

   // Configure EPWM1 for ADC trigger and to generate an interrupt when trigger is initiated

   // Assumes ePWM1 clock is already enabled in InitSysCtrl();
   EPwm1Regs.ETSEL.bit.SOCAEN = 1;        // Enable SOC on A group
   EPwm1Regs.ETSEL.bit.SOCASEL = 4;       // Select SOC from from CPMA on upcount
   EPwm1Regs.ETPS.bit.SOCAPRD = 1;        // Generate pulse on 1st event
   // Enable interrupt at SOC
   EPwm1Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;     // Select INT on Zero event
   EPwm1Regs.ETSEL.bit.INTEN = PWM1_INT_ENABLE;  // Enable INT
   EPwm1Regs.ETPS.bit.INTPRD = ET_1ST;           // Generate INT on 1st event
   //EPwm1Regs.CMPA.half.CMPA = 0x0752;	  // Set compare A value	//40kHz
   //EPwm1Regs.TBPRD = 0x0752;              // Set period for ePWM1
   EPwm1Regs.CMPA.half.CMPA = 0x03A9;	  // Set compare A value	//80kHz
   EPwm1Regs.TBPRD = 0x03A9;              // Set period for ePWM1
   //EPwm1Regs.CMPA.half.CMPA = 0x04CD;	  // Set compare A value	//61kHz
   //EPwm1Regs.TBPRD =0x04CD;              // Set period for ePWM1
   EPwm1Regs.TBCTL.bit.CTRMODE = 0;		  // count up and start
   //NOTE: PWM TBCLK is 150MHz/2 = 75MHz
   //Timing formula: Tpwm = (TBPRD+1)*(1/75MHz)

   GpioDataRegs.GPASET.bit.GPIO0 = 1; 	// Initially set OutA = V+ and OutB = GND
   GpioDataRegs.GPASET.bit.GPIO1 = 1; 	// Initially set OutA = V+ and OutB = GND
   GpioDataRegs.GPASET.bit.GPIO2 = 1; 	// Initially set OutA = V+ and OutB = GND
   GpioDataRegs.GPASET.bit.GPIO3 = 1; 	// Initially set OutA = V+ and OutB = GND
   DELAY_US(1000000);
   GpioDataRegs.GPBSET.bit.GPIO32 = 1;  // Enable H-bridge operation ...connect to PWMH pin
   GpioDataRegs.GPBSET.bit.GPIO33 = 1;  // Enable H-bridge operation ...connect to PWMH pin
   GpioDataRegs.GPBSET.bit.GPIO49 = 1;  // Enable H-bridge operation ...connect to PWMH pin
   GpioDataRegs.GPBSET.bit.GPIO62 = 1;  // Enable H-bridge operation ...connect to PWMH pin


// Wait for in idle until new ADC samples received, then perform current and position control
   for(;;)
   {
      IdleCount++;	// Idle operation time

      //Coordinate_Transform();

      if(BB_cntrl_flag == 1)
      {
    	  DINT;  // Disable interrupts, should not be necessary with correct calculation timing relative to

    	  if(Position_PID_cntrl_flag_x == 1)			// Perform Position control algorithm when flag is set
    	  {

    		  delta_I_x = Position_PID_Cntrl_x();


    		  Current_Cmd[0] =	Cur_bias + delta_I_x;

    		  Current_Cmd[1] =	Cur_bias - delta_I_x;


    		  // FIGURE out better way to code this! *********************



    		  if(Current_Cmd[0] > Max_Current)
    		  	{
    			  Current_Cmd[0] = Max_Current;
    		  	}

    		  if(Current_Cmd[0] < Min_Current)
    		  	{
    			  Current_Cmd[0] = Min_Current;
    		  	}

    		  if(Current_Cmd[1] > Max_Current)
    		  	{
    			  Current_Cmd[1] = Max_Current;
    		  	}

    		  if(Current_Cmd[1] < Min_Current)
    		  	{
    			  Current_Cmd[1] = Min_Current;
    		  	}


    		  // FIGURE out better way to code this! ^^^^^


    		  Position_PID_cntrl_flag_x = 0;		// Reset flag


    	  }

    	  else if(Position_PID_cntrl_flag_y == 1)			// Perform Position control algorithm when flag is set
    	  {


    		  delta_I_y = Position_PID_Cntrl_y();

    		  Current_Cmd[2] =	Cur_bias + delta_I_y;

    		  Current_Cmd[3] =	Cur_bias - delta_I_y;


    		  // FIGURE out better way to code this! *********************

    		  if(Current_Cmd[2] > Max_Current)
    		  {
    			  Current_Cmd[2] = Max_Current;
    		  }

    		  if(Current_Cmd[2] < Min_Current)
    		  {
    			  Current_Cmd[2] = Min_Current;
    		  }

    		  if(Current_Cmd[3] > Max_Current)
    		  {
    			  Current_Cmd[3] = Max_Current;
    		  }

    		  if(Current_Cmd[3] < Min_Current)
    		  {
    			  Current_Cmd[3] = Min_Current;
    		  }

    		  // FIGURE out better way to code this! ^^^^^


    		  Position_PID_cntrl_flag_y = 0;		// Reset flag

    	  }



    	  Bang_Bang_Cntrl(Current_Cmd);	// Current control performed after each sampling interval

    	  BB_cntrl_flag = 0;	// Reset flag

    	  GpioDataRegs.GPBCLEAR.bit.GPIO58 = 1;

    	  EINT;  // Enable interrupts
      }

   }

}


//void Bang_Bang_Cntrl(float32 const *Current_Cmd)
void Bang_Bang_Cntrl(float32 *Current_Cmd)
{

	float32 Current_Meas[coil_num] = {0};
	Uint16 Current_IO_set,Current_IO_clr;
	Uint16 i;


	#define CLR_IO_mask 0xFFF0

	Current_IO_set = 0;
	Current_IO_clr = 0;

	//Current_Avg_converted = (Current_samp1 + Current_samp2 + Current_samp3 + Current_samp4)*DivBy4*ADC_CONVERSION;

	//Current_Avg_converted = ((Current_samp1 + Current_samp2 + Current_samp3 + Current_samp4)*Current_Conv_Constant)+(Current_Conv_Offset);

	Current_Meas[0] = ((Current_samp1 + Current_samp2)*Current_Conv_Constant)+(Current_Conv_Offset);

	Current_Meas[1] = ((Current_samp3 + Current_samp4)*Current_Conv_Constant)+(Current_Conv_Offset);

	Current_Meas[2] = ((Current_samp5 + Current_samp6)*Current_Conv_Constant)+(Current_Conv_Offset);

	Current_Meas[3] = ((Current_samp7 + Current_samp8)*Current_Conv_Constant)+(Current_Conv_Offset);


	// Bang-Bang compare


	for( i=0; i<coil_num ; i++ )
	{
		if(Current_Cmd[i] > Current_Meas[i])
		{
			Current_IO_set = Current_IO_set | (1<<i);  // Set direction for pin
		}
	}

	Current_IO_clr = CLR_IO_mask ^ (~Current_IO_set);

	GpioDataRegs.GPASET.all = Current_IO_set;
	GpioDataRegs.GPACLEAR.all = Current_IO_clr;


}


float32 Position_PID_Cntrl_x(void) //Previously was Position_PID_Cntrl_x_test_new_deriv_v2
{
	float32 Position_Avg_converted;
	float32 Error_pos;
	float32 Error_pos_filtered;
	//static float32 Error_pos_prev = 0;
	static float32 Error_pos_filtered_prev = 0;
	static float32 Int_pos_prev = 0;
	static float32 Der_pos_prev = 0;
	static float32 delta_I_x_calc;
	float32 Prop_pos, Int_pos, Der_pos;


	//Position_Avg_converted = ((Position_samp1 + Position_samp2 + Position_samp3 + Position_samp4)*Position_Conv_Constant)+Position_Conv_Offset;

	Position_Avg_converted = ((Position_sampx1 + Position_sampx2 + Position_sampx3 + Position_sampx4)*Position_Conv_Constant_x)+Position_Conv_Offset_x;


	Error_pos = Position_Avg_converted - Pos_bias;

	//Error_pos = Position_Avg_converted - Pos_bias + Delta_Setpoint;

	Error_pos_filtered = Digital_Filter_2nd_order_x(Error_pos);

	// Proportional Term
	Prop_pos = Error_pos_filtered*KP_pos;

	// Integral Term

	Int_pos = (KI_pos*(Error_pos_filtered + Error_pos_filtered_prev)*(Pos_sample_time*DivBy2))+Int_pos_prev;

	// Integral Wind-up Saturation Limits
	if(Int_pos > Integral_max)
	{
		Int_pos = Integral_max;
	}

	if(Int_pos < Integral_min)
	{
		Int_pos = Integral_min;
	}

	// Derivative Term

	Der_pos = (KD_pos*(Error_pos_filtered - Error_pos_filtered_prev)-(Der_pos_prev*mTfplusPt5Ts))*one_div_plusTfplusPt5Ts;


	// Compute Current Threshold


	if((Error_pos_filtered > 0.0005)|(Error_pos_filtered < (-0.0005)))	// Integral Reset // PD control used when error is large
	{
		delta_I_x_calc = Prop_pos + Der_pos;		// PD control
		Int_pos = 0;
	}

	else
	{
		delta_I_x_calc = Prop_pos + Int_pos + Der_pos;	//PID control
	}


	Int_pos_prev = Int_pos; // This was moved below the Int_pos = 0 statement

	Der_pos_prev = Der_pos;

	Error_pos_filtered_prev = Error_pos_filtered;

	//asm ("      nop");

	// Limit max and min position controller output

	return delta_I_x_calc;


}

float32 Position_PID_Cntrl_y(void)
{
	float32 Position_Avg_converted;
	float32 Error_pos;
	float32 Error_pos_filtered;
	//static float32 Error_pos_prev = 0;
	static float32 Error_pos_filtered_prev = 0;
	static float32 Int_pos_prev = 0;
	static float32 Der_pos_prev = 0;
	static float32 delta_I_y_calc;
	float32 Prop_pos, Int_pos, Der_pos;


	Position_Avg_converted = ((Position_sampy1 + Position_sampy2 + Position_sampy3 + Position_sampy4)*Position_Conv_Constant_y)+Position_Conv_Offset_y;


	//Error_pos = Position_Avg_converted - Pos_bias;

	Error_pos = Position_Avg_converted - Pos_bias + Delta_Setpoint;

	Error_pos_filtered = Digital_Filter_2nd_order_y(Error_pos);

	// Proportional Term
	Prop_pos = Error_pos_filtered*KP_pos;

	// Integral Term

	Int_pos = (KI_pos*(Error_pos_filtered + Error_pos_filtered_prev)*(Pos_sample_time*DivBy2))+Int_pos_prev;

	// Integral Wind-up Saturation Limits
	if(Int_pos > Integral_max)
	{
		Int_pos = Integral_max;
	}

	if(Int_pos < Integral_min)
	{
		Int_pos = Integral_min;
	}

	// Derivative Term

	Der_pos = (KD_pos*(Error_pos_filtered - Error_pos_filtered_prev)-(Der_pos_prev*mTfplusPt5Ts))*one_div_plusTfplusPt5Ts;


	// Compute Current Threshold


	if((Error_pos_filtered > 0.0005)|(Error_pos_filtered < (-0.0005)))	// Integral Reset // PD control used when error is large
	{
		delta_I_y_calc = Prop_pos + Der_pos;		// PD control
		Int_pos = 0;
	}

	else
	{
		delta_I_y_calc = Prop_pos + Int_pos + Der_pos;	//PID control
	}


	Int_pos_prev = Int_pos; // This was moved below the Int_pos = 0 statement

	Der_pos_prev = Der_pos;

	Error_pos_filtered_prev = Error_pos_filtered;

	//asm ("      nop");

	// Limit max and min position controller output

	return delta_I_y_calc;


}


float32 Digital_Filter_2nd_order_x(float32 x_k) // SECOND ORDER
{
	// Input array
	static float32 x_input[3] = { 0, 0, 0 };
	// Output array
	static float32 y_output[3] = { 0, 0, 0 };
	// Index value
	static Uint16 k=0,k_m1=0,k_m2=0;

	k++;

	if(k>2)
	{
		k = 0;
	}

	switch(k)
	{
		case 0:
			k_m1 = 2;
			k_m2 = 1;
			break;
		case 1:
			k_m1 = 0;
			k_m2 = 2;
			break;
		case 2:
			k_m1 = 1;
			k_m2 = 0;
			break;
	}

	x_input[k] = x_k;

	y_output[k] = b0*x_input[k] + b1*x_input[k_m1] + b2*x_input[k_m2] - a1*y_output[k_m1] - a2*y_output[k_m2];

	return y_output[k];

}

float32 Digital_Filter_2nd_order_y(float32 x_k) // SECOND ORDER
{
	// Input array
	static float32 x_input[3] = { 0, 0, 0 };
	// Output array
	static float32 y_output[3] = { 0, 0, 0 };
	// Index value
	static Uint16 k=0,k_m1=0,k_m2=0;

	k++;

	if(k>2)
	{
		k = 0;
	}

	switch(k)
	{
		case 0:
			k_m1 = 2;
			k_m2 = 1;
			break;
		case 1:
			k_m1 = 0;
			k_m2 = 2;
			break;
		case 2:
			k_m1 = 1;
			k_m2 = 0;
			break;
	}

	x_input[k] = x_k;

	y_output[k] = b0*x_input[k] + b1*x_input[k_m1] + b2*x_input[k_m2] - a1*y_output[k_m1] - a2*y_output[k_m2];

	return y_output[k];

}



interrupt void  adc_isr(void)
{

	DINT;

	GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;	// Instrumentation for timing
	GpioDataRegs.GPBSET.bit.GPIO58 = 1;

	BB_cntrl_flag = 1;		// Set flag for Current controller

	//samp1 = AdcRegs.ADCRESULT0 >>4;  	// 	In simultaneous sampling mode, the first two data readings are incorrect
	//samp2 = AdcRegs.ADCRESULT1 >>4;	//	and must be thrown out, thus, I will not read the values

	Current_samp1 = AdcRegs.ADCRESULT2 >>4;
	Current_samp2 = AdcRegs.ADCRESULT3 >>4;
	Current_samp3 = AdcRegs.ADCRESULT4 >>4;
	Current_samp4 = AdcRegs.ADCRESULT5 >>4;
	Current_samp5 = AdcRegs.ADCRESULT6 >>4;
	Current_samp6 = AdcRegs.ADCRESULT7 >>4;
	Current_samp7 = AdcRegs.ADCRESULT8 >>4;
	Current_samp8 = AdcRegs.ADCRESULT9 >>4;

// ------------ Initiates step change in setpoint for debug purposes

	/*

	if(Time_counter == 200000)
		{
			GpioDataRegs.GPBCLEAR.bit.GPIO48 = 1;
		}



	if(Time_counter == 400000)
		{

			if(Delta_Setpoint == Setpoint_val1)
				{
				Delta_Setpoint = Setpoint_val2;
				}
			else
				{
				Delta_Setpoint = Setpoint_val1;
				}

			Time_counter = 0; // Clear time counter (5 seconds has transpired!)

			GpioDataRegs.GPBSET.bit.GPIO48 = 1;
		}

		*/

// ------------

	if(Position_Sampled_Flag_x == 1)
		{
			Position_PID_cntrl_flag_x = 1;	// Set flag to perform position controller algorithm in main()
			Position_Sampled_Flag_x = 0;		// Clear flag

			Position_sampx1 = AdcRegs.ADCRESULT10 >>4;
			Position_sampx2 = AdcRegs.ADCRESULT11 >>4;
			Position_sampx3 = AdcRegs.ADCRESULT12 >>4;
			Position_sampx4 = AdcRegs.ADCRESULT13 >>4;

			// Next cycle will only convert current sensor data
			ADC_B_CUR_SAMPLING;				// Macro

		}


	else if(Position_Sampled_Flag_y == 1)
	{
			Position_PID_cntrl_flag_y= 1;	// Set flag to perform position controller algorithm in main()
			Position_Sampled_Flag_y = 0;		// Clear flag

			Position_sampy1 = AdcRegs.ADCRESULT10 >>4;
			Position_sampy2 = AdcRegs.ADCRESULT11 >>4;
			Position_sampy3 = AdcRegs.ADCRESULT12 >>4;
			Position_sampy4 = AdcRegs.ADCRESULT13 >>4;

			// Next cycle will only convert current sensor data
			ADC_B_CUR_SAMPLING;				// Macro
	}

	// an "else" here may be more efficient... test this!!
	else
	{

	if(InterruptCount == (Pos_Sample_x))
		{
			// Next ADC cycle will convert read X position and current sensor data
			ADC_A_POS_CUR_SAMPLING_x;			// Macro
			Position_Sampled_Flag_x = 1;		// Set flag so next ADC cycle the converted position data for Y is read
			//InterruptCount = 0;				// Reset counter

		}

	if(InterruptCount == (Pos_Sample_y))
		{
			// Next ADC cycle will convert read Y position and current sensor data
			ADC_A_POS_CUR_SAMPLING_y;			// Macro
			Position_Sampled_Flag_y = 1;		// Set flag so next ADC cycle the converted position data for X is read
			InterruptCount = 0;					// Reset counter

		}

	}

	InterruptCount++;


	// Reinitialize for next ADC sequence
	AdcRegs.ADCTRL2.bit.RST_SEQ1 = 1;         // Reset SEQ1
	AdcRegs.ADCST.bit.INT_SEQ1_CLR = 1;       // Clear INT SEQ1 bit
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;   // Acknowledge interrupt to PIE

// ------------
	Time_counter++;
// ------------


	EINT;

	return;
}


interrupt void  EPWM1_isr(void)
{

	// Clear INT flag for this timer
	EPwm1Regs.ETCLR.bit.INT = 1;

	// Acknowledge this interrupt to receive more interrupts from group 3
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
	return;
}




void GPIO_setup(void)
{

	   // GPIO (GENERAL PURPOSE I/O) CONFIG
	   //--------------------------------------------------------------------------------------
	   //--------------------------------------------------------------------------------------
	   // QUICK NOTES on GPIO CONFIG USAGE:
	   //----------------------------------
	   // If GpioCtrlRegs.GP?MUX?bit.GPIO?= 1, 2 or 3 (i.e. Non GPIO func), then leave
	   //	rest of lines commented
	   // If GpioCtrlRegs.GP?MUX?bit.GPIO?= 0 (i.e. GPIO func), then:
	   //	  1) uncomment GpioCtrlRegs.GP?DIR.bit.GPIO? = ? and choose pin to be IN or OUT direc.
	   //	  2) If IN, can leave next two lines commented
	   //	  3) If OUT, uncomment line with ..GPACLEAR.. to force pin LOW or
	   //			     uncomment line with ..GPASET.. to force pin HIGH
	   //--------------------------------------------------------------------------------------

		EALLOW;

	   //--------------------------------------------------------------------------------------
	   //  GPIO-00 - PIN FUNCTION = --Spare--
	   	GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 0;		// 0=GPIO,  1=EPWM1A,  2=Resv,  3=Resv
	   	GpioCtrlRegs.GPADIR.bit.GPIO0 = 1;		// 1=OUTput,  0=INput
	   	GpioDataRegs.GPACLEAR.bit.GPIO0 = 1;	// uncomment if --> Set Low initially
	   //	GpioDataRegs.GPASET.bit.GPIO0 = 1;		// uncomment if --> Set High initially
	   //--------------------------------------------------------------------------------------
	   //  GPIO-01 - PIN FUNCTION = --Spare--
	   	GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 0;		// 0=GPIO,  1=EPWM1B,  2=ECAP6,  3=MFSR-B
	   	GpioCtrlRegs.GPADIR.bit.GPIO1 = 1;		// 1=OUTput,  0=INput
	   	GpioDataRegs.GPACLEAR.bit.GPIO1 = 1;	// uncomment if --> Set Low initially
	   //	GpioDataRegs.GPASET.bit.GPIO1 = 1;		// uncomment if --> Set High initially
	   //--------------------------------------------------------------------------------------
	   //  GPIO-02 - PIN FUNCTION = --Spare--
	   	GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 0;		// 0=GPIO,  1=EPWM2A,  2=Resv,  3=Resv
	   	GpioCtrlRegs.GPADIR.bit.GPIO2 = 1;		// 1=OUTput,  0=INput
	   	GpioDataRegs.GPACLEAR.bit.GPIO2 = 1;	// uncomment if --> Set Low initially
	   //	GpioDataRegs.GPASET.bit.GPIO2 = 1;		// uncomment if --> Set High initially
	   //--------------------------------------------------------------------------------------
	   //  GPIO-03 - PIN FUNCTION = --Spare--
	   	GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 0;		// 0=GPIO,  1=EPWM2B,  2=ECAP5,  3=MCLKR-B
	   	GpioCtrlRegs.GPADIR.bit.GPIO3 = 1;		// 1=OUTput,  0=INput
	   	GpioDataRegs.GPACLEAR.bit.GPIO3 = 1;	// uncomment if --> Set Low initially
	   //	GpioDataRegs.GPASET.bit.GPIO3 = 1;		// uncomment if --> Set High initially
	   //--------------------------------------------------------------------------------------
	   //  GPIO-04 - PIN FUNCTION = --Spare--
	   	GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 0;		// 0=GPIO,  1=EPWM3A,  2=Resv,  3=Resv
	   	GpioCtrlRegs.GPADIR.bit.GPIO4 = 1;		// 1=OUTput,  0=INput
	   	GpioDataRegs.GPACLEAR.bit.GPIO4 = 1;	// uncomment if --> Set Low initially
	   //	GpioDataRegs.GPASET.bit.GPIO4 = 1;		// uncomment if --> Set High initially
	   //--------------------------------------------------------------------------------------
	   //  GPIO-05 - PIN FUNCTION = --Spare--
	   	GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 0;		// 0=GPIO,  1=EPWM3B,  2=MFSR-A,  3=ECAP1
	   	GpioCtrlRegs.GPADIR.bit.GPIO5 = 1;		// 1=OUTput,  0=INput
	   	GpioDataRegs.GPACLEAR.bit.GPIO5 = 1;	// uncomment if --> Set Low initially
	   //	GpioDataRegs.GPASET.bit.GPIO5 = 1;		// uncomment if --> Set High initially
	   //--------------------------------------------------------------------------------------
	   //  GPIO-06 - PIN FUNCTION = --Spare--
	   	GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 0;		// 0=GPIO,  1=EPWM4A,  2=SYNCI,  3=SYNCO
	   	GpioCtrlRegs.GPADIR.bit.GPIO6 = 1;		// 1=OUTput,  0=INput
	   	GpioDataRegs.GPACLEAR.bit.GPIO6 = 1;	// uncomment if --> Set Low initially
	   //	GpioDataRegs.GPASET.bit.GPIO6 = 1;		// uncomment if --> Set High initially
	   //--------------------------------------------------------------------------------------
	   //  GPIO-07 - PIN FUNCTION = --Spare--
	   	GpioCtrlRegs.GPAMUX1.bit.GPIO7 = 0;		// 0=GPIO,  1=EPWM4B,  2=MCLKR-A,  3=ECAP2
	   	GpioCtrlRegs.GPADIR.bit.GPIO7 = 1;		// 1=OUTput,  0=INput
	   	GpioDataRegs.GPACLEAR.bit.GPIO7 = 1;	// uncomment if --> Set Low initially
	   //	GpioDataRegs.GPASET.bit.GPIO7 = 1;		// uncomment if --> Set High initially
	   //--------------------------------------------------------------------------------------
	   //  GPIO-08 - PIN FUNCTION = --Spare--
	   	GpioCtrlRegs.GPAMUX1.bit.GPIO8 = 0;		// 0=GPIO,  1=EPWM5A,  2=CANTX-B,  3=ADCSOC-A
	   	GpioCtrlRegs.GPADIR.bit.GPIO8 = 1;		// 1=OUTput,  0=INput
	   	GpioDataRegs.GPACLEAR.bit.GPIO8 = 1;	// uncomment if --> Set Low initially
	   //	GpioDataRegs.GPASET.bit.GPIO8 = 1;		// uncomment if --> Set High initially
	   //--------------------------------------------------------------------------------------
	   //  GPIO-09 - PIN FUNCTION = --Spare--
	   	GpioCtrlRegs.GPAMUX1.bit.GPIO9 = 0;		// 0=GPIO,  1=EPWM5B,  2=SCITX-B,  3=ECAP3
	   	GpioCtrlRegs.GPADIR.bit.GPIO9 = 1;		// 1=OUTput,  0=INput
	   	GpioDataRegs.GPACLEAR.bit.GPIO9 = 1;	// uncomment if --> Set Low initially
	   //	GpioDataRegs.GPASET.bit.GPIO9 = 1;		// uncomment if --> Set High initially
	   //--------------------------------------------------------------------------------------
	   //  GPIO-10 - PIN FUNCTION = --Spare--
	   	GpioCtrlRegs.GPAMUX1.bit.GPIO10 = 0;	// 0=GPIO,  1=EPWM6A,  2=CANRX-B,  3=ADCSOC-B
	   	GpioCtrlRegs.GPADIR.bit.GPIO10 = 1;		// 1=OUTput,  0=INput
	   	GpioDataRegs.GPACLEAR.bit.GPIO10 = 1;	// uncomment if --> Set Low initially
	   //	GpioDataRegs.GPASET.bit.GPIO10 = 1;		// uncomment if --> Set High initially
	   //--------------------------------------------------------------------------------------
	   //  GPIO-11 - PIN FUNCTION = --Spare--
	   	GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 0;	// 0=GPIO,  1=EPWM6B,  2=SCIRX-B,  3=ECAP4
	   	GpioCtrlRegs.GPADIR.bit.GPIO11 = 1;		// 1=OUTput,  0=INput
	   	GpioDataRegs.GPACLEAR.bit.GPIO11 = 1;	// uncomment if --> Set Low initially
	   //	GpioDataRegs.GPASET.bit.GPIO11 = 1;		// uncomment if --> Set High initially
	   //--------------------------------------------------------------------------------------
	   //  GPIO-12 - PIN FUNCTION = --Spare--
	   	GpioCtrlRegs.GPAMUX1.bit.GPIO12 = 0;	// 0=GPIO,  1=TZ1,  2=CANTX-B,  3=MDX-B
	   	GpioCtrlRegs.GPADIR.bit.GPIO12 = 1;		// 1=OUTput,  0=INput
	   	GpioDataRegs.GPACLEAR.bit.GPIO12 = 1;	// uncomment if --> Set Low initially
	   //	GpioDataRegs.GPASET.bit.GPIO12 = 1;		// uncomment if --> Set High initially
	   //--------------------------------------------------------------------------------------
	   //  GPIO-13 - PIN FUNCTION = --Spare--
	   	GpioCtrlRegs.GPAMUX1.bit.GPIO13 = 0;	// 0=GPIO,  1=TZ2,  2=CANRX-B,  3=MDR-B
	   	GpioCtrlRegs.GPADIR.bit.GPIO13 = 1;		// 1=OUTput,  0=INput
	   	GpioDataRegs.GPACLEAR.bit.GPIO13 = 1;	// uncomment if --> Set Low initially
	   //	GpioDataRegs.GPASET.bit.GPIO13 = 1;		// uncomment if --> Set High initially
	   //--------------------------------------------------------------------------------------
	   //  GPIO-14 - PIN FUNCTION = --Spare--
	   	GpioCtrlRegs.GPAMUX1.bit.GPIO14 = 0;	// 0=GPIO,  1=TZ3,  2=SCITX-B,  3=MCLKX-B
	   	GpioCtrlRegs.GPADIR.bit.GPIO14 = 1;		// 1=OUTput,  0=INput
	   	GpioDataRegs.GPACLEAR.bit.GPIO14 = 1;	// uncomment if --> Set Low initially
	   //	GpioDataRegs.GPASET.bit.GPIO14 = 1;		// uncomment if --> Set High initially
	   //--------------------------------------------------------------------------------------
	   //  GPIO-15 - PIN FUNCTION = --Spare--
	   	GpioCtrlRegs.GPAMUX1.bit.GPIO15 = 0;	// 0=GPIO,  1=TZ4,  2=SCIRX-B,  3=MFSX-B
	   	GpioCtrlRegs.GPADIR.bit.GPIO15 = 1;		// 1=OUTput,  0=INput
	   	GpioDataRegs.GPACLEAR.bit.GPIO15 = 1;	// uncomment if --> Set Low initially
	   //	GpioDataRegs.GPASET.bit.GPIO15 = 1;		// uncomment if --> Set High initially
	   //--------------------------------------------------------------------------------------
	   //--------------------------------------------------------------------------------------
	   //  GPIO-16 - PIN FUNCTION = --Spare--
	   	GpioCtrlRegs.GPAMUX2.bit.GPIO16 = 0;	// 0=GPIO,  1=SPISIMO-A,  2=CANTX-B,  3=TZ5
	   	GpioCtrlRegs.GPADIR.bit.GPIO16 = 1;		// 1=OUTput,  0=INput
	   	GpioDataRegs.GPACLEAR.bit.GPIO16 = 1;	// uncomment if --> Set Low initially
	   //	GpioDataRegs.GPASET.bit.GPIO16 = 1;		// uncomment if --> Set High initially
	   //--------------------------------------------------------------------------------------
	   //  GPIO-17 - PIN FUNCTION = --Spare--
	   	GpioCtrlRegs.GPAMUX2.bit.GPIO17 = 0;	// 0=GPIO,  1=SPISOMI-A,  2=CANRX-B,  3=TZ6
	   	GpioCtrlRegs.GPADIR.bit.GPIO17 = 1;		// 1=OUTput,  0=INput
	   	GpioDataRegs.GPACLEAR.bit.GPIO17 = 1;	// uncomment if --> Set Low initially
	   //	GpioDataRegs.GPASET.bit.GPIO17 = 1;		// uncomment if --> Set High initially
	   //--------------------------------------------------------------------------------------
	   //  GPIO-18 - PIN FUNCTION = --Spare--
	   	GpioCtrlRegs.GPAMUX2.bit.GPIO18 = 0;	// 0=GPIO,  1=SPICLK-A,  2=SCITX-B,  3=CANRX-A
	   	GpioCtrlRegs.GPADIR.bit.GPIO18 = 1;		// 1=OUTput,  0=INput
	   	GpioDataRegs.GPACLEAR.bit.GPIO18 = 1;	// uncomment if --> Set Low initially
	   //	GpioDataRegs.GPASET.bit.GPIO18 = 1;		// uncomment if --> Set High initially
	   //--------------------------------------------------------------------------------------
	   //  GPIO-19 - PIN FUNCTION = --Spare--
	   	GpioCtrlRegs.GPAMUX2.bit.GPIO19 = 0;	// 0=GPIO,  1=SPISTE-A,  2=SCIRX-B,  3=CANTX-A
	   	GpioCtrlRegs.GPADIR.bit.GPIO19 = 1;		// 1=OUTput,  0=INput
	   	GpioDataRegs.GPACLEAR.bit.GPIO19 = 1;	// uncomment if --> Set Low initially
	   //	GpioDataRegs.GPASET.bit.GPIO19 = 1;		// uncomment if --> Set High initially
	   //--------------------------------------------------------------------------------------
	   //  GPIO-20 - PIN FUNCTION = --Spare--
	   	GpioCtrlRegs.GPAMUX2.bit.GPIO20 = 0;	// 0=GPIO,  1=EQEPA-1,  2=MDX-A,  3=CANTX-B
	   	GpioCtrlRegs.GPADIR.bit.GPIO20 = 1;		// 1=OUTput,  0=INput
	   	GpioDataRegs.GPACLEAR.bit.GPIO20 = 1;	// uncomment if --> Set Low initially
	   //	GpioDataRegs.GPASET.bit.GPIO20 = 1;		// uncomment if --> Set High initially
	   //--------------------------------------------------------------------------------------
	   //  GPIO-21 - PIN FUNCTION = --Spare--
	   	GpioCtrlRegs.GPAMUX2.bit.GPIO21 = 0;	// 0=GPIO,  1=EQEPB-1,  2=MDR-A,  3=CANRX-B
	   	GpioCtrlRegs.GPADIR.bit.GPIO21 = 1;		// 1=OUTput,  0=INput
	   	GpioDataRegs.GPACLEAR.bit.GPIO21 = 1;	// uncomment if --> Set Low initially
	   //	GpioDataRegs.GPASET.bit.GPIO21 = 1;		// uncomment if --> Set High initially
	   //--------------------------------------------------------------------------------------
	   //  GPIO-22 - PIN FUNCTION = --Spare--
	   	GpioCtrlRegs.GPAMUX2.bit.GPIO22 = 0;	// 0=GPIO,  1=EQEPS-1,  2=MCLKX-A,  3=SCITX-B
	   	GpioCtrlRegs.GPADIR.bit.GPIO22 = 1;		// 1=OUTput,  0=INput
	   	GpioDataRegs.GPACLEAR.bit.GPIO22 = 1;	// uncomment if --> Set Low initially
	   //	GpioDataRegs.GPASET.bit.GPIO22 = 1;		// uncomment if --> Set High initially
	   //--------------------------------------------------------------------------------------
	   //  GPIO-23 - PIN FUNCTION = --Spare--
	   	GpioCtrlRegs.GPAMUX2.bit.GPIO23 = 0;	// 0=GPIO,  1=EQEPI-1,  2=MFSX-A,  3=SCIRX-B
	   	GpioCtrlRegs.GPADIR.bit.GPIO23 = 1;		// 1=OUTput,  0=INput
	   	GpioDataRegs.GPACLEAR.bit.GPIO23 = 1;	// uncomment if --> Set Low initially
	   //	GpioDataRegs.GPASET.bit.GPIO23 = 1;		// uncomment if --> Set High initially
	   //--------------------------------------------------------------------------------------
	   //  GPIO-24 - PIN FUNCTION = --Spare--
	   	GpioCtrlRegs.GPAMUX2.bit.GPIO24 = 0;	// 0=GPIO,  1=ECAP1,  2=EQEPA-2,  3=MDX-B
	   	GpioCtrlRegs.GPADIR.bit.GPIO24 = 1;		// 1=OUTput,  0=INput
	   	GpioDataRegs.GPACLEAR.bit.GPIO24 = 1;	// uncomment if --> Set Low initially
	   //	GpioDataRegs.GPASET.bit.GPIO24 = 1;		// uncomment if --> Set High initially
	   //--------------------------------------------------------------------------------------
	   //  GPIO-25 - PIN FUNCTION = --Spare--
	   	GpioCtrlRegs.GPAMUX2.bit.GPIO25 = 0;	// 0=GPIO,  1=ECAP2,  2=EQEPB-2,  3=MDR-B
	   	GpioCtrlRegs.GPADIR.bit.GPIO25 = 1;		// 1=OUTput,  0=INput
	   	GpioDataRegs.GPACLEAR.bit.GPIO25 = 1;	// uncomment if --> Set Low initially
	   //	GpioDataRegs.GPASET.bit.GPIO25 = 1;		// uncomment if --> Set High initially
	   //--------------------------------------------------------------------------------------
	   //  GPIO-26 - PIN FUNCTION = --Spare--
	   	GpioCtrlRegs.GPAMUX2.bit.GPIO26 = 0;	// 0=GPIO,  1=ECAP3,  2=EQEPI-2,  3=MCLKX-B
	   	GpioCtrlRegs.GPADIR.bit.GPIO26 = 1;		// 1=OUTput,  0=INput
	   	GpioDataRegs.GPACLEAR.bit.GPIO26 = 1;	// uncomment if --> Set Low initially
	   //	GpioDataRegs.GPASET.bit.GPIO26 = 1;		// uncomment if --> Set High initially
	   //--------------------------------------------------------------------------------------
	   //  GPIO-27 - PIN FUNCTION = --Spare--
	   	GpioCtrlRegs.GPAMUX2.bit.GPIO27 = 0;	// 0=GPIO,  1=ECAP4,  2=EQEPS-2,  3=MFSX-B
	   	GpioCtrlRegs.GPADIR.bit.GPIO27 = 1;		// 1=OUTput,  0=INput
	   	GpioDataRegs.GPACLEAR.bit.GPIO27 = 1;	// uncomment if --> Set Low initially
	   //	GpioDataRegs.GPASET.bit.GPIO27 = 1;		// uncomment if --> Set High initially
	   //--------------------------------------------------------------------------------------
	   //  GPIO-28 - PIN FUNCTION = SCI-RX
	   	GpioCtrlRegs.GPAMUX2.bit.GPIO28 = 0;	// 0=GPIO,  1=SCIRX-A,  2=Resv,  3=Resv
	   	GpioCtrlRegs.GPADIR.bit.GPIO28 = 1;		// 1=OUTput,  0=INput
	   	GpioDataRegs.GPACLEAR.bit.GPIO28 = 1;	// uncomment if --> Set Low initially
	   //	GpioDataRegs.GPASET.bit.GPIO28 = 1;		// uncomment if --> Set High initially
	   //--------------------------------------------------------------------------------------
	   //  GPIO-29 - PIN FUNCTION = SCI-TX
	   	GpioCtrlRegs.GPAMUX2.bit.GPIO29 = 0;	// 0=GPIO,  1=SCITXD-A,  2=XA19,  3=Resv
	   	GpioCtrlRegs.GPADIR.bit.GPIO29 = 1;		// 1=OUTput,  0=INput
	   	GpioDataRegs.GPACLEAR.bit.GPIO29 = 1;	// uncomment if --> Set Low initially
	   //	GpioDataRegs.GPASET.bit.GPIO29 = 1;		// uncomment if --> Set High initially
	   //--------------------------------------------------------------------------------------
	   //  GPIO-30 - PIN FUNCTION = --Spare--
	   	GpioCtrlRegs.GPAMUX2.bit.GPIO30 = 0;	// 0=GPIO,  1=CANRX-A,  2=XA18,  3=Resv
	   	GpioCtrlRegs.GPADIR.bit.GPIO30 = 1;		// 1=OUTput,  0=INput
	   	GpioDataRegs.GPACLEAR.bit.GPIO30 = 1;	// uncomment if --> Set Low initially
	   //	GpioDataRegs.GPASET.bit.GPIO30 = 1;		// uncomment if --> Set High initially
	   //--------------------------------------------------------------------------------------
	   //  GPIO-31 - PIN FUNCTION = LED2 (for Release 1.1 and up F2833x controlCARDs)
	   	GpioCtrlRegs.GPAMUX2.bit.GPIO31 = 0;	// 0=GPIO,  1=CANTX-A,  2=XA17,  3=Resv
	   	GpioCtrlRegs.GPADIR.bit.GPIO31 = 1;		// 1=OUTput,  0=INput
	   //	GpioDataRegs.GPACLEAR.bit.GPIO31 = 1;	// uncomment if --> Set Low initially
	   //	GpioDataRegs.GPASET.bit.GPIO31 = 1;		// uncomment if --> Set High initially
	   //--------------------------------------------------------------------------------------
	   //--------------------------------------------------------------------------------------
	   //  GPIO-32 - PIN FUNCTION = --Spare--
	   	GpioCtrlRegs.GPBMUX1.bit.GPIO32 = 0;	// 0=GPIO,  1=I2C-SDA,  2=SYNCI,  3=ADCSOCA
	   	GpioCtrlRegs.GPBDIR.bit.GPIO32 = 1;		// 1=OUTput,  0=INput
	   	GpioDataRegs.GPBCLEAR.bit.GPIO32 = 1;	// uncomment if --> Set Low initially
	   //	GpioDataRegs.GPBSET.bit.GPIO32 = 1;		// uncomment if --> Set High initially
	   //--------------------------------------------------------------------------------------
	   //  GPIO-33 - PIN FUNCTION = --Spare--
	   	GpioCtrlRegs.GPBMUX1.bit.GPIO33 = 0;	// 0=GPIO,  1=I2C-SCL,  2=SYNCO,  3=ADCSOCB
	   	GpioCtrlRegs.GPBDIR.bit.GPIO33 = 1;		// 1=OUTput,  0=INput
	   	GpioDataRegs.GPBCLEAR.bit.GPIO33 = 1;	// uncomment if --> Set Low initially
	   //	GpioDataRegs.GPBSET.bit.GPIO33 = 1;		// uncomment if --> Set High initially
	   //--------------------------------------------------------------------------------------
	   //  GPIO-34 - PIN FUNCTION = LED3 (for Release 1.1 and up F2833x controlCARDs)
	   	GpioCtrlRegs.GPBMUX1.bit.GPIO34 = 0;	// 0=GPIO,  1=ECAP1,  2=Resv,  3=Resv
	   	GpioCtrlRegs.GPBDIR.bit.GPIO34 = 1;		// 1=OUTput,  0=INput
	   	GpioDataRegs.GPBCLEAR.bit.GPIO34 = 1;	// uncomment if --> Set Low initially
	   	//	GpioDataRegs.GPBSET.bit.GPIO34 = 1;		// uncomment if --> Set High initially
	   //--------------------------------------------------------------------------------------
	   //--------------------------------------------------------------------------------------
	   //  GPIO-35 - PIN FUNCTION = --Spare-- (SCI-TX on R1 F2833x controlCARD)
	   	GpioCtrlRegs.GPBMUX1.bit.GPIO35 = 0;	// 0=GPIO,  1=SCIA-TX,  2=Resv,  3=Resv
	   	GpioCtrlRegs.GPBDIR.bit.GPIO35 = 1;		// 1=OUTput,  0=INput
	   	GpioDataRegs.GPBCLEAR.bit.GPIO35 = 1;	// uncomment if --> Set Low initially
	   //	GpioDataRegs.GPBSET.bit.GPIO35 = 1;		// uncomment if --> Set High initially
	   //--------------------------------------------------------------------------------------
	   //  GPIO-36 - PIN FUNCTION = --Spare-- (SCI-RX on R1 F2833x controlCARD)
	   	GpioCtrlRegs.GPBMUX1.bit.GPIO36 = 0;	// 0=GPIO,  1=SCIA-RX,  2=Resv,  3=Resv
	   	GpioCtrlRegs.GPBDIR.bit.GPIO36 = 1;		// 1=OUTput,  0=INput
	   	GpioDataRegs.GPBCLEAR.bit.GPIO36 = 1;	// uncomment if --> Set Low initially
	   //	GpioDataRegs.GPBSET.bit.GPIO36 = 1;		// uncomment if --> Set High initially
	   //--------------------------------------------------------------------------------------
	   //  GPIO-38 - PIN FUNCTION = LED2 (for Release 1 F2833x controlCARDs)
	   	GpioCtrlRegs.GPBMUX1.bit.GPIO38 = 0;	// 0=GPIO,  1=Resv,  2=Resv,  3=Resv
	   	GpioCtrlRegs.GPBDIR.bit.GPIO38 = 1;		// 1=OUTput,  0=INput
	   	GpioDataRegs.GPBCLEAR.bit.GPIO38 = 1;	// uncomment if --> Set Low initially
	   //	GpioDataRegs.GPBSET.bit.GPIO38 = 1;		// uncomment if --> Set High initially
	   //--------------------------------------------------------------------------------------
	   //  GPIO-39 - PIN FUNCTION = LED3 (for Release 1 F2833x controlCARDs)
	   	GpioCtrlRegs.GPBMUX1.bit.GPIO39 = 0;	// 0=GPIO,  1=Resv,  2=XA16,  3=Resv
	   	GpioCtrlRegs.GPBDIR.bit.GPIO39 = 1;		// 1=OUTput,  0=INput
	   	GpioDataRegs.GPBCLEAR.bit.GPIO39 = 1;	// uncomment if --> Set Low initially
	   //	GpioDataRegs.GPBSET.bit.GPIO39 = 1;		// uncomment if --> Set High initially
	   //--------------------------------------------------------------------------------------

	   //  GPIO-48 - PIN FUNCTION = --Spare--
	   	GpioCtrlRegs.GPBMUX2.bit.GPIO48 = 0;	// 0=GPIO,  1=ECAP5,  2=XD31,  3=Resv
	   	GpioCtrlRegs.GPBDIR.bit.GPIO48 = 1;		// 1=OUTput,  0=INput
	   	GpioDataRegs.GPBCLEAR.bit.GPIO48 = 1;	// uncomment if --> Set Low initially
	   //	GpioDataRegs.GPBSET.bit.GPIO48 = 1;		// uncomment if --> Set High initially
	   //--------------------------------------------------------------------------------------
	   //  GPIO-49 - PIN FUNCTION = --Spare--
	   	GpioCtrlRegs.GPBMUX2.bit.GPIO49 = 0;	// 0=GPIO,  1=ECAP6,  2=XD30,  3=Resv
	   	GpioCtrlRegs.GPBDIR.bit.GPIO49 = 1;		// 1=OUTput,  0=INput
	   	GpioDataRegs.GPBCLEAR.bit.GPIO49 = 1;	// uncomment if --> Set Low initially
	   //	GpioDataRegs.GPBSET.bit.GPIO49 = 1;		// uncomment if --> Set High initially
	   //--------------------------------------------------------------------------------------

	   //  GPIO-58 - PIN FUNCTION = --Spare--
	   	GpioCtrlRegs.GPBMUX2.bit.GPIO58 = 0;	// 0=GPIO,  1=MCLKR-A,  2=XD21,  3=Resv
	   	GpioCtrlRegs.GPBDIR.bit.GPIO58 = 1;		// 1=OUTput,  0=INput
	   	GpioDataRegs.GPBCLEAR.bit.GPIO58 = 1;	// uncomment if --> Set Low initially
	   //	GpioDataRegs.GPBSET.bit.GPIO58 = 1;		// uncomment if --> Set High initially
	   //--------------------------------------------------------------------------------------
	   //  GPIO-59 - PIN FUNCTION = --Spare--
	   	GpioCtrlRegs.GPBMUX2.bit.GPIO59 = 0;	// 0=GPIO,  1=MFSR-A,  2=XD20,  3=Resv
	   	GpioCtrlRegs.GPBDIR.bit.GPIO59 = 1;		// 1=OUTput,  0=INput
	   	GpioDataRegs.GPBCLEAR.bit.GPIO59 = 1;	// uncomment if --> Set Low initially
	   //	GpioDataRegs.GPBSET.bit.GPIO59 = 1;		// uncomment if --> Set High initially
	   //--------------------------------------------------------------------------------------
	   //  GPIO-60 - PIN FUNCTION = --Spare--
	   	GpioCtrlRegs.GPBMUX2.bit.GPIO60 = 0;	// 0=GPIO,  1=MCLKR-B,  2=XD19,  3=Resv
	   	GpioCtrlRegs.GPBDIR.bit.GPIO60 = 1;		// 1=OUTput,  0=INput
	   	GpioDataRegs.GPBCLEAR.bit.GPIO60 = 1;	// uncomment if --> Set Low initially
	   //	GpioDataRegs.GPBSET.bit.GPIO60 = 1;		// uncomment if --> Set High initially
	   //--------------------------------------------------------------------------------------
	   //  GPIO-61 - PIN FUNCTION = --Spare--
	   	GpioCtrlRegs.GPBMUX2.bit.GPIO61 = 0;	// 0=GPIO,  1=MFSR-B,  2=XD18,  3=Resv
	   	GpioCtrlRegs.GPBDIR.bit.GPIO61 = 1;		// 1=OUTput,  0=INput
	   	GpioDataRegs.GPBCLEAR.bit.GPIO61 = 1;	// uncomment if --> Set Low initially
	   //	GpioDataRegs.GPBSET.bit.GPIO61 = 1;		// uncomment if --> Set High initially
	   //--------------------------------------------------------------------------------------
	   //  GPIO-62 - PIN FUNCTION = --Spare--
	   	GpioCtrlRegs.GPBMUX2.bit.GPIO62 = 0;	// 0=GPIO,  1=SCIRX-C,  2=XD17,  3=Resv
	   	GpioCtrlRegs.GPBDIR.bit.GPIO62 = 1;		// 1=OUTput,  0=INput
	   	GpioDataRegs.GPBCLEAR.bit.GPIO62 = 1;	// uncomment if --> Set Low initially
	   //	GpioDataRegs.GPBSET.bit.GPIO62 = 1;		// uncomment if --> Set High initially
	   //--------------------------------------------------------------------------------------
	   //  GPIO-63 - PIN FUNCTION = --Spare--
	   	GpioCtrlRegs.GPBMUX2.bit.GPIO63 = 0;	// 0=GPIO,  1=SCITX-C,  2=XD16,  3=Resv
	   	GpioCtrlRegs.GPBDIR.bit.GPIO63 = 1;		// 1=OUTput,  0=INput
	   	GpioDataRegs.GPBCLEAR.bit.GPIO63 = 1;	// uncomment if --> Set Low initially
	   //	GpioDataRegs.GPBSET.bit.GPIO63 = 1;		// uncomment if --> Set High initially
	   //--------------------------------------------------------------------------------------

	   //  GPIO-84 - PIN FUNCTION = --Spare--
	   	GpioCtrlRegs.GPCMUX2.bit.GPIO84 = 0;	// 0=GPIO,  1=GPIO,  2=XA12,  3=Resv
	   	GpioCtrlRegs.GPCDIR.bit.GPIO84 = 1;		// 1=OUTput,  0=INput
	   	GpioDataRegs.GPCCLEAR.bit.GPIO84 = 1;	// uncomment if --> Set Low initially
	   //	GpioDataRegs.GPCSET.bit.GPIO84 = 1;		// uncomment if --> Set High initially
	   //--------------------------------------------------------------------------------------
	   //  GPIO-85 - PIN FUNCTION = --Spare--
	   	GpioCtrlRegs.GPCMUX2.bit.GPIO85 = 0;	// 0=GPIO,  1=GPIO,  2=XA13,  3=Resv
	   	GpioCtrlRegs.GPCDIR.bit.GPIO85 = 1;		// 1=OUTput,  0=INput
	   	GpioDataRegs.GPCCLEAR.bit.GPIO85 = 1;	// uncomment if --> Set Low initially
	   //	GpioDataRegs.GPCSET.bit.GPIO85 = 1;		// uncomment if --> Set High initially
	   //--------------------------------------------------------------------------------------
	   //  GPIO-86 - PIN FUNCTION = --Spare--
	   	GpioCtrlRegs.GPCMUX2.bit.GPIO86 = 0;	// 0=GPIO,  1=GPIO,  2=XA14,  3=Resv
	   	GpioCtrlRegs.GPCDIR.bit.GPIO86 = 1;		// 1=OUTput,  0=INput
	   	GpioDataRegs.GPCCLEAR.bit.GPIO86 = 1;	// uncomment if --> Set Low initially
	   //	GpioDataRegs.GPCSET.bit.GPIO86 = 1;		// uncomment if --> Set High initially
	   //--------------------------------------------------------------------------------------
	   //  GPIO-87 - PIN FUNCTION = --Spare--
	   	GpioCtrlRegs.GPCMUX2.bit.GPIO87 = 0;	// 0=GPIO,  1=GPIO,  2=XA15,  3=Resv
	   	GpioCtrlRegs.GPCDIR.bit.GPIO87 = 1;		// 1=OUTput,  0=INput
	   	GpioDataRegs.GPCCLEAR.bit.GPIO87 = 1;	// uncomment if --> Set Low initially
	   //	GpioDataRegs.GPCSET.bit.GPIO87 = 1;		// uncomment if --> Set High initially
	   //--------------------------------------------------------------------------------------

	   	EDIS;	// Disable register access


}

/*
void Coordinate_Transform(void)
{

	int16 X_s_distance = 1;
	int16 Y_s_distance = 1;

	int16 X_R_distance;
	int16 Y_R_distance;

	int16 Rotor_Angle = 45;



	//#define cos 1
	//#define sin 1

	GpioDataRegs.GPASET.bit.GPIO5 = 1;

	X_R_distance = cos(0.78539)*X_s_distance;

	GpioDataRegs.GPACLEAR.bit.GPIO5 = 1;

	//Y_R_distance = -1*sin(2*3.14)*X_s_distance + cos(2*3.14)*Y_s_distance;

}
*/
