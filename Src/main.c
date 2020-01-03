/* USER CODE BEGIN Header */
/*
Firmware for the stm32f051 mcu for controlling brushless or brushed motors, current accepts servo signal input or dshot 300 ( bi-directional )
other modes disabled for now.


/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "eeprom.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

COMP_HandleTypeDef hcomp1;

IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim14;
TIM_HandleTypeDef htim15;
TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim17;
DMA_HandleTypeDef hdma_tim2_up;
DMA_HandleTypeDef hdma_tim2_ch3;
DMA_HandleTypeDef hdma_tim15_ch1_up_trig_com;

/* USER CODE BEGIN PV */
#define MP6531
//#define FD6288

uint16_t VirtAddVarTab[NB_OF_VAR] = {0x5555, 0x6666, 0x7777};
uint16_t VarDataTab[NB_OF_VAR] = {0, 0, 0};
uint16_t VarValue,VarDataTmp;

uint32_t ee_status;

// variables to use for eeprom function,  ie: EE_WriteVariable(VirtAddVarTab[EEvehiclemode],  vehicle_mode ;

enum userVars{
	EEvehiclemode = 0,
	EEdirection = 1,
	EEbidirection = 2,
//	EEbrake_on_stop = 3
};
char brushed_mode = 0;
char vehicle_mode = 5;    // 1 = quad mode / eeprom load mode , 2 = crawler / thruster mode,  3 = rc car mode,  4 = like car mode but with auto reverse after stop  5 = no eeprom !!!!
char sine_mode_range = 52;    // 0-52 at 0 there is no sine mode at 52 gives a input range 47-99 sine

char speed_control_mode = 0;
char bi_polar = 0;
char stall_protection = 1;
int boost_level = 1;     // for stall protection the amount that gets added to duty cycle.

char polling_mode = 0;  // for better low speed accuracy


int dead_time = 60;           // change to 60 for qfn

int test_comp_output = 0;
int level = 0;
int dir_reversed = 0;   // global direction reversed set in eeprom
int step = 1;
int startcount = 0;
int deviation = 0;
uint32_t extiline;

int forcedcomcount = 0;
int commandcount = 0;
char bad_commutation = 0;

int bi_direction = 0;
char comp_pwm = 1;                      // for complementary pwm , 0 for diode freewheeling
int brake = 1;                          // apply full motor brake on stop
int start_power = 200;
char prop_brake = 0;
int prop_brake_strength = 300;
int IC_buffer_size = 64;


char prop_brake_active = 0;
int adjusted_input;

int dshotcommand = 0;
uint8_t calcCRC;
uint8_t checkCRC;
int error = 0;
int stop_time =0;

int quietmode = 0;
int count = 0;
int tempbrake = 0;

char advancedivisor = 12;                    // increase divisor to decrease advance,
char advancedivisorup = 3;
char advancedivisordown = 3;
int advance_multiplier = 0;
int min_advance_multiplier = 4;                // * 2.25 to get advance degrees to 10 would be 22.5 degrees advance.
int max_advance_multiplier = 10;

int dither_count;
int dither_amount = 10;


int thiszctime = 0;
int lastzctime = 0;
int sensorless = 0;
int commutation_interval = 0;
int advance = 0;                       // set proportianal to commutation time. with advance divisor
int blanktime;
int pwm_settle = 50;
int demagtime = 5;
int waitTime = 0;
char filter_level = 1;
char compit = 0;
int filter_delay = 2;

int total;

int max_servo_deviation = 90;

int filter_level_up = 8;
int filter_level_down = 8;
int forcedcount = 0;
int control_loop_count;
int zctimeout = 0;
int zc_timeout_threshold = 2400;   // depends on speed of main loop

int signaltimeout = 0;
int signal_timeout_threshold = 20000;

int temp_step;
int ROC = 1;
int tocheck = 0;


int tim2_start_arr = 9000;
int startupcountdown = 0;

int duty_cycle = 100;
int adjusted_duty_cycle = 0;

int pwm = 1;
int floating = 2;
int lowside = 3;
int bemf_counts;
int k_erpm;

int degree_time;

int forward = 1;
int rising = 1;
int running = 0;
int started = 0;
char armed = 0;
int armedcount = 0;

int input_buffer_size = 64;
int smallestnumber = 20000;
uint32_t dma_buffer[64];
int propulse[4] = {0,0,0,0};
int dpulse[16] =  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

int input = 0;
int newinput =0;
int servorawinput = 0;

int voltageraw = 0;
int currentraw = 0;
int tempraw = 0;
uint32_t ADC1ConvertedValues[2] = {0,0};
int timestamp;

int compcount = 0;
int upcompcount = 0;
int falsecount = 0;
int falsethreshold = 2;
int zcfound = 1;
int threshold = 6;
int upthreshold = 6;
int forced_com_done = 0;

char dshot = 0;
char proshot = 0;
char multishot = 0;
char oneshot42 = 0;
char oneshot125 = 0;
char servoPwm = 0;


char brushed_direction_set = 0;

char inputSet = 0;

const int pwmSin[] = {128,130,132,134,136,139,141,143,
145,147,150,152,154,156,158,160,
163,165,167,169,171,173,175,177,
179,181,183,185,187,189,191,193,
195,197,199,201,202,204,206,208,
209,211,213,214,216,218,219,221,
222,224,225,227,228,229,231,232,
233,234,236,237,238,239,240,241,
242,243,244,245,246,247,247,248,
249,249,250,251,251,252,252,253,
253,253,254,254,254,255,255,255,
255,255,255,255,255,255,255,255,
254,254,254,253,253,253,252,252,
251,251,250,249,249,248,247,247,
246,245,244,243,242,241,240,239,
238,237,236,234,233,232,231,229,
228,227,225,224,222,221,219,218,
216,214,213,211,209,208,206,204,
202,201,199,197,195,193,191,189,
187,185,183,181,179,177,175,173,
171,169,167,165,163,160,158,156,
154,152,150,147,145,143,141,139,
136,134,132,130,128,125,123,121,
119,116,114,112,110,108,105,103,
101,99,97,95,92,90,88,86,
84,82,80,78,76,74,72,70,
68,66,64,62,60,58,56,54,
53,51,49,47,46,44,42,41,
39,37,36,34,33,31,30,28,
27,26,24,23,22,21,19,18,
17,16,15,14,13,12,11,10,
9,8,8,7,6,6,5,4,
4,3,3,2,2,2,1,1,
1,0,0,0,0,0,0,0,
0,0,0,0,1,1,1,2,
2,2,3,3,4,4,5,6,
6,7,8,8,9,10,11,12,
13,14,15,16,17,18,19,21,
22,23,24,26,27,28,30,31,
33,34,36,37,39,41,42,44,
46,47,49,51,53,54,56,58,
60,62,64,66,68,70,72,74,
76,78,80,82,84,86,88,90,
92,95,97,99,101,103,105,108,
110,112,114,116,119,121,123,125};

int phase_A_position;
int phase_B_position;
int phase_C_position;
int step_delay  = 4000;
char stepper_sine = 0;
int gate_drive_offset = 100;
int sine_mode = 0;
int timer_one_period= 1999;

int input_override;


uint32_t gcrtest[23] =  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
uint32_t gcr[23] =  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
int gcr_encode_table[16] = { 0b11001,
		0b11011,
		0b10010,
		0b10011,
		0b11101,
		0b10101,
		0b10110,
		0b10111,
		0b11010,
		0b01001,
		0b01010,
		0b01011,
		0b11110,
		0b01101,
		0b01110,
		0b01111
};

int tim1_arr = 0;
int timer_2_period = 55;
int timer_2_prescaler =2;
int high_bit_length = 65;
int timer_15_prescaler = 0;
char is_output = 0;
uint32_t gcrnumber;
int e_com_time = 65408;
int lastnumber = 0;
int shift_amount = 0;
char dshot_telemetry = 0;
char delay_before_output = 0;
int wait_after = 2500;
int wait_before = 700;       // delay after getting dshot signal before changing over to output
int dshot_full_number;
int timer_16_period = 6000;

char blanktime_int = 0;
int duration_in_microseconds;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC_Init(void);
static void MX_COMP1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM15_Init(void);
static void MX_IWDG_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM14_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM17_Init(void);
/* USER CODE BEGIN PFP */
static void MX_TIM15_Init_PWM(void);


long map(long x, long in_min, long in_max, long out_min, long out_max)
{
	if (x < in_min){
		x = in_min;
	}
	if (x > in_max){
		x = in_max;
	}
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;

}


void storeEEpromConfig(){

	 EE_WriteVariable(VirtAddVarTab[EEvehiclemode], vehicle_mode);
	 EE_WriteVariable(VirtAddVarTab[EEdirection], dir_reversed);
	 EE_WriteVariable(VirtAddVarTab[EEbidirection], bi_direction);
	// EE_WriteVariable(VirtAddVarTab[EEbrake_on_stop], EEbrake_on_stop);

	// playEEpromSavedTune();
}


void loadEEpromConfig(){
	 EE_ReadVariable(VirtAddVarTab[EEvehiclemode], &VarDataTab[EEvehiclemode]);
	 EE_ReadVariable(VirtAddVarTab[EEdirection], &VarDataTab[EEdirection]);
	 EE_ReadVariable(VirtAddVarTab[EEbidirection], &VarDataTab[EEbidirection]);
//	 EE_ReadVariable(VirtAddVarTab[EEbrake_on_stop], &VarDataTab[EEbrake_on_stop]);

	 if (VarDataTab[EEvehiclemode] == 0){             // nothing in the eeprom
     storeEEpromConfig();            // store default values
	 }else{
	 vehicle_mode = VarDataTab[EEvehiclemode];
	 dir_reversed = VarDataTab[EEdirection];
	 bi_direction = VarDataTab[EEbidirection];
//	 brake = VarDataTab[EEbrake_on_stop];
	 }
}


int getAbsDif(int number1, int number2){
	int result = number1 - number2;
	if (result < 0) {
	    result = -result;
	}
	return result;
}

//////////////////////////////////PHASE 1//////////////////////
#ifdef MP6531
void phaseAPWM() {  // phaseB qfnf051 , phase A qfp32
#endif
#ifdef FD6288
void phaseBPWM() {
#endif


		if(!comp_pwm  || prop_brake_active){            // for future
		LL_GPIO_SetPinMode(GPIOB, GPIO_PIN_0, LL_GPIO_MODE_OUTPUT);
		GPIOB->BRR = GPIO_PIN_0;
		}else{
		LL_GPIO_SetPinMode(GPIOB, GPIO_PIN_0, LL_GPIO_MODE_ALTERNATE); // low
		}
		LL_GPIO_SetPinMode(GPIOA, GPIO_PIN_9, LL_GPIO_MODE_ALTERNATE);  // high

	}

#ifdef MP6531
void phaseAFLOAT() {  // phaseB qfnf051 , phase A qfp32
#endif
#ifdef FD6288
void phaseBFLOAT() {
#endif



		LL_GPIO_SetPinMode(GPIOB, GPIO_PIN_0, LL_GPIO_MODE_OUTPUT);
		GPIOB->BRR = GPIO_PIN_0;
		LL_GPIO_SetPinMode(GPIOA, GPIO_PIN_9, LL_GPIO_MODE_OUTPUT);
		GPIOA->BRR = GPIO_PIN_9;
	}

#ifdef MP6531
void phaseALOW() {  // phaseB qfnf051 , phase A qfp32
#endif
#ifdef FD6288
void phaseBLOW() {
#endif

	        // low mosfet on
		LL_GPIO_SetPinMode(GPIOB, GPIO_PIN_0, LL_GPIO_MODE_OUTPUT);
		GPIOB->BSRR = GPIO_PIN_0;
		LL_GPIO_SetPinMode(GPIOA, GPIO_PIN_9, LL_GPIO_MODE_OUTPUT);
		GPIOA->BRR = GPIO_PIN_9;
	}



//////////////////////////////PHASE 2//////////////////////////////////////////////////

#ifdef MP6531
void phaseBPWM() {                                // phase c qfn , phase b qfp
#endif
#ifdef FD6288
void phaseCPWM() {
#endif

		if (!comp_pwm || prop_brake_active){
			LL_GPIO_SetPinMode(GPIOA, GPIO_PIN_7, LL_GPIO_MODE_OUTPUT);
			GPIOA->BRR = GPIO_PIN_7;
		}else{
		LL_GPIO_SetPinMode(GPIOA, GPIO_PIN_7, LL_GPIO_MODE_ALTERNATE);
		}
		LL_GPIO_SetPinMode(GPIOA, GPIO_PIN_8, LL_GPIO_MODE_ALTERNATE);

	}

#ifdef MP6531
void phaseBFLOAT() {                                // phase c qfn , phase b qfp
#endif
#ifdef FD6288
void phaseCFLOAT() {
#endif

	         // floating
		LL_GPIO_SetPinMode(GPIOA, GPIO_PIN_7, LL_GPIO_MODE_OUTPUT);
		GPIOA->BRR = GPIO_PIN_7;
		LL_GPIO_SetPinMode(GPIOA, GPIO_PIN_8, LL_GPIO_MODE_OUTPUT);
		GPIOA->BRR = GPIO_PIN_8;
	}


#ifdef MP6531
void phaseBLOW() {                                // phase c qfn , phase b qfp
#endif
#ifdef FD6288
void phaseCLOW() {
#endif

	            // lowside
		LL_GPIO_SetPinMode(GPIOA, GPIO_PIN_7, LL_GPIO_MODE_OUTPUT);
		GPIOA->BSRR = GPIO_PIN_7;
		LL_GPIO_SetPinMode(GPIOA, GPIO_PIN_8, LL_GPIO_MODE_OUTPUT);
		GPIOA->BRR = GPIO_PIN_8;
	}



///////////////////////////////////////////////PHASE 3 /////////////////////////////////////////////////


#ifdef MP6531
void phaseCPWM() {                    // phaseA qfn , phase C qfp
#endif
#ifdef FD6288
void phaseAPWM() {
#endif

		if (!comp_pwm || prop_brake_active){
			LL_GPIO_SetPinMode(GPIOB, GPIO_PIN_1, LL_GPIO_MODE_OUTPUT);
			GPIOB->BRR = GPIO_PIN_1;
			}else{
		LL_GPIO_SetPinMode(GPIOB, GPIO_PIN_1, LL_GPIO_MODE_ALTERNATE);
			}
		LL_GPIO_SetPinMode(GPIOA, GPIO_PIN_10, LL_GPIO_MODE_ALTERNATE);

	}


#ifdef MP6531
void phaseCFLOAT() {                    // phaseA qfn , phase C qfp
#endif
#ifdef FD6288
void phaseAFLOAT() {
#endif

		LL_GPIO_SetPinMode(GPIOB, GPIO_PIN_1, LL_GPIO_MODE_OUTPUT);
		GPIOB->BRR = GPIO_PIN_1;
		LL_GPIO_SetPinMode(GPIOA, GPIO_PIN_10, LL_GPIO_MODE_OUTPUT);
		GPIOA->BRR = GPIO_PIN_10;
	}


#ifdef MP6531
void phaseCLOW() {                    // phaseA qfn , phase C qfp
#endif
#ifdef FD6288
void phaseALOW() {
#endif


		LL_GPIO_SetPinMode(GPIOB, GPIO_PIN_1, LL_GPIO_MODE_OUTPUT);
		GPIOB->BSRR = GPIO_PIN_1;
		LL_GPIO_SetPinMode(GPIOA, GPIO_PIN_10, LL_GPIO_MODE_OUTPUT);
		GPIOA->BRR = GPIO_PIN_10;
	}



void  comStep (int newStep){
//TIM14->CNT = 0;
switch(newStep)
{

        case 1:			//A-B
        	phaseAPWM();
        	phaseBLOW();
        	phaseCFLOAT();
        	break;


        case 2:		// C-B
        	phaseAFLOAT();
        	phaseBLOW();
        	phaseCPWM();
        	break;



        case 3:	// C-A
        	phaseALOW();
        	phaseBFLOAT();
        	phaseCPWM();
        	break;


        case 4:// B-A
        	phaseALOW();
        	phaseBPWM();
        	phaseCFLOAT();
        	break;


        case 5:    // B-C
        	phaseAFLOAT();
        	phaseBPWM();
        	phaseCLOW();
        	break;


        case 6:      // A-C
        	phaseAPWM();
        	phaseBFLOAT();
        	phaseCLOW();
        	break;
	}

//stop_time = TIM14->CNT;

}


void allOff() {                   // coast
	phaseAFLOAT();
	phaseBFLOAT();
	phaseCFLOAT();
}

void fullBrake(){                     // full braking shorting all low sides
	phaseALOW();
	phaseBLOW();
	phaseCLOW();
}

void proBrake(){                    // duty cycle controls braking strength
//	prop_brake_active = 1;       // will turn off lower fets so only high side is active
	phaseAPWM();
	phaseBPWM();
	phaseCPWM();
}


void changeCompInput() {
//	TIM3->CNT = 0;
//	HAL_COMP_Stop_IT(&hcomp1);            // done in comparator interrupt routine

	if (step == 1 || step == 4) {   // c floating
	//	hcomp1.Init.InvertingInput = COMP_INVERTINGINPUT_IO1;
		COMP->CSR = 0b1100001;
	}

	if (step == 2 || step == 5) {     // a floating
#ifdef MP6531
		//COMP->CSR = 0x40;
	//	hcomp1.Init.InvertingInput = COMP_INVERTINGINPUT_DAC1;
		COMP->CSR = 0b1000001;                        /// if f051k6  step 2 , 5 is dac 1 ( swap comp input)
#endif
#ifdef FD6288
	//hcomp1.Init.InvertingInput = COMP_INVERTINGINPUT_DAC2;
		COMP->CSR = 0b1010001;
#endif
	}

	if (step == 3 || step == 6) {      // b floating
#ifdef MP6531
		//COMP->CSR = 0x50
	//	hcomp1.Init.InvertingInput = COMP_INVERTINGINPUT_DAC2;
		COMP->CSR = 0b1010001;
#endif
#ifdef FD6288
	//	hcomp1.Init.InvertingInput = COMP_INVERTINGINPUT_DAC1;
		COMP->CSR = 0b1000001;
#endif
	}
	if (rising){

		EXTI->RTSR = 0x0;
	EXTI->FTSR = 0x200000;

	//	hcomp1.Init.TriggerMode = COMP_TRIGGERMODE_IT_FALLING;   // polarity of comp output reversed
	}else{                          // falling bemf

	EXTI->FTSR = 0x0;
	EXTI->RTSR = 0x200000;
	//	hcomp1.Init.TriggerMode = COMP_TRIGGERMODE_IT_RISING;
	}

//	if (HAL_COMP_Init(&hcomp1) != HAL_OK) {
//		_Error_Handler(__FILE__, __LINE__);
//	}
////stop_time = TIM3->CNT;
}
void pollingChangeCompInput() {

	HAL_COMP_Stop(&hcomp1);

	if (step == 1 || step == 4) {   // c floating
		hcomp1.Init.InvertingInput = COMP_INVERTINGINPUT_IO1;
	}

	if (step == 2 || step == 5) {     // a floating
#ifdef MP6531
			hcomp1.Init.InvertingInput = COMP_INVERTINGINPUT_DAC1;  /// if f051k6  step 2 , 5 is dac 1 ( swap comp input)
#endif
#ifdef FD6288
	hcomp1.Init.InvertingInput = COMP_INVERTINGINPUT_DAC2;
#endif
	}

	if (step == 3 || step == 6) {      // b floating
#ifdef MP6531
			hcomp1.Init.InvertingInput = COMP_INVERTINGINPUT_DAC2;  /// if f051k6  step 2 , 5 is dac 1 ( swap comp input)
#endif
#ifdef FD6288
	hcomp1.Init.InvertingInput = COMP_INVERTINGINPUT_DAC1;
#endif
	}

	if (HAL_COMP_Init(&hcomp1) != HAL_OK) {
		Error_Handler();
	}

	if (HAL_COMP_Start(&hcomp1) != HAL_OK) {
		/* Initialization Error */
		Error_Handler();
	}

}

void commutate() {
//	TIM2->CNT = 0;
	if (forward == 1){
		step++;
		if (step > 6) {
			step = 1;
		}
		if (step == 1 || step == 3 || step == 5) {
			rising = 1;                                // is back emf rising or falling
		}
		if (step == 2 || step == 4 || step == 6) {
			rising = 0;
		}
	}
	if (forward == 0){
		step--;
		if (step < 1) {
			step = 6;
		}
		if (step == 1 || step == 3 || step == 5) {
			rising = 0;
		}
		if (step == 2 || step == 4 || step == 6) {
			rising = 1;
		}
	}

	//TIM2->CNT = 0;
	if (input > 47){

//	TIM1->CNT = 0;
comStep(step);
	}

	if((bemf_counts > 50 && duty_cycle > 180)){
		polling_mode = 0;

	}else{
		polling_mode = 1;
	}
	if (duty_cycle < 180 || commutation_interval > 5000){
		polling_mode = 1;
	}

	if(!polling_mode){
		changeCompInput();
	//	TIM6->CNT = 0;
	//	__HAL_TIM_ENABLE_IT(&htim6, TIM_IT_UPDATE);
	//	blanktime_int = 1;
	//	TIM6->ARR = blanktime;           // exit and set timer for blanktime interrupt
		EXTI->IMR |= (1 << 21);
	}else{
		pollingChangeCompInput();
	}
	zcfound = 0;
    //	TIM6->CNT = 0;
    	commutation_interval = ((2*commutation_interval) + thiszctime) /3;
    		degree_time = commutation_interval >> 5;                          // about 1.85 degrees per unit divided by 32
    			advance = degree_time * advance_multiplier;                     //  * 16 would be about 30 degrees
    //	advance = commutation_interval>>2 ;
    	waitTime = (commutation_interval >> 1) - advance;
    	if (waitTime < 0){
    			waitTime = 0;
    		}
    			blanktime = commutation_interval >>4 ;                               // divided by 8
    			bemf_counts++;
}




void startMotor() {

 startcount++;

    char decaystate = comp_pwm;
    sensorless = 0;
	if (running == 0){
		EXTI->IMR &= ~(1 << 21);
		EXTI->PR &=~(1 << 21);
		comp_pwm = 1;
	commutate();
	commutation_interval = 10000;
	TIM3->CNT = 0;
	running = 1;
	if(!polling_mode){
	EXTI->IMR |= (1 << 21);
	}
	}else{
		if (HAL_COMP_Start(&hcomp1) != HAL_OK) {
			/* Initialization Error */
			Error_Handler();
		}
	}
	comp_pwm = decaystate;    // return to normal
	sensorless = 1;
	startupcountdown =0;
	bemf_counts = 0;

}

void forcedCommutation(){
	HAL_COMP_Stop_IT(&hcomp1);
    TIM3->CNT = commutation_interval / 2;
    commutate();
    while (TIM3->CNT - commutation_interval / 2  <  blanktime){}
	if (HAL_COMP_Start_IT(&hcomp1) != HAL_OK) {
		/* Initialization Error */
		Error_Handler();
	}

}



void HAL_COMP_TriggerCallback(COMP_HandleTypeDef *hcomp) {
// TIM17->CNT = 0;
//	EXTI->IMR &= (0 << 21);
//		EXTI->PR &=(0 << 21);
	if ((TIM3->CNT < commutation_interval >> 1)&& bemf_counts > 3 ){
//	EXTI->IMR |= (1 << 21);
	return;
}

if(commutation_interval > 500){
while (TIM3->CNT - thiszctime < filter_delay){
}
}

compit +=1;
if (compit > 100){
	EXTI->IMR &= ~(1 << 21);
	EXTI->PR &=~(1 << 21);
//		input = 0;
	error = 1;
	return;
}

for (int i = 0; i < filter_level; i++){
if (rising == (COMP1->CSR &  1<<14) >> 14){     // if the comparator output is not what is expected
//		GPIOA->BRR = GPIO_PIN_15;

//	EXTI->IMR |= (1 << 21);
	return;
//	EXTI->IMR |= (1 << 21);
}
}
		EXTI->IMR &= ~(1 << 21);               // turn off interrupts and pending requests.
		EXTI->PR &=~(1 << 21);
		thiszctime = TIM3->CNT;
		TIM3->CNT = 0;
		zctimeout = 0;
		TIM6->CNT = 0;
		TIM6->ARR = waitTime;
		__HAL_TIM_ENABLE_IT(&htim6, TIM_IT_UPDATE);

}


void checkForZeroCross(){
	if(!zcfound){

			//			  if (step == 2 || step == 4 || step == 6){
			if (rising == 0){
				if (HAL_COMP_GetOutputLevel(&hcomp1) == COMP_OUTPUTLEVEL_LOW){
					falsecount++;
					if (falsecount > falsethreshold){
					compcount = 0;
					zcfound = 0;
					falsecount = 0;
					}

				}

				if (HAL_COMP_GetOutputLevel(&hcomp1) == COMP_OUTPUTLEVEL_HIGH){

					compcount++;
				}
					if (compcount > threshold){
							zcfound = 1;
							zctimeout = 0;
							compcount = 0;
							bemf_counts++;
							falsecount = 0;
							thiszctime = TIM3->CNT;
							TIM3->CNT = 0;
							forced_com_done = 0;

						    commutation_interval = ((2*commutation_interval) + thiszctime) / 3;
							degree_time = commutation_interval >> 5;                          // about 1.85 degrees per unit
							advance = degree_time * advance_multiplier;                     //  * 16 would be about 30 degrees
							waitTime = (commutation_interval >> 1) - advance;

							if (sensorless){
								while (TIM3->CNT  < waitTime){
								}
								commutate();

							}
							zcfound = 0;

						}
			}

			if (rising == 1){
				if (HAL_COMP_GetOutputLevel(&hcomp1) == COMP_OUTPUTLEVEL_HIGH){
					falsecount++;
					if (falsecount > falsethreshold){
					upcompcount = 0;
					zcfound = 0;
					falsecount = 0;
					}

				}

				if (HAL_COMP_GetOutputLevel(&hcomp1) == COMP_OUTPUTLEVEL_LOW){
					//		GPIOA->BSRR = GPIO_PIN_15;
					upcompcount++;
				}
					if (upcompcount > upthreshold){
							zcfound = 1;
							zctimeout = 0;
							upcompcount = 0;
							falsecount = 0;
							bemf_counts++;
				//			GPIOA->BSRR = GPIO_PIN_15;
								thiszctime = TIM3->CNT;
								TIM3->CNT = 0;
								forced_com_done = 0;

							    commutation_interval = ((2*commutation_interval) + thiszctime) / 3;
								degree_time = commutation_interval >> 5;                          // about 1.85 degrees per unit
								advance = degree_time * advance_multiplier;
								waitTime = (commutation_interval >> 1) - advance;
								if (sensorless){
									while (TIM3->CNT < waitTime){
									}
									commutate();
								}
								zcfound = 0;
		//						lastzctime = thiszctime;
						}
			}
	}
}

void playStartupTune(){
	TIM1->PSC = 75;
	TIM1->CCR1 = 5;
	TIM1->CCR2 = 5;
	TIM1->CCR3 = 5;
	comStep(3);
	HAL_Delay(100);
	TIM1->PSC = 50;
	HAL_Delay(100);
	TIM1->PSC = 25;
	HAL_Delay(100);
	allOff();
	TIM1->PSC = 0;
}

void playInputTune(){
	TIM1->PSC = 100;
	TIM1->CCR1 = 5;
	TIM1->CCR2 = 5;
	TIM1->CCR3 = 5;
	comStep(6);
	HAL_Delay(100);
	TIM1->PSC = 50;
	HAL_Delay(100);
	allOff();
	TIM1->PSC = 0;
}

void getADCs(){
	//voltageraw = ADC1ConvertedValues[0];
	currentraw = ADC1ConvertedValues[0];
	tempraw = ADC1ConvertedValues[1];

}




void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	getADCs();
}


void detectInput(){
	smallestnumber = 20000;
	dshot = 0;
	proshot = 0;
	multishot = 0;
	oneshot42 = 0;
	oneshot125 = 0;
	servoPwm = 0;
//	int lastnumber = dma_buffer[0];
	for ( int j = 1 ; j < input_buffer_size; j++){

		if(dma_buffer[j]  < smallestnumber){ // blank space
			smallestnumber = dma_buffer[j];
		}

	}

	if ((smallestnumber > 3)&&(smallestnumber < 20)){
		dshot = 1;
		timer_15_prescaler = 24;
		TIM15->PSC = timer_15_prescaler;
		IC_buffer_size = 32;
		TIM16->ARR = 8000;
	}
	if ((smallestnumber > 20)&&(smallestnumber < 40)){
			dshot = 1;
			timer_15_prescaler = 36;
			TIM15->PSC = timer_15_prescaler;
			TIM16->PSC = 1;
			TIM16->ARR = 8000;
			IC_buffer_size = 32;
		}

	if ((smallestnumber > 40)&&(smallestnumber < 55)){
			dshot = 1;
			timer_15_prescaler = 64;
			TIM2->PSC = 1;
			TIM15->PSC = timer_15_prescaler;
			TIM16->PSC = 1;
			TIM16->ARR = 8000;
			IC_buffer_size = 32;
		}

//	if ((smallestnumber > 40 )&&(smallestnumber < 80)){
//		proshot = 1;
//		TIM15->PSC = 5;
//		IC_buffer_size = 8;
//		TIM16->ARR = 8000;
//	}
//	if ((smallestnumber > 100 )&&(smallestnumber < 400)){
//		multishot = 1;
//	}
//	if ((smallestnumber > 2000 )&&(smallestnumber < 3000)){
//		oneshot42 = 1;
//	}
//	if ((smallestnumber > 3000 )&&(smallestnumber < 7000)){
//		oneshot125 = 1;
//	}
	if (smallestnumber > 100){
		servoPwm = 1;
		TIM15->PSC = 47;
//		TIM16->PSC = 47;
//		TIM16->ARR = 10000;
		HAL_TIM_Base_Stop(&htim16);
		IC_buffer_size = 6;

	}

	if (smallestnumber == 0){
		inputSet = 0;
	}else{

		inputSet = 1;

		HAL_Delay(50);
		//	playInputTune();
	}
	HAL_TIM_IC_Start_DMA(&htim15, TIM_CHANNEL_1, dma_buffer , IC_buffer_size);
}

void computeProshotDMA(){

total = dma_buffer[1]+ dma_buffer[2] + dma_buffer[3] + dma_buffer[4]+ dma_buffer[5] + dma_buffer[6] + dma_buffer[7];

   if (( total < 118 && total > 98)&& (dma_buffer[0]> 100)){
   for (int i = 1; i < 8; i +=2){
    propulse[(i-1) / 2] = (dma_buffer[i] - 7);
					}
   }else{

	   return;
   }
	calcCRC = ((propulse[0]^propulse[1]^propulse[2])<<3
							|(propulse[0]^propulse[1]^propulse[2])<<2
							|(propulse[0]^propulse[1]^propulse[2])<<1
							|(propulse[0]^propulse[1]^propulse[2]));

	checkCRC = (propulse[3]<<3 | propulse[3]<<2 | propulse[3]<<1 | propulse[3]);
    if ( checkCRC == calcCRC){
    	tocheck = ((propulse[0]<<7 | propulse[1]<<3 | propulse[2]>>1));
    }else{
 //   	error++;
    }
    if (tocheck > 47 && tocheck < 2048){
    	newinput = tocheck ;
    				commandcount = 0;



    			}else if (tocheck > 1 && tocheck < 48 && input == 0){

    					dshotcommand = tocheck ;
    			}else{
    				commandcount++;
    				if (commandcount > 1){
    				newinput = tocheck ;
    			    commandcount = 0;
    			}
    			}

}

void computeDshotDMA(){
	if (dma_buffer[0] > 10){
		for (int i = 1; i < 32; i+=2){
		dpulse[(i-1)>>1] = dma_buffer[i];
	}
	}
	if (dma_buffer[1] > 10){
		for (int i = 2; i <= 32; i+=2){
				dpulse[(i-1)>>1] = dma_buffer[i];
			}
	}

	calcCRC = ((dpulse[0]^dpulse[4]^dpulse[8])<<3
				          |(dpulse[1]^dpulse[5]^dpulse[9])<<2
						|(dpulse[2]^dpulse[6]^dpulse[10])<<1
						|(dpulse[3]^dpulse[7]^dpulse[11])
					);
		 checkCRC = (dpulse[12]<<3 | dpulse[13]<<2 | dpulse[14]<<1 | dpulse[15]);
if(!armed){
	if (dshot_telemetry == 0){
		 if(calcCRC == ~checkCRC+16){
			 dshot_telemetry = 1;
			 is_output = 1;        // so that setupinput() is called on next timer16 timeout
		 }
	}
}
if(dshot_telemetry){
	checkCRC= ~checkCRC+16;
}
				if(calcCRC == checkCRC){
					tocheck = (
						dpulse[0]<<10 | dpulse[1]<<9 | dpulse[2]<<8 | dpulse[3]<<7
						| dpulse[4]<<6 | dpulse[5]<<5 | dpulse[6]<<4 | dpulse[7]<<3
						| dpulse[8]<<2 | dpulse[9]<<1 | dpulse[10]);
	//				success++;
					}else{
						error++;
					}


//	 calcCRC = ((dma_buffer[1]^dma_buffer[9]^dma_buffer[17])<<3
//			          |(dma_buffer[3]^dma_buffer[11]^dma_buffer[19])<<2
//					|(dma_buffer[5]^dma_buffer[13]^dma_buffer[21])<<1
//					|(dma_buffer[7]^dma_buffer[15]^dma_buffer[23])
//				);
////	 calcCRC = ((dpulse[0]^dpulse[4]^dpulse[8])<<3
////			          |(dpulse[1]^dpulse[5]^dpulse[9])<<2
////					|(dpulse[2]^dpulse[6]^dpulse[10])<<1
////					|(dpulse[3]^dpulse[7]^dpulse[11])
////				);
////	 checkCRC = (dpulse[12]<<3 | dpulse[13]<<2 | dpulse[14]<<1 | dpulse[15]);
//	 checkCRC = (dma_buffer[25]<<3 | dma_buffer[27]<<2 | dma_buffer[29]<<1 | dma_buffer[31]);
//
//	 if (dshot_telemetry){
//	 	 checkCRC= ~checkCRC+16;
//	 }
//
////
//			if(calcCRC == checkCRC){
//				tocheck = (
//						dma_buffer[1]<<10 | dma_buffer[3]<<9 | dma_buffer[5]<<8 | dma_buffer[7]<<7
//					| dma_buffer[9]<<6 | dma_buffer[11]<<5 | dma_buffer[13]<<4 | dma_buffer[15]<<3
//					| dma_buffer[17]<<2 | dma_buffer[19]<<1 | dma_buffer[21]);
////				success++;
//				}else{
//					error++;
//				}

			if (tocheck > 47 && tocheck < 2048){
				newinput = tocheck;
				commandcount = 0;
			}else if (tocheck > 1 && tocheck < 48 && input == 0){

					dshotcommand = tocheck ;


			}else{
				commandcount++;
				if (commandcount > 1){
				newinput = tocheck ;
			    commandcount = 0;
			}
			}

}


void computeMSInput(){

	int lastnumber = dma_buffer[0];
	for ( int j = 1 ; j < 2; j++){

		if(((dma_buffer[j] - lastnumber) < 1500) && ((dma_buffer[j] - lastnumber) > 0)){ // blank space

			newinput = map((dma_buffer[j] - lastnumber),243,1200, 0, 2000);
			break;
		}
		lastnumber = dma_buffer[j];
	}
}

void computeOS125Input(){

	int lastnumber = dma_buffer[0];
	for ( int j = 1 ; j < 2; j++){

		if(((dma_buffer[j] - lastnumber) < 12300) && ((dma_buffer[j] - lastnumber) > 0)){ // blank space

			newinput = map((dma_buffer[j] - lastnumber),6500,12000, 0, 2000);
			break;
		}
		lastnumber = dma_buffer[j];
	}
}

void computeOS42Input(){

	int lastnumber = dma_buffer[0];
	for ( int j = 1 ; j < 2; j++){

		if(((dma_buffer[j] - lastnumber) < 4500) && ((dma_buffer[j] - lastnumber) > 0)){ // blank space

			newinput = map((dma_buffer[j] - lastnumber),2020, 4032, 0, 2000);
			break;
		}
		lastnumber = dma_buffer[j];
	}
}




void computeServoInput(){

	if ( dma_buffer[1] < 2000 && dma_buffer[1] > 1000){
		if(dma_buffer[2]< 1000 || dma_buffer[2] > 2500){

		servorawinput = map(dma_buffer[1], 1100,2000,0,2000);
		}
	}else if( dma_buffer[2] < 2000 && dma_buffer[2] > 1000) {
		if(dma_buffer[1]< 1000 || dma_buffer[1] > 2500){
		servorawinput = map(dma_buffer[2], 1100,2000,0,2000);
		}
	}


	if (servorawinput - newinput > max_servo_deviation){
		newinput += max_servo_deviation;
	}else if(newinput - servorawinput > max_servo_deviation){
		newinput -= max_servo_deviation;
	}else{
		newinput = servorawinput;
	}


}

void make_dshot_package(){
//	TIM8->CNT = 0;

	  e_com_time = commutation_interval * 6 / 2 ;
      if(!running){
    	  e_com_time = 65535;
      }
//	calculate shift amount for data in format eee mmm mmm mmm, first 1 found in first seven bits of data determines shift amount
// this allows for a range of up to 65408 microseconds which would be shifted 0b111 (eee) or 7 times.
for (int i = 15; i >= 9 ; i--){
	if(e_com_time >> i == 1){
		shift_amount = i+1 - 9;
		break;
	}else{
		shift_amount = 0;
	}
}
// shift the commutation time to allow for expanded range and put shift amount in first three bits
	dshot_full_number = ((shift_amount << 9) | (e_com_time >> shift_amount));
//calculate checksum
	uint16_t  csum = 0;
	uint16_t csum_data = dshot_full_number;
		  for (int i = 0; i < 3; i++) {
		      csum ^=  csum_data;   // xor data by nibbles
		      csum_data >>= 4;
		  }
		  csum = ~csum;       // invert it
		  csum &= 0xf;

		  dshot_full_number = (dshot_full_number << 4)  | csum; // put checksum at the end of 12 bit dshot number

// GCR RLL encode 16 to 20 bit

		  gcrnumber = gcr_encode_table[(dshot_full_number >> 12)] << 15  // first set of four digits
		  | gcr_encode_table[(((1 << 4) - 1) & (dshot_full_number >> 8))] << 10  // 2nd set of 4 digits
		  | gcr_encode_table[(((1 << 4) - 1) & (dshot_full_number >> 4))] << 5  //3rd set of four digits
		  | gcr_encode_table[(((1 << 4) - 1) & (dshot_full_number >> 0))];  //last four digits
//GCR RLL encode 20 to 21bit output

//	  gcrnumber = 0b1000000000000000001;
//		  gcr[0] = 0;
//		  lastnumber = 0;
//		  for( int i= 21; i >= 0; i--){
//						if((gcrnumber & ( 1 << (i-1)))){   // if the bit is a 1
//							lastnumber = !lastnumber; // invert the bit
//							gcr[22-i] = (!lastnumber * high_bit_length); // since the output is inverted electrical a 0 becomes period + 1
//						}else{ // if the bit is zero
//						//	gcr[20-i] = (max_duty)- lastnumber*(max_duty); // output same as last one
//							gcr[22-i] = (!lastnumber * high_bit_length);
//						}
//		  }
//		  gcr[1] = high_bit_length;        //  since pwm is inverted
//          gcr[22] = 0;

		  gcr[1] = 64;
		  for( int i= 19; i >= 0; i--){              // each digit in gcrnumber
			  gcr[20-i+1] = ((((gcrnumber &  1 << i )) >> i) ^ (gcr[20-i]>>6)) << 6;        // exclusive ored with number before it multiplied by 64 to match output timer.
		  }
          gcr[0] = 0;


}

void senddshotburst(){

 //count++;
	if (HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_3, gcr, 23) != HAL_OK)
	     {
	       /* Starting Error */
	       Error_Handler();
	     }
	DMA1_Channel1->CCR = 0xa93;         // turn off half transfer interrupt.
	//GPIOA->BSRR = GPIO_PIN_15;
	count++;
}



void advanceincrement(){        // called by timer interrupt when in forced sinusoidal mode.
	  if (sine_mode){
if (!forward){
	phase_A_position ++;
    if (phase_A_position > 359){
	   phase_A_position = 0 ;
    }

	    phase_B_position ++;
	     if (phase_B_position > 359){
		phase_B_position = 0 ;
	}
	    phase_C_position ++;
	     if (phase_C_position > 359){
		phase_C_position = 0 ;
	}
}else{
	phase_A_position --;
	    if (phase_A_position < 0){
		   phase_A_position = 359 ;
	    }

		    phase_B_position --;
		     if (phase_B_position < 0){
			phase_B_position = 359;
		}
		    phase_C_position --;
		     if (phase_C_position < 0){
			phase_C_position = 359 ;
		}
}
//	if (phase_A_position >= 29 && phase_A_position <= 88){                // this is wrong changr this!!!  commutation angle should be 90 degrees from holding angle
//		step = 6;
//	}
//	if (phase_A_position >= 99 && phase_A_position <= 158){
//			step = 1;
//		}
//	if (phase_A_position >= 159 && phase_A_position <= 218){
//			step = 2;
//		}
//	if (phase_A_position >= 219 && phase_A_position <= 278){
//				step = 3;
//			}
//	 if (phase_A_position >= 279 && phase_A_position <= 328){
//			step = 4;
//		}
//	if (phase_A_position >= 329 || phase_A_position <= 28){
//			step = 5;
//		}
		    TIM1->CCR1 = (pwmSin[phase_A_position])+gate_drive_offset;												// set duty cycle to 50 out of 768 to start.
		    TIM1->CCR2 = (pwmSin[phase_B_position])+gate_drive_offset;
		    TIM1->CCR3 = (pwmSin[phase_C_position])+gate_drive_offset;
	  }

}
void changeToOutput(){
	TIM16->ARR = 65535;
	__HAL_TIM_DISABLE_DMA(&htim15, TIM_DMA_CC1);
	__HAL_TIM_DISABLE_DMA(&htim15, TIM_DMA_UPDATE);
	 GPIOA->AFR[0] = 0x200;
	 is_output = 1;
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if (htim->Instance==TIM6)  // commutation timer
				{
		__HAL_TIM_DISABLE_IT(&htim6, TIM_IT_UPDATE);  // disable update interrupt
//		if(blanktime_int == 1){
//		EXTI->IMR |= (1 << 21);
//		blanktime_int = 0;
//		return;
//		}
		TIM17->CNT = 0;
		 commutate();
        duration_in_microseconds = TIM17->CNT;
		 return;
			}


			if (htim->Instance==TIM16)  // input timeout reset timer
			{
			if(!is_output){
			if(inputSet == 1){
		     HAL_TIM_IC_Stop_DMA(&htim15,TIM_CHANNEL_1);
			 TIM15->CNT = 0;
				 HAL_TIM_IC_Start_DMA(&htim15, TIM_CHANNEL_1, dma_buffer , IC_buffer_size);
			}
		}else{
			if(delay_before_output == 1){
				changeToOutput();
				make_dshot_package();
					senddshotburst();
				TIM16->ARR = timer_16_period;
				delay_before_output = 0;
					}
		}
			}


			if (htim->Instance==TIM14) /// sinusoidal interval timer
						{
advanceincrement();
						}

}



void setupInput(){
	TIM16->CNT = 0;
	TIM16->ARR = timer_16_period;
	GPIOA->AFR[0] = 0x00;                  // set alternate function to zero
	is_output = 0;
}


void transferComplete(){
//	TIM3->CNT = 0;
//	compit = 0;
	signaltimeout = 0;
	count++;
//	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_3);
	if (is_output){        // if transfer complete happened on output the tansfer to dma is done and we can switch back lines and start input capture
		setupInput();
		HAL_TIM_IC_Stop_DMA(&htim15,TIM_CHANNEL_1);
		HAL_TIM_IC_Start_DMA(&htim15, TIM_CHANNEL_1, dma_buffer , 33);
		return;
	}

	if (inputSet == 1){
		if((dshot_telemetry) && (!is_output) ){
			if ((dma_buffer[0] > 10)||(dma_buffer[1]> 10)){
			TIM16->CNT = 0;
			TIM16->ARR = 65535;
			is_output = 1;
			TIM16->ARR = wait_before;
			computeDshotDMA();
			dma_buffer[0] =0;
			dma_buffer[1]= 0;
            delay_before_output = 1;
			}
		//	return;
		}



if(!dshot_telemetry){

		if (dshot == 1){
			computeDshotDMA();
			return;
		}
		if (proshot == 1){
			computeProshotDMA();
			return;
		}

		if  (servoPwm == 1){
			computeServoInput();

				HAL_TIM_IC_Start_DMA(&htim15, TIM_CHANNEL_1, dma_buffer , 3);

		}
//		if  (multishot){
//			computeMSInput();
//			HAL_TIM_IC_Start_DMA(&htim15, TIM_CHANNEL_1, dma_buffer , 3);
//
//		}
//		if  (oneshot125){
//			computeOS125Input();
//			HAL_TIM_IC_Start_DMA(&htim15, TIM_CHANNEL_1, dma_buffer , 3);
//
//		}
//		if  (oneshot42){
//			computeOS42Input();
//			HAL_TIM_IC_Start_DMA(&htim15, TIM_CHANNEL_1, dma_buffer , 3);
//          }
		}

	}

}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  HAL_FLASH_Unlock();
  EE_Init();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC_Init();
  MX_COMP1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM15_Init();
//  MX_IWDG_Init();
  MX_TIM16_Init();
  MX_TIM14_Init();
  MX_TIM6_Init();
  MX_TIM17_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

	HAL_TIM_Base_Start(&htim2);
	HAL_TIM_Base_Start(&htim6);           // commutation timer
	//HAL_TIM_Base_Start_IT(&htim14);
	HAL_TIM_Base_Start(&htim17);
	HAL_TIM_Base_Start(&htim3);
	//  HAL_Delay(500);
	for ( int i = 0; i < vehicle_mode; i++){
	playStartupTune();
	HAL_Delay(100);
	}
	MX_IWDG_Init();
	if (vehicle_mode == 1){                    // quad single direction
		loadEEpromConfig();
	}
	if (vehicle_mode == 2){                   // crawler or thruster
		 bi_direction = 1;
		 comp_pwm = 1;                      // for complementary pwm , 0 for diode freewheeling
		 brake = 1;                          // apply full motor brake on stop
	//	 start_power = 150;

	}
	if (vehicle_mode == 3){                 // rc car 50 percent brake on reverse.
		 bi_direction = 1;
		 comp_pwm = 0;                      // for complementary pwm , 0 for diode freewheeling
		 brake = 0;                          // apply full motor brake on stop
	//	 start_power = 150;
		 prop_brake = 1;
		 prop_brake_strength = 900;
	}
	if (vehicle_mode == 4){                 // rc car 50 percent brake on reverse.
			 bi_direction = 1;
			 comp_pwm = 0;                      // for complementary pwm , 0 for diode freewheeling
			 brake = 0;                          // apply full motor brake on stop
	//		 start_power = 150;
			 prop_brake = 1;
			 prop_brake_strength = 800;
		}

	if (vehicle_mode == 5){                    // quad single direction no eeprom

	}

//	if (HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_4) != HAL_OK)
//	{
//		Error_Handler();
//	}
//	HAL_TIM_IC_Start_DMA(&htim15, TIM_CHANNEL_1, dma_buffer , 64);
//	if (HAL_ADC_Start_DMA(&hadc, (uint32_t*)ADC1ConvertedValues, 2) != HAL_OK){            // for adc turn this on!!!!
//		Error_Handler();
//
//	}
		if(HAL_COMP_Start_IT(&hcomp1) != HAL_OK)
		{
			/* Initialization Error */
			Error_Handler();
		}
	//
//	if(HAL_IWDG_Init(&hiwdg) != HAL_OK)
//	{
//		/* Initialization Error */
//		Error_Handler();
//	}

	if(bi_direction){
		newinput = 1001;
	//	start_power = 175;
	}

	if (dir_reversed == 1){
		forward = 0;
	}else{
		forward = 1;
	}

	//proBrake();
	TIM1->CCR1 = 1;
	TIM1->CCR2 = 1;
	TIM1->CCR3 = 1;

	TIM1->CCR4 = 800;

	if (!brushed_mode && bi_polar){          // sanity check, turn off bipolar pwm if brushed mode is not selected
		bi_polar = 0;
	}
	if(bi_polar){
		comp_pwm = 1;
	}
	 phase_A_position = 0;
	  phase_B_position = 119;
	  phase_C_position = 239;

	  if(vehicle_mode != 2){
		  sine_mode_range = 0;
	  }
	  if (sine_mode_range > 52 || sine_mode_range < 0){
		  sine_mode_range = 0;
	   }


	  DMA1_Channel2->CCR = 0xa90; // for output compare
	   DMA1_Channel1->CCR = 0xa91;     // for output compare
	   DMA1_Channel5->CCR = 0x981;    // for input capture
	   __HAL_TIM_ENABLE_DMA(&htim2, TIM_DMA_UPDATE);
	   __HAL_TIM_ENABLE_DMA(&htim2, TIM_DMA_CC3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	  HAL_Delay(1);
//	  changeToOutput();
//		make_dshot_package();
//	//    first_transfer = 1;
//		senddshotburst();
////		HAL_Delay(1);
////		//voltage = voltageraw;




			if (count > 100000) {
				count = 0;
			}
			compit = 0;
			if (HAL_IWDG_Refresh(&hiwdg) != HAL_OK)              // watchdog refresh
					{
				/* Refresh Error */
				Error_Handler();
			}
			control_loop_count++;
			if (control_loop_count > 2) {


				control_loop_count = 0;

				//	  		1-5: beep (1= low freq. 5 = high freq.)
				//	  		6: ESC info request (FW Version and SN sent over the tlm wire)
				//	  		7: rotate in one direction
				//	  		8: rotate in the other direction
				//	  		9: 3d mode off
				//	  		10: 3d mode on
				//	  		11: ESC settings request (saved settings over the TLM wire) (planed but not there yet)
				//	  		12: save Settings

				if (dshotcommand > 0) {
					if (dshotcommand == 2) {
						playInputTune();
					}
					if (dshotcommand == 21) {
						forward = dir_reversed;

					}
					if (dshotcommand == 20) {     // forward = 1 if dir_reversed = 0
						forward = 1 - dir_reversed;
					}
					if (dshotcommand == 7) {
						dir_reversed = 0;

					}
					if (dshotcommand == 8) {
						dir_reversed = 1;

					}
					if (dshotcommand == 9) {
						bi_direction = 0;
						armed = 0;

					}
					if (dshotcommand == 10) {
						bi_direction = 1;
						armed = 0;
					}
					if (dshotcommand == 12) {
						storeEEpromConfig();
						while (1) {   // resets esc as iwdg times out

						}
					}
					dshotcommand = 0;
				}

				if (bi_direction == 1 && (proshot == 0 && dshot == 0)) {
					//char oldbrake = brake;

					if (newinput > 1100) {
						if (forward == dir_reversed) {
							adjusted_input = 0;
							prop_brake_active = 1;
							brushed_direction_set = 0;
							forward = 1 - dir_reversed;
							//	HAL_Delay(1);
						}

						if (prop_brake_active == 0) {

							adjusted_input = (newinput - 1099) * 3;
							//	tempbrake = 0;
							//	}
						}
					}
					if (newinput < 760) {
						if (forward == (1 - dir_reversed)) {
							prop_brake_active = 1;
							adjusted_input = 0;
							forward = dir_reversed;
							brushed_direction_set = 0;

						}
						if (prop_brake_active == 0) {
							adjusted_input = ((760 - newinput) * 3);

						}
						//	tempbrake = 0;
					}
				//	if (zctimeout >= zc_timeout_threshold) {
						//	adjusted_input = 0;
						if (vehicle_mode != 3) { // car mode requires throttle return to center before direction change
							prop_brake_active = 0;
				//		}

				//		startupcountdown = 0;
				//		bemf_counts = 0;
					}

					if (newinput >= 760 && newinput < 1100) {
						adjusted_input = 0;
						prop_brake_active = 0;
					}

				} else if ((proshot || dshot) && bi_direction) {
					if (newinput > 1097) {

						if (forward == dir_reversed) {
							forward = 1 - dir_reversed;
							bemf_counts = 0;
							brushed_direction_set = 0;
						}
						adjusted_input = (newinput - 1100) * 2 + 100;
					}
					if (newinput <= 1047 && newinput > 0) {
					//	startcount++;

						if (forward == (1 - dir_reversed)) {
							bemf_counts = 0;
							forward = dir_reversed;
							brushed_direction_set = 0;

						}
						adjusted_input = (newinput - 90) * 2;
					}
					if ((newinput > 1047 && newinput < 1098) || newinput <= 120) {
						adjusted_input = 0;
					}

				} else {
					adjusted_input = newinput;
				}

				//	if ((adjusted_input > 100)&&(startupcountdown>250)){
				if (adjusted_input > 2000) {
					adjusted_input = 2000;
				}
//adjusted_input = adjusted_input  / 2;
	//			if ((adjusted_input - input > 5 && bemf_counts < 50)) {
	//				input = input + 1;
	//			//	advance_multiplier = 12;
	//			} else if ((input - adjusted_input > 5) && ( bemf_counts < 50)) {
	//				input = input - 1;
	//			} else {
				if (input_override > 1){
					input = input_override;
				}else{

//				if (adjusted_input < 300 && input < adjusted_input){
//					input++;
//				//	HAL_Delay(5);
//				}else if (adjusted_input < 300 && input > adjusted_input){
//									input--;
//					//				HAL_Delay(5);
//				}else{
					input = adjusted_input;
//				}
				}
				//	advance_multiplier = map((commutation_interval), 150, 300, 8, 8);
		//		}

			}



	//		if ( commutation_interval < 120 ){        // desync
	//		input = 0;
	//		}
	if(brushed_mode){
	dither_count++;
	if(dither_count > 2){
		dither_count = 0;
	}
	if(input > 1990){               // keep slightly below 100 percent duty cycle for some drivers
		input = 1990;
	}
	bemf_counts = 200;

	if(!brushed_direction_set && !prop_brake_active){

	if (!bi_polar){
		if(forward){
			comStep(6);
		}else{
			comStep(3);
		}
		brushed_direction_set = 1;
	}else{      // bipolar pwm  caution!!
		phaseAPWM();
		phaseCPWM();
			brushed_direction_set = 1;
	}
	}
	}
	//		if (bemf_counts > 100){                      // NEVER TURN THIS ON!!! for musement only
	//bemf_counts = 0;
	//		input=0;
	//		forward = 1-forward;
	//			dir_reversed = 1-dir_reversed;
	//
	//		}


	//		if (duty_cycle < 500 && commutation_interval < 200){         // stuck commutation at pwm speed
	//			zctimeout = zc_timeout_threshold;
	//		}

			advance_multiplier = map((commutation_interval), 150, 3000, max_advance_multiplier, min_advance_multiplier);
			if (inputSet == 0) {
				HAL_Delay(10);
				detectInput();

			}
			if (!armed) {
					if ((inputSet == 1) && (input == 0)) {
						armedcount++;
						HAL_Delay(1);
						if (armedcount > 2000) {
							armed = 1;
							playInputTune();
						}
					}
					if (input > 0) {
						armedcount = 0;
					}
				}


			if ((input >= (100 - (52-sine_mode_range)-(10*running))) && (armed == 1)) {
if (sine_mode == 1){
			    sine_mode = 0;
			    TIM1->ARR = timer_one_period;
			    TIM1->CNT = 0;
		//	    allOff();
			    running = 1;
			    commutate();
}
				prop_brake_active = 0;
				started = 1;
				start_power = map((input), 47, 1998, 150, 600);

//				if((input - 60) > duty_cycle){
//					duty_cycle +=2;
//				}
//				if((input - 60) < duty_cycle){
//					duty_cycle -=2;
//				}
				duty_cycle = (input  - 20);

				if (bemf_counts < 20) {
	//				if (duty_cycle < 150) {
	//					duty_cycle = 150;
	//				}
					if (duty_cycle > 500) {
						duty_cycle = 500;
					}
				}

				if (bemf_counts < 5 ){

						duty_cycle = start_power;

				}

				if (running) {
					if (duty_cycle > 1998) {                             // safety!!!
						duty_cycle = 1998;
					}
					if (duty_cycle < 60) {
						duty_cycle = 60;
					}

					if (stall_protection && vehicle_mode == 2){
						if (commutation_interval > 10000){

boost_level = map(commutation_interval,10000,20000,10,80);
							duty_cycle = duty_cycle + boost_level;
						}
					}

					if(bi_polar){
	                 if (dither_count == 0){
					 if (forward){
						TIM1->CCR2 = (TIM1->ARR /2) + (input / 2);
						TIM1->CCR3 = (TIM1->ARR /2) - (input / 2);
					 }else{
						 TIM1->CCR2 = (TIM1->ARR /2) - (input / 2);
						 TIM1->CCR3 = (TIM1->ARR /2) + (input / 2);
					 }
	                 }
	                 if (dither_count == 1){
	                 				 if (forward){
	                 					TIM1->CCR2 = (TIM1->ARR /2) + (input / 2) + dither_amount;
	                 					TIM1->CCR3 = (TIM1->ARR /2) - (input / 2) - dither_amount;
	                 				 }else{
	                 					 TIM1->CCR2 = (TIM1->ARR /2) - (input / 2)- dither_amount;
	                 					 TIM1->CCR3 = (TIM1->ARR /2) + (input / 2) + dither_amount;
	                 				 }
	                                  }
	                 if (dither_count == 2){
	                 				 if (forward){
	                 					TIM1->CCR2 = (TIM1->ARR /2) + (input / 2)- dither_amount;
	                 					TIM1->CCR3 = (TIM1->ARR /2) - (input / 2) + dither_amount;
	                 				 }else{
	                 					 TIM1->CCR2 = (TIM1->ARR /2) - (input / 2) + dither_amount;
	                 					 TIM1->CCR3 = (TIM1->ARR /2) + (input / 2) - dither_amount;
	                 				 }
	                                  }
					}else{

					tim1_arr = map(commutation_interval, 84, 168, 1000, 2000);
					adjusted_duty_cycle = (duty_cycle * tim1_arr)/2000 - 2;
					TIM1->ARR = tim1_arr;
//					if (speed_control_mode){
//						if(desired_interval > commutation_interval){
//
//						}
//					}else{
					TIM1->CCR1 = adjusted_duty_cycle;
					TIM1->CCR2 = adjusted_duty_cycle;
					TIM1->CCR3 = adjusted_duty_cycle;
//					}
//					TIM1->CCR1 = duty_cycle;
//					TIM1->CCR2 = duty_cycle;
//					TIM1->CCR3 = duty_cycle;
					//	TIM1->CCR4 = duty_cycle;
					}
				}

			}
			//

			signaltimeout++;
			if (signaltimeout > signal_timeout_threshold) {
				input = 0;
				armed = 0;
				dshot_telemetry = 0;
				armedcount = 0;
				error = 1;
				inputSet = 0;
				TIM15->PSC=0;
				TIM16->PSC=0;
				dshot = 0;
				proshot = 0;
				servoPwm = 0;
				HAL_TIM_Base_Start_IT(&htim16);
				IC_buffer_size = 64;
				for (int i=0; i < 64; i++){
					dma_buffer[i] = 0;
				}
				HAL_TIM_IC_Start_DMA(&htim15, TIM_CHANNEL_1, dma_buffer , 64);
				//	  duty_cycle = 0;          //mid point
			}

			if (input <= 47) {

				sine_mode = 0;
		        if((brushed_mode) && (brushed_direction_set)){

		        	brushed_direction_set = 0;
		        }
				if (brake == 1){
				EXTI->IMR &= ~(1 << 21);
				}
				//	sensorless = 0;
				forcedcomcount = 0;
				started = 0;
				//running = 0;
				if (!brake && !prop_brake_active) {
					allOff();
				}
				duty_cycle = 0;
				if ((brake || tempbrake)&& (!bi_polar)) {
					fullBrake();
					duty_cycle = 0;
					bemf_counts = 0;
					//	HAL_COMP_Stop_IT(&hcomp1);
				}

				if (prop_brake && prop_brake_active) {
					//	prop_brake_active = 1;
					duty_cycle = prop_brake_strength;
					proBrake();
				}

				if(bi_polar){
					TIM1->CCR2 = (TIM1->ARR) / 2;
					TIM1->CCR3 = (TIM1->ARR) / 2;
				}else{
				TIM1->CCR1 = duty_cycle;// set duty cycle to 50 out of 768 to start.
				TIM1->CCR2 = duty_cycle;
				TIM1->CCR3 = duty_cycle;
				}
	//					if (commutation_interval > 60000){
	//						HAL_COMP_Stop_IT(&hcomp1);
	//					//	prop_brake_active = 0;
	//					}

			}
	//		if (bemf_counts < 100) {


			if (vehicle_mode == 1){
			if (bemf_counts < 40 || commutation_interval > 1000 || duty_cycle < 200) {
				filter_delay = 15;
				filter_level = 10;
			} else {
				filter_level = 5;

				filter_delay = 0;
			}
			if (duty_cycle > 600 && bemf_counts > 75){
				filter_level = 2;
			//	filter_delay = 0;
			}

			if (commutation_interval < 100 && bemf_counts > 100){
				filter_level = 2;
			}

			}

			if (vehicle_mode == 2|| vehicle_mode == 3 ) {    // crawler much fewer poles, much more filtering time needed
				if (bemf_counts < 25 || commutation_interval > 4000 || duty_cycle < 200) {
					filter_delay = 15;
					filter_level = 15;
				} else {
					filter_level = 8;

					filter_delay = 0;
				}
			}
			if (vehicle_mode == 5 ){

			if(bemf_counts < 15 || commutation_interval > 12000){
				filter_level = 10;
			}else{
				filter_level = 2;
				wait_before = 700;
			}
			if(commutation_interval < 100){
				filter_level = 1;
				wait_before = 500;
			}
			}

			if (started == 1) {
				if (running == 0) {
					if(brushed_mode){
						running = 1;
					}else{

					zctimeout = 0;
					startMotor(); // safety on for input testing   ************************************************
					}
				}
			}

			if (polling_mode && running == 1){
				checkForZeroCross();

	//			if ((TIM3->CNT > (commutation_interval + commutation_interval / 2)) && commutation_interval > 10000 ){
	//				if(!forced_com_done){
	//					commutate();
	//					TIM3->CNT = 0;
	//					forced_com_done = 1;
	//				}
	//			}
			}




	if(!brushed_mode || !sine_mode){
	//		if (duty_cycle < 300 && bemf_counts > 10) {
	//			zc_timeout_threshold = 800;
	//		} else {
	//			zc_timeout_threshold = 700;
	//		}
	////		if (TIM3->CNT > 60000 && duty_cycle < 1000){
	////			running = 0;
	////			started = 0;
	////			EXTI->IMR &= ~(1 << 21);
	////			EXTI->PR &=~(1 << 21);
	////			//input = 0;
	////
	////		}
		zctimeout++;
			if (zctimeout > zc_timeout_threshold) {
				bemf_counts = 0;
				//		prop_brake_active = 0;
				bad_commutation = 0;
				sensorless = 0;
				EXTI->IMR &= (0 << 21);
			//	HAL_COMP_Stop_IT(&hcomp1);
			//	count++;
				running = 0;
			//	duty_cycle = 0;
			}

	}
	if(( input > 47 && input < (100-(52-sine_mode_range)-(10*running)))&&( armed)){
		if(running){
			EXTI->IMR &= ~(1 << 21);
			running = 0;
			started = 0;
		}
		if (sine_mode == 0){

			proBrake();
//			TIM1->ARR = 1500;
			sine_mode = 1;
		}


		step_delay = map(input,48,80,2000,500);
		TIM14->ARR = step_delay;
		if (TIM14->CNT >= TIM14->ARR){
			TIM14->CNT = TIM14->ARR - 2;
		}
	}





    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14
                              |RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = ENABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted. 
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted. 
  */
  sConfig.Channel = ADC_CHANNEL_6;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted. 
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief COMP1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_COMP1_Init(void)
{

  /* USER CODE BEGIN COMP1_Init 0 */

  /* USER CODE END COMP1_Init 0 */

  /* USER CODE BEGIN COMP1_Init 1 */

  /* USER CODE END COMP1_Init 1 */
  hcomp1.Instance = COMP1;
  hcomp1.Init.InvertingInput = COMP_INVERTINGINPUT_DAC2;
  hcomp1.Init.NonInvertingInput = COMP_NONINVERTINGINPUT_IO1;
  hcomp1.Init.Output = COMP_OUTPUT_NONE;
  hcomp1.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
  hcomp1.Init.Hysteresis = COMP_HYSTERESIS_NONE;
  hcomp1.Init.Mode = COMP_MODE_HIGHSPEED;
  hcomp1.Init.WindowMode = COMP_WINDOWMODE_DISABLE;
  hcomp1.Init.TriggerMode = COMP_TRIGGERMODE_IT_RISING_FALLING;
  if (HAL_COMP_Init(&hcomp1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN COMP1_Init 2 */

  /* USER CODE END COMP1_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_16;
  hiwdg.Init.Window = 4095;
  hiwdg.Init.Reload = 4000;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = timer_one_period;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 40;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 63;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_TIM_DISABLE_OCxPRELOAD(&htim2, TIM_CHANNEL_3);
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 23;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 23;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 65535;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 10;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 4000;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

}

/**
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 0;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 0xffff;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchro(&htim15, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim15, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 0;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 9000;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 47;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 65535;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
  /* DMA1_Channel4_5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_5_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

  /**/
  LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_15);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_15;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
static void MX_TIM15_Init_PWM(void)
{

//TIM8->CNT = 0;
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
 // TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};


//  htim15.Instance = TIM15;
    htim15.Init.Prescaler = 1;
//  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 95;
//  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
//  htim15.Init.RepetitionCounter = 0;
//  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
//	TIM15->ARR = timer_15_period;
//	TIM15->PSC = timer_15_prescaler;
  if (HAL_TIM_SlaveConfigSynchronization(&htim15, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }


  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  GPIO_InitTypeDef GPIO_InitStruct = {0};
      __HAL_RCC_GPIOA_CLK_ENABLE();
      /**TIM15 GPIO Configuration
      PA2     ------> TIM15_CH1
      */
      GPIO_InitStruct.Pin = GPIO_PIN_2;
      GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
      GPIO_InitStruct.Pull = GPIO_PULLUP;
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
      GPIO_InitStruct.Alternate = GPIO_AF0_TIM15;
      HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
