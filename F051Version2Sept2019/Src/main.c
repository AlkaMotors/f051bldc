/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"

/* USER CODE BEGIN Includes */
#include "eeprom.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

COMP_HandleTypeDef hcomp1;

IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim15;
TIM_HandleTypeDef htim16;
DMA_HandleTypeDef hdma_tim15_ch1_up_trig_com;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

//#define MP6531
#define FD6288

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
char vehicle_mode = 2;    // 1 = quad mode / eeprom load mode , 2 = crawler / thruster mode,  3 = rc car mode,  4 = like car mode but with auto reverse after stop
int dead_time = 60;           // change to 60 for qfn

int dir_reversed = 0;   // global direction reversed set in eeprom
int step = 1;
int startcount = 0;
int deviation = 0;
uint32_t extiline;

int forcedcomcount = 0;
int commandcount = 0;
char bad_commutation = 0;

int bi_direction = 0;
char slow_decay = 1;                      // for complementary pwm , 0 for diode freewheeling
int brake = 1;                          // apply full motor brake on stop
int start_power = 150;
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
int advance_multiplier = 10;


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
int zc_timeout_threshold = 2000;   // depends on speed of main loop

int signaltimeout = 0;
int signal_timeout_threshold = 20000;

int temp_step;
int ROC = 1;
int tocheck = 0;

int zcfound = 1;
int threshold = 1;
int upthreshold = 1;
int tim2_start_arr = 9000;
int startupcountdown = 0;

int duty_cycle = 100;

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


char dshot = 0;
char proshot = 0;
char multishot = 0;
char oneshot42 = 0;
char oneshot125 = 0;
char servoPwm = 0;


char inputSet = 0;

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
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


		if(!slow_decay  || prop_brake_active){            // for future
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

		if (!slow_decay || prop_brake_active){
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

		if (!slow_decay || prop_brake_active){
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
//	TIM2->CNT = 0;
//	HAL_COMP_Stop_IT(&hcomp1);            // done in comparator routine

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
	//	hcomp1.Init.InvertingInput = COMP_INVERTINGINPUT_DAC2;
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
		//hcomp1.Init.InvertingInput = COMP_INVERTINGINPUT_DAC1;
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
//stop_time = TIM2->CNT;
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
comStep(step);
	}
	changeCompInput();
//	zcfound = 0;
//	falseAlarm = 0;
//	compCount = 0;
//	upcompCount = 0;

	//stop_time = TIM2->CNT;
}



void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

			if (htim->Instance==TIM16)
			{

			if(inputSet == 1){
				 if (!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2)){

		     HAL_TIM_IC_Stop_DMA(&htim15,TIM_CHANNEL_1);
			 TIM15->CNT = 0;
//			 for (int i = 0; i < 8; i++){
//			 							dma_buffer[i]=0;
//			 						}


				 HAL_TIM_IC_Start_DMA(&htim15, TIM_CHANNEL_1, dma_buffer , IC_buffer_size);
			 }


			 }

		}
}

void startMotor() {

 startcount++;

    char decaystate = slow_decay;
    sensorless = 0;
	if (running == 0){
		EXTI->IMR &= (0 << 21);
		EXTI->PR &=(0 << 21);


	//	HAL_COMP_Stop_IT(&hcomp1);
		slow_decay = 1;


	commutate();
	//HAL_Delay(1);
	//commutate();
	commutation_interval = 15000;
	TIM3->CNT = 0;
//	TIM2->CNT = 0;
//	TIM2->ARR = commutation_interval * 2;
	running = 1;
//	if (HAL_COMP_Start_IT(&hcomp1) != HAL_OK) {
			/* Initialization Error */
//			Error_Handler();
//		}
	EXTI->IMR |= (1 << 21);
	}

	slow_decay = decaystate;    // return to normal
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


if ((TIM3->CNT < commutation_interval >> 1)&& bemf_counts > 10 ){
	return;
}

//	while(TIM3->CNT - thiszctime  < 10){
//
//	}
		if (rising){
		//	advancedivisor = advancedivisorup;
			for (int i = 0; i < filter_level; i++){
			if (HAL_COMP_GetOutputLevel(&hcomp1) == COMP_OUTPUTLEVEL_HIGH){
		//		GPIOA->BRR = GPIO_PIN_15;


				return;
			}
			}


		}else{
		//	advancedivisor = advancedivisordown;
			for (int i = 0; i < filter_level; i++){
			if (HAL_COMP_GetOutputLevel(&hcomp1) == COMP_OUTPUTLEVEL_LOW){
		//		GPIOA->BRR = GPIO_PIN_15;

				return;
			}
			}

		}
		thiszctime = TIM3->CNT;
		compit +=1;
	TIM3->CNT = 0;
	zctimeout = 0;
	//	HAL_COMP_Stop_IT(&hcomp1);
	EXTI->IMR &= (0 << 21);
	EXTI->PR &=(0 << 21);
	if (waitTime < 0){
			waitTime = 0;
		}
	while (TIM3->CNT  < waitTime){


	}

	TIM1->CNT = duty_cycle;
	commutate();

	commutation_interval = (( commutation_interval) + thiszctime) / 2;
		degree_time = commutation_interval >> 5;                          // about 1.85 degrees per unit
			advance = degree_time * advance_multiplier;                     //  * 16 would be about 30 degrees

//	advance = commutation_interval>>2 ;
	waitTime = (commutation_interval >> 1) - advance;
			blanktime = commutation_interval >>4 ;                               // divided by 4
			if(bemf_counts > 200){
			blanktime = commutation_interval >>3 ;                              // by 8
			}
			bemf_counts++;
			while (TIM3->CNT < waitTime + blanktime){

			}

			if (compit > 10){
				EXTI->IMR &= (0 << 21);
				EXTI->PR &=(0 << 21);
		//		input = 0;
			//	error = 1;
				return;
			}

			EXTI->IMR |= (1 << 21);




}



void playStartupTune(){
	TIM1->PSC = 75;
	TIM1->CCR1 = 5;
	TIM1->CCR2 = 5;
	TIM1->CCR3 = 5;
	comStep(2);
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
	comStep(2);
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

	if ((smallestnumber > 3)&&(smallestnumber < 22)){
		dshot = 1;
		TIM15->PSC = 20;
		IC_buffer_size = 32;
		TIM16->ARR = 8000;
	}
	if ((smallestnumber > 22)&&(smallestnumber < 30)){
			dshot = 1;
			TIM15->PSC = 32;
			TIM16->PSC = 1;
			TIM16->ARR = 8000;
			IC_buffer_size = 32;
		}

	if ((smallestnumber > 40 )&&(smallestnumber < 80)){
		proshot = 1;
		TIM15->PSC = 5;
		IC_buffer_size = 8;
		TIM16->ARR = 8000;
	}
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

	if (dma_buffer[0] < 20){
		return;
	}

//	for (int i = 1; i < 32; i+=2){
//		dpulse[(i-1)>>1] = dma_buffer[i] ;
//	}
	 calcCRC = ((dma_buffer[1]^dma_buffer[9]^dma_buffer[17])<<3
			          |(dma_buffer[3]^dma_buffer[11]^dma_buffer[19])<<2
					|(dma_buffer[5]^dma_buffer[13]^dma_buffer[21])<<1
					|(dma_buffer[7]^dma_buffer[15]^dma_buffer[23])
				);
//	 calcCRC = ((dpulse[0]^dpulse[4]^dpulse[8])<<3
//			          |(dpulse[1]^dpulse[5]^dpulse[9])<<2
//					|(dpulse[2]^dpulse[6]^dpulse[10])<<1
//					|(dpulse[3]^dpulse[7]^dpulse[11])
//				);
//	 checkCRC = (dpulse[12]<<3 | dpulse[13]<<2 | dpulse[14]<<1 | dpulse[15]);
	 checkCRC = (dma_buffer[25]<<3 | dma_buffer[27]<<2 | dma_buffer[29]<<1 | dma_buffer[31]);
//
			if(calcCRC == checkCRC){
				tocheck = (
						dma_buffer[1]<<10 | dma_buffer[3]<<9 | dma_buffer[5]<<8 | dma_buffer[7]<<7
					| dma_buffer[9]<<6 | dma_buffer[11]<<5 | dma_buffer[13]<<4 | dma_buffer[15]<<3
					| dma_buffer[17]<<2 | dma_buffer[19]<<1 | dma_buffer[21]);
//				success++;
				}else{
					error++;
				}

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


void transferComplete(){
//	TIM2->CNT = 0;
//	compit = 0;
	signaltimeout = 0;
//	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_3);


	if (inputSet == 1){




		if (dshot == 1){
			computeDshotDMA();
//			HAL_TIM_IC_Start_DMA(&htim15, TIM_CHANNEL_1, dma_buffer , 64);
	//		stop_time = TIM2->CNT;
			return;
		}
		if (proshot == 1){
			computeProshotDMA();
	//		stop_time = TIM2->CNT;
	//		HAL_TIM_IC_Start_DMA(&htim15, TIM_CHANNEL_1, dma_buffer , 16);
			return;
		}

		if  (servoPwm == 1){
			computeServoInput();

		//	if (!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2)){
			//	HAL_TIM_IC_Start_DMA(&htim15, TIM_CHANNEL_1, dma_buffer , 3);
		//	}else{
				HAL_TIM_IC_Start_DMA(&htim15, TIM_CHANNEL_1, dma_buffer , 3);
		//	}

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
//
//		}



}

}
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  HAL_FLASH_Unlock();
  EE_Init();
//  HAL_Delay(10);
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
  MX_IWDG_Init();
  MX_TIM16_Init();

  /* USER CODE BEGIN 2 */
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

	HAL_TIM_Base_Start(&htim2);
	HAL_TIM_Base_Start_IT(&htim16);
	HAL_TIM_Base_Start(&htim3);
	//  HAL_Delay(500);
	playStartupTune();


	if (vehicle_mode == 1){                    // quad single direction
		loadEEpromConfig();
	}
	if (vehicle_mode == 2){                   // crawler or thruster
		 bi_direction = 1;
		 slow_decay = 1;                      // for complementary pwm , 0 for diode freewheeling
		 brake = 1;                          // apply full motor brake on stop
		 start_power = 150;

	}
	if (vehicle_mode == 3){                 // rc car 50 percent brake on reverse.
		 bi_direction = 1;
		 slow_decay = 0;                      // for complementary pwm , 0 for diode freewheeling
		 brake = 0;                          // apply full motor brake on stop
	//	 start_power = 150;
		 prop_brake = 1;
		 prop_brake_strength = 900;
	}
	if (vehicle_mode == 4){                 // rc car 50 percent brake on reverse.
			 bi_direction = 1;
			 slow_decay = 0;                      // for complementary pwm , 0 for diode freewheeling
			 brake = 0;                          // apply full motor brake on stop
	//		 start_power = 150;
			 prop_brake = 1;
			 prop_brake_strength = 800;
		}



	if (HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_4) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}
	HAL_TIM_IC_Start_DMA(&htim15, TIM_CHANNEL_1, dma_buffer , 64);
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
	if(HAL_IWDG_Init(&hiwdg) != HAL_OK)
	{
		/* Initialization Error */
		Error_Handler();
	}

	if(bi_direction){
		newinput = 1001;
	//	start_power = 175;
	}

	if (dir_reversed == 1){
		forward = 0;
	}else{
		forward = 1;
	}
	TIM1->CCR1 = 1;
	TIM1->CCR2 = 1;
	TIM1->CCR3 = 1;

	TIM1->CCR4 = 800;
//	changeCompInput();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{


	//voltage = voltageraw;
		k_erpm = ((100000000 / (commutation_interval * 208 * 6)) * 6)/10 ;

		count++;
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
						forward = 1 - dir_reversed;
						//	HAL_Delay(1);
					}

					if (prop_brake_active == 0) {

						adjusted_input = (newinput - 1050) * 3;
						//	tempbrake = 0;
						//	}
					}
				}
				if (newinput < 800) {
					if (forward == (1 - dir_reversed)) {
						prop_brake_active = 1;
						adjusted_input = 0;
						forward = dir_reversed;
						//	HAL_Delay(1);

					}
					if (prop_brake_active == 0) {
						adjusted_input = ((800 - newinput) * 3) + 100;

					}
					//	tempbrake = 0;
				}
				if (zctimeout >= zc_timeout_threshold) {
					//	adjusted_input = 0;
					if (vehicle_mode != 3) { // car mode requires throttle return to center before direction change
						prop_brake_active = 0;
					}

					startupcountdown = 0;
					bemf_counts = 0;
				}

				if (newinput > 800 && newinput < 1100) {
					adjusted_input = 0;
					prop_brake_active = 0;
				}

			} else if ((proshot || dshot) && bi_direction) {
				if (newinput > 1097) {

					if (forward == dir_reversed) {
						forward = 1 - dir_reversed;
						bemf_counts = 0;
					//	startcount++;
					}
					adjusted_input = (newinput - 1100) * 2 + 100;
				}
				if (newinput <= 1047 && newinput > 0) {
				//	startcount++;

					if (forward == (1 - dir_reversed)) {
						bemf_counts = 0;
						forward = dir_reversed;

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

			if (adjusted_input - input > 5) {
				input = input + 5;
			//	advance_multiplier = 12;
			} else if (input - adjusted_input > 5) {
				input = input - 5;
			} else {
				input = adjusted_input;
			//	advance_multiplier = map((commutation_interval), 150, 300, 8, 8);
			}

		}

		if ( commutation_interval < 120 ){        // desync
		input = 0;
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

		advance_multiplier = map((commutation_interval), 150, 3000, 14, 6);
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


		if ((input > 47) && (armed == 1)) {
			prop_brake_active = 0;
			started = 1;

			duty_cycle = input  - 20;

			if (bemf_counts < 25) {
				if (duty_cycle < 150) {
					duty_cycle = 150;
				}
				if (duty_cycle > 350) {
					duty_cycle = 350;
				}
			}

			if (bemf_counts < 199 ){
				if (duty_cycle > 500){
					duty_cycle = 500;
				}
			}

			if (running) {
				if (duty_cycle > 1998) {                             // safety!!!
					duty_cycle = 1999;
				}
				if (duty_cycle < 60) {
					duty_cycle = 60;
				}

				TIM1->CCR1 = duty_cycle;// set duty cycle to 50 out of 768 to start.
				TIM1->CCR2 = duty_cycle;
				TIM1->CCR3 = duty_cycle;
				//	TIM1->CCR4 = duty_cycle;
			}

		}
		//

		signaltimeout++;
		if (signaltimeout > signal_timeout_threshold) {
			input = 0;
			armed = 0;
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
	//		error = 1;
			if (brake == 1){
			EXTI->IMR &= (0 << 21);
			}
			//	sensorless = 0;
			forcedcomcount = 0;
			started = 0;
			if (!brake && !prop_brake_active) {
				allOff();
			}
			duty_cycle = 0;
			if (brake || tempbrake) {
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

			TIM1->CCR1 = duty_cycle;// set duty cycle to 50 out of 768 to start.
			TIM1->CCR2 = duty_cycle;
			TIM1->CCR3 = duty_cycle;

//					if (commutation_interval > 60000){
//						HAL_COMP_Stop_IT(&hcomp1);
//					//	prop_brake_active = 0;
//					}

		}
//		if (bemf_counts < 100) {

		if (bemf_counts < 50 || commutation_interval > 2000 || duty_cycle < 200) {
		//	filter_delay = 5;
			filter_level = 10;
		} else {
			filter_level = 4;
	//		filter_delay = 5;
		}
		if (duty_cycle > 600){
			filter_level = 3;
		}
//		if(bemf_counts < 25){
//			filter_level = 20;
//		}
//		if (bemf_counts >= 200 && commutation_interval < 2000) {
//
//		//	filter_delay = 3;
//			filter_level = 8;
//		}
		if (commutation_interval < 200 && bemf_counts > 100){
			filter_level = 3;
		}

		if (started == 1) {
			if (running == 0) {
				//		allOff();
				upthreshold = 2;
				threshold = 2;
				zctimeout = 0;
				startMotor(); // safety on for input testing   ************************************************
			}
		}

		if (duty_cycle < 300 && bemf_counts > 10) {
			zc_timeout_threshold = 10000;
		} else {
			zc_timeout_threshold = 2000;
		}

		zctimeout++;
		if (zctimeout > zc_timeout_threshold) {
			bemf_counts = 0;
			//		prop_brake_active = 0;
			bad_commutation = 0;
			sensorless = 0;
			EXTI->IMR &= (0 << 21);
		//	HAL_COMP_Stop_IT(&hcomp1);
			//			if(tempbrake){
			//
			//			}else{
			//			allOff();
			//			}
			running = 0;
			//		commutation_interval = 0;
			zctimeout = zc_timeout_threshold + 1;
			duty_cycle = 0;
		}



//		if ((bemf_counts > 15
//				&& TIM3->CNT > commutation_interval + commutation_interval / 2)
//				&& duty_cycle < 300) {
//			if (forcedcount < 1) {
//				forcedcount++;
//				forcedCommutation();
//			}
//		//	bemf_counts = 0;
//		}



  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
//			}
//		}
	}
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14
                              |RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC init function */
static void MX_ADC_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
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
  hadc.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_CC4;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc.Init.DMAContinuousRequests = ENABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
//  sConfig.Channel = ADC_CHANNEL_3;
//  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
//  {
//    _Error_Handler(__FILE__, __LINE__);
//  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* COMP1 init function */
static void MX_COMP1_Init(void)
{

  hcomp1.Instance = COMP1;
  hcomp1.Init.InvertingInput = COMP_INVERTINGINPUT_DAC1;
  hcomp1.Init.NonInvertingInput = COMP_NONINVERTINGINPUT_IO1;
  hcomp1.Init.Output = COMP_OUTPUT_NONE;
  hcomp1.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
  hcomp1.Init.Hysteresis = COMP_HYSTERESIS_NONE;
  hcomp1.Init.Mode = COMP_MODE_HIGHSPEED;
  hcomp1.Init.WindowMode = COMP_WINDOWMODE_DISABLE;
  hcomp1.Init.TriggerMode = COMP_TRIGGERMODE_IT_RISING;
  if (HAL_COMP_Init(&hcomp1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* IWDG init function */
static void MX_IWDG_Init(void)
{

	hiwdg.Instance = IWDG;
	hiwdg.Init.Prescaler = IWDG_PRESCALER_16;
	hiwdg.Init.Window = IWDG_WINDOW_DISABLE;
	hiwdg.Init.Reload = 1000;
	if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
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
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 30;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim1);

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 10;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 50000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 10;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM15 init function */
static void MX_TIM15_Init(void)
{

  TIM_SlaveConfigTypeDef sSlaveConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_IC_InitTypeDef sConfigIC;

  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 0;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 0xffff;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_IC_Init(&htim15) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchronization(&htim15, &sSlaveConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim15, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM16 init function */
static void MX_TIM16_Init(void)
{

  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 0;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 9000;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

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
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel4_5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_5_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_5_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  LL_GPIO_InitTypeDef GPIO_InitStruct;

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

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while(1)
	{
	}
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
