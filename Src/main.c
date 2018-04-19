
#include "main.h"
#include "stm32f0xx_hal.h"

/* USER CODE BEGIN Includes */

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
DMA_HandleTypeDef hdma_tim15_ch1_up_trig_com;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
int step = 1;
int pot = 1000;

int smoothedinput = 0;
const int numReadings = 1000;     // the readings from the analog input
int readIndex = 0;              // the index of the current reading
int total = 0;

int dshotcommand = 0;
uint8_t calcCRC;
uint8_t checkCRC;

int quietmode = 0;
int sine_array[20] = {80, 80, 90, 90 , 95 , 95 ,95, 100, 100,100, 100 ,100, 100,95,95,95,90,90,80,80};

int count = 0;
int compCount = 0;
int upcompCount = 0;
int falseAlarm = 0;
int falseThreshold = 2;
int upthiszctime = 0;
int uplastzctime = 0;
int brake = 0;
char fastdecay = 1;                      // for complementary pwm , 0 for diode freewheeling

char advancedivisor = 3;                    // increase divisor to decrease advance,  3 to 6 tested 48 blows up motors
char advancedivisorup = 3;

int thiszctime = 0;
int lastzctime = 0;
int sensorless = 0;
int commutation_interval = 0;
int advance = 0;                       // set proportianal to commutation time. with advance divisor
int blanktime;
int pwm_settle = 50;
int demagtime = 5;
int waitTime = 0;

int inputcapture = 0;

int control_loop_count;
int zctimeout = 0;
int zc_timeout_threshold = 1000;   // depends on speed of main loop

int ROC = 1;

int zcfound = 1;
int threshold = 1;
int upthreshold = 1;
int tim2_start_arr = 9000;
int startupcountdown = 0;

int duty_cycle = 100;

int pwm = 1;
int floating = 2;
int lowside = 3;

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

int voltageraw = 0;
int currentraw = 0;
uint32_t ADC1ConvertedValues[2] = {0,0};



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





void phaseA(int newPhase) {
	if (newPhase == pwm) {
		if(!fastdecay){            // for future
		LL_GPIO_SetPinMode(GPIOB, GPIO_PIN_0, LL_GPIO_MODE_OUTPUT);
		GPIOB->BRR = GPIO_PIN_0;
		}else{
		LL_GPIO_SetPinMode(GPIOB, GPIO_PIN_0, LL_GPIO_MODE_ALTERNATE); // low
		}
		LL_GPIO_SetPinMode(GPIOA, GPIO_PIN_9, LL_GPIO_MODE_ALTERNATE);  // high

	}

	if (newPhase == floating) {
		LL_GPIO_SetPinMode(GPIOB, GPIO_PIN_0, LL_GPIO_MODE_OUTPUT);
		GPIOB->BRR = GPIO_PIN_0;
		LL_GPIO_SetPinMode(GPIOA, GPIO_PIN_9, LL_GPIO_MODE_OUTPUT);
		GPIOA->BRR = GPIO_PIN_9;
	}

	if (newPhase == lowside) {          // low mosfet on
		LL_GPIO_SetPinMode(GPIOB, GPIO_PIN_0, LL_GPIO_MODE_OUTPUT);
		GPIOB->BSRR = GPIO_PIN_0;
		LL_GPIO_SetPinMode(GPIOA, GPIO_PIN_9, LL_GPIO_MODE_OUTPUT);
		GPIOA->BRR = GPIO_PIN_9;
	}

}

void phaseB(int newPhase) {

	if (newPhase == pwm) {  // pwm
		if (!fastdecay){
			LL_GPIO_SetPinMode(GPIOA, GPIO_PIN_7, LL_GPIO_MODE_OUTPUT);
			GPIOA->BRR = GPIO_PIN_7;
		}else{
		LL_GPIO_SetPinMode(GPIOA, GPIO_PIN_7, LL_GPIO_MODE_ALTERNATE);
		}
		LL_GPIO_SetPinMode(GPIOA, GPIO_PIN_8, LL_GPIO_MODE_ALTERNATE);

	}

	if (newPhase == floating) {            // floating
		LL_GPIO_SetPinMode(GPIOA, GPIO_PIN_7, LL_GPIO_MODE_OUTPUT);
		GPIOA->BRR = GPIO_PIN_7;
		LL_GPIO_SetPinMode(GPIOA, GPIO_PIN_8, LL_GPIO_MODE_OUTPUT);
		GPIOA->BRR = GPIO_PIN_8;
	}

	if (newPhase == lowside) {              // lowside
		LL_GPIO_SetPinMode(GPIOA, GPIO_PIN_7, LL_GPIO_MODE_OUTPUT);
		GPIOA->BSRR = GPIO_PIN_7;
		LL_GPIO_SetPinMode(GPIOA, GPIO_PIN_8, LL_GPIO_MODE_OUTPUT);
		GPIOA->BRR = GPIO_PIN_8;
	}
}

void phaseC(int newPhase) {
	if (newPhase == pwm) {
		if (!fastdecay){
			LL_GPIO_SetPinMode(GPIOB, GPIO_PIN_1, LL_GPIO_MODE_OUTPUT);
			GPIOB->BRR = GPIO_PIN_1;
			}else{
		LL_GPIO_SetPinMode(GPIOB, GPIO_PIN_1, LL_GPIO_MODE_ALTERNATE);
			}
		LL_GPIO_SetPinMode(GPIOA, GPIO_PIN_10, LL_GPIO_MODE_ALTERNATE);

	}

	if (newPhase == floating) {
		LL_GPIO_SetPinMode(GPIOB, GPIO_PIN_1, LL_GPIO_MODE_OUTPUT);
		GPIOB->BRR = GPIO_PIN_1;
		LL_GPIO_SetPinMode(GPIOA, GPIO_PIN_10, LL_GPIO_MODE_OUTPUT);
		GPIOA->BRR = GPIO_PIN_10;
	}

	if (newPhase == lowside) {
		LL_GPIO_SetPinMode(GPIOB, GPIO_PIN_1, LL_GPIO_MODE_OUTPUT);
		GPIOB->BSRR = GPIO_PIN_1;
		LL_GPIO_SetPinMode(GPIOA, GPIO_PIN_10, LL_GPIO_MODE_OUTPUT);
		GPIOA->BRR = GPIO_PIN_10;
	}

}

void comStep(int newStep) {

	if (newStep == 1) {			//A-B
		phaseA(pwm);
		phaseB(lowside);
		phaseC(floating);
	}

	if (newStep == 2) {			// C-B
		phaseA(floating);
		phaseB(lowside);
		phaseC(pwm);
	}

	if (newStep == 3) {		// C-A
		phaseA(lowside);
		phaseB(floating);
		phaseC(pwm);
	}

	if (newStep == 4) {    // B-A
		phaseA(lowside);
		phaseB(pwm);
		phaseC(floating);
	}

	if (newStep == 5) {          // B-C
		phaseA(floating);
		phaseB(pwm);
		phaseC(lowside);
	}

	if (newStep == 6) {       // A-C
		phaseA(pwm);
		phaseB(floating);
		phaseC(lowside);
	}

}

void allOff() {
	phaseA(floating);
	phaseB(floating);
	phaseC(floating);
}

void fullBrake(){
	phaseA(lowside);
	phaseB(lowside);
	phaseC(lowside);
}

void align(){
	phaseA(pwm);
	phaseB(floating);
	phaseC(lowside);
}

void changeCompInput() {

	HAL_COMP_Stop(&hcomp1);

	if (step == 1 || step == 4) {   // c floating
		hcomp1.Init.InvertingInput = COMP_INVERTINGINPUT_IO1;
	}

	if (step == 2 || step == 5) {     // a floating
		hcomp1.Init.InvertingInput = COMP_INVERTINGINPUT_DAC1;
	}

	if (step == 3 || step == 6) {      // b floating
		hcomp1.Init.InvertingInput = COMP_INVERTINGINPUT_DAC2;
	}

	if (HAL_COMP_Init(&hcomp1) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	if (HAL_COMP_Start(&hcomp1) != HAL_OK) {
		/* Initialization Error */
		Error_Handler();
	}

}


void commutate() {

	if (forward == 1){
		step++;
		if (step > 6) {
			step = 1;
		}
		if (step == 1 || step == 3 || step == 5) {
			rising = 1;
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


	comStep(step);
	changeCompInput();
	zcfound = 0;
	falseAlarm = 0;
	compCount = 0;
	upcompCount = 0;
	TIM2->CNT = 0;
	TIM2->ARR = commutation_interval;
}



void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) { // for forced commutation -- open loop

	if (htim->Instance == TIM2)

	{

		if (!sensorless) {
			waitTime = TIM2->ARR / 2;

			if (running == 1) {
				GPIOA->BRR = GPIO_PIN_15;
				commutate();


				//				step++;
				//				if (step > 6) {
				//					step = 1;
				//				}
				//				comStep(step);
				//				changeCompInput();
				//				zcfound = 0;
				//				falseAlarm = 0;
				//				compCount = 0;
			}
		}
	}
}



void startMotor() {
    char decaystate = fastdecay;

	if (commutation_interval == 0){
		fastdecay = 1;

	TIM1->CCR1 = 100;				// set duty cycle to 110 out of 999 to start.
	TIM1->CCR2 = 100;
	TIM1->CCR3 = 100;

//	align();
//	HAL_Delay(300);
//	step = 6;



	for (int i = 8; i > 0; i--){

		//		TIM2->ARR = i;
		//		TIM2->CNT = 0;

		//		step++;
		//		if (step > 6) {
		//		step=1;
		//		}
		//		comStep(step);
		commutate();
		HAL_Delay(i);
        }

//	zcfound = 1; //supress bemf detection for speedup

	TIM2->ARR = tim2_start_arr-1000;
	commutation_interval = tim2_start_arr- 3000;
	HAL_Delay(5);
	}

	fastdecay = decaystate;    // return to normal

	TIM2->CNT = 0;
	running = 1;
	sensorless = 1;
	startupcountdown =0;

	//					sensorless = 1;
}


//void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
//{
// if (htim->Instance==TIM15)
//  {
//     inputcapture=TIM15->CNT;
//     TIM15->CNT = 0;
//  }
//}


void HAL_COMP_TriggerCallback(COMP_HandleTypeDef *hcomp) {          // not used
	/* Turn On LED3 */

	if (TIM1->CNT > duty_cycle) {
		GPIOA->BSRR = GPIO_PIN_15;

		if (!zcfound) {
			compCount++;
		}
	}
}

void playStartupTune(){
	TIM1->PSC = 75;
	TIM1->CCR1 = 5;
	TIM1->CCR2 = 5;
	TIM1->CCR3 = 5;
	comStep(1);
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
	comStep(1);
	HAL_Delay(100);
	TIM1->PSC = 50;
	HAL_Delay(100);
	allOff();
	TIM1->PSC = 0;
}

void getADCs(){
	voltageraw = ADC1ConvertedValues[0];
	currentraw = ADC1ConvertedValues[1];
	//pot = ADC1ConvertedValues[2];

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
	int lastnumber = dma_buffer[0];
	for ( int j = 1 ; j < input_buffer_size; j++){

		if((dma_buffer[j] - lastnumber) < smallestnumber){ // blank space
			smallestnumber = dma_buffer[j] - lastnumber;

		}
		lastnumber = dma_buffer[j];
	}

	if ((smallestnumber > 3)&&(smallestnumber < 22)){
		dshot = 1;
	}
	if ((smallestnumber > 40 )&&(smallestnumber < 80)){
		proshot = 1;
		TIM15->PSC=1;
		TIM15->CNT = 0xffff;
	}
	if ((smallestnumber > 100 )&&(smallestnumber < 400)){
		multishot = 1;
	}
	if ((smallestnumber > 2000 )&&(smallestnumber < 3000)){
		oneshot42 = 1;
	}
	if ((smallestnumber > 3000 )&&(smallestnumber < 15000)){
		oneshot125 = 1;
	}
	if (smallestnumber > 15000 ){
		servoPwm = 1;
		TIM15->PSC = 47;
		TIM15->CNT = 0xffff;
	}

	if (smallestnumber == 0){
		inputSet = 0;
	}else{

		inputSet = 1;

		HAL_Delay(50);
		//	playInputTune();
	}
	HAL_TIM_IC_Start_DMA(&htim15, TIM_CHANNEL_1, dma_buffer , 64);
}

void computeProshotDMA(){
	int lastnumber = dma_buffer[0];
	for ( int j = 1 ; j < 9; j++){

		if(((dma_buffer[j] - lastnumber) > 1500) && ((dma_buffer[j] - lastnumber) < 50000)){ // blank space
			if ((dma_buffer[j+7] - dma_buffer[j])<10000){
				//			for ( int i = 0; i < 8; i+= 2){
				//
				//			propulse[i>>1] =map((dma_buffer[j+i+1] - dma_buffer[j+i]),48, 141, 0, 15);
				//			}

				//		for ( int i = 0; i < 8; i+= 2){
				//			 propulse[i>>1] = ((dma_buffer[j+i+1] - dma_buffer[j+i]) - 46)*11>>6;
				//		}
				for (int i = 0; i < 4; i++){

					propulse[i] = (((dma_buffer[j + i*2 +1] - dma_buffer[j + i*2])) - 23)/3;


				}

				calcCRC = ((propulse[0]^propulse[1]^propulse[2])<<3
						|(propulse[0]^propulse[1]^propulse[2])<<2
						|(propulse[0]^propulse[1]^propulse[2])<<1
						|(propulse[0]^propulse[1]^propulse[2]));
				checkCRC = (propulse[3]<<3 | propulse[3]<<2 | propulse[3]<<1 | propulse[3]);
			}


            if (calcCRC == checkCRC){
			int tocheck = ((propulse[0]<<7 | propulse[1]<<3 | propulse[2]>>1));
			if (tocheck > 2047 || tocheck < 0){
				break;
			}else{
				if(tocheck > 47){
					newinput = tocheck;
					dshotcommand = 0;
				}
				if ((tocheck <= 47)&& (tocheck > 0)){
					newinput = 0;
					dshotcommand = tocheck;    //  todo
				}
				if (tocheck == 0){
					newinput = 0;
					dshotcommand = 0;
				}
			}
            }
			break;
		}
		lastnumber = dma_buffer[j];
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

	int lastnumber = dma_buffer[0];
	for ( int j = 1 ; j < 3; j++){

		if(((dma_buffer[j] - lastnumber) >1000 ) && ((dma_buffer[j] - lastnumber) < 2010)){ // blank space

			newinput = map((dma_buffer[j] - lastnumber), 1090, 2000, 0, 2000);
			break;
		}
		lastnumber = dma_buffer[j];
	}
}


void computeDshotDMA(){

	int lastnumber = dma_buffer[0];
	for ( int j = 1 ; j < input_buffer_size; j++){

		if(((dma_buffer[j] - lastnumber) > 50) && ((dma_buffer[j] - lastnumber) < 65000)){ // blank space

			for (int i = 0; i < 16; i++){
				dpulse[i] = ((dma_buffer[j + i*2 +1] - dma_buffer[j + i*2]) / 13) - 1;
			}

			uint8_t calcCRC = ((dpulse[0]^dpulse[4]^dpulse[8])<<3
					|(dpulse[1]^dpulse[5]^dpulse[9])<<2
					|(dpulse[2]^dpulse[6]^dpulse[10])<<1
					|(dpulse[3]^dpulse[7]^dpulse[11])
			);
			uint8_t checkCRC = (dpulse[12]<<3 | dpulse[13]<<2 | dpulse[14]<<1 | dpulse[15]);
			//

			int tocheck = (
					dpulse[0]<<10 | dpulse[1]<<9 | dpulse[2]<<8 | dpulse[3]<<7
					| dpulse[4]<<6 | dpulse[5]<<5 | dpulse[6]<<4 | dpulse[7]<<3
					| dpulse[8]<<2 | dpulse[9]<<1 | dpulse[10]);

			if(calcCRC == checkCRC){

				if (tocheck > 47){
					newinput = tocheck;
                    dshotcommand = 0;
				}
			}
			if ((tocheck <= 47)&& (tocheck > 0)){
				newinput = 0;
				dshotcommand = tocheck;    //  todo
			}
			if (tocheck == 0){
				newinput = 0;
			//	dshotcommand = 0;
			}



			break;
		}
		lastnumber = dma_buffer[j];
	}
}

void transferComplete(){
	//	TIM15->CNT = 1;

	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_3);


	if (inputSet == 1){
		if (dshot == 1){
			computeDshotDMA();
			HAL_TIM_IC_Start_DMA(&htim15, TIM_CHANNEL_1, dma_buffer , 64);
		}
		if (proshot == 1){
			computeProshotDMA();
			HAL_TIM_IC_Start_DMA(&htim15, TIM_CHANNEL_1, dma_buffer , 16);
		}

		if  (servoPwm == 1){
			computeServoInput();
			HAL_TIM_IC_Start_DMA(&htim15, TIM_CHANNEL_1, dma_buffer , 3);

		}
		if  (multishot){
			computeMSInput();
			HAL_TIM_IC_Start_DMA(&htim15, TIM_CHANNEL_1, dma_buffer , 3);

		}
		if  (oneshot125){
			computeOS125Input();
			HAL_TIM_IC_Start_DMA(&htim15, TIM_CHANNEL_1, dma_buffer , 3);

		}
		if  (oneshot42){
			computeOS42Input();
			HAL_TIM_IC_Start_DMA(&htim15, TIM_CHANNEL_1, dma_buffer , 3);

		}
	}
}


void changeDutyCycleWithSin(){

	if (!rising){
	duty_cycle = (duty_cycle * sine_array[((TIM2->CNT*10)/TIM2->ARR)+9])/100;          // last ten elements in sin array
	}else{
	duty_cycle = (duty_cycle * sine_array[(TIM2->CNT*10)/TIM2->ARR])/100;           // first ten elements in sin array
	}

	TIM1->CCR1 = duty_cycle;
	TIM1->CCR2 = duty_cycle;
    TIM1->CCR3 = duty_cycle;

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

	/* USER CODE BEGIN 2 */
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);


	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Base_Start(&htim3);
	//  HAL_Delay(500);
	playStartupTune();


	if (HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_4) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	HAL_TIM_IC_Start_DMA(&htim15, TIM_CHANNEL_1, dma_buffer , 64);

	if (HAL_ADC_Start_DMA(&hadc, (uint32_t*)ADC1ConvertedValues, 2) != HAL_OK)
		return 0;

	if(HAL_COMP_Start(&hcomp1) != HAL_OK)
	{
		/* Initialization Error */
		Error_Handler();
	}
	if(HAL_IWDG_Init(&hiwdg) != HAL_OK)
	{
		/* Initialization Error */
		Error_Handler();
	}


	TIM1->CCR1 = duty_cycle;												// set duty cycle to 50 out of 768 to start.
	TIM1->CCR2 = duty_cycle;
	TIM1->CCR3 = duty_cycle;

	TIM1->CCR4 = 800;
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{

		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_15);
		if(HAL_IWDG_Refresh(&hiwdg) != HAL_OK)                   // watchdog refresh
				{
					/* Refresh Error */
					Error_Handler();
				}

	//	control_loop_count++;
	//	if (control_loop_count > 100){


		if (dshotcommand > 0){
			if (dshotcommand == 2){
			//	playInputTune();
			}
			if (dshotcommand == 21){
				forward = 0;
			}
			if (dshotcommand == 20){
				forward = 1;
			}
		}

		//	  getADCs();

		if (newinput > 100){
			if (newinput > 2000){
				newinput = 2000;
			}
			if (newinput > input){

				if (newinput - input > 25){
					upthreshold = 1;                 // needs two bemf counts for fast acceleration up.. more stable
					threshold = 0;                   //
					blanktime = TIM2->ARR / 5;       // but a bigger blanktime for bast accell
					count++;
					if (commutation_interval > 600){  // when turning slowly
						if (count > 5){
						input += 1;
						count = 0;

					}
						advancedivisor = 6;
						advancedivisorup = 3;
					}else{
						input++;
						advancedivisor = 3;
						advancedivisorup = 3;

					}

				}else{
					input = newinput;
					upthreshold = 1;
					threshold = 1;
					advancedivisor = 3;
					advancedivisorup = 3;
					blanktime = TIM2->ARR / 20;
				}

			}
			if (newinput < input){
				if (input - newinput > 50){
					input--;
				}else{
					input = newinput;
				}
			}
		}else{
			input = newinput;
		}

		//		getSmoothedInput();

		if (inputSet == 0){
			HAL_Delay(10);
			detectInput();

		}

		if (!armed){
			if ((inputSet == 1)&&(input == 0)){
				armedcount++;
				HAL_Delay(1);
				if (armedcount > 1000){
					armed = 1;
					playInputTune();
				}
			}
			if (input > 1){
				armedcount = 0;
			}
		}


		if ((input > 100)&&(armed == 1)) {
			started = 1;
			if (startupcountdown < 5000){
				input  = 125 + (25-(25*fastdecay));
				startupcountdown++;
			}else{

			duty_cycle = 40 + (input - 100)/2 + (50-(50*fastdecay));        // slightlty higher throttle for slow decay
			}
			if (quietmode){
				changeDutyCycleWithSin();
			}

			if (running){
			TIM1->CCR1 = duty_cycle;												// set duty cycle to 50 out of 768 to start.
			TIM1->CCR2 = duty_cycle;
			TIM1->CCR3 = duty_cycle;
			//	TIM1->CCR4 = duty_cycle;
			}

		}
//
//
		if (input <= 100) {
			started = 0;
			running = 0;
			threshold = 1;
			upthreshold = 2;
			if (!brake){
				allOff();
			}
			if(brake){
				fullBrake();
			}
			TIM1->CCR1 = 0;												// set duty cycle to 50 out of 768 to start.
			TIM1->CCR2 = 0;
			TIM1->CCR3 = 0;

		}

	//	}

		zctimeout++;
		if (zctimeout > zc_timeout_threshold) {
			allOff();
			running = 0;
			commutation_interval = 0;
			zctimeout = 0;
			//			}
		}

		if (started == 1) {
			if (running == 0) {
			allOff();
			startMotor();  // safety on for input testing   ************************************************
			}
		}


		if ((TIM2 ->CNT > blanktime)&& (TIM1 ->CNT > pwm_settle)){

				//			  if (step == 2 || step == 4 || step == 6){
				if (rising == 0){
					if (HAL_COMP_GetOutputLevel(&hcomp1) == COMP_OUTPUTLEVEL_LOW){
							compCount = 0;
							zcfound = 0;

					}

					if (HAL_COMP_GetOutputLevel(&hcomp1) == COMP_OUTPUTLEVEL_HIGH){

						compCount++;
					}
						if (compCount > threshold){
								zcfound = 1;
								zctimeout = 0;
								compCount = 0;

								thiszctime = TIM3->CNT;

								if (thiszctime > lastzctime){
									if ((thiszctime - lastzctime > (commutation_interval * 3)) || (thiszctime - lastzctime < commutation_interval)){

										commutation_interval = commutation_interval;

									}else{
										commutation_interval = (thiszctime - lastzctime)/2;
										advance = commutation_interval / advancedivisor;
										waitTime = commutation_interval /2 - advance;
									}

								}
								if (sensorless){
									while (TIM3->CNT - thiszctime < waitTime){
										if (quietmode){
										changeDutyCycleWithSin();
										}

									}
									commutate();

								}

								lastzctime = thiszctime;
							}

				}


				if (rising == 1){
					if (HAL_COMP_GetOutputLevel(&hcomp1) == COMP_OUTPUTLEVEL_HIGH){

							upcompCount = 0;
							zcfound = 0;


					}

					if (HAL_COMP_GetOutputLevel(&hcomp1) == COMP_OUTPUTLEVEL_LOW){
						//		GPIOA->BSRR = GPIO_PIN_15;
						upcompCount++;
					}

							if (upcompCount > upthreshold){
								zcfound = 1;
								zctimeout = 0;
								upcompCount = 0;

					//			GPIOA->BSRR = GPIO_PIN_15;
									upthiszctime = TIM3->CNT;

									if (upthiszctime > uplastzctime){
										if ((upthiszctime - uplastzctime > (commutation_interval * 2) + 300) || (upthiszctime - uplastzctime < commutation_interval)){

											commutation_interval = commutation_interval;

										}else{
											commutation_interval = (upthiszctime - uplastzctime)/2;
											advance = commutation_interval/ advancedivisorup;
											waitTime = commutation_interval / 2 - advance;
										}

									}


									if (sensorless){


										while (TIM3->CNT - upthiszctime < waitTime){
											if (quietmode){
												changeDutyCycleWithSin();

										}

										}
										commutate();
									}


									uplastzctime = upthiszctime;

							}
					//	}
					//}
				}
	//	}


				//GPIOA->BRR = GPIO_PIN_15;

				/* USER CODE END WHILE */

				/* USER CODE BEGIN 3 */
		//	}
		}
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
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI14|RCC_OSCILLATORTYPE_LSI
			|RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
	RCC_OscInitStruct.HSI14CalibrationValue = 16;
	RCC_OscInitStruct.LSIState = RCC_LSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
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
	sConfig.Channel = ADC_CHANNEL_3;
	sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
	sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
	if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure for the selected ADC regular channel to be converted.
	 */
	sConfig.Channel = ADC_CHANNEL_6;
	if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* COMP1 init function */
static void MX_COMP1_Init(void)
{

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
	htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
	htim1.Init.Period = 999;
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

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC4REF;
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
	sBreakDeadTimeConfig.DeadTime = 60;
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

	TIM_ClockConfigTypeDef sClockSourceConfig;
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

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	if (HAL_TIM_IC_Init(&htim15) != HAL_OK)
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

/** 
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) 
{
	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Channel1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
	/* DMA1_Channel4_5_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel4_5_IRQn, 0, 0);
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
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOF);
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
