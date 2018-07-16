/**
*			KEIL PROJECT
*			PLL SYNC WITH GRID
*			SVPWM 
**/
// LIBRARIES
#include <math.h>
#include "stm32f4xx.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_adc.h"
#include "stm32f4xx_conf.h"
#include "arm_math.h"

/** 	
Va 		-> 	PC3 	ADC_Ch13		ADC1
Vb 		-> 	PC0 	ADC_Ch10		ADC2
Vc 		-> 	PA6 	ADC_Ch06		ADC1
Ia 		-> 	PF5 	ADC_Ch15		ADC3
Ib 		-> 	PF3 	ADC_Ch09		ADC3
Ic 		-> 	PB0 	ADC_Ch08		ADC1	

VCII 	-> 	PA0 	ADC_Ch00		ADC2
P/ma 	-> 	PA3 	ADC_Ch03		ADC3
Q/d 	-> 	PC4 	ADC_Ch14 		ADC2

ADC1 {Va, 	Ic,		Vc}
ADC2 {VCII,	Vb,		Q/d}
ADC3 {Ia, 	Ib,		P/m}
**/

#define BUFFERSIZE 9

#define PERIOD 4500
#define ADC_PER 4499

volatile uint16_t adc[BUFFERSIZE] = {0};
volatile int k=0,l=0;

volatile float va = 0, vb = 0, vc = 0;
volatile float ia = 0, ib = 0, ic = 0;
volatile float vcii = 0;

//PLL VARIABLES
/*
	ANALOG: Kp = 180; Ki = 3200;
	Ts = 1/20e3;
	a = 1/V|peak * (Kp + Ki*Ts/2) * Ts
	b = 1/V|peak * (-Kp + Ki*Ts/2) * Ts
//volatile float a = 2.768169E-5, b = -2.765709E-5;
	ANALOG: Kp = 180; Ki = 3200;
	Ts = 1/40e3;
*/
volatile float a = 2.768169E-5, b = -2.765709E-5;
volatile float w = 1.570796E-2, phi = 0.0, psi = 0.0;

// variable_ : old value
volatile float valpha = 0, vbeta = 0, ialpha=0, ibeta=0;
volatile float vd = 340.0, vq = 0.0, v0 = 0.0, vdf = 0, vqf = 0, v0f = 0;
volatile float vd_ = 340.0, vq_ = 0.0, v0_ = 0.0;

volatile float id=0.0, iq=0.0, i0=0.0, p=0.0, q=0.0;

//PID VARIABLES
volatile int i = 0;
volatile float apq = 3.116319E-3, bpq= -3.114450E-3, ai= 1.788036E1, bi = -1.786963E1; 
volatile float ep = 0.0, eq = 0.0, ep_ = 0.0, eq_ = 0.0, eid=0.0, eiq=0.0, eid_ = 0.0, eiq_ = 0.0;
volatile float pref = 0.0, qref = 0.0, pref_ = 0.0, qref_ = 0.0, pref__ = 0.0, qref__ = 0.0; 
volatile float id_ref = 0.0, iq_ref = 0.0, vdi = 24.2, vqi = 3;
volatile float mH = 0.9, mL = 0.0, deltaH= 3.141593, deltaL=-3.141593;
volatile float delta=0, delta_ = 0, delta__ = 0;
volatile float p_sum = 0, q_sum = 0;
volatile float avci = 2.880203E2, bvci = -2.879797E2, evcii = 0, evcii_ = 0;

// SVPWM VARIABLES
volatile int j = 0;
volatile float m_sum= 0.0, d_sum= 0.0;
volatile float theta = 0;
volatile float m = 0.35, m_ = 0.35, m__ = 0.35, m_norm = 0.0,  theta_ = 0.0, theta__ = 0.0;
volatile float T1 = 0.0, T2 = 0.0, T0 = 0.0, Sa_ = 0.0, Sb_ = 0.0, Sc_ = 0.0;
uint8_t sextant = 0;

// Functions
void PLL_DQ (void);
void pq_ref(void);
void PID (void);
void SVPWM (void);
void PWM_SetDutyCycle(TIM_TypeDef* TIMx, float Duty, uint8_t OCx);
void PWM_DC_Calc(float ma, float theta_);
void Var_Init (void);
void m_delta (void);

void GPIO_INIT (void) {
	
	GPIO_InitTypeDef GPIO_InitStruct_AN; //GPIO Instantiation for ANALOG
	GPIO_InitTypeDef  GPIO_InitStructure; //GPIO Instantiation for OUTPUT
	
	RCC_AHB1PeriphClockCmd (RCC_AHB1Periph_GPIOA, ENABLE);	
	RCC_AHB1PeriphClockCmd (RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_AHB1PeriphClockCmd (RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_AHB1PeriphClockCmd (RCC_AHB1Periph_GPIOE, ENABLE);
	RCC_AHB1PeriphClockCmd (RCC_AHB1Periph_GPIOF, ENABLE);
	
	//Initialize Analog GPIO
	GPIO_InitStruct_AN.GPIO_OType = GPIO_OType_PP;	
	GPIO_InitStruct_AN.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_InitStruct_AN.GPIO_PuPd = GPIO_PuPd_NOPULL ;
	
	GPIO_InitStruct_AN.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStruct_AN.GPIO_Pin = GPIO_Pin_0;
	GPIO_Init(GPIOA, &GPIO_InitStruct_AN);
	GPIO_Init(GPIOB, &GPIO_InitStruct_AN);
	GPIO_Init(GPIOC, &GPIO_InitStruct_AN);
	
	GPIO_InitStruct_AN.GPIO_Pin = GPIO_Pin_3;
	GPIO_Init(GPIOA, &GPIO_InitStruct_AN);
	GPIO_Init(GPIOC, &GPIO_InitStruct_AN);
	GPIO_Init(GPIOF, &GPIO_InitStruct_AN);
	
	GPIO_InitStruct_AN.GPIO_Pin = GPIO_Pin_4;
	GPIO_Init(GPIOC, &GPIO_InitStruct_AN);
	
	GPIO_InitStruct_AN.GPIO_Pin = GPIO_Pin_5;
	GPIO_Init(GPIOF, &GPIO_InitStruct_AN);
	
	GPIO_InitStruct_AN.GPIO_Pin = GPIO_Pin_6;
	GPIO_Init(GPIOA, &GPIO_InitStruct_AN);
	GPIO_Init(GPIOF, &GPIO_InitStruct_AN);
	
	//Initialize OUTPUT GPIO
	/*
	CH1			A8
	CH1N		E8
	CH2			A9
	CH2N		E10
	CH3			E13
	CH3N		E12
	*/
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource8, GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource10, GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource13, GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource12, GPIO_AF_TIM1);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_Init (GPIOA, &GPIO_InitStructure);
	GPIO_ResetBits (GPIOA, GPIO_Pin_8 | GPIO_Pin_9);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_10 | GPIO_Pin_12 | GPIO_Pin_13;
	GPIO_Init (GPIOE, &GPIO_InitStructure);
	GPIO_ResetBits (GPIOE, GPIO_Pin_8 | GPIO_Pin_10 | GPIO_Pin_12 | GPIO_Pin_13);
	
	// INTERCONNECTION RELAY SWITCH
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_Init (GPIOC, &GPIO_InitStructure);
	
	// READ AUTO MANUAL FOR INTERCONNECTION
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
	GPIO_Init (GPIOC, &GPIO_InitStructure);
	
}

void TIM1_INIT (void) {
	
	RCC_APB2PeriphClockCmd (RCC_APB2Periph_TIM1, ENABLE);
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;            
	TIM_TimeBaseStructure.TIM_Prescaler = 0; 
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned3;
	TIM_TimeBaseStructure.TIM_Period = PERIOD; 
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	
	TIM_TimeBaseInit (TIM1, &TIM_TimeBaseStructure);

}

void BDTR_INIT (void) {  
	TIM_OCInitTypeDef TIM_OCStruct;
  TIM_BDTRInitTypeDef TIM_BDTRInitStructure;

  TIM_OCStruct.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCStruct.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCStruct.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCStruct.TIM_OutputNState = TIM_OutputNState_Enable;
	TIM_OCStruct.TIM_OCNPolarity = TIM_OCNPolarity_High; 
	TIM_OCStruct.TIM_Pulse = 0;
	
	TIM_OC1Init(TIM1, &TIM_OCStruct);
  TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
  TIM_OC2Init(TIM1, &TIM_OCStruct);
  TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC3Init(TIM1, &TIM_OCStruct);
  TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
	
	TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
  TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
  TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_OFF;
	// DT = 0x59 or 89 => ~500ns dead time
	// DT = 0x96 or 150 => ~1us dead time
	// DT =      or 110 => ~600ns
	// DT =      or 130 => ~700ns
  TIM_BDTRInitStructure.TIM_DeadTime = 89;
  TIM_BDTRInitStructure.TIM_Break = TIM_Break_Disable;
  TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_Low;
  TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Disable;

	TIM_BDTRConfig(TIM1, &TIM_BDTRInitStructure);
 
  TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_OCMode_PWM1);
	TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);
	TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Enable);
	TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_OCMode_PWM1);
	TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable);
	TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Enable);
	TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_OCMode_PWM1);
	TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable);
	TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Enable);
	
	TIM_CtrlPWMOutputs(TIM1, ENABLE);
	
}

void ADC_INIT (void) {
	
	//ADC Instantiation
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	ADC_InitTypeDef ADC_InitStructure;
	
	//Timer Instantiation
	TIM_TimeBaseInitTypeDef TIM_BaseStruct_ST;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3, ENABLE);	
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	
	//Initialize Timers
	TIM_BaseStruct_ST.TIM_Prescaler = 0;
	TIM_BaseStruct_ST.TIM_CounterMode = TIM_CounterMode_Up;
	// 899 => 100kHz, 2249 => 40kHz, 4499 => 20kHz, 8999 => 10kHz
	TIM_BaseStruct_ST.TIM_Period = ADC_PER; 
	TIM_BaseStruct_ST.TIM_ClockDivision = TIM_CKD_DIV1;
	// Initialize TIM2 
  TIM_TimeBaseInit(TIM2, &TIM_BaseStruct_ST);
	// Enable update interrupt 
	TIM_SelectOutputTrigger(TIM2, TIM_TRGOSource_Update);  //Select output trigger to trigger ADC
	
	//ADC Initialization	
  // ADC Common Init 
  ADC_CommonInitStructure.ADC_Mode = ADC_TripleMode_RegSimult;
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_1;
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
  ADC_CommonInit(&ADC_CommonInitStructure);

  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  ADC_InitStructure.ADC_ScanConvMode = ENABLE; // 3 Channels
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE; // Conversions Triggered
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Rising;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T2_TRGO;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfConversion = 3;
  ADC_Init(ADC1, &ADC_InitStructure);
	
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None; //Master is ADC1
	ADC_Init(ADC2, &ADC_InitStructure);
	ADC_Init(ADC3, &ADC_InitStructure);
	
	ADC_RegularChannelConfig(ADC1, ADC_Channel_13, 	1, ADC_SampleTime_15Cycles); //Va 	PC3	Ch13 ADC1
	ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 	2, ADC_SampleTime_15Cycles); //Ic 	PB0	Ch8 ADC1
	ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 	3, ADC_SampleTime_15Cycles); 	//Vc 	PA6	Ch06 ADC1
	
	ADC_RegularChannelConfig(ADC2, ADC_Channel_0, 	1, ADC_SampleTime_15Cycles); //VCII PA0	Ch00 ADC2
	ADC_RegularChannelConfig(ADC2, ADC_Channel_10, 	2, ADC_SampleTime_15Cycles); //Vb 	PC0	Ch10 ADC2
	ADC_RegularChannelConfig(ADC2, ADC_Channel_14, 	3, ADC_SampleTime_15Cycles); //Q/d 	PC4	Ch14 ADC2
	
	ADC_RegularChannelConfig(ADC3, ADC_Channel_15, 	1, ADC_SampleTime_15Cycles); //Ia 	PF5	Ch15 ADC3
	ADC_RegularChannelConfig(ADC3, ADC_Channel_9, 	2, ADC_SampleTime_15Cycles); //Ib PF3	Ch09 ADC3
	ADC_RegularChannelConfig(ADC3, ADC_Channel_3, 	3, ADC_SampleTime_15Cycles); //P-ma PA3	Ch03 ADC3
	
}

void DMA_INIT (void) {
	
	//DMA Instantiation
	DMA_InitTypeDef DMA_InitStructure;
	NVIC_InitTypeDef ADCNVICConfig;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
	
	DMA_DeInit ( DMA2_Stream4 );
	
	DMA_InitStructure.DMA_Channel = DMA_Channel_0;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC->CDR; //Source address
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)(&adc[0]); //Destination address
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_BufferSize = BUFFERSIZE; //Buffer size
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; //source size - 16bit
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord; // destination size = 16b
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

	DMA_Init(DMA2_Stream4, &DMA_InitStructure); 

	ADCNVICConfig.NVIC_IRQChannel = DMA2_Stream4_IRQn;
	ADCNVICConfig.NVIC_IRQChannelPreemptionPriority = 0;
	ADCNVICConfig.NVIC_IRQChannelSubPriority = 1;
	ADCNVICConfig.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&ADCNVICConfig);
	
}

void ENABLE_PERIPH (void) {
	
	// Start count on TIM2 
  TIM_Cmd(TIM2, ENABLE);
	
	ADC_DMACmd(ADC1, ENABLE);
	ADC_MultiModeDMARequestAfterLastTransferCmd(ENABLE);
		
	TIM_Cmd (TIM1, ENABLE);
	
  ADC_Cmd(ADC1, ENABLE);
	ADC_Cmd(ADC2, ENABLE);
	ADC_Cmd(ADC3, ENABLE);
	
	DMA_Cmd(DMA2_Stream4, ENABLE); //Enable the DMA2 - Stream 4
	DMA_ITConfig(DMA2_Stream4, DMA_IT_TC, ENABLE);
	
}

int main (void) {
	
	SystemInit();
	
	GPIO_INIT();
	TIM1_INIT();
	BDTR_INIT();
	ADC_INIT();
	DMA_INIT();
	ENABLE_PERIPH();
	
	PWM_SetDutyCycle(TIM1, 0, 1);
	PWM_SetDutyCycle(TIM1, 0, 2);
	PWM_SetDutyCycle(TIM1, 0, 3);
	
	while(1);
}

void DMA2_Stream4_IRQHandler(void) {

	if(DMA_GetITStatus(DMA2_Stream4, DMA_IT_TCIF4) != RESET)
	{
		DMA_ClearITPendingBit(DMA2_Stream4, DMA_IT_TCIF4);

		PLL_DQ ();
		
		
		uint8_t am_switch = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_14);
		
   	if (!am_switch) {
			GPIO_ResetBits(GPIOC, GPIO_Pin_13);
			pq_ref();
			PID ();
			//m_delta();
		}
		else if (am_switch) {
			GPIO_SetBits(GPIOC, GPIO_Pin_13);
			Var_Init();
		}
		vcii  = 5.241090e-2f * ((float)adc[1] - 17);
		SVPWM ();	
	}
}


void PLL_DQ (void) {
		/*
		{00, 01, 02, 03, 04, 05, 06, 07, 08}
		{Va, VC, Ia, Ic, Vb, Ib, Vc, Qd, Pm}
		*/
	
		// STEP 0
		va = 1.736402E-1f * (float) (adc[0] - 2117);
		vb = 1.731740E-1f * (float) (adc[4] - 2090);
		vc = 1.752522E-1f * (float) (adc[6] - 2102);
	
		ia = 3.746655E-3f * (float) (adc[2] - 2109);
		ib = 3.826395E-3f * (float) (adc[5] - 2115);
		ic = 3.954506E-3f * (float) (adc[3] - 2187);
		
		// STEP 1
		phi = phi + w;
		phi = fmodf(phi,6.283185f);
		
		// STEP 2
		valpha = 6.666667E-1f * va - 3.333333E-1f * vb - 3.333333E-1f * vc;
		vbeta = 5.773502E-1f * vb - 5.773502E-1f * vc;
		
		vd = sinf(phi) * valpha - cosf(phi) * vbeta;
		vq = cosf(phi) * valpha + sinf(phi) * vbeta;
		v0 = 3.333333E-1f * (va + vb + vc);
		
		// STEP 3
		w = w + (a*vq)+(b*vq_);
	
		psi = phi + 0.5f*w;
		psi = fmodf (psi,6.283185f);
		
		vq_ = vq;
}



void pq_ref (void) {
	
	//pref__ = 2.442002E-1f * (float) adc[8];
	qref__ = 1.221001E-1f * (float)adc[7];
	
	
	
	//p_sum = p_sum + pref__;
	q_sum = q_sum + qref__;
	i +=1;
	
	if (i==100) {
		//pref_ = 0.01f * p_sum;
		qref_ = 0.01f * q_sum;
		//if (((pref_ - pref) > 10.0f ) || ((pref_ - pref) < -10.0f ))
		//pref = pref_;
		if (((qref_ - qref) > 10.0f ) || ((qref_ - qref) < -10.0f ))
		qref = qref_;
		//p_sum = 0.0f;
		q_sum = 0.0f;
		i = 0;
	}
}

void PID(void) {
	// fs=20000 
	
	vcii  = 5.241090e-2f * ((float)adc[1] - 17);
	evcii = vcii - 120;
	pref = pref + ( avci * evcii ) + ( bvci * evcii_ );
	evcii_ = evcii;
	
	ialpha = 6.666667E-1f * ia - 3.333333E-1f * ib - 3.333333E-1f * ic;
	ibeta = 5.773502E-1f * ib - 5.773502E-1f * ic;	
	
	id = sinf(phi) * ialpha - cosf(phi) * ibeta;
	iq = cosf(phi) * ialpha + sinf(phi) * ibeta;
	i0 = 3.333333E-1f * (ia + ib + ic);
	
	p= 1.5f * ( vd * id + vq * iq + 2.0f * v0 * i0 );
	q= 1.5f * ( vq * id - vd * iq );
	
	// a=+Kp+0.5*Ki*Ts, b=-Kp+0.5*Ki*Ts; 
	// P,Q PIs: Kp=0.0405*(1/13), Ki=0.486*(1/13) : a=0.003116319230769, b=-0.003114450000000
	// id_ref = id_ref + a*ep + b*ep_; & iq_ref = iq_ref + a*eq + b*eq_; 
	
	ep = pref - p;
	eq = q - qref;
	id_ref = id_ref + ( apq * ep ) + ( bpq * ep_ );
	iq_ref = iq_ref + ( apq * eq ) + ( bpq * eq_ );
	ep_ = ep;
	eq_ = eq;
	
	// I PIs: Kp=1.375*(13), Ki=16.5*(13) : a=17.880362500000000, b=-17.869637500000000
	//vdi=vdi+ a*eid + b*eid_ & vqi=vqi+ a*eiq + b*eiq_
	
	eid = id_ref - id;
	eiq = iq_ref - iq;
	vdi = vdi + ( ai * eid ) + ( bi * eid_); 
	vqi = vqi + ( ai * eiq ) + ( bi * eiq_);
	eid_ = eid;
	eiq_ = eiq;
	
	//m=(1/120)*sqrt(3)*
	m = 1.443376E-2f * sqrt( vdi*vdi + vqi*vqi );
	if (m>mH)  m=mH;
	if (m<mL)  m=mL; 
		
	if ((vqi != 0) || (vdi != 0))
		delta = atan2(vqi,vdi);  
	
}

void m_delta (void) {
	
	//{00, 01, 02, 03, 04, 05, 06, 07, 08}
	//{Va, VC, Ia, Ic, Vb, Ib, Vc, Qd, Pm}
	m__ = 0.2f + 1.953602E-4f * (float) adc[8];
	delta__ = 2.131049E-4f * (float)adc[7];
	
	m_sum = m_sum + m__;
	d_sum = d_sum + delta__;
	j +=1;
	if (j==100) {
		m_ = 0.01f * m_sum;
		delta_ = 0.01f * d_sum;
		m_sum = 0.0f;
		d_sum = 0.0f;
		j =0;
	}
	if (((m - m_) > 0.01f ) || ((m - m_) < -0.01f ))
		m = m_;
	if (((delta - delta_) > 0.02f ) || ((delta - delta_) < -0.02f ))
		delta = delta_;		
	  
}

void SVPWM(void) {
	theta  = psi + 4.712389f + delta;
	theta=fmodf(theta,6.283185f);
	
	if (theta < 1.047198f) {
		sextant = 1;
		theta_ = theta;
		PWM_DC_Calc(m, theta_);
		PWM_SetDutyCycle(TIM1, Sa_, 1);
		PWM_SetDutyCycle(TIM1, Sb_, 2);
		PWM_SetDutyCycle(TIM1, Sc_, 3);
	}
	else if (theta < 2.094395f) {
		sextant = 2;
		theta_ = 2.094395f - theta;
		PWM_DC_Calc(m, theta_);
		PWM_SetDutyCycle(TIM1, Sb_, 1);
		PWM_SetDutyCycle(TIM1, Sa_, 2);
		PWM_SetDutyCycle(TIM1, Sc_, 3);
	}
	else if (theta < 3.141593f) {
		sextant = 3;
		theta_ = -2.094395f + theta;
		PWM_DC_Calc(m, theta_);
		PWM_SetDutyCycle(TIM1, Sc_, 1);
		PWM_SetDutyCycle(TIM1, Sa_, 2);
		PWM_SetDutyCycle(TIM1, Sb_, 3);
	}
	else if (theta < 4.188790f) {
		sextant = 4;
		theta_ = 4.188790f - theta;
		PWM_DC_Calc(m, theta_);
		PWM_SetDutyCycle(TIM1, Sc_, 1);
		PWM_SetDutyCycle(TIM1, Sb_, 2);
		PWM_SetDutyCycle(TIM1, Sa_, 3);
	}
	else if (theta < 5.235988f) {
		sextant = 5;
		theta_ = -4.188790f + theta;
		PWM_DC_Calc(m, theta_);
		PWM_SetDutyCycle(TIM1, Sb_, 1);
		PWM_SetDutyCycle(TIM1, Sc_, 2);
		PWM_SetDutyCycle(TIM1, Sa_, 3);
	}
	else if (theta < 6.283185f) {
		sextant = 6;
		theta_ = 6.283185f - theta;
		PWM_DC_Calc(m, theta_);
		PWM_SetDutyCycle(TIM1, Sa_, 1);
		PWM_SetDutyCycle(TIM1, Sc_, 2);
		PWM_SetDutyCycle(TIM1, Sb_, 3);
	}
	
}

void PWM_SetDutyCycle(TIM_TypeDef* TIMx, float DUTY, uint8_t OCx) {
	switch(OCx)
	{
	case 1: TIMx->CCR1 = ((uint32_t) DUTY) ; break;
	case 2: TIMx->CCR2 = ((uint32_t) DUTY) ; break;
	case 3: TIMx->CCR3 = ((uint32_t) DUTY) ; break;
	}
}

void PWM_DC_Calc(float m, float theta_) {
	
  theta__ = theta_ + 5.235988E-1f;
	
	T1 = m * cosf(theta__);
	T2 = m * sinf(theta_ );
	T0 = 1 - T1 - T2;

	Sa_ = (T1 + T2 + T0/2) ;
	Sb_ = (T2 + T0/2) ;
	Sc_ = (T0/2) ;
	
	Sa_ = Sa_ * (float) PERIOD;
	Sb_ = Sb_ * (float) PERIOD;
	Sc_ = Sc_ * (float) PERIOD;
}

void Var_Init (void) {
		id_ref = 0;
		iq_ref = 0;
		ep_ = 0;
		eq_ = 0;
		vdi = 24.2 ;
		vqi = 3 ;
		eid_ = 0.0;
		eiq_ = 0.0;
		m= 0.35f;
		delta = 0.0f;
}
