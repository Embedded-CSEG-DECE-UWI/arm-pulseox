#include <atmel_start.h>
#include <stdio.h>
#include <FreeRTOS.h>
#include <FreeRTOSConfig.h>
#include <semphr.h>
#include <task.h>
#include <list.h>
#include <time.h>
SemaphoreHandle_t xMutex = NULL;
char name[10];

int infraredpeak;
double infraredbpm;
int irdc;

int redpeak;
double redbpm;
int reddc;
void adc_init(void)
{
	REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID_ADC;
	while (GCLK->STATUS.bit.SYNCBUSY);
	/* -------------------------------------------------
	* 1) Enable bus clock to APBC mask
	*/
	REG_PM_APBCMASK |=  PM_APBCMASK_ADC;
	
	/* -------------------------------------------------
	* 3) reset ADC to its initial settings and disable
	*/
	ADC->CTRLA.reg = ADC_CTRLA_SWRST;

	uint32_t bias = (*((uint32_t *) ADC_FUSES_BIASCAL_ADDR) & ADC_FUSES_BIASCAL_Msk) >> ADC_FUSES_BIASCAL_Pos;
	uint32_t linearity = (*((uint32_t *) ADC_FUSES_LINEARITY_0_ADDR) & ADC_FUSES_LINEARITY_0_Msk) >> ADC_FUSES_LINEARITY_0_Pos;
	linearity |= ((*((uint32_t *) ADC_FUSES_LINEARITY_1_ADDR) & ADC_FUSES_LINEARITY_1_Msk) >> ADC_FUSES_LINEARITY_1_Pos) << 5;

	/* Wait for bus synchronization. */
	while (ADC->STATUS.bit.SYNCBUSY) {};
	ADC->CALIB.reg = ADC_CALIB_BIAS_CAL(bias) | ADC_CALIB_LINEARITY_CAL(linearity);
	
	/* ------------------------------------------------- -------------------------------------------------
	* 5) Setup voltage reference 
	*/
	ADC->INPUTCTRL.bit.GAIN = ADC_INPUTCTRL_GAIN_1X_Val;    // Gain Factor Selection
	ADC->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_AREFA_Val; // Vcc voltage reference
	ADC->REFCTRL.bit.REFCOMP = 0;                           //  enable reference compensation
	
	
	ADC->CTRLB.bit.PRESCALER = ADC_CTRLB_PRESCALER_DIV8_Val;
	
	// 10-bit resolution
	ADC->CTRLB.bit.RESSEL = ADC_CTRLB_RESSEL_12BIT_Val;
	
	// no correction enable
	ADC->CTRLB.bit.CORREN = 0;     
	
	// set free running 
	ADC->CTRLB.bit.FREERUN = 1;
	
	// set right Adjustment
	ADC->CTRLB.bit.LEFTADJ = 0;
	
	// turn off differential mode
	ADC->CTRLB.bit.DIFFMODE = 0;

	ADC->WINCTRL.bit.WINMODE = 0;
	
    ADC->INPUTCTRL.reg = ADC_INPUTCTRL_MUXPOS_PIN0 |
    ADC_INPUTCTRL_MUXNEG_GND |
    ADC_INPUTCTRL_GAIN_DIV2;
    
	ADC->CTRLA.reg = ADC_CTRLA_ENABLE;
	
}

int32_t adc_readchannel(uint8_t channel)
{
	// set positive MUX input selection
	ADC->INPUTCTRL.bit.MUXPOS = channel;
	
	
	
	// start conversion then flush
	// ADC will pick up where it left off
	ADC->SWTRIG.reg = ADC_SWTRIG_START ;

	// wait for analog conversion to complete
	while (ADC->INTFLAG.bit.RESRDY == 0)
	{

	}

	// return the result of the ADC
	return ADC->RESULT.reg;
}

void ble_format1(int bpm,int spo2){
	
	printf("\"%s\": {\"SpO2\": %d, \"BPM\": %d},\r\n",name,spo2,bpm);
}
void ble_format2(int bpm,int spo2){
	
	printf("%s: SpO2-%d, BPM-%d\r\n",name,spo2,bpm);
}
void speaker()
{
	REG_PORT_DIR0 |= 1<<21;
	for(int h = 0; h < 5; h++)
	{
		REG_PORT_OUT0 |= 1<<21;
		delay_ms(50);
		REG_PORT_OUT0 &= (~(1<<21));
		delay_ms(50);
	}
	
}
void taskStart(void*p)
{
	
	for(;;)
	{
		if (xSemaphoreTake(xMutex, portMAX_DELAY)==1)
		{
			
			int c;
		
			REG_PORT_OUT0 |= 1 << 21;
			scanf("%10s",name);
			printf("Hello %s\r\n",name);
			while ((c=fgetc(stdin))!='\n'&& c!=EOF);
			

			while (1)
			{
				if(REG_PORT_IN0 & PORT_PA20)
				{
					
					break;
				}
				REG_PORT_OUT0 ^= 1 << 21;
				delay_ms(100);
			}
			REG_PORT_OUT0 |= 1 << 21;

			xSemaphoreGive(xMutex);
		}

		vTaskDelay(pdMS_TO_TICKS(12000));
	
	}
	vTaskDelete(NULL);
}

void taskLED(void*p)
{
		
	for(;;)
	{
		//infrared led
		if (xSemaphoreTake(xMutex, portMAX_DELAY)==1)
		{
			
			ADC->INPUTCTRL.reg |= ADC_INPUTCTRL_MUXPOS_PIN19; 
			

			REG_PORT_OUT1 |= 1<<11;
			delay_ms(500);
		
			int peaksum=0;
			int peakcount=0;
			int ratesum=0;
			int previndx=0;
			int values[150];
			values[0]=adc_readchannel(19);
			for (int i =0;i<150;i++)
			{
				REG_PORT_OUT0 ^= 1 << 21;
				delay_ms(20);
				values[i]=adc_readchannel(19);
			}
			for (int i=1;i<149;i++)
			{
				
				if((values[i-1]<values[i])&&(values[i+1]<values[i])&&(values[i]>1200))
				{
					peaksum=peaksum+values[i];
					if (peakcount>0)
					{
						ratesum=ratesum+((i-previndx));
						
					}
					peakcount=peakcount+1;
					previndx=i;
					
					
				}
			}
			infraredpeak = peaksum/peakcount;
			infraredbpm  =60/((ratesum*0.035)/(peakcount-1));

			
			irdc = adc_readchannel(5);
			irdc = adc_readchannel(5);
			
	
			//red led
			REG_PORT_OUT1 &= (~(1<<11));
			
			REG_PORT_OUT1 |= 1 << 10;
			delay_ms(500);
			
			peaksum=0;
			peakcount=0;
			ratesum=0;
			previndx=0;
			values[0]=adc_readchannel(18);
			for (int i =0;i<150;i++)
			{
				REG_PORT_OUT0 ^= 1 << 21;
				delay_ms(20);
				values[i]=adc_readchannel(18);
				printf("&d, "values[i]);0
				
			}
			for (int i=1;i<149;i++)
			{
				
				if((values[i-1]<values[i])&&(values[i+1]<values[i])&&(values[i]>800))
				{
					peaksum=peaksum+values[i];
					if (peakcount>0)
					{
						ratesum=ratesum+((i-previndx));
						
					}
					peakcount=peakcount+1;
					previndx=i;
					
					
				}
			}
			redpeak = peaksum/peakcount;

			redbpm = 60/((ratesum*0.035)/(peakcount-1));
			
			reddc = adc_readchannel(4);
			reddc = adc_readchannel(4);
			
			REG_PORT_OUT1 &= (~(1<<10));
			REG_PORT_OUT0 &= (~(1<<21));
			xSemaphoreGive(xMutex);
			
			
		}
		vTaskDelay(pdMS_TO_TICKS(12000));

	}
	vTaskDelete(NULL);
}

void taskCalc(void*p)
{
	for(;;)
	{
		int heartrate = (infraredbpm+redbpm)/2;
		int spo2 = (redpeak/reddc)/(infraredpeak/irdc);
		if(REG_PORT_IN0 & PORT_PA16)
		{
			ble_format1(heartrate,spo2);
		}
		else
		{
			ble_format2(heartrate,spo2);
		}
		if (spo2==0){
			speaker();
		}
		vTaskDelay(pdMS_TO_TICKS(12000));

		
	}
	vTaskDelete(NULL);
}


int main(void)
{
	BaseType_t xReturned;
	BaseType_t xReturned2;
	BaseType_t xReturned3;
	
	atmel_start_init();
	adc_init();
	SystemInit();

	REG_PORT_DIR0 |= 1 << 21;
	REG_PORT_DIR1 |= 1 << 10;
	REG_PORT_DIR1 |= 1 << 11;
	REG_PORT_DIR0 |= 0 << 20;
	PORT->Group[0].PINCFG[20].bit.INEN =1;
	REG_PORT_DIR0 |= 0 << 16;
	PORT->Group[0].PINCFG[16].bit.INEN =1;
	REG_PORT_OUT1 |= 0 << 11;
	REG_PORT_OUT1 |= 0 << 10;

	
	xMutex = xSemaphoreCreateMutex();

	xReturned = xTaskCreate(
	taskStart,
	"Task Start",
	1000,
	NULL,
	3,
	NULL
	);
	
	xReturned2 = xTaskCreate(
	taskLED,
	"Task LED",
	1500,
	NULL,
	2,
	NULL
	);
	
	xReturned3 = xTaskCreate(
	taskCalc,
	"Task Calc",
	500,
	NULL,
	1,
	NULL
	);

	

	
	if(xReturned == pdPASS && xReturned2 == pdPASS && xReturned3==pdPASS )
	{
		vTaskStartScheduler();
	}
	while (1) 
	{
		

	}
}
