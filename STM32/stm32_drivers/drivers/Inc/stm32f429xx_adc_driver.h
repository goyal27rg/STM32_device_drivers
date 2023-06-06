/*
 * stm32f429xx_adc_driver.h
 *
 *  Created on: 21-May-2023
 *      Author: rakshit
 */

#ifndef INC_STM32F429XX_ADC_DRIVER_H_
#define INC_STM32F429XX_ADC_DRIVER_H_


/*
 * Configuration structure for ADC peripheral
 */
typedef struct {
	uint8_t ADC_ConversionMode;
	uint8_t ADC_ScanEnOrDi;
	uint8_t ADC_DataAlignment;
	uint8_t ADC_NumChannels;
	uint8_t* ADC_ConversionSequence;
} ADC_Config_t;


/*
 * Configuration structure for ADC peripheral
 */
typedef struct {
	ADC_RegDef_t *pADCx;
	ADC_Config_t ADC_Config;

} ADC_Handle_t;

/*
 * ADC Conversion modes
 */
#define ADC_CONVERSION_MODE_SINGLE            0
#define ADC_CONVERSION_MODE_CONTINUOUS        1

/*
 * ADC Data Alignment
 */
#define ADC_DATA_ALIGNMENT_RIGHT_ALIGN        0
#define ADC_DATA_ALIGNMENT_LEFT_ALIGN         1

/*
 * ADC Number of Channels
 */
#define ADC_NUM_CHANNELS_1     1
#define ADC_NUM_CHANNELS_2     2
#define ADC_NUM_CHANNELS_3     3
#define ADC_NUM_CHANNELS_4     4
#define ADC_NUM_CHANNELS_5     5
#define ADC_NUM_CHANNELS_6     6
#define ADC_NUM_CHANNELS_7     7
#define ADC_NUM_CHANNELS_8     8
#define ADC_NUM_CHANNELS_9     9
#define ADC_NUM_CHANNELS_10    10
#define ADC_NUM_CHANNELS_11    11
#define ADC_NUM_CHANNELS_12    12
#define ADC_NUM_CHANNELS_13    13
#define ADC_NUM_CHANNELS_14    14
#define ADC_NUM_CHANNELS_15    15
#define ADC_NUM_CHANNELS_16    16


/*
 * ADCx Peripheral clock setup
 */
void ADC_PeriClockControl(ADC_RegDef_t* pADCx, uint8_t EnorDi);


/*
 * ADCx Init and DeInit
 */
void ADC_Init(ADC_Handle_t *pADCHandle);
void ADC_DeInit(ADC_Handle_t *pADCHandle);


/*
 * IRQ Handling
 */

void ADC_IRQITConfig(uint8_t IRQNumber, uint8_t EnorDi);
void ADC_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);

/*
 * Control functions
 */
void ADC_StartConversion(ADC_Handle_t *pADCHandle);
void ADC_StopConversion(ADC_Handle_t *pADCHandle);
uint32_t ADC_ConvertChannel(ADC_Handle_t *pADCHandle, uint8_t ChannelNum);

//Misc functions
uint8_t ADC_GetFlagStatus(ADC_RegDef_t* pADCx, uint8_t flag);

// Event Callback
void ADC_ApplicationEventCallback(ADC_Handle_t *pADCHandle, uint8_t event);


#endif /* INC_STM32F429XX_ADC_DRIVER_H_ */
