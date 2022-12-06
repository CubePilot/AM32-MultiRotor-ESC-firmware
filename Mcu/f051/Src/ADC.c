/*
 * ADC.c
 *
 *  Created on: May 20, 2020
 *      Author: Alka
 */
#include "ADC.h"
#include "comparator.h"
#include <string.h>

uint16_t ADCDataDMA[2*NUM_SAMPLES];


// extern uint16_t ADC_raw_temp;
// extern uint16_t ADC_raw_volts;
// extern uint16_t ADC_raw_current;
// extern uint16_t ADC_raw_input;
// extern uint16_t ADC_raw_bemf[4];
extern uint8_t phase_num;
// uint8_t phase_level;
// uint8_t iszcfound;

extern void interruptRoutine();
extern uint32_t running_time;
extern uint32_t runtime;
// uint32_t lowpct = 30;
// uint32_t highpct = 20;

// uint8_t trig_time;
// uint8_t last_trig_time;
// uint8_t  use_last_trig = 0;
#define MIN_CUTOFF (0x2AA/8)
#define MAX_CUTOFF (0x555*3/2)
// uint32_t dc_voltage = 0;// = 0x555;
// uint8_t irq_counter = 0;
// uint8_t last_phase = 0;
extern uint8_t old_routine;
extern uint8_t bemfcounter;
// uint8_t eos_cnt;

// void ADC_DMA_Callback() {  // read dma buffer and set extern variables
//     dc_voltage = ((0xFF*dc_voltage + (ADCDataDMA[1]) )>>8);
//     const uint8_t phase_checked = 2;
//     if (last_phase != phase_num) {
//         if (LL_ADC_REG_IsConversionOngoing(ADC1)) {
//             LL_ADC_REG_StopConversion(ADC1);
//             while (LL_ADC_REG_IsStopConversionOngoing(ADC1));
//         }
//         if (phase_num == 0) {
//             LL_ADC_REG_SetSequencerChannels(ADC1, LL_ADC_CHANNEL_0 | LL_ADC_CHANNEL_6); // PHASE A
//         } else if (phase_num == 1) {
//             LL_ADC_REG_SetSequencerChannels(ADC1, LL_ADC_CHANNEL_4 | LL_ADC_CHANNEL_6); // PHASE B
//         } else if (phase_num == 2) {
//             LL_ADC_REG_SetSequencerChannels(ADC1, LL_ADC_CHANNEL_5 | LL_ADC_CHANNEL_6); // PHASE C
//         }
//         iszcfound = 0;
//         last_phase = phase_num;
//         LL_ADC_REG_StartConversion(ADC1);
//         return;
//     }
//     // if (phase_num == phase_checked) {
//     //     LL_GPIO_TogglePin(GPIOA, LL_GPIO_PIN_11);
//     // }
//     uint16_t RISING_CUTOFF, FALLING_CUTOFF;
//     RISING_CUTOFF = MIN_CUTOFF;//(dc_voltage>>3);
//     FALLING_CUTOFF = MAX_CUTOFF;//(dc_voltage*3)>>2;

//     if (ADCDataDMA[0] >= RISING_CUTOFF && !iszcfound && rising) {
//         if (phase_num == 0)
//             // LL_GPIO_TogglePin(GPIOA, LL_GPIO_PIN_11);
//         if (running_time >= runtime) {
//             interruptRoutine();
//         }
//         iszcfound = 1;
//         phase_level = 1;
//         // LL_ADC_REG_StopConversion(ADC1);
//         // return;
//     }
//     if (ADCDataDMA[0] <= FALLING_CUTOFF && !iszcfound && !rising) {
//         if (phase_num == 0)
//             // LL_GPIO_TogglePin(GPIOA, LL_GPIO_PIN_11);
//         if (running_time >= runtime) {
//             interruptRoutine();
//         }
//         iszcfound = 1;
//         phase_level = 0;
//         // LL_ADC_REG_StopConversion(ADC1);
//         // return;
//     }
//     // nothing found, look on next on time
//     LL_ADC_REG_StartConversion(ADC1);
//     LL_GPIO_TogglePin(GPIOA, LL_GPIO_PIN_11);
// }

void enableADC_DMA(){    // enables channel

	NVIC_SetPriority(DMA1_Channel1_IRQn, 0);
	NVIC_EnableIRQ(DMA1_Channel1_IRQn);

  LL_DMA_ConfigAddresses(DMA1,
                         LL_DMA_CHANNEL_1,
                         LL_ADC_DMA_GetRegAddr(ADC1, LL_ADC_DMA_REG_REGULAR_DATA),
                         (uint32_t)&ADCDataDMA,
                         LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  /* Set DMA transfer size */
#ifdef USE_ADC_INPUT
  LL_DMA_SetDataLength(DMA1,
                       LL_DMA_CHANNEL_1,
                       2*NUM_SAMPLES);
#else
  LL_DMA_SetDataLength(DMA1,
                       LL_DMA_CHANNEL_1,
                       2*NUM_SAMPLES);

#endif
  /* Enable DMA transfer interruption: transfer complete */
  // LL_DMA_EnableIT_TC(DMA1,
  //                    LL_DMA_CHANNEL_1);

  /* Enable DMA transfer interruption: transfer error */
  LL_DMA_EnableIT_TE(DMA1,
                     LL_DMA_CHANNEL_1);

  /*## Activation of DMA #####################################################*/
  /* Enable the DMA transfer */
  LL_DMA_EnableChannel(DMA1,
                       LL_DMA_CHANNEL_1);
}


void activateADC(){          // called right after enable regular conversions are started by software and DMA interrupt happens at end of transfer

	  __IO uint32_t wait_loop_index = 0;



	    LL_ADC_StartCalibration(ADC1);

	    /* Poll for ADC effectively calibrated */


	    while (LL_ADC_IsCalibrationOnGoing(ADC1) != 0)
	    {

	    }

	    /* Delay between ADC end of calibration and ADC enable.                   */
	    /* Note: Variable divided by 2 to compensate partially                    */
	    /*       CPU processing cycles (depends on compilation optimization).     */
	    wait_loop_index = (LL_ADC_DELAY_CALIB_ENABLE_ADC_CYCLES >> 1);
	    while(wait_loop_index != 0)
	    {
	      wait_loop_index--;
	    }

	    /* Enable ADC */
	    LL_ADC_Enable(ADC1);

	    /* Poll for ADC ready to convert */


	    while (LL_ADC_IsActiveFlag_ADRDY(ADC1) == 0)
	    {

	    }
      LL_ADC_REG_StartConversion(ADC1);
}

void ADC_Init(void)
{
  LL_ADC_InitTypeDef ADC_InitStruct = {0};
  LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_ADC1);

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  /**ADC GPIO Configuration
  PA3   ------> ADC_IN3
  PA6   ------> ADC_IN6
  */

  GPIO_InitStruct.Pin = LL_GPIO_PIN_0;  // Phase A
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_1; // Center
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

#ifdef USE_ADC_INPUT
  GPIO_InitStruct.Pin = LL_GPIO_PIN_2;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
#endif


  GPIO_InitStruct.Pin = LL_GPIO_PIN_3;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);


  GPIO_InitStruct.Pin = LL_GPIO_PIN_4; // Phase B
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);


  GPIO_InitStruct.Pin = LL_GPIO_PIN_5; // Phase C
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);


  GPIO_InitStruct.Pin = LL_GPIO_PIN_6;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);


  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_1, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MODE_CIRCULAR);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PDATAALIGN_HALFWORD);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MDATAALIGN_HALFWORD);


  LL_ADC_REG_SetSequencerChAdd(ADC1, LL_ADC_CHANNEL_0); // PHASE A
  LL_ADC_REG_SetSequencerChAdd(ADC1, LL_ADC_CHANNEL_1); // Center

#ifdef USE_ADC_INPUT
  // LL_ADC_REG_SetSequencerChAdd(ADC1, LL_ADC_CHANNEL_2);
#endif

  // LL_ADC_REG_SetSequencerChAdd(ADC1, LL_ADC_CHANNEL_3);

  // LL_ADC_REG_SetSequencerChAdd(ADC1, LL_ADC_CHANNEL_4); // PHASE B
  // LL_ADC_REG_SetSequencerChAdd(ADC1, LL_ADC_CHANNEL_5); // PHASE C

//   LL_ADC_REG_SetSequencerChAdd(ADC1, LL_ADC_CHANNEL_6);
  /** Configure Regular Channel
  */
  // LL_ADC_REG_SetSequencerChAdd(ADC1, LL_ADC_CHANNEL_TEMPSENSOR);
  /** Configure Internal Channel
  */
  // LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(ADC1), LL_ADC_PATH_INTERNAL_TEMPSENSOR);
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  ADC_InitStruct.Clock = LL_ADC_CLOCK_SYNC_PCLK_DIV4;
  ADC_InitStruct.Resolution = LL_ADC_RESOLUTION_8B;
  ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
  ADC_InitStruct.LowPowerMode = LL_ADC_LP_MODE_NONE;
  LL_ADC_Init(ADC1, &ADC_InitStruct);
//   ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_EXT_TIM1_CH4;
    ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_SOFTWARE;

  ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
  ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_CONTINUOUS;
//   ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_UNLIMITED;
//   ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_SINGLE;
  ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_LIMITED;
  ADC_REG_InitStruct.Overrun = LL_ADC_REG_OVR_DATA_PRESERVED;

  // LL_ADC_REG_SetTriggerEdge(ADC1, LL_ADC_REG_TRIG_EXT_RISING);
  LL_ADC_REG_SetSequencerScanDirection(ADC1, LL_ADC_REG_SEQ_SCAN_DIR_FORWARD);
  LL_ADC_SetSamplingTimeCommonChannels(ADC1, LL_ADC_SAMPLINGTIME_7CYCLES_5);
  LL_ADC_REG_Init(ADC1, &ADC_REG_InitStruct);


    // LL_ADC_SetAnalogWDMonitChannels(ADC1, LL_ADC_AWD_CHANNEL_0_REG);
    // LL_ADC_ConfigAnalogWDThresholds(ADC1, 0x2AA, 0x0);

  LL_ADC_DisableIT_EOC(ADC1);
  LL_ADC_DisableIT_EOS(ADC1);
}

// uint16_t old_data[2][2*NUM_SAMPLES];
uint8_t buf_count = 0;
extern COMP_TypeDef* active_COMP;
void ADC_DMA_Callback()
{
    // for (uint8_t i = 0; i < NUM_SAMPLES; i++) {
    //     dc_voltage = (63*ADCDataDMA[1] + dc_voltage) >> 6;
    // }

    // // copy and sort data array from ADCDataDMA in one go
    // for (uint8_t i = 0; i < NUM_SAMPLES; i++) {
    //     data[i] = ADCDataDMA[i];
    // }
    // // sort data from highest to lowest
    // for (uint8_t i = 0; i < NUM_SAMPLES; i++) {
    //     for (uint8_t j = i + 1; j < NUM_SAMPLES; j++) {
    //         if (data[i] < data[j]) {
    //             uint16_t temp = data[i];
    //             data[i] = data[j];
    //             data[j] = temp;
    //         }
    //     }
    // }


    // if (ADCDataDMA[5] > ADCDataDMA[1]/2) {
    //     LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_11);
    // } else {
    //     LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_11);
    // }

    // if (LL_ADC_IsActiveFlag_AWD1(ADC1)) {
    //     LL_ADC_ClearFlag_AWD1(ADC1);
    //     if (phase_num == 1) {
    //         LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_11);
    //     }
    //     phase_level = 1;
    // } else {
    //     if (phase_num == 1) {
    //         LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_11);
    //     }
    //     phase_level = 0;
    // }

    // LL_ADC_ConfigAnalogWDThresholds(ADC1, (dc_voltage*6)/7, 0);
    // // LL_ADC_ClearFlag_AWD1(ADC1);
    // LL_GPIO_TogglePin(GPIOB, LL_GPIO_PIN_7);
    // if (!LL_COMP_ReadOutputLevel(active_COMP)) {
    //     LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_7);
    // } else {
    //     LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_7);
    // }
    // // add and compare
    // high_sum = 0;
    // low_sum = 0;
    // for (uint8_t i = 1; i < (2*NUM_SAMPLES); i+=2) {
    //     temp = ((ADCDataDMA[i-1] + ADCDataDMA[i+1])>>1);
    //     if (ADCDataDMA[i] > temp) {
    //         high_sum++;
    //     } else if (ADCDataDMA[i] < temp) {
    //         low_sum++;
    //     }
    // }
    // if (high_sum > low_sum) {
    //     LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_11);
    // } else {
    //     LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_11);
    // }
    // LL_GPIO_TogglePin(GPIOA, LL_GPIO_PIN_11);

    // if (buf_count < 2) {
    //     memcpy(old_data[buf_count], ADCDataDMA, sizeof(ADCDataDMA));
    //     buf_count++;
    // }
    // LL_ADC_REG_StartConversion(ADC1);
}
