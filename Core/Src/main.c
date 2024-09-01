/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>

#include "arm_math.h"  
#include "arm_const_structs.h"//���ͷ�ļ�


#include "AD9833.h"
#include "AD9833_2.h"
#include "DSP_MATH.h"
//#include ��dsp/window_functions.h��

#define  FFT_LENGTH        1024        //FFT���ȣ�Ĭ����1024��FFT
#define  SAMPLE_FREQ       102400      //����Ƶ��

//-------TEXT

#define PointMax 500 //�����С��500
#define SCALE_FACTOR (4095 / 3.3)

#define MAX_FFT_RESULTS 100 // ���������СΪ100
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define adc_SIZE 1024// ADC ������С�Ķ���
uint32_t adcConvertValue[adc_SIZE]={0};// ��� ADC �������ݵ�����
int flag=0;// ���λ�����ڱ�ʶ ADC �����Ƿ����
float32_t frequency ;// ���ڴ�ż�������Ƶ�ʱ���
float32_t frequency1 ;// ���ڴ�ż�������Ƶ�ʱ���

float32_t frequency_add;
float32_t frequency1_add[MAX_FFT_RESULTS]; // ���Ƶ�ʽ��������
float32_t frequency1_end;
// FFT ��ز����Ķ���
#define FFT_SIZE 1024
#define FFT_LEN FFT_SIZE 
#define SAMPLING_FREQUENCY 10000
//���������޼���:12M/(12.5+Sampling Time=2.5)= 800k
//����Ƶ��:���Ƶ�ʾ��붨ʱ���������й�;150M/146(73)=1024kHZ
//������λ
#define PHASE_CHANGE_THRESHOLD 1.0 // ��λ�仯��ֵ����λΪ��

float32_t inputSignal[FFT_SIZE*2];// FFT �����ź�����
float32_t fftOutput[FFT_SIZE/2];// FFT �������

float32_t vpp;

int index_;// ��� FFT ��������ֵ������
int index1_;// ��� FFT ��������ֵ������

float phase;//FFT���õĳ�ʼ��λ
double pos_angle;//ת��Ϊ�ӳ�0-2pi��ȫ��Ϊ�����ĽǶ�

float phase2;//FFT���õĳ�ʼ��λ
double pos2_angle;//ת��Ϊ�ӳ�0-2pi��ȫ��Ϊ�����ĽǶ�

float mag_1harmo,mag_3harmo,k;//һ�κ�����г���ϵ����������ֵ
float mag_5harmo,k_1;//���г���ϵ�����
unsigned short harmo3_index,harmo5_index;//�����ź����κ����г��Ƶ�ʶ�Ӧ������ֵ
float mag_1harmo1,mag_3harmo1,k1;//һ�κ�����г���ϵ����������ֵ
float mag_5harmo1;//���г���ϵ�����
unsigned short harmo3_index1,harmo5_index1;//�����ź����κ����г��Ƶ�ʶ�Ӧ������ֵ
//----------------------------------------------------------
//���ڷ���
//֡ͷ
uint8_t  USART2_Head = 0x7D;

//----------------------------------------------------------
//���ڽ���
#define len 6     //�������ֽ���
uint8_t USART2_RX_BUF[len];  //��������
uint8_t rxIndex = 0;
uint8_t frameStartDetected = 0; // ����֡��ʼ��־uint16_t aRxBuffer;			//�����жϻ���
uint16_t aRxBuffer;			//�����жϻ���
uint8_t Uart2_Rx_Cnt = 0;		//���ջ������
//----------------------------------------------------------
//ģʽ   
/*   MODE     Sign_A     Sign_B
      0         ����      ����
			1         ����      ����
*/
uint8_t  MODE = 0x00;
uint8_t  MODE1 = 0x00;


//----------------------------------------------------------
//256�㺺����
 const float hanning_win_table[1024] = 
     {0.000000, 0.000009, 0.000038, 0.000085, 0.000151, 0.000236, 0.000339, 0.000462, 0.000603, 0.000764, 0.000943, 0.001141, 0.001357, 0.001593, 0.001847, 0.002120, 0.002412, 0.002723, 0.003052, 0.003401, 0.003768, 0.004153, 0.004558, 0.004981, 0.005422, 0.005883, 0.006362, 0.006859, 0.007376, 0.007910, 0.008464, 0.009036, 0.009626, 0.010235, 0.010862, 0.011508, 0.012173, 0.012855, 0.013556, 0.014276, 0.015014, 0.015770, 0.016544, 0.017336, 0.018147, 0.018976, 0.019823, 0.020688, 0.021572, 0.022473, 0.023392, 0.024330, 0.025285, 0.026258, 0.027249, 0.028258, 0.029285, 0.030329, 0.031391, 0.032471, 0.033568, 0.034683, 0.035816, 0.036966, 0.038134, 0.039319, 0.040521, 0.041741, 0.042978, 0.044232, 0.045503, 0.046792, 0.048098, 0.049420, 0.050760, 0.052117, 0.053490, 0.054881, 0.056288, 0.057712, 0.059153, 0.060610, 0.062084, 0.063574, 0.065081, 0.066604, 0.068143, 0.069699, 0.071271, 0.072860, 0.074464, 0.076084, 0.077721, 0.079373, 0.081041, 0.082725, 0.084425, 0.086141, 0.087872, 0.089618, 0.091380, 0.093158, 0.094951, 0.096759, 0.098582, 0.100421, 0.102274, 0.104143, 0.106026, 0.107924, 0.109838, 0.111765, 0.113708, 0.115665, 0.117637, 0.119623, 0.121623, 0.123638, 0.125666, 0.127709, 0.129766, 0.131837, 0.133922, 0.136021, 0.138133, 0.140259, 0.142399, 0.144552, 0.146718, 0.148898, 0.151091, 0.153297, 0.155517, 0.157749, 0.159994, 0.162252, 0.164523, 0.166806, 0.169102, 0.171411, 0.173732, 0.176065, 0.178410, 0.180768, 0.183137, 0.185519, 0.187912, 0.190317, 0.192734, 0.195163, 0.197603, 0.200054, 0.202517, 0.204990, 0.207476, 0.209972, 0.212479, 0.214996, 0.217525, 0.220064, 0.222614, 0.225174, 0.227745, 0.230326, 0.232917, 0.235518, 0.238129, 0.240750, 0.243381, 0.246021, 0.248671, 0.251331, 0.254000, 0.256678, 0.259365, 0.262062, 0.264767, 0.267482, 0.270205, 0.272936, 0.275677, 0.278425, 0.281183, 0.283948, 0.286721, 0.289503, 0.292292, 0.295090, 0.297895, 0.300708, 0.303528, 0.306355, 0.309190, 0.312033, 0.314882, 0.317738, 0.320601, 0.323471, 0.326347, 0.329230, 0.332120, 0.335016, 0.337918, 0.340826, 0.343740, 0.346660, 0.349586, 0.352518, 0.355455, 0.358397, 0.361345, 0.364298, 0.367256, 0.370220, 0.373188, 0.376161, 0.379138, 0.382121, 0.385107, 0.388098, 0.391093, 0.394092, 0.397096, 0.400103, 0.403114, 0.406128, 0.409146, 0.412168, 0.415193, 0.418221, 0.421252, 0.424286, 0.427323, 0.430363, 0.433405, 0.436450, 0.439497, 0.442547, 0.445598, 0.448652, 0.451708, 0.454765, 0.457824, 0.460885, 0.463948, 0.467011, 0.470076, 0.473142, 0.476209, 0.479277, 0.482346, 0.485415, 0.488485, 0.491555, 0.494626, 0.497697, 0.500768, 0.503839, 0.506909, 0.509980, 0.513050, 0.516120, 0.519189, 0.522257, 0.525325, 0.528391, 0.531457, 0.534521, 0.537584, 0.540645, 0.543705, 0.546764, 0.549820, 0.552875, 0.555928, 0.558978, 0.562027, 0.565073, 0.568116, 0.571157, 0.574196, 0.577231, 0.580264, 0.583294, 0.586320, 0.589343, 0.592363, 0.595379, 0.598392, 0.601401, 0.604406, 0.607408, 0.610405, 0.613398, 0.616387, 0.619371, 0.622351, 0.625326, 0.628297, 0.631263, 0.634223, 0.637179, 0.640129, 0.643075, 0.646015, 0.648949, 0.651878, 0.654801, 0.657718, 0.660629, 0.663534, 0.666433, 0.669326, 0.672212, 0.675092, 0.677965, 0.680831, 0.683691, 0.686544, 0.689389, 0.692228, 0.695059, 0.697883, 0.700700, 0.703509, 0.706310, 0.709103, 0.711889, 0.714666, 0.717436, 0.720197, 0.722950, 0.725695, 0.728431, 0.731158, 0.733877, 0.736587, 0.739288, 0.741979, 0.744662, 0.747336, 0.750000, 0.752655, 0.755300, 0.757936, 0.760562, 0.763178, 0.765784, 0.768380, 0.770966, 0.773542, 0.776107, 0.778662, 0.781207, 0.783741, 0.786264, 0.788776, 0.791278, 0.793768, 0.796248, 0.798716, 0.801173, 0.803619, 0.806053, 0.808476, 0.810887, 0.813286, 0.815673, 0.818049, 0.820413, 0.822764, 0.825103, 0.827430, 0.829745, 0.832047, 0.834337, 0.836614, 0.838879, 0.841130, 0.843369, 0.845595, 0.847808, 0.850007, 0.852194, 0.854367, 0.856527, 0.858673, 0.860806, 0.862925, 0.865030, 0.867122, 0.869200, 0.871264, 0.873314, 0.875350, 0.877372, 0.879379, 0.881372, 0.883351, 0.885315, 0.887265, 0.889200, 0.891121, 0.893027, 0.894917, 0.896793, 0.898655, 0.900501, 0.902331, 0.904147, 0.905948, 0.907733, 0.909503, 0.911257, 0.912996, 0.914719, 0.916427, 0.918119, 0.919795, 0.921455, 0.923099, 0.924728, 0.926340, 0.927936, 0.929517, 0.931081, 0.932628, 0.934160, 0.935675, 0.937173, 0.938655, 0.940121, 0.941570, 0.943002, 0.944418, 0.945817, 0.947199, 0.948564, 0.949912, 0.951243, 0.952557, 0.953854, 0.955134, 0.956397, 0.957643, 0.958871, 0.960082, 0.961276, 0.962452, 0.963611, 0.964752, 0.965876, 0.966983, 0.968071, 0.969142, 0.970195, 0.971231, 0.972249, 0.973249, 0.974231, 0.975195, 0.976141, 0.977070, 0.977980, 0.978872, 0.979746, 0.980603, 0.981441, 0.982260, 0.983062, 0.983846, 0.984611, 0.985358, 0.986086, 0.986796, 0.987488, 0.988162, 0.988817, 0.989454, 0.990072, 0.990671, 0.991253, 0.991815, 0.992359, 0.992885, 0.993392, 0.993880, 0.994350, 0.994801, 0.995233, 0.995647, 0.996042, 0.996418, 0.996776, 0.997115, 0.997435, 0.997736, 0.998018, 0.998282, 0.998527, 0.998753, 0.998961, 0.999149, 0.999319, 0.999470, 0.999602, 0.999715, 0.999809, 0.999884, 0.999941, 0.999979, 0.999998, 0.999998, 0.999979, 0.999941, 0.999884, 0.999809, 0.999715, 0.999602, 0.999470, 0.999319, 0.999149, 0.998961, 0.998753, 0.998527, 0.998282, 0.998018, 0.997736, 0.997435, 0.997115, 0.996776, 0.996418, 0.996042, 0.995647, 0.995233, 0.994801, 0.994350, 0.993880, 0.993392, 0.992885, 0.992359, 0.991815, 0.991253, 0.990671, 0.990072, 0.989454, 0.988817, 0.988162, 0.987488, 0.986796, 0.986086, 0.985358, 0.984611, 0.983846, 0.983062, 0.982260, 0.981441, 0.980603, 0.979746, 0.978872, 0.977980, 0.977070, 0.976141, 0.975195, 0.974231, 0.973249, 0.972249, 0.971231, 0.970195, 0.969142, 0.968071, 0.966983, 0.965876, 0.964752, 0.963611, 0.962452, 0.961276, 0.960082, 0.958871, 0.957643, 0.956397, 0.955134, 0.953854, 0.952557, 0.951243, 0.949912, 0.948564, 0.947199, 0.945817, 0.944418, 0.943002, 0.941570, 0.940121, 0.938655, 0.937173, 0.935675, 0.934160, 0.932628, 0.931081, 0.929517, 0.927936, 0.926340, 0.924728, 0.923099, 0.921455, 0.919795, 0.918119, 0.916427, 0.914719, 0.912996, 0.911257, 0.909503, 0.907733, 0.905948, 0.904147, 0.902331, 0.900501, 0.898655, 0.896793, 0.894917, 0.893027, 0.891121, 0.889200, 0.887265, 0.885315, 0.883351, 0.881372, 0.879379, 0.877372, 0.875350, 0.873314, 0.871264, 0.869200, 0.867122, 0.865030, 0.862925, 0.860806, 0.858673, 0.856527, 0.854367, 0.852194, 0.850007, 0.847808, 0.845595, 0.843369, 0.841130, 0.838879, 0.836614, 0.834337, 0.832047, 0.829745, 0.827430, 0.825103, 0.822764, 0.820413, 0.818049, 0.815673, 0.813286, 0.810887, 0.808476, 0.806053, 0.803619, 0.801173, 0.798716, 0.796248, 0.793768, 0.791278, 0.788776, 0.786264, 0.783741, 0.781207, 0.778662, 0.776107, 0.773542, 0.770966, 0.768380, 0.765784, 0.763178, 0.760562, 0.757936, 0.755300, 0.752655, 0.750000, 0.747336, 0.744662, 0.741979, 0.739288, 0.736587, 0.733877, 0.731158, 0.728431, 0.725695, 0.722950, 0.720197, 0.717436, 0.714666, 0.711889, 0.709103, 0.706310, 0.703509, 0.700700, 0.697883, 0.695059, 0.692228, 0.689389, 0.686544, 0.683691, 0.680831, 0.677965, 0.675092, 0.672212, 0.669326, 0.666433, 0.663534, 0.660629, 0.657718, 0.654801, 0.651878, 0.648949, 0.646015, 0.643075, 0.640129, 0.637179, 0.634223, 0.631263, 0.628297, 0.625326, 0.622351, 0.619371, 0.616387, 0.613398, 0.610405, 0.607408, 0.604406, 0.601401, 0.598392, 0.595379, 0.592363, 0.589343, 0.586320, 0.583294, 0.580264, 0.577231, 0.574196, 0.571157, 0.568116, 0.565073, 0.562027, 0.558978, 0.555928, 0.552875, 0.549820, 0.546764, 0.543705, 0.540645, 0.537584, 0.534521, 0.531457, 0.528391, 0.525325, 0.522257, 0.519189, 0.516120, 0.513050, 0.509980, 0.506909, 0.503839, 0.500768, 0.497697, 0.494626, 0.491555, 0.488485, 0.485415, 0.482346, 0.479277, 0.476209, 0.473142, 0.470076, 0.467011, 0.463948, 0.460885, 0.457824, 0.454765, 0.451708, 0.448652, 0.445598, 0.442547, 0.439497, 0.436450, 0.433405, 0.430363, 0.427323, 0.424286, 0.421252, 0.418221, 0.415193, 0.412168, 0.409146, 0.406128, 0.403114, 0.400103, 0.397096, 0.394092, 0.391093, 0.388098, 0.385107, 0.382121, 0.379138, 0.376161, 0.373188, 0.370220, 0.367256, 0.364298, 0.361345, 0.358397, 0.355455, 0.352518, 0.349586, 0.346660, 0.343740, 0.340826, 0.337918, 0.335016, 0.332120, 0.329230, 0.326347, 0.323471, 0.320601, 0.317738, 0.314882, 0.312033, 0.309190, 0.306355, 0.303528, 0.300708, 0.297895, 0.295090, 0.292292, 0.289503, 0.286721, 0.283948, 0.281183, 0.278425, 0.275677, 0.272936, 0.270205, 0.267482, 0.264767, 0.262062, 0.259365, 0.256678, 0.254000, 0.251331, 0.248671, 0.246021, 0.243381, 0.240750, 0.238129, 0.235518, 0.232917, 0.230326, 0.227745, 0.225174, 0.222614, 0.220064, 0.217525, 0.214996, 0.212479, 0.209972, 0.207476, 0.204990, 0.202517, 0.200054, 0.197603, 0.195163, 0.192734, 0.190317, 0.187912, 0.185519, 0.183137, 0.180768, 0.178410, 0.176065, 0.173732, 0.171411, 0.169102, 0.166806, 0.164523, 0.162252, 0.159994, 0.157749, 0.155517, 0.153297, 0.151091, 0.148898, 0.146718, 0.144552, 0.142399, 0.140259, 0.138133, 0.136021, 0.133922, 0.131837, 0.129766, 0.127709, 0.125666, 0.123638, 0.121623, 0.119623, 0.117637, 0.115665, 0.113708, 0.111765, 0.109838, 0.107924, 0.106026, 0.104143, 0.102274, 0.100421, 0.098582, 0.096759, 0.094951, 0.093158, 0.091380, 0.089618, 0.087872, 0.086141, 0.084425, 0.082725, 0.081041, 0.079373, 0.077721, 0.076084, 0.074464, 0.072860, 0.071271, 0.069699, 0.068143, 0.066604, 0.065081, 0.063574, 0.062084, 0.060610, 0.059153, 0.057712, 0.056288, 0.054881, 0.053490, 0.052117, 0.050760, 0.049420, 0.048098, 0.046792, 0.045503, 0.044232, 0.042978, 0.041741, 0.040521, 0.039319, 0.038134, 0.036966, 0.035816, 0.034683, 0.033568, 0.032471, 0.031391, 0.030329, 0.029285, 0.028258, 0.027249, 0.026258, 0.025285, 0.024330, 0.023392, 0.022473, 0.021572, 0.020688, 0.019823, 0.018976, 0.018147, 0.017336, 0.016544, 0.015770, 0.015014, 0.014276, 0.013556, 0.012855, 0.012173, 0.011508, 0.010862, 0.010235, 0.009626, 0.009036, 0.008464, 0.007910, 0.007376, 0.006859, 0.006362, 0.005883, 0.005422, 0.004981, 0.004558, 0.004153, 0.003768, 0.003401, 0.003052, 0.002723, 0.002412, 0.002120, 0.001847, 0.001593, 0.001357, 0.001141, 0.000943, 0.000764, 0.000603, 0.000462, 0.000339, 0.000236, 0.000151, 0.000085, 0.000038, 0.000009, 0.000000, 
		 };

		 
/* 
AdcConvEnd�������ADC�Ƿ�ɼ����
0��û�вɼ����
1���ɼ���ϣ���stm32f1xx_it���DMA����жϽ����޸�
 */
__IO uint8_t AdcConvEnd = 0;
__IO uint8_t do_ad_flag1 = 0;
__IO uint8_t do_ad_flag2 = 0;
//uint8_t do_ad_flag1=0;
		 


float currentPhase2 ; // ��ǰ��λ�������FFT�л�ȡ
int phase2_index = 0; // ������������
int fre2_index = 0; // ������������
float error2;
float phaseAdjustment2 ;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


int fputc(int ch, FILE *f)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xffff);
	HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xffff);
  return ch;
}

int fgetc(FILE *f)
{
  uint8_t ch = 0;
  HAL_UART_Receive(&huart2, &ch, 1, 0xffff);
	HAL_UART_Receive(&huart1, &ch, 1, 0xffff);
  return ch;
}
uint8_t T_data[6]={0x7D,0x00,0x00,0x00,0x00,0x00};
uint16_t T_datasize =sizeof(T_data)/sizeof(T_data[0]);

//-------------------------------------------
//��λ

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_ADC2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

//-----------------------------------------------------------------
////����������Σ��䵱�źŷ�����
//get_sin_tab(*sinData);

//������ʱ��
	HAL_TIM_Base_Start(&htim3);

	HAL_ADC_Start_DMA(&hadc2, (uint32_t *)adcConvertValue,adc_SIZE);
	while (!AdcConvEnd)                                   //�ȴ�ת�����
    ;

//	for(int i=0;i<1023;i++){printf("%d\n",adcConvertValue[i]);}
	fftCalculate();
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//---------------------------------------------------------------------------
//AD9833D/A���A'��B'
//    HAL_GPIO_WritePin(LED_TEST_GPIO_Port,LED_TEST_Pin,0);
//		HAL_Delay(500);
		
    HAL_ADC_Start_DMA(&hadc2, (uint32_t *)adcConvertValue,adc_SIZE); //����ADC��DMA
		while (!AdcConvEnd)                                   //�ȴ�ת�����
    ;
//		for(int i=0;i<1023;i++){printf("%d\n",adcConvertValue[i]);}
		fftCalculate();



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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 75;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void fftCalculate(void)// FFT ���㺯��
{

     float temp_float;


				//// ����ƽ��ֵ��ȥ�������źŵ�ֱ���ɷ�
			float sum = 0;
			for (int i = 0; i < FFT_LENGTH; i++)
						{
								adcConvertValue[i] = adcConvertValue[i]*3.3/4096.0;
						}
			for (int i = 0; i < FFT_LENGTH; i++)
						{
								sum += adcConvertValue[i];
						}
						
			float average;
			average = sum / FFT_LENGTH;
      
//			for (int i = 0; i < FFT_LENGTH; i++) 
//						{
//								inputSignal[2 * i] -= average;
//						}
						
		for(int j=0;j<FFT_SIZE;j++)//��������ת��
		{

			inputSignal[j*2]=(adcConvertValue[j]- average);//12λ��������ֵ��ӦΪ0-3.3��	*3.3/4096.0 /10000000
		
			inputSignal[j*2+1]=0;//������㣬������ݵ��鲿

		}						
		
		for (int i = 0; i < FFT_LENGTH; i++) 
					{
							inputSignal[2 * i] *= hanning_win_table[i]; // ʵ�����Ժ�����
					}
								
    arm_cfft_f32(&arm_cfft_sR_f32_len1024, inputSignal, 0, 1);// ִ�� FFT ����
    arm_cmplx_mag_f32(inputSignal, fftOutput, FFT_LEN/2);// ���� FFT ����ķ���
					fftOutput[0]=fftOutput[0]/102400;
					fftOutput[1]=fftOutput[1]/102400;
					fftOutput[2]=fftOutput[2]/102400;
			 for(int i = 1; i < 512 ; i++)//FFT��������������ҶԳƵģ�ֻ��Ҫ��һ��
         {fftOutput[i] = fftOutput[i]/51200;}	
					
//-----------------------------------------------------------------------------------------------------
//ģ������  fA ��fB 
//		fftOutput[50]=	1.9;	//PA5  index1_  ���Ƚ�С��ΪfA
//		fftOutput[90]=	1.95;	//PA4  index_   ���Ƚϴ��ΪfB
//					

		index_ = 0;// ���� FFT ����е����ֵ
		index1_ = 0;// ���� FFT ����е����ֵ
    float32_t maxValue = fftOutput[1];
    float32_t maxValue1 = fftOutput[1];
//		arm_max_f32(&fftOutput[1], FFT_LEN/2, &maxValue, &index_); // ʹ�� arm_max_f32 ���������ҵ� FFT ����е����ֵ��������

		for(int j=0;j<6;j++)//��������ת��
		{

			fftOutput[j]= 0.0000000000f;                                 //Ϊ�˷����ҵ����ֵ

		}		
//    for (uint32_t i = 1; i < FFT_LEN/2; i++)
//    {

//       printf("%.3f\n", fftOutput[i]); 
//    }
		findTwoMaxPeaks(fftOutput, FFT_LEN/2, &maxValue, &index_, &maxValue1, &index1_);



//		printf("Peak:%.3f V\n",maxValue); //���ݴ�ӡ���鿴���Ƚ��
//		frequency = (float32_t)index_ * (float32_t)SAMPLE_FREQ / (float32_t)FFT_SIZE /1000;// �������ֵ�����������źŵ�Ƶ��  (float32_t)SAMPLING_FREQUENCY / (float32_t)FFT_SIZE=1k              
//    if(frequency<=1.5){frequency = 0;}
		
		frequency1 = (float32_t)index1_ * (float32_t)SAMPLE_FREQ / (float32_t)FFT_SIZE /1000;// �������ֵ�����������źŵ�Ƶ��  (float32_t)SAMPLING_FREQUENCY / (float32_t)FFT_SIZE=1k              
    frequency1 = (frequency1-0.8999)*1000;//����
		if(frequency1<0)frequency1=0;
		printf("%f\n",frequency1);
		
		if(frequency1>0.2 && fre2_index<99)
		{
    // ��Ƶ�ʴ�������
    frequency1_add[fre2_index] = frequency1;
    fre2_index++; // ��������λ�ã�׼�������һ��Ƶ��ֵ
		}
//		printf("index_:%d \n", index_);//�������
//    printf("frequencyMAX:%.3f kHz\r\n", frequency);//���Ƶ��

//		printf("%3d", index_);
		
//		printf("Peak1:%.3f V\n",maxValue1); //���ݴ�ӡ���鿴���* 3.3 / 4095.00
//		frequency1 = (float32_t)index1_ * (float32_t)SAMPLING_FREQUENCY / (float32_t)FFT_SIZE /1000;// �������ֵ�����������źŵ�Ƶ��  (float32_t)SAMPLING_FREQUENCY / (float32_t)FFT_SIZE=1k 



}



/**
  * ��������: �����ⲿ�жϻص�����
  * �������: GPIO_Pin���ж�����
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
 // �����жϻص�����

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
//    HAL_Delay(50); // ���������ӳ�


    if (HAL_GPIO_ReadPin(GPIOB, GPIO_Pin) == GPIO_PIN_SET)
    {

    switch (GPIO_Pin)
    {
        case GPIO_PIN_1:
//            do_ad_flag1 = 1;//��ȫ�ֱ�־��λ����ʾ����ͬ����ʼADת����
//            do_ad_flag2 = 1;
				flag=1;
            break;

            break;

    }
  	}
		
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);
  /* NOTE: This function Should not be modified, when the callback is needed,
           the HAL_UART_TxCpltCallback could be implemented in the user file
   */
	if(Uart2_Rx_Cnt >= 100)  //����ж�
	{
		Uart2_Rx_Cnt = 0;
		memset(USART2_RX_BUF,0x00,sizeof(USART2_RX_BUF));
		HAL_UART_Transmit(&huart1, (uint8_t *)"ERROR", 10,0xFFFF); 	
        
	}
	else
	{

 // ����Ƿ���֡��ʼ
        if (!frameStartDetected && aRxBuffer == USART2_Head) {
            frameStartDetected = 1; // ����֡��ʼ��־
            USART2_RX_BUF[rxIndex] = aRxBuffer; // �洢֡ͷ
        } 
				else if (frameStartDetected) {
            // �洢ʣ������
					rxIndex++;
            USART2_RX_BUF[rxIndex] = aRxBuffer;
            
            // ����Ƿ��������������
            if (rxIndex  >= 5) {
               // ���ý���������֡��ʼ��־
               frameStartDetected = 0;
               rxIndex = 0;
 

//								// ��ӡ������������֤
//								printf("GUSART2_RX_BUF��1��: %d\r\n", USART2_RX_BUF[1]);
//								printf("USART2_RX_BUF��2��: %d\r\n", USART2_RX_BUF[2]);
//								printf("USART2_RX_BUF��3��: %d\r\n", USART2_RX_BUF[3]);
//							
//							  printf("USART2_RX_BUF��4��: %x\r\n", do_ad_flag1);

										}
        }
        



	}


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
  __disable_irq();
  while (1)
  {
  }
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
