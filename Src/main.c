#include "string.h"
#include "main.h"
#include "stm32f7xx_hal.h"

/* Definations */
#define SAMPLE_NUM	128				// Number of bytes transmitted from thermal camera 
#define LoopList_Length	8192		
#define MAX_MSG_LENGTH	8
#define CUR_SAMPLE_NUM	500
#define MAX_EVENT_NUM	4
#define EVENT_7	3
#define EVENT_2	0
#define EVENT_3 1
#define EVENT_6	2
#define UP			1
#define DOWN 		0

/* enums */
typedef enum{
	OK										= 0x00,
	Error_Overflow				= 0x07,
	Error_EmptyBuffer			= 0x08
}Error_Type;

typedef enum{
  Idle						= 0x00,
  GetThermal 			= 0x01,
	Start						= 0x02,
	CmdParse				= 0x03,
	ReadCur					= 0x04,
	Reset						= 0x05
}Control_State;

typedef struct{
	uint8_t buffer[LoopList_Length];				
	uint16_t head;
	uint16_t tail;
}rx_loop_list;

typedef struct{
	uint8_t buffer[LoopList_Length];				
	uint16_t head;
	uint16_t tail;
}tx_loop_list;

typedef struct{
	uint8_t event_type;
	uint8_t event_num;
	uint32_t occur_time;
}event_stamp_str;

/* Global variables */
uint8_t ctrl_state;
uint8_t cr_flag = 0;
uint8_t lf_flag = 0;
uint8_t msg_flag = 0;
uint8_t event_flag = 0;
uint8_t thermal_flag = 0;
uint8_t comma_flag = 0;
uint8_t msg_offset = 0;
//uint8_t reset_flag;

uint8_t data_buf[CUR_SAMPLE_NUM * 16];
uint8_t data_r[SAMPLE_NUM];
uint8_t msg_buf[MAX_MSG_LENGTH];
uint8_t event_tx_time[64];
uint8_t data_t[SAMPLE_NUM*4];

uint16_t sample_cnt = 0;
uint16_t event_tx_len;
uint16_t subdex;

uint32_t tick_start;
uint32_t tick_former;
uint32_t tick_duration;
uint32_t read_start;
uint32_t time_stamp ;
uint32_t timer;

tx_loop_list tx_list;
rx_loop_list rx_list;
event_stamp_str event_stamp;
/* System variables */
I2C_HandleTypeDef hi2c3;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart6;
UART_HandleTypeDef huart2;

#ifdef __GNUC__
#define PUTCHAR _PROTOTYPE int __io_putchar(int ch)
	#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
	#endif /* _GNUC__*/
PUTCHAR_PROTOTYPE
{
	HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);
	return ch;
}

/* System function declarations */
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C3_Init(void);

uint8_t CmdStart[] = "Start\r\n";
uint8_t CmdReset[] = "Reset\r\n";
uint8_t cmd12[] = "READ?\r\n";
uint8_t head[] = "{\"thermal\":[";

/* Private function declarations */
uint8_t mystrcmp(uint8_t *str1, uint8_t *str2);
uint8_t mystrlen(uint8_t *str);

uint8_t Cmd_GetThermal(void);
uint8_t Cmd_Start(void);
uint8_t Cmd_Reset(void);
uint8_t Parse(UART_HandleTypeDef *huart);
void Meter_Init(void);
void Read_Current(void);
uint8_t Idling(void);

uint8_t UART_Handler(UART_HandleTypeDef *huart);
uint8_t Add_Txlist(UART_HandleTypeDef *huart,uint8_t *pData, uint16_t Size);
uint8_t Thermal_Data_Transmit(uint8_t* data_i);
uint8_t Thermal_Data_Check(uint8_t* data_i);

/*******************************IRQ functions**********************************************/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
		ctrl_state = GetThermal;
		HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_2);
}

uint8_t Add_Txlist(UART_HandleTypeDef *huart,uint8_t *pData, uint16_t Size){
	uint16_t i;
	
	for(i = 0;i < Size; i++){
				tx_list.buffer[tx_list.tail++] = *pData++;
				if(tx_list.tail > LoopList_Length - 1)
					tx_list.tail = 0;
	}
			
	return OK;
}

uint8_t UART_Handler(UART_HandleTypeDef *huart){
	uint32_t isrflags   = READ_REG(huart->Instance->ISR);
  uint32_t cr1its     = READ_REG(huart->Instance->CR1);
  uint32_t cr3its     = READ_REG(huart->Instance->CR3);
	uint32_t errorflags;

	errorflags = (isrflags & (uint32_t)(USART_ISR_PE | USART_ISR_FE | USART_ISR_ORE | USART_ISR_NE));
	/* If some errors occur */
  if( (errorflags != RESET) && (((cr3its & USART_CR3_EIE) != RESET)|| ((cr1its & (USART_CR1_RXNEIE | USART_CR1_PEIE)) != RESET)) ){
    /* UART parity error interrupt occurred -------------------------------------*/
    if(((isrflags & USART_ISR_PE) != RESET) && ((cr1its & USART_CR1_PEIE) != RESET)){
      __HAL_UART_CLEAR_IT(huart, UART_CLEAR_PEF);
      huart->ErrorCode |= HAL_UART_ERROR_PE;
    }
    /* UART frame error interrupt occurred --------------------------------------*/
    if(((isrflags & USART_ISR_FE) != RESET) && ((cr3its & USART_CR3_EIE) != RESET)){
      __HAL_UART_CLEAR_IT(huart, UART_CLEAR_FEF);

      huart->ErrorCode |= HAL_UART_ERROR_FE;
    }
    /* UART noise error interrupt occurred --------------------------------------*/
    if(((isrflags & USART_ISR_NE) != RESET) && ((cr3its & USART_CR3_EIE) != RESET)){
      __HAL_UART_CLEAR_IT(huart, UART_CLEAR_NEF);

      huart->ErrorCode |= HAL_UART_ERROR_NE;
    }
    
    /* UART Over-Run interrupt occurred -----------------------------------------*/
    if(((isrflags & USART_ISR_ORE) != RESET) &&(((cr1its & USART_CR1_RXNEIE) != RESET) || ((cr3its & USART_CR3_EIE) != RESET))){
      __HAL_UART_CLEAR_IT(huart, UART_CLEAR_OREF);

      huart->ErrorCode |= HAL_UART_ERROR_ORE;
    }
	}
	//Transmit IT
 if(((isrflags & USART_ISR_TXE) != RESET) && ((cr1its & USART_CR1_TXEIE) != RESET)){
			if(huart == &huart2){
	//			if (huart->gState == HAL_UART_STATE_BUSY_TX){
					if(tx_list.head != tx_list.tail){																		 
					huart->Instance->TDR = tx_list.buffer[tx_list.head++];
					if(tx_list.head > LoopList_Length - 1)
						tx_list.head = 0;
					}
					else{
						      CLEAR_BIT(huart->Instance->CR1, USART_CR1_TXEIE);
						//			huart->gState = HAL_UART_STATE_READY;
					}
	//			}
			}
			else if(huart == &huart6){
				if (huart->gState == HAL_UART_STATE_BUSY_TX){
					if(huart->TxXferCount == 0){
					/* Disable the UART Transmit Data Register Empty Interrupt */
					CLEAR_BIT(huart->Instance->CR1, USART_CR1_TXEIE);
					/* Enable the UART Transmit Complete Interrupt */
					SET_BIT(huart->Instance->CR1, USART_CR1_TCIE);
					}
					else{
						huart->TxXferCount--;				 
						huart->Instance->TDR = (uint8_t)(*huart->pTxBuffPtr++ & (uint8_t)0xFFU);	
					}
				}
		}
}

 /* UART in mode Transmitter (transmission end) -----------------------------*/
 else if(((isrflags & USART_ISR_TC) != RESET) && ((cr1its & USART_CR1_TCIE) != RESET)){
      CLEAR_BIT(huart->Instance->CR1, USART_CR1_TCIE);
			huart->gState = HAL_UART_STATE_READY;
 }
	
//Receive IT
  else if(((isrflags & USART_ISR_RXNE) != RESET) && ((cr1its & USART_CR1_RXNEIE) != RESET)){
		if(huart->RxState == HAL_UART_STATE_BUSY_RX){
			if(huart == &huart6)
				rx_list.buffer[rx_list.tail] = (uint8_t)huart->Instance->RDR;	
			else if(huart == &huart2)
				msg_buf[msg_offset++] = (uint8_t)huart->Instance->RDR;
			
			if(rx_list.buffer[rx_list.tail] == ','){
				comma_flag = 1;
				//time stamp
				if(sample_cnt % 4 == 0)
					time_stamp += 3;
				else
					time_stamp += 4;
				sample_cnt++;
			}
			if(rx_list.buffer[rx_list.tail] == '\r'){
					cr_flag = 1;
					rx_list.buffer[rx_list.tail] = ',';
			}
			else if(rx_list.buffer[rx_list.tail] == '\n'){
					lf_flag = 1;
					comma_flag = 1;
					rx_list.buffer[rx_list.tail] = ',';
					//time stamp
					if(sample_cnt % 4 == 0)
						time_stamp += 3;
					else
						time_stamp += 4;
					sample_cnt++;
					
					if(rx_list.tail != 0)
						rx_list.tail--;
					else
						rx_list.tail = LoopList_Length - 1;
			}
			
			if(msg_offset == MAX_MSG_LENGTH - 1)
					msg_flag = 1;
			
			if(huart == &huart6){
				rx_list.tail ++;			
				if(rx_list.tail > LoopList_Length - 1 )
					rx_list.tail = 0;				 
			}
		
		}
			
		else
				__HAL_UART_SEND_REQ(huart, UART_RXDATA_FLUSH_REQUEST);					
	}

	return OK;
}

//启动的时候会有一个三个引脚都接收上拉的过程
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){  
	 if(GPIO_Pin==GPIO_PIN_3){
		event_stamp.occur_time = HAL_GetTick() - tick_start; 
		event_stamp.event_num = EVENT_3;
		 if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_3) == GPIO_PIN_SET)
			 event_stamp.event_type = UP;			
		 else
			  event_stamp.event_type = DOWN;			 
	 }
	 else if(GPIO_Pin==GPIO_PIN_2){
		event_stamp.occur_time = HAL_GetTick() - tick_start; 
		event_stamp.event_num = EVENT_2;
		 if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_2) == GPIO_PIN_SET)
			 event_stamp.event_type = UP;			
		 else
			  event_stamp.event_type = DOWN;	
	 }
	 else if(GPIO_Pin==GPIO_PIN_7){
		event_stamp.occur_time = HAL_GetTick() - tick_start; 
		event_stamp.event_num = EVENT_7;
		 if(HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_7) == GPIO_PIN_SET)
			 event_stamp.event_type = UP;			
		 else
			  event_stamp.event_type = DOWN;
	 }
	 	else if(GPIO_Pin==GPIO_PIN_6){
		event_stamp.occur_time = HAL_GetTick() - tick_start; 
		event_stamp.event_num = EVENT_6;
		 if(HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_6) == GPIO_PIN_SET)
			 event_stamp.event_type = UP;			
		 else
			  event_stamp.event_type = DOWN;	
	 } 
	 
	 event_tx_len = sprintf((char*)event_tx_time,"event:%d,type:%d,time:%d\r\n",event_stamp.event_num,event_stamp.event_type,event_stamp.occur_time);
	 event_flag = 1;
}

/******************************************main**************************************************/
int main(void)
{
  HAL_Init();

  SystemClock_Config();

  MX_GPIO_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_I2C3_Init();
  MX_TIM4_Init();
  MX_USART6_UART_Init();

	rx_list.head = 0;
	rx_list.tail = 0;
	tx_list.head = 0;
	tx_list.tail = 0;
	sample_cnt = 0;
	ctrl_state = Idle;

	//Start receive
  __HAL_LOCK(&huart6);
	SET_BIT(huart6.Instance->CR1,USART_CR1_RXNEIE | USART_CR1_PEIE);
	UART_MASK_COMPUTATION(&huart6);
  huart6.RxState = HAL_UART_STATE_BUSY_RX;
  __HAL_UNLOCK(&huart6);
	
	 __HAL_LOCK(&huart2);
	SET_BIT(huart2.Instance->CR1,USART_CR1_RXNEIE | USART_CR1_PEIE);
	SET_BIT(huart2.Instance->CR3, USART_CR3_EIE);
	UART_MASK_COMPUTATION(&huart2);
  huart2.RxState = HAL_UART_STATE_BUSY_RX;
  __HAL_UNLOCK(&huart2);
	
	Meter_Init();
	
  while (1){
		switch(ctrl_state){
			case GetThermal:	Cmd_GetThermal();break;			
			case Idle:				Idling();break;
			case ReadCur:			Read_Current();break;
			case Start:				Cmd_Start();break;
			case Reset:				Cmd_Reset();break;
			default:					ctrl_state = Idle;break;
		}
  }

}

/**********************functions*********************************************/
uint8_t Cmd_Start(void){
	rx_list.head = 0;
	rx_list.tail = 0;
	tx_list.head = 0;
	tx_list.tail = 0;
	
	time_stamp = 33;
	
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_SET);
	tick_start = HAL_GetTick();
	sample_cnt = 0;
	event_flag = 0;
	thermal_flag = 0;
	HAL_TIM_Base_Start_IT(&htim3);
	
	HAL_UART_Transmit(&huart6,cmd12,mystrlen(cmd12),0x0028);	

	ctrl_state = Idle;
	return OK;
}

uint8_t Cmd_Reset(void){
	ctrl_state = Idle;
	
  HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
  HAL_NVIC_DisableIRQ(EXTI2_IRQn);
  HAL_NVIC_DisableIRQ(EXTI3_IRQn);
	
	HAL_TIM_Base_Stop_IT(&htim3);

	
	CLEAR_BIT(huart6.Instance->CR1, (USART_CR1_RXNEIE | USART_CR1_PEIE));
	huart6.RxState = HAL_UART_STATE_READY;	
	HAL_Delay(2);		// flush output buffer
	
	CLEAR_BIT(huart2.Instance->CR1, USART_CR1_TXEIE);
	tx_list.tail = tx_list.head;
	HAL_NVIC_SystemReset();
	
	return OK;
}

uint8_t Idling(void){
	if(comma_flag == 1){
		Parse(&huart6);
		comma_flag = 0;
	}
	
	if(msg_flag == 1){
		msg_offset = 0;
		msg_flag = 0;
		msg_buf[MAX_MSG_LENGTH - 1] = '\0';
		
		if(mystrcmp(msg_buf,CmdStart) == 0)
			ctrl_state = Start;
		else if(mystrcmp(msg_buf,CmdReset) == 0)
			ctrl_state = Reset;
	}	
	
		if(tx_list.tail != tx_list.head){
			SET_BIT(huart2.Instance->CR1, USART_CR1_TXEIE);
	}			

	return OK;
}


uint8_t Parse(UART_HandleTypeDef *huart){
	uint16_t data_len;
	uint16_t i;
	char time_msg_buf[200];
	uint8_t time_msg_len;
	
	CLEAR_BIT(huart->Instance->CR1, (USART_CR1_RXNEIE | USART_CR1_PEIE));
	huart->RxState = HAL_UART_STATE_READY;
	
	if(huart == &huart6){

		//move current data to tx list
		if(rx_list.tail > rx_list.head){
				data_len = rx_list.tail - rx_list.head;
				//Check if the data is broken. If not, abandon the data
				if(data_len == 16){
					//add time stamp
					time_msg_len = sprintf(time_msg_buf,"%d\r\n",time_stamp);	
					for(i = 0; i < data_len; i++){
						tx_list.buffer[tx_list.tail++] = rx_list.buffer[rx_list.head++];
						if(tx_list.tail > LoopList_Length - 1)
							tx_list.tail = 0;
					}
					for(i = 0;i < time_msg_len; i++){
						tx_list.buffer[tx_list.tail++] = time_msg_buf[i];
						if(tx_list.tail > LoopList_Length - 1)
							tx_list.tail = 0;
					}
				}
		}
			else if(rx_list.tail == rx_list.head){
				return Error_Overflow;
			}		
			else if(rx_list.tail < rx_list.head){
				data_len = rx_list.tail + LoopList_Length - rx_list.head;
				if(data_len == 16){
					//add time stamp
					time_msg_len = sprintf(time_msg_buf,"%d\r\n",time_stamp);
					for(i = rx_list.head; i < LoopList_Length; i++){
						tx_list.buffer[tx_list.tail++] = rx_list.buffer[i];
						if(tx_list.tail > LoopList_Length - 1)
							tx_list.tail = 0;
					}
					for(i = 0;i < rx_list.tail;i++){
						tx_list.buffer[tx_list.tail++] = rx_list.buffer[i];
						if(tx_list.tail > LoopList_Length - 1)
							tx_list.tail = 0;
					}
					for(i = 0;i < time_msg_len; i++){
						tx_list.buffer[tx_list.tail++] = time_msg_buf[i];
						if(tx_list.tail > LoopList_Length - 1)
							tx_list.tail = 0;
					}
				}
			}
		rx_list.head = rx_list.tail ;
			
		if(event_flag == 1){
			Add_Txlist(&huart2,event_tx_time,event_tx_len);
			event_flag = 0;
		}
		if(thermal_flag == 1){
			Add_Txlist(&huart2,head,mystrlen(head));
			Add_Txlist(&huart2,data_t,subdex + 1);
			thermal_flag = 0;
		}
		
		if(lf_flag == 1 && cr_flag == 1){ 
			ctrl_state = ReadCur;					//next circle
			lf_flag = 0;
			cr_flag = 0;
		}

	}
	
	__HAL_LOCK(huart);
	SET_BIT(huart->Instance->CR1, USART_CR1_PEIE | USART_CR1_RXNEIE);
	huart->RxState = HAL_UART_STATE_BUSY_RX;
	__HAL_UNLOCK(huart);
	
	return OK;
}

void Read_Current(void){
	sample_cnt = 0;
	time_stamp = HAL_GetTick() - tick_start + 33;
	HAL_UART_Transmit(&huart6,cmd12,mystrlen(cmd12),0x0028);
	ctrl_state = Idle;
}
/**********************************commands*****************************************/
/* Command Get Thermal Data:
		Allow the PC to fetch a certain time thermal image data at the rate of 10Hz.
		If the device is unable to give valid data in 10s, a timeout error will be reported.
*/
uint8_t Cmd_GetThermal(){
	uint8_t data[SAMPLE_NUM];
	uint8_t valid_flag = 1;
	uint8_t trys = 5;
	
		//fetch data from I2C line
		while(HAL_I2C_Master_Receive(&hi2c3,0xd2,data,SAMPLE_NUM,0x00ff) != HAL_OK && trys != 0){trys--;}
		valid_flag = Thermal_Data_Check(data);
			if(valid_flag == 0){
				Thermal_Data_Transmit(data_r);
			}
		
		ctrl_state = Idle;
	//ctrl_state = ReadCur;
		
		return OK;
}

void Meter_Init(){
	uint8_t cls[] = "*cls\r\n";
	uint8_t cmd1[] = "conf:curr:dc 0.1\r\n";			//range 400mA.¡£¡£¡£¡£¡£¡£¡£¡£¡£¡£¡£¡£¡£
	uint8_t cmd2[] = "curr:dc:nplc 0.2\r\n";			//shortest integration time
//	uint8_t cmd1[] = "conf:volt:dc 3\r\n";
//	uint8_t cmd2[] = "volt:dc:nplc 0.2\r\n";
	uint8_t cmd3[] = "zero:auto 0\r\n";
	uint8_t cmd4[] = "trig:sour imm\r\n";
	uint8_t cmd5[] = "trig:del 0\r\n";
	uint8_t cmd6[] = "trig:coun 1\r\n";
	uint8_t cmd8[] = "syst:rem\r\n";
	uint8_t cmd9[] = "samp:coun 500\r\n";
	
	HAL_UART_Transmit(&huart6,cls,mystrlen(cls),0x00ff);
	HAL_UART_Transmit(&huart6,cmd1,mystrlen(cmd1),0x00ff);
	HAL_UART_Transmit(&huart6,cmd2,mystrlen(cmd2),0x00ff);
	HAL_UART_Transmit(&huart6,cmd3,mystrlen(cmd3),0x00ff);
	HAL_UART_Transmit(&huart6,cmd4,mystrlen(cmd4),0x00ff);
	HAL_UART_Transmit(&huart6,cmd5,mystrlen(cmd5),0x00ff);
	HAL_UART_Transmit(&huart6,cmd6,mystrlen(cmd6),0x00ff);
	HAL_UART_Transmit(&huart6,cmd8,mystrlen(cmd8),0x00ff);
	HAL_UART_Transmit(&huart6,cmd9,mystrlen(cmd9),0x00ff);
	
}

	/* Thermal data check:
	At the start of receiving data for CJMCU, there will be a series of invalid data(0x0000). 
	I call them bad data zone. In addition, there is an invalid data block between 2 valid data blocks for some reason.
	Therefore, if more than 96 data(half of valid data) in a whole block is 0x00, the data block is invalid and should be abandoned.
	I grep 8 data blocks and return the first valid data block.
	*/
uint8_t Thermal_Data_Check(uint8_t* data_i){
	int valid_flag = 1;
	int loop_cnt, zero_cnt = 0;

	for(loop_cnt = 0; loop_cnt < SAMPLE_NUM; loop_cnt ++){
		if(data_i[loop_cnt] == 0x00){
			zero_cnt++;
		}
	}

	if(zero_cnt < 96){		//valid data
			for(loop_cnt = 0; loop_cnt <SAMPLE_NUM; loop_cnt ++){
					data_r[loop_cnt] = data_i[loop_cnt];
		}
			valid_flag = 0;
	}	
	else 
		valid_flag = 1;
			
	return valid_flag;
}

/* Thermal data encoding and transmitting:
		Temperature data of each pixel is 12 bit data and 2 byte data.
		1LSB has 12 bit resolution(11 bit + sign) which is equivalent to 0.25¡æ 
		and it is indicated as tow's complement form. 
		
		e.g.
		temperature				Binary Number		
		+125¡æ						0001_1111_0100
		+25¡æ							0000_0110_0100
		+0.25¡æ						0000_0000_0001
		0¡æ								0000_0000_0000
		-0.25¡æ						1111_1111_1111
		-25¡æ							1111_1001_1100
*/
uint8_t Thermal_Data_Transmit(uint8_t* data_i){
	int offset;
	uint16_t thermal,tmp,thermal_int, thermal_dem;

	subdex = 0;
	for(offset = 0; offset < SAMPLE_NUM; offset += 2){
		//+ or -
		if(data_i[offset + 1] > 1){
			data_t[subdex] = '-';
			subdex ++;
			thermal = ~(data_i[offset] - 1);
		}
		//data bit
		else if(data_i[offset + 1] == 0){
			thermal = data_i[offset];
		}
		else 
			thermal = data_i[offset] + SAMPLE_NUM*4;
		
		//from hundred's digit to percentile digit
		thermal_int = thermal/4;
		thermal_dem = thermal % 4;
		
		tmp = thermal_int/100;
		if(tmp != 0){
			data_t[subdex] = tmp + 48;
			subdex ++;
		}
		
		tmp = thermal_int/10;
		tmp %= 10;
		if(tmp != 0){
			data_t[subdex] = tmp + 48;
			subdex ++;
		}
		
		tmp = thermal_int % 10;
		data_t[subdex] = tmp + 48;
		subdex ++;
	
		if(thermal_dem != 0){
			data_t[subdex] = '.';
			subdex ++;
			thermal_dem = thermal_dem * 25;
			tmp = thermal_dem/10;
			data_t[subdex] = tmp + 48;
			subdex ++;
			tmp = thermal_dem%10;
			data_t[subdex] = tmp + 48;
			subdex ++;
		}
		//end of number
		data_t[subdex] = ',';
		subdex ++;
	}
	//end of a block
	subdex --;
	data_t[subdex] = ']';
	data_t[++subdex] = '}';
	data_t[++subdex] = '\r';
	data_t[++subdex] = '\n';
	
	thermal_flag = 1;
	
	return OK;
}

/**********************************self made functions**********************************/
uint8_t mystrcmp(uint8_t *str1, uint8_t *str2){
	uint8_t i = 0;
	while(str1[i] == str2[i]){
		if(str1[i] != '\0')
			i++;
		else
			return 0;
	}
	return i + 1;
}

uint8_t mystrlen(uint8_t *str){
	uint8_t i = 0;
	
	while(str[i] != '\0'){
		i++;
	}
	
	return i;
}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 108;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Activate the Over-Drive mode 
    */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_USART6
                              |RCC_PERIPHCLK_UART4|RCC_PERIPHCLK_UART5
                              |RCC_PERIPHCLK_UART7|RCC_PERIPHCLK_UART8
                              |RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_I2C2
                              |RCC_PERIPHCLK_I2C3;
  PeriphClkInitStruct.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInitStruct.Usart6ClockSelection = RCC_USART6CLKSOURCE_PCLK2;
  PeriphClkInitStruct.I2c3ClockSelection = RCC_I2C3CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
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

/* I2C3 init function */
static void MX_I2C3_Init(void)
{

  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x6000030D;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure Analogue filter 
    */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure Digital filter 
    */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 1080-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 20000-1;
//	htim3.Init.Period = 4000-1;
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

}

/* TIM4 init function */
static void MX_TIM4_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 10800 - 1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 2000 - 1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }

}

/* USART6 init function */
static void MX_USART6_UART_Init(void)
{

  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  huart6.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart6.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }

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

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC2 PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA2 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PD6 PD7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
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
