/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include <stdio.h>
#include <stdbool.h>

#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

/* Private variables ---------------------------------------------------------*/
#define USART_RX_BUF_SIZE 60
uint8_t USART_RX_BUF[USART_RX_BUF_SIZE];
volatile int USART_RX_STA = 0;
volatile bool USART_RX_COMPLETE = false;

/* Private function prototypes -----------------------------------------------*/
void USART_Configuration(void);
void USART2_Configuration(void);
void LED_Init(void);
void LED_Toggle(void);
	
	// 发送数据帧
void USART_SendFrame(USART_TypeDef *USART,uint8_t* frame, uint16_t length) {
    for (uint16_t i = 0; i < length; i++) {
        USART_SendData(USART, frame[i]);
        while (USART_GetFlagStatus(USART, USART_FLAG_TXE) == RESET);
    }
}

// 接收数据帧
void USART_ReceiveFrame(USART_TypeDef *USART,uint8_t* buffer, uint16_t length) {
    volatile uint16_t i = 0;
    for (uint16_t i = 0; i < length; i++) {
        while (USART_GetFlagStatus(USART, USART_FLAG_RXNE) == RESET){
        i++;
        if(i>10000) return;
        }
        buffer[i] = USART_ReceiveData(USART);
    }
}

// 计算和校验
uint8_t CalculateChecksum(uint8_t* data, uint16_t length) {
    uint8_t sum = 0;
    for (uint16_t i = 0; i < length; i++) {
        sum += data[i];
    }
    return sum;
}

//设置激光器开关
void SetLaserSwitch(uint8_t sw) {
    uint8_t frame[7] = {0xEF, 0xEF, 0x04, 0x9E, 0x00, sw, 0x00};
    frame[6] = CalculateChecksum(frame, 6);

    // 发送设置开关命令
    //USART_SendFrame(frame, 7);

    // 接收返回数据
    uint8_t response[7];
    //USART_ReceiveFrame(response, 7);

    // 解析返回数据
    if (response[0] == 0xED && response[1] == 0xFA) {
        if (response[3] == 0x9F && response[4] == 0x00 && response[5] == sw) {
            // 成功设置激光器开关，翻转LED
            LED_Toggle();
        }
    }
}

//查询激光开关
void QueryLaserSwitch(void) {
    uint8_t frame[6] = {0xEF, 0xEF, 0x03, 0x9F, 0x00, 0x00};
    frame[5] = CalculateChecksum(frame, 5);

    // 发送查询开关命令
    //USART_SendFrame(frame, 6);

    // 接收返回数据
    uint8_t response[7];
    //USART_ReceiveFrame(response, 7);

    // 解析返回数据
    if (response[0] == 0xED && response[1] == 0xFA) {
        uint8_t sw = response[5];
        if (sw) {
            // 激光器开关为打开状态，翻转LED
            LED_Toggle();
        }
    }
}

//清空串口缓冲区
void USART_ClearBuffer(uint8_t *buffer, uint16_t length) {
    for(int i = 0; i < length; i++) {
        buffer[i] = 0;
    }
}

// 延时函数
void Delay(uint32_t count) {
    for (; count != 0; count--);
}

void LED_Toggle(void) {
    static uint8_t led_state = 0;
    GPIO_WriteBit(GPIOC, GPIO_Pin_13, led_state);
    led_state = ~led_state;
}


/*******************************************************************************
* Function Name  : main
* Description    : Main program
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
int main(void)
{
	USART_Configuration();
	USART2_Configuration();
	LED_Init();

	    // 初始时打开激光器
   // SetLaserSwitch(1);
    /* Infinite loop */
    while (1){
        // 每秒查询一次激光器开关状态
        //QueryLaserSwitch();
        //Delay(7200000); // 大约1秒的延时（具体时间根据系统时钟频率调整）
       //LED_Toggle();
       if(USART_RX_COMPLETE) {
            USART_SendFrame(USART2, USART_RX_BUF, USART_RX_STA);
            USART_ClearBuffer(USART_RX_BUF, USART_RX_STA);
            USART_RX_STA = 0;
            USART_RX_COMPLETE = false;
            
        }
    }
}


/*******************************************************************************
* Function Name  : USART_Configuration
* Description    : Configure USART1 
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void USART_Configuration(void)
{ 
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure; 
  NVIC_InitTypeDef NVIC_InitStructure;

  RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1,ENABLE);
  /*
  *  USART1_TX -> PA9 , USART1_RX ->	PA10
  */				
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;	         
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
  GPIO_Init(GPIOA, &GPIO_InitStructure);		   

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;	        
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  USART_Init(USART1, &USART_InitStructure); 

    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  
    NVIC_Init(&NVIC_InitStructure);



	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);


  USART_Cmd(USART1, ENABLE);
}

void USART2_Configuration(void)
{ 
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure; 

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA , ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    // 配置PA2为USART2 TX
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // 配置PA3为USART2 RX
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    USART_Init(USART2, &USART_InitStructure); 
    USART_Cmd(USART2, ENABLE);
}

// LED 初始化
void LED_Init(void) {
    GPIO_InitTypeDef GPIO_InitStructure;

    // 使能GPIOC时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

    // 配置PC13为推挽输出模式
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    // 默认关闭LED
    GPIO_SetBits(GPIOC, GPIO_Pin_13);
}

void USART1_IRQHandler(void)
{
    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
        USART_RX_BUF[USART_RX_STA] = USART_ReceiveData(USART1);
        USART_RX_STA++;
        if(USART_RX_STA >= USART_RX_BUF_SIZE) {
            USART_RX_STA = 0;
        }
    }

    if(USART_GetITStatus(USART1, USART_IT_IDLE) != RESET) {
        USART_ReceiveData(USART1);
        USART_RX_COMPLETE = true;
    }
     
}


/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART */
  USART_SendData(USART1, (uint8_t) ch);

  /* Loop until the end of transmission */
  while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
  {}

  return ch;
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/*********************************************************************************************************
      END FILE
*********************************************************************************************************/

