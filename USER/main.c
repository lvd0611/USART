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
	
	// ��������֡
void USART_SendFrame(USART_TypeDef *USART,uint8_t* frame, uint16_t length) {
    for (uint16_t i = 0; i < length; i++) {
        USART_SendData(USART, frame[i]);
        while (USART_GetFlagStatus(USART, USART_FLAG_TXE) == RESET);
    }
}

// ��������֡
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

// �����У��
uint8_t CalculateChecksum(uint8_t* data, uint16_t length) {
    uint8_t sum = 0;
    for (uint16_t i = 0; i < length; i++) {
        sum += data[i];
    }
    return sum;
}

//���ü���������
void SetLaserSwitch(uint8_t sw) {
    uint8_t frame[7] = {0xEF, 0xEF, 0x04, 0x9E, 0x00, sw, 0x00};
    frame[6] = CalculateChecksum(frame, 6);

    // �������ÿ�������
    //USART_SendFrame(frame, 7);

    // ���շ�������
    uint8_t response[7];
    //USART_ReceiveFrame(response, 7);

    // ������������
    if (response[0] == 0xED && response[1] == 0xFA) {
        if (response[3] == 0x9F && response[4] == 0x00 && response[5] == sw) {
            // �ɹ����ü��������أ���תLED
            LED_Toggle();
        }
    }
}

//��ѯ���⿪��
void QueryLaserSwitch(void) {
    uint8_t frame[6] = {0xEF, 0xEF, 0x03, 0x9F, 0x00, 0x00};
    frame[5] = CalculateChecksum(frame, 5);

    // ���Ͳ�ѯ��������
    //USART_SendFrame(frame, 6);

    // ���շ�������
    uint8_t response[7];
    //USART_ReceiveFrame(response, 7);

    // ������������
    if (response[0] == 0xED && response[1] == 0xFA) {
        uint8_t sw = response[5];
        if (sw) {
            // ����������Ϊ��״̬����תLED
            LED_Toggle();
        }
    }
}

//��մ��ڻ�����
void USART_ClearBuffer(uint8_t *buffer, uint16_t length) {
    for(int i = 0; i < length; i++) {
        buffer[i] = 0;
    }
}

// ��ʱ����
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

	    // ��ʼʱ�򿪼�����
   // SetLaserSwitch(1);
    /* Infinite loop */
    while (1){
        // ÿ���ѯһ�μ���������״̬
        //QueryLaserSwitch();
        //Delay(7200000); // ��Լ1�����ʱ������ʱ�����ϵͳʱ��Ƶ�ʵ�����
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

    // ����PA2ΪUSART2 TX
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // ����PA3ΪUSART2 RX
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

// LED ��ʼ��
void LED_Init(void) {
    GPIO_InitTypeDef GPIO_InitStructure;

    // ʹ��GPIOCʱ��
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

    // ����PC13Ϊ�������ģʽ
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    // Ĭ�Ϲر�LED
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

