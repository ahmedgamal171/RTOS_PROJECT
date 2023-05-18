#include <stdint.h>
#include "tm4c123gh6pm.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

#include "queue.h"
#include "semphr.h"

#include <math.h>

//______

#include "TM4C123.h"    // Device header

#define LCD GPIOB    		//LCD port with Tiva C 
#define RS 0x01				 	//RS -> PB0 (0x01)
#define RW 0x02         //RW -> PB1 (0x02)
#define EN 0x04  		 	 	//EN -> PB2 (0x04)


//Functions Declaration
void delayUs(int);   														   //Delay in Micro Seconds
void delayMs(int);   														   //Delay in Milli Seconds
void LCD4bits_Init(void);													 //Initialization of LCD Dispaly
void LCD_Write4bits(unsigned char, unsigned char); //Write data as (4 bits) on LCD
void LCD_WriteString(char*);											 //Write a string on LCD 
void LCD4bits_Cmd(unsigned char);									 //Write command 
void LCD4bits_Data(unsigned char);								 //Write a character
//_______

# define CLK_FREQ  16000000
#define DELAY_DEBOUNCE	CLK_FREQ/1000
#define Get_Bit(Register, Bit) (Register & ( 1 << Bit )) >> Bit

#define configUSE_COUNTING_SEMAPHORES 1

void Delay(unsigned long c);

void PortF_Init (void);

void printString(char * string);
void printChar(char c);
void printInt(char c);
char readChar(void);

/*-----------------------------------------------------------*/

static void vTask1( void *pvParameters );
static void vTask2( void *pvParameters );
static void vTask3( void *pvParameters );
static void vTask4( void *pvParameters );


xQueueHandle xUARTQueue; //UARTQueue, used by Task1 and Task2
xQueueHandle xLCDQueue; //UARTQueue, used by Task1 and Task3
xQueueHandle xBuzzerQueue; //UARTQueue, used by Task1 and Task4

/*-----------------------------------------------------------*/

int main( void )

{
	PortF_Init();

	xUARTQueue = xQueueCreate( 1, sizeof( int ) );
	xLCDQueue = xQueueCreate( 1, sizeof( int ) ); //changed to 1 so that we can have the newest values in the queue more quickly
	xBuzzerQueue = xQueueCreate( 1, sizeof( int ) );
	
	xTaskCreate( vTask1, (const portCHAR *)"MainController", configMINIMAL_STACK_SIZE, NULL, 1, NULL );
	xTaskCreate( vTask2, (const portCHAR *)"UARTController", configMINIMAL_STACK_SIZE, NULL, 1, NULL );
	xTaskCreate( vTask3, (const portCHAR *)"LCDController", configMINIMAL_STACK_SIZE, NULL, 1, NULL );
	xTaskCreate( vTask4, (const portCHAR *)"BuzzerController", configMINIMAL_STACK_SIZE, NULL, 1, NULL );

	vTaskStartScheduler();
		
	for( ;; ){
	}
}

/*-----------------------------------------------------------*/
/*Main Task. reads temperature and setpoint values. turns heater on if temperature<setpoint
Turns heater off if temperature>setpoint
sends temperature and setpoint values to LCD queue to be displayed
and sends 1 or 0 to Buzzer queue to turn on/off Buzzer if temperature exceeds alarm value*/
/*-----------------------------------------------------------*/
static void vTask1( void *pvParameters )
{
	unsigned int adc_value;    //variable that contains adc value 
	
	/*Analog to Digital conversion of Sensor reading*/
    /* Enable Clock to ADC0 and GPIO pins*/
    SYSCTL->RCGCGPIO |= (1<<4);   /* Enable Clock to GPIOE or PE3/AN0 */
	  delayMs(10);									/* 10 msec delay to enable the clock */
    SYSCTL->RCGCADC |= (1<<0);    /* ADC0 clock enable*/
    
    /* initialize PE3 for AN0 input  */
    GPIOE->AFSEL |= (1<<3);       /* enable alternate function */
    GPIOE->DEN &= ~(1<<3);        /* disable digital function */
    GPIOE->AMSEL |= (1<<3);       /* enable analog function */
   
    /* initialize sample sequencer3 */
    ADC0->ACTSS &= ~(1<<3);        				/* disable SS3 during configuration */
    ADC0->EMUX &= ~0xF000;    						/* software trigger conversion */
    ADC0->SSMUX3 = 0;         	 					/* get input from channel 0 */
    ADC0->SSCTL3 |= (1<<1)|(1<<2);        /* take one sample at a time, set flag at 1st sample */
    ADC0->ACTSS |= (1<<3);         				/* enable ADC0 sequencer 3 */
    
	  /*Iniitialize PF3 as a digital output pin */
		SYSCTL->RCGCGPIO |= 0x20;  /* turn on bus clock for GPIOF */
		delayMs(10);							 /* 10 msec delay to enable the clock */
		GPIOF->DIR       |= 0x08;  /* set GREEN pin as a digital output pin */
		GPIOF->DEN       |= 0x08;  /* Enable PF3 pin as a digital pin */ 
		//____________________
		
	//stores temperature reading in Txt1 and setpoint in Txt2
	typedef struct Message{
		unsigned char Txt1;
		unsigned char Txt2;
	} AMessage;
	
	AMessage msg;
	
	//variables for Buzzer
	char on = 1;
	char off = 0;
	
	//Initial setpoint temperature can be changed from PuTTy
	volatile unsigned char setpoint = 20;
	
	volatile int Temperature;
	volatile unsigned char t;
	
	unsigned const char AlarmValue = 30; //if exceeded. buzzer is turned on
	
	on = 1;
	off = 0;
	//volatile int lReceivedValue; //VOLATILE SO WE CAN SEE ITS VALUE IN DEBUGGING (AND NOT OUT OF SCOPE)
portBASE_TYPE xStatus;
//const portTickType xTicksToWait = 100 / portTICK_RATE_MS;

	for( ;; )
	{
		//receive setpoint temperature from UART queue
		xStatus = xQueueReceive( xUARTQueue, &setpoint, 0 );
		//ADC reading
		ADC0->PSSI |= (1<<3);        		/* Enable SS3 conversion or start sampling data from AN0 */
    while((ADC0->RIS & 8) == 0) ;   /* Wait untill sample conversion completed*/
    adc_value = ADC0->SSFIFO3; 			/* read adc coversion result from SS3 FIFO*/
    ADC0->ISC = 8;
		
		Temperature = adc_value * 0.8; //DATA TYPES FIHA MOSHKLA
		Temperature = Temperature / 100;
		
		t = Temperature;
	
		if (Temperature < setpoint)
		{
			GPIOF->DATA  |= 0x08; /* turn on green LED. HEATER ON. */
		}
		else
		{
			GPIOF->DATA  &= 0xF7; /* turn off green LED. HEATER OFF. */
		}
		
		msg.Txt1 = t; //store temperature in Txt1
		msg.Txt2 = setpoint; //store setpoint in Txt1
		
		xStatus = xQueueSend( xLCDQueue, &msg, 0 ); //send temperature and setpoint to LCDQueue to be displayed
		
		if (Temperature > AlarmValue)
			xQueueSend(xBuzzerQueue, &on, 0); //turn Buzzer on
		else
			xQueueSend(xBuzzerQueue, &off, 0); //turn Buzzer off
	}
//}
}
//_______________________________________________
/*Task 2 is the UART task
reads setpoint temperature entered by user in PuTTy terminal
*/

static void vTask2( void *pvParameters )
{

volatile char c;
	volatile char Total;
	volatile char* str = 0x00000000; 
	char s [16] = {}; str= s;
	//volatile int y;
	int i;
		portBASE_TYPE xStatus;
		const portTickType xTicksToWait = 100 / portTICK_RATE_MS;
	while(1){
	
		printString( "\n\r\n\rEnter setpoint temperature: " );
		c = 0;
		Total = 0;
		i=0;
		while(1){
			c = readChar();
		printChar(c);
			
			if (c=='\r') break;
			//y=c;
			//Total2 = y; //Total2 = y;
			s[i] = c;
			c = c-'0';
			Total = 10*Total + c; // di fiha l hexa value
			i=i+1;
		}
		str=s;
		xStatus = xQueueSendToBack( xUARTQueue, &Total, xTicksToWait ); //2allel xticks to wait
		//xStatus = xQueueSendToBack( xLCDQueue2, &str, xTicksToWait ); //2allel xticks to wait
		printString("\n\rTemperature setpoint changed");
	}
}

//_______________________________________________
static void vTask3( void *pvParameters ) {
	
	typedef struct Message{
		unsigned char Txt1;
		unsigned char Txt2;
	} AMessage;
	
	AMessage msg;
	
	volatile char s2[16]; volatile unsigned char bin = 5; volatile unsigned char n = 1;
volatile unsigned char b = bin;
	
	volatile char s3[16]; volatile unsigned char bin3 = 5; volatile unsigned char n3 = 1;
volatile unsigned char b3 = bin3;
	
	LCD4bits_Init();									//Initialization of LCD
	
	portBASE_TYPE xStatus;
	volatile char* str = 0x00000000;
	volatile int j =0;
	
	LCD4bits_Cmd(0x01);								//Clear the display
	
	while (1){
		s2[0] = ' '; s2[1] = ' '; s2[2] = ' '; s2[3] = ' '; s2[4] = ' '; s2[5] = ' '; s2[6] = ' '; s2[7] = ' '; s2[8] = ' ';  s2[9] = ' ';
		s3[0] = ' '; s3[1] = ' '; s3[2] = ' '; s3[3] = ' '; s3[4] = ' '; s3[5] = ' '; s3[6] = ' '; s3[7] = ' '; s3[8] = ' ';  s3[9] = ' ';
		LCD4bits_Cmd(0x01);								//Clear the display
	LCD4bits_Cmd(0x80);               //Force the cursor to beginning of 1st line
	//delayMs(500);											//delay 500 ms for LCD (MCU is faster than LCD)
		//xStatus = xQueueReceive( xLCDQueue2, &str, 0 );
		xStatus = xQueueReceive( xLCDQueue, &msg, 0 );
		bin = msg.Txt1;
		b = bin;
		bin3 = msg.Txt2;
		b3 = bin3;
		
		//FUNCTION BARRA. ALSO "MEASURED ABD SETPOINT:"
		//INSTEAD OF S3[J-1] IT'S S3[J-1+11]
		//w volatile char s2[16] = {"Measured: "}
		j=0;
		while (b!=0){
	b/=10; j+=1;
}  
	while (j>=0){
    s2[j-1] = (bin % 10) + '0';
        bin /= 10;
	//s2[j-2] = (bin % 10) + '0';
		j--;
	}
	
	j=0;
	while (b3!=0){
	b3/=10; j+=1;
}  
	while (j>=0){
    s3[j-1] = (bin3 % 10) + '0';
        bin3 /= 10;
	//s2[j-2] = (bin % 10) + '0';
		j--;
	}
		
		LCD_WriteString(s2);
		delayMs(100);
		LCD4bits_Cmd(0xC0);               //Force the cursor to beginning of 1st line
	delayMs(100);											//delay 500 ms for LCD (MCU is faster than LCD)
	LCD_WriteString(s3);							//Write the string on LCD
	delayMs(1000);											//Delay 500 ms to let the LCD diplays the d
	//erase l string
	}

}
//_______________________________________________

static void vTask4( void *pvParameters ) {
	
	char BuzzerState;
	
	for (;;){
	
	xQueueReceive( xBuzzerQueue, &BuzzerState, 0 );
		if (BuzzerState == 1)
			GPIOF->DATA  |= 0x04; /* turn on green LED*/
		else
			GPIOB->DATA  &=(0xFB); /* turn on green LED*/
	
	
	}



}

//_______________________________________________
void PortF_Init(void){
	
SYSCTL_RCGCGPIO_R |= 0X20;
GPIO_PORTF_DIR_R |= 0X0E;
GPIO_PORTF_DEN_R |= 0X0E;
GPIO_PORTF_AMSEL_R = 0X00;
GPIO_PORTF_PCTL_R = 0X00000000;
GPIO_PORTF_AFSEL_R = 0X00;
GPIO_PORTF_DEN_R = 0X0E;


/*SYSCTL_RCGCGPIO_R |= 0X4;
	GPIO_PORTC_DIR_R |= 0X10;
	GPIO_PORTC_AMSEL_R = 0X00;
GPIO_PORTC_PCTL_R = 0X00000000;
GPIO_PORTC_AFSEL_R = 0X00;
GPIO_PORTC_DEN_R |= 0X10;*/

	
  GPIO_PORTF_LOCK_R = 0x4C4F434B;
  GPIO_PORTF_CR_R = 0x1F;
  GPIO_PORTF_PUR_R = 0x11;
  GPIO_PORTF_DEN_R = 0x1F;
	//dir
	
	SYSCTL_RCGCUART_R |= 0X01;
	SYSCTL_RCGCGPIO_R |= 0X01;
	
	GPIO_PORTA_AFSEL_R = (1<<1)|(1<<0);
	GPIO_PORTA_PCTL_R = (1<<0)|(1<<4);
	GPIO_PORTA_DEN_R = (1<<0)|(1<<1);
	
	UART0_CTL_R = 0;         /* UART0 module disbable */
	
    //UART0_IBRD_R = 65;      /* for 9600 baud rate, integer = 104 */
UART0_IBRD_R = 104;  

	//UART0_FBRD_R = 7;       /* for 9600 baud rate, fractional = 11*/
	UART0_FBRD_R = 11;
	
	
    UART0_CC_R = 0x5;          /*select system clock*/
    UART0_LCRH_R = 0x60;     /* data lenght 8-bit, not parity bit, no FIFO */
    UART0_CTL_R = 0x301;		/* Enable UART5 module, Rx and Tx */
		
		
		//GPIO_PORTF_IS_R &= ~0x10;
//GPIO_PORTF_IBE_R &= ~0x10;
//GPIO_PORTF_IEV_R &= ~0x10;
//GPIO_PORTF_ICR_R |= 0x10;
//GPIO_PORTF_IM_R |= 0x10;

//NVIC_PRI7_R = 7<<21;
//NVIC_PRI7_R |=0xE00000;
//NVIC_PRI7_R |=0x200000;
//NVIC_EN0_R |= 1<<30;
//__enable_irq();
	
}

void printChar(char c){

	while((UART0_FR_R & (1<<5)) != 0);
	UART0_DR_R = c;

}

void printInt(char c){

	while((UART0_FR_R & (1<<5)) != 0);
	UART0_DR_R = c;

}


void printString(char * string) {

	while(*string){
	
		printChar(*(string++));
	}
	
}


char readChar(void) {

	char c;
	while((UART0_FR_R & (1<<4)) != 0); //wait for key press
	c = (char) UART0_DR_R; //read data user typed and save it in c
	return c;


}


void Delay(unsigned long c)
{
  unsigned long i = 0;
  for(i = 0; i<c; i++);	
}
//_________________________


void LCD4bits_Init(void)
{
	SYSCTL->RCGCGPIO |= 0x02;    //enable clock for PORTB
	delayMs(10);                 //delay 10 ms for enable the clock of PORTB
  LCD->DIR = 0xFF;             //let PORTB as output pins
	LCD->DEN = 0xFF;             //enable PORTB digital IO pins
	LCD4bits_Cmd(0x28);          //2 lines and 5x7 character (4-bit data, D4 to D7)
	LCD4bits_Cmd(0x06);          //Automatic Increment cursor (shift cursor to right)
	LCD4bits_Cmd(0x01);					 //Clear display screen
	LCD4bits_Cmd(0x0F);          //Display on, cursor blinking
}


void LCD_Write4bits(unsigned char data, unsigned char control)
{
	data &= 0xF0;                       //clear lower nibble for control 
	control &= 0x0F;                    //clear upper nibble for data
	LCD->DATA = data | control;         //Include RS value (command or data ) with data 
	LCD->DATA = data | control | EN;    //pulse EN
	delayUs(0);													//delay for pulsing EN
	LCD->DATA = data | control;					//Turn off the pulse EN
	LCD->DATA = 0;                      //Clear the Data 
}

void LCD_WriteString(char * str)
{  
	volatile int i = 0;          //volatile is important 
	
	while(*(str+i) != '\0')       //until the end of the string
	{
		LCD4bits_Data(*(str+i));    //Write each character of string
		i++;                        //increment for next character
	}
}

void LCD4bits_Cmd(unsigned char command)
{
	LCD_Write4bits(command & 0xF0 , 0);    //upper nibble first
	LCD_Write4bits(command << 4 , 0);			 //then lower nibble
	
	if(command < 4)
		delayMs(2);       //commands 1 and 2 need up to 1.64ms
	else
		delayUs(40);      //all others 40 us
}

void LCD4bits_Data(unsigned char data)
{
	LCD_Write4bits(data & 0xF0 , RS);   //upper nibble first
	LCD_Write4bits(data << 4 , RS);     //then lower nibble
	delayUs(40);												//delay for LCD (MCU is faster than LCD)
}

void delayMs(int n)
{  
	volatile int i,j;             //volatile is important for variables incremented in code
	for(i=0;i<n;i++)
		for(j=0;j<3180;j++)         //delay for 1 msec
		{}
}

void delayUs(int n)             
{
	volatile int i,j;							//volatile is important for variables incremented in code
	for(i=0;i<n;i++)
		for(j=0;j<3;j++)            //delay for 1 micro second
		{}
}