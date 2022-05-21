//@author Sai Bodanki


//The application Demonstrates usage of Queues, Semaphores, Tasking model.

//Application - Glow LED[7:0] for first interrupt and Glow LED[15:8] for next interrupt --> Repeat

//Flow diagram
// GPIO Interrupt (DIP Switch) --> ( ISR )Send a Semaphore --> Task 1 (Catch the Semaphore) -->
// -->Task 1 - Send a Queue to Task -2 --> Task 2 Receive the queue --> Write to GPIO (LED)
//
// Assumptions:
//   o GPIO_0 is a 16-bit, output-only GPIO port connected to the LEDs.
//   o GPIO_1 is capable of generating an interrupt and is connect to the switches and buttons
//     (see project #2 write-up for details)
//   o AXI Timer 0 is a dual 32-bit timer with the Timer 0 interrupt used to generate
//     the FreeRTOS systick.

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"

/* BSP includes. */
#include "xtmrctr.h"
#include "xgpio.h"
#include "sleep.h"
#include "nexys4io.h"
#include "PmodOLEDrgb.h"
#include "PmodENC544.h"
#include "xwdttb.h"

#define mainQUEUE_LENGTH					( 1 )

/* A block time of 0 simply means, "don't block". */
#define mainDONT_BLOCK						( portTickType ) 0

// Clock frequencies
#define CPU_CLOCK_FREQ_HZ		XPAR_CPU_CORE_CLOCK_FREQ_HZ
#define AXI_CLOCK_FREQ_HZ		XPAR_CPU_M_AXI_DP_FREQ_HZ

// AXI timer parameters
#define AXI_TIMER_DEVICE_ID		XPAR_AXI_TIMER_0_DEVICE_ID
#define AXI_TIMER_BASEADDR		XPAR_AXI_TIMER_0_BASEADDR
#define AXI_TIMER_HIGHADDR		XPAR_AXI_TIMER_0_HIGHADDR
#define TmrCtrNumber			0

// Definitions for peripheral NEXYS4IO
#define NX4IO_DEVICE_ID		XPAR_NEXYS4IO_0_DEVICE_ID
#define NX4IO_BASEADDR		XPAR_NEXYS4IO_0_S00_AXI_BASEADDR
#define NX4IO_HIGHADDR		XPAR_NEXYS4IO_0_S00_AXI_HIGHADDR

// Definitions for peripheral PMODOLEDRGB
#define RGBDSPLY_DEVICE_ID		XPAR_PMODOLEDRGB_0_DEVICE_ID
#define RGBDSPLY_GPIO_BASEADDR	XPAR_PMODOLEDRGB_0_AXI_LITE_GPIO_BASEADDR
#define RGBDSPLY_GPIO_HIGHADDR	XPAR_PMODOLEDRGB_0_AXI_LITE_GPIO_HIGHADD
#define RGBDSPLY_SPI_BASEADDR	XPAR_PMODOLEDRGB_0_AXI_LITE_SPI_BASEADDR
#define RGBDSPLY_SPI_HIGHADDR	XPAR_PMODOLEDRGB_0_AXI_LITE_SPI_HIGHADDR

// Definitions for peripheral PMODENC
#define PMODENC_DEVICE_ID		XPAR_PMODENC544_0_DEVICE_ID
#define PMODENC_BASEADDR		XPAR_PMODENC544_0_S00_AXI_BASEADDR
#define PMODENC_HIGHADDR		XPAR_PMODENC544_0_S00_AXI_HIGHADDR

// Watchdog definitions
#define WDTTB_DEVICE_ID		XPAR_WDTTB_0_DEVICE_ID
#define INTC_DEVICE_ID		XPAR_INTC_0_DEVICE_ID
#define WDTTB_IRPT_INTR		XPAR_INTC_0_WDTTB_0_WDT_INTERRUPT_VEC_ID

// Definitions for Switch Masks
#define K_SWITCH_MASK           0x000Cu
#define INCREMENT_MASK          0x0030u
#define SETPOINT_MASK           0x0003u

// Maximum RPM Definitions
#define MAX_RPM					3000

//Create Instances
static XGpio xOutputGPIOInstance, xInputGPIOInstance;
XWdtTb WatchdogTimebase;
PmodOLEDrgb	pmodOLEDrgb_inst;

//Function Declarations
static void prvSetupHardware( void );

//Declare a Semaphore
xSemaphoreHandle binary_sem;

/* The queue used by the queue send and queue receive tasks. */
static xQueueHandle xQueueKParams = NULL, xQueueDesSpdDir = NULL; //Display, xQueueDesSpdDir = NULL;

volatile uint8_t buttons = 0;
volatile uint16_t switches = 0;
volatile uint8_t pressed = 0;

volatile uint32_t ROTENC_CNT= 0;
volatile uint32_t OLD_ROTENC_CNT = 0;

volatile uint16_t leds = 0;

int counter = 0;

void PMDIO_itoa(int32_t value, char *string, int32_t radix);
void PMDIO_putnum(PmodOLEDrgb* InstancePtr, int32_t num, int32_t radix);
void Update_Seven_Seg(uint16_t DesiredRPM);
void Reset_Seven_Seg();

//ISR, to handle interrupt of GPIO dip switch
//Can also use Push buttons.
//Give a Semaphore
static void gpio_intr (void *pvUnused)
{
	//xSemaphoreGiveFromISR(binary_sem,NULL);
	pressed = 1;

	buttons = XGpio_DiscreteRead(&xInputGPIOInstance, 1);
	switches = XGpio_DiscreteRead(&xInputGPIOInstance, 2);

	XGpio_InterruptClear( &xInputGPIOInstance, 1 );
	XGpio_InterruptClear( &xInputGPIOInstance, 2 );

}

//A task which takes the Interrupt Semaphore and sends a queue to task 2.
void parameter_input_thread (void *p)
{
	unsigned long long ulKParamsToSend = 0, ulDesDirSpdToSend = 0;
	unsigned int uiIncrementSw = 0, uiKParamSwitch = 0, uiSetPointSwitch = 0;
	unsigned int uiKp = 0, uiKi = 0, uiKd = 0;
	unsigned int uiIncVal = 1, uiSetPointVal = 1;
	unsigned long uiDirection = 0;					//PWM 0 is clockwise
	int iSetPointRPM = 0;

	while(1)
	if(pressed==1){
		pressed = 0;
		uiIncrementSw = (switches & INCREMENT_MASK)>>4;
		uiKParamSwitch = (switches & K_SWITCH_MASK)>>2;
		uiSetPointSwitch = (switches & SETPOINT_MASK);
		xil_printf("Increment: %d\r\n",uiIncrementSw);
		xil_printf("K: %d\r\n",uiKParamSwitch);
		xil_printf("SetPoint: %d\r\n",uiSetPointSwitch);

		if(uiIncrementSw == 3 || uiIncrementSw == 2){
			uiIncVal = 10;
		}
		else if(uiIncrementSw == 1){
			uiIncVal = 5;
		}
		else{
			uiIncVal = 1;
		}

		if(uiSetPointSwitch == 3 || uiSetPointSwitch == 2){
			uiSetPointVal = 10;
		}
		else if(uiSetPointSwitch == 1){
			uiSetPointVal = 5;
		}
		else{
			uiSetPointVal = 1;
		}

		switch(buttons){
			case 4:
				xil_printf("Button D Pressed\r\n");
				if(uiKParamSwitch == 3 && uiKp > 0){
					uiKp -= uiIncVal;
					if(uiKp < 0){
						uiKp = 0;
					}
				}
				else if(uiKParamSwitch == 2 && uiKi > 0) {
					uiKi -= uiIncVal;
					if(uiKi < 0){
						uiKi = 0;
					}
				}
				else if(uiKParamSwitch == 1 && uiKd > 0) {
					uiKd -= uiIncVal;
					if(uiKd < 0){
						uiKd = 0;
					}
				}
				break;
			case 8:
				xil_printf("Button U Pressed\r\n");
				if(uiKParamSwitch == 3 && uiKp < 255){
					uiKp += uiIncVal;
					if(uiKp > 255){
						uiKp = 255;
					}
				}
				else if(uiKParamSwitch == 2 && uiKi < 255) {
					uiKi += uiIncVal;
					if(uiKi > 255){
						uiKi = 255;
					}
				}
				else if(uiKParamSwitch == 1 && uiKd < 255) {
					uiKd += uiIncVal;
					if(uiKd > 255){
						uiKd = 255;
					}
				}
				break;
			case 16:
				xil_printf("Button C Pressed\r\n");
				iSetPointRPM = 0;
				uiKp = 0;
				uiKi = 0;
				uiKd = 0;
				break;
		}
		xil_printf("Kp: %d\r\n",uiKp);
		xil_printf("Ki: %d\r\n",uiKi);
		xil_printf("Kd: %d\r\n",uiKd);
		ulKParamsToSend = (uiKParamSwitch << 24) | (uiKp << 16) | (uiKi << 8) | uiKd;
		ulDesDirSpdToSend = (uiDirection << 16) | (iSetPointRPM);
		xQueueSend( xQueueKParams, &ulKParamsToSend, mainDONT_BLOCK );
		xQueueSend( xQueueDesSpdDir, &ulDesDirSpdToSend, mainDONT_BLOCK );
	}
	else{
		uiIncrementSw = (switches & INCREMENT_MASK)>>4;
		uiKParamSwitch = (switches & K_SWITCH_MASK)>>2;
		uiSetPointSwitch = (switches & SETPOINT_MASK);
		uiDirection = (PMODENC544_getBtnSwReg())>>1;
	    ROTENC_CNT = -1*(PMODENC544_getRotaryCount());
	    if(ROTENC_CNT!=OLD_ROTENC_CNT){

	    	xil_printf("Rotary Encoder Direction: %d\r\n",uiDirection);
	    	iSetPointRPM += (ROTENC_CNT-OLD_ROTENC_CNT)*uiSetPointVal;
	    	if(iSetPointRPM < 0){
	    		iSetPointRPM = 0;
	    	}
	    	else if(iSetPointRPM > MAX_RPM) {
	    		iSetPointRPM = MAX_RPM;
	    	}
	    	xil_printf("Set Point RPM: %d\r\n",iSetPointRPM);
	    	ulKParamsToSend = (uiKp << 16) | (uiKi << 8) | uiKd;
	    	ulDesDirSpdToSend = (uiDirection << 16) | (iSetPointRPM);
	    	xQueueSend( xQueueKParams, &ulKParamsToSend, mainDONT_BLOCK );
	    	xQueueSend( xQueueDesSpdDir, &ulDesDirSpdToSend, mainDONT_BLOCK );
	    	OLD_ROTENC_CNT = ROTENC_CNT;
	    }
	}
	//Could use to take rotary encoder values (A trick, coz pmodENC dosen't support interrupt.)
}

void display_thread (void *p)
{
	unsigned long ulKParamsReceived, ulDesSpdDirReceived;
	while(1){
		xQueueReceive( xQueueKParams, &ulKParamsReceived, portMAX_DELAY );
		xQueueReceive( xQueueDesSpdDir, &ulDesSpdDirReceived, portMAX_DELAY );
		//Write to LED.
		//XGpio_DiscreteWrite( &xOutputGPIOInstance, 1, ulReceivedValue );
		xil_printf("Queue Received: %d\r\n",ulKParamsReceived);
		xil_printf("Queue Received: %d\r\n",ulDesSpdDirReceived);
		//Write to OLED.
		OLEDrgb_SetFontColor(&pmodOLEDrgb_inst,OLEDrgb_BuildRGB(200, 12, 44));
		OLEDrgb_Clear(&pmodOLEDrgb_inst);
		OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 0, 1);
		OLEDrgb_PutString(&pmodOLEDrgb_inst,"Kp:");
		OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 3, 1);
		PMDIO_putnum(&pmodOLEDrgb_inst, (int)((ulKParamsReceived>>16)&0x00000000000000FF), 10);

		OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 0, 3);
		OLEDrgb_PutString(&pmodOLEDrgb_inst,"Ki:");
		OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 3, 3);
		PMDIO_putnum(&pmodOLEDrgb_inst, (int)((ulKParamsReceived>>8)&0x00000000000000FF), 10);

		OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 0, 5);
		OLEDrgb_PutString(&pmodOLEDrgb_inst,"Kd:");
		OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 3, 5);
		PMDIO_putnum(&pmodOLEDrgb_inst, (int)(ulKParamsReceived&0x00000000000000FF), 10);

		Update_Seven_Seg((uint16_t)(ulDesSpdDirReceived)&0x0000FFFF);

		leds = (leds & 0xFFF0) | (0xFFFF&(ulKParamsReceived>>24));
		XGpio_DiscreteWrite( &xOutputGPIOInstance, 1, leds );

	}
}

int main(void)
{


	// Announcement
	xil_printf("Hello from FreeRTOS Example\r\n");

	//Initialize the HW
	prvSetupHardware();

	Reset_Seven_Seg();

	//Create Semaphore
	vSemaphoreCreateBinary(binary_sem);

	/* Create the queue */
	xQueueKParams = xQueueCreate( mainQUEUE_LENGTH, sizeof( unsigned long ) );
	xQueueDesSpdDir = xQueueCreate( mainQUEUE_LENGTH, sizeof( unsigned long ) );

	/* Sanity check that the queue was created. */
	configASSERT( xQueueKParams );
	configASSERT( xQueueDesSpdDir );

	//Create Task1
	xTaskCreate( parameter_input_thread,
				 ( const char * ) "TX",
				 2048,
				 NULL,
				 1,
				 NULL );

	//Create Task2
	xTaskCreate( display_thread,
				"RX",
				2048,
				NULL,
				2,
				NULL );



	//Start the Scheduler
	xil_printf("Starting the scheduler\r\n");
	xil_printf("Flip one of the switches to change the LED pattern\r\n\r\n");
	vTaskStartScheduler();

	return -1;
}


static void prvSetupHardware( void )
{
portBASE_TYPE xStatus;
uint32_t status;
const unsigned char ucSetToOutput = 0U;
const unsigned char ucSetToInput = 0xFFU;

	xil_printf("Initializing GPIO's\r\n");

	/* Initialize the GPIO for the LEDs. */

	status = (uint32_t) NX4IO_initialize(NX4IO_BASEADDR);

	NX4IO_SSEG_setSSEG_DATA(SSEGHI, 0x0058E30E);
	NX4IO_SSEG_setSSEG_DATA(SSEGLO, 0x00144116);

	xStatus = PMODENC544_initialize(PMODENC_BASEADDR);
	OLEDrgb_begin(&pmodOLEDrgb_inst, RGBDSPLY_GPIO_BASEADDR, RGBDSPLY_SPI_BASEADDR);
	xStatus = XGpio_Initialize( &xOutputGPIOInstance, XPAR_AXI_GPIO_0_DEVICE_ID );

	if( xStatus == XST_SUCCESS )
	{
		/* All bits on this channel are going to be outputs (LEDs). */
		XGpio_SetDataDirection( &xOutputGPIOInstance, 1, ucSetToOutput );

		xil_printf("Check that LEDs work\r\n");

		XGpio_DiscreteWrite( &xOutputGPIOInstance, 1, 0x5555 );
		usleep(10000);
		XGpio_DiscreteWrite( &xOutputGPIOInstance, 1, 0xAAAA );
		usleep(10000);

	}

	/* Initialise the GPIO for the button inputs. */
	if( xStatus == XST_SUCCESS )
	{
		xStatus = XGpio_Initialize( &xInputGPIOInstance, XPAR_AXI_GPIO_1_DEVICE_ID );
	}


	if( xStatus == XST_SUCCESS )
	{
		/* Install the handler defined in this task for the button input.
		*NOTE* The FreeRTOS defined xPortInstallInterruptHandler() API function
		must be used for this purpose. */
		xStatus = xPortInstallInterruptHandler( XPAR_MICROBLAZE_0_AXI_INTC_AXI_GPIO_1_IP2INTC_IRPT_INTR, gpio_intr, NULL );


		if( xStatus == pdPASS )
		{
			xil_printf("Buttons and Switches interrupt handler installed\r\n");

			/* Set switches and buttons to input. */
			XGpio_SetDataDirection( &xInputGPIOInstance, 1, ucSetToInput );
			XGpio_SetDataDirection( &xInputGPIOInstance, 2, ucSetToInput );

			/* Enable the button input interrupts in the interrupt controller.
			*NOTE* The vPortEnableInterrupt() API function must be used for this
			purpose. */

			vPortEnableInterrupt( XPAR_MICROBLAZE_0_AXI_INTC_AXI_GPIO_1_IP2INTC_IRPT_INTR );

			/* Enable GPIO channel interrupts. */
			XGpio_InterruptEnable( &xInputGPIOInstance, 1 );
			XGpio_InterruptEnable( &xInputGPIOInstance, 2 );
			XGpio_InterruptGlobalEnable( &xInputGPIOInstance );
		}
	}

	configASSERT( ( xStatus == pdPASS ) );
}

/****************************************************************************/
/**
* Write a 32-bit number in Radix "radix" to LCD display
*
* Writes a 32-bit number to the LCD display starting at the current
* cursor position. "radix" is the base to output the number in.
*
* @param num is the number to display
*
* @param radix is the radix to display number in
*
* @return *NONE*
*
* @note
* No size checking is done to make sure the string will fit into a single line,
* or the entire display, for that matter.  Watch your string sizes.
*****************************************************************************/
void PMDIO_putnum(PmodOLEDrgb* InstancePtr, int32_t num, int32_t radix)
{
  char  buf[16];

  PMDIO_itoa(num, buf, radix);
  OLEDrgb_PutString(InstancePtr,buf);

  return;
}

void PMDIO_itoa(int32_t value, char *string, int32_t radix)
{
	char tmp[33];
	char *tp = tmp;
	int32_t i;
	uint32_t v;
	int32_t  sign;
	char *sp;

	if (radix > 36 || radix <= 1)
	{
		return;
	}

	sign = ((10 == radix) && (value < 0));
	if (sign)
	{
		v = -value;
	}
	else
	{
		v = (uint32_t) value;
	}

  	while (v || tp == tmp)
  	{
		i = v % radix;
		v = v / radix;
		if (i < 10)
		{
			*tp++ = i+'0';
		}
		else
		{
			*tp++ = i + 'a' - 10;
		}
	}
	sp = string;

	if (sign)
		*sp++ = '-';

	while (tp > tmp)
		*sp++ = *--tp;
	*sp = 0;

  	return;
}

//Reset the Seven Segment Display Decimal points
void Reset_Seven_Seg()
{
	NX4IO_SSEG_setSSEG_DATA(SSEGHI, 0xFFFFFFFF);
	NX4IO_SSEG_setSSEG_DATA(SSEGLO, 0xFFFFFFFF);
	NX4IO_SSEG_setDecPt(SSEGLO,DIGIT7,false);
	NX4IO_SSEG_setDecPt(SSEGLO,DIGIT6,false);
	NX4IO_SSEG_setDecPt(SSEGLO,DIGIT5,false);
	NX4IO_SSEG_setDecPt(SSEGLO,DIGIT4,false);
	NX4IO_SSEG_setDecPt(SSEGHI,DIGIT3,false);
	NX4IO_SSEG_setDecPt(SSEGHI,DIGIT2,false);
	NX4IO_SSEG_setDecPt(SSEGHI,DIGIT1,false);
	NX4IO_SSEG_setDecPt(SSEGHI,DIGIT0,false);
}

void Update_Seven_Seg(uint16_t DesiredRPM)
{
	NX4IO_SSEG_setDigit(SSEGLO,DIGIT3,((DesiredRPM/1000)%10));
	NX4IO_SSEG_setDigit(SSEGLO,DIGIT2,((DesiredRPM/100)%10));
	NX4IO_SSEG_setDigit(SSEGLO,DIGIT1,((DesiredRPM/10)%10));
	NX4IO_SSEG_setDigit(SSEGLO,DIGIT0,(DesiredRPM%10));
	usleep(5000);
}

