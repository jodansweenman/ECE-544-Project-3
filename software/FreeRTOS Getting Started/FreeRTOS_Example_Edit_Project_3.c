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
#include "xparameters.h"
#include "xstatus.h"
#include "microblaze_sleep.h"
#include "nexys4io.h"
#include "PmodOLEDrgb.h"
#include "PmodENC544.h"

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

#define K_SWITCH_MASK           0x000Cu
#define INCREMENT_MASK          0x0030u
#define SETPOINT_MASK           0x0003u

//Create Instances
static XGpio xOutputGPIOInstance, xInputGPIOInstance;

//Function Declarations
static void prvSetupHardware( void );

//Declare a Semaphore
xSemaphoreHandle binary_sem;

/* The queue used by the queue send and queue receive tasks. */
static xQueueHandle displayQueue, paramQueue = NULL;

volatile uint32_t ROTENC_CNT= 0;
volatile uint32_t OLD_ROTENC_CNT = 0;

volatile uint16_t NEXYS_SW = 0;
volatile uint16_t K_SWITCH = 0;
volatile uint16_t INC_SWITCH = 0;
volatile uint16_t SP_SWITCH = 0;

volatile uint8_t  INC_VAL = 1;
volatile uint8_t  SP_VAL = 1;

volatile uint16_t DESIRED_RPM = 0;
volatile uint8_t  DESIRED_DIR = 0;      //0 Clockwise, 1 Counter-Clockwise

volatile uint8_t  K_P = 0;
volatile uint8_t  K_I = 0;
volatile uint8_t  K_D = 0;

int ROT_DIFF = 0;


//ISR, to handle interrupt of GPIO dip switch
//Can also use Push buttons.
//Give a Semaphore
static void gpio_intr (void *pvUnused)
{
	xSemaphoreGiveFromISR(binary_sem,NULL);

	XGpio_InterruptClear( &xInputGPIOInstance, 1 );

}

//A task which takes the Interrupt Semaphore and sends a queue to task 2.
void parameter_input_thread (void *p)
{
    unsigned long displaySend, paramSend;
    while(1)
	if(xSemaphoreTake(binary_sem,500)){
        OLD_ROTENC_CNT = ROTENC_CNT;
        ROTENC_CNT = PMODENC544_getRotaryCount();
        ROT_DIFF = ROTENC_CNT - OLD_ROTENC_CNT;

        NEXYS_SW = NX4IO_getSwitches();

        DESIRED_DIR = (uint8_t)(PMODENC544_getBtnSwReg()>>1);

        INC_SWITCH = (NEXYS_SW & INCREMENT_MASK)>>4;
        K_SWITCH = (NEXYS_SW & K_SWITCH_MASK)>>2;
        SP_SWITCH = NEXYS4IO & SETPOINT_MASK;


        if(INC_SWITCH == 3 || INC_SWITCH == 2){
            INC_VAL = 10;
        }
        else if(INC_SWITCH == 1){
            INC_VAL = 5;
        }
        else{
            INC_VAL = 1;
        }

        if(SP_SWITCH == 3 || SP_SWITCH == 2){
            SP_VAL = 10;
        }
        else if(SP_SWITCH == 1){
            SP_VAL = 5;
        }
        else{
            SP_VAL = 1;
        }

        DESIRED_RPM += ROT_DIFF*SP_VAL;

        if (NX4IO_isPressed(BTNU)) {
            if(K_SWITCH == 3 && K_P < 255){
                K_P += INC_VAL;
            }
            else if(K_SWITCH == 2 && K_I < 255) {
                K_I += INC_VAL;
            }
            else if(K_SWITCH == 1 && K_D < 255) {
                K_D += INC_VAL;
            }
        }
        else if (NX4IO_isPressed(BTND)) {
            if(K_SWITCH == 3 && K_P > 0){
                K_P -= INC_VAL;
            }
            else if(K_SWITCH == 2 && K_I > 0) {
                K_I -= INC_VAL;
            }
            else if(K_SWITCH == 1 && K_D > 0) {
                K_D -= INC_VAL;
            }
        }
        else if (NX4IO_isPressed(BTNC))
        {
            DESIRED_RPM = 0;
        }
        displaySend = (K_P << 16) | (K_I << 8) | K_D;
        paramSend = (DESIRED_DIR << 40) | (DESIRED_RPM << 24) | (K_P << 16) | (K_I << 8) | K_D;
		xil_printf("Display Param Sent: %d\r\n",displaySend);
		xQueueSend( displayQueue, &displaySend, mainDONT_BLOCK );
        xil_printf("PID Params Sent: %d\r\n", paramSend);
		xQueueSend( paramQueue, &paramSend, mainDONT_BLOCK );
	}
	else
		xil_printf("Semaphore time out\r\n");
	//Could use to take rotary encoder values (A trick, coz pmodENC dosen't support interrupt.)
}

void display_thread (void *p)
{
	unsigned long ulInputThreadRecieve;
	while(1){
		xQueueReceive( displayQueue, &ulInputThreadRecieve, portMAX_DELAY );
		//Write to OLED.
		OLEDrgb_SetFontColor(&pmodOLEDrgb_inst,OLEDrgb_BuildRGB(200, 12, 44));
        OLEDrgb_Clear(&pmodOLEDrgb_inst);
        OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 0, 1);
        OLEDrgb_PutString(&pmodOLEDrgb_inst,"Kp:");
        OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 3, 1);
        PMDIO_putnum(&pmodOLEDrgb_inst, (int)((ulInputThreadRecieve>>16)&0x00000000000000FF), 10);
        OLD_RED = RED;

        OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 0, 3);
        OLEDrgb_PutString(&pmodOLEDrgb_inst,"Ki:");
        OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 3, 3);
        PMDIO_putnum(&pmodOLEDrgb_inst, (int)((ulInputThreadRecieve>>8)&0x00000000000000FF), 10);
        OLD_GREEN = GREEN;

        OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 0, 5);
        OLEDrgb_PutString(&pmodOLEDrgb_inst,"Kd:");
        OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 3, 5);
        PMDIO_putnum(&pmodOLEDrgb_inst, (int)(ulInputThreadRecieve&0x00000000000000FF), 10);
		xil_printf("Queue Received: %d\r\n",ulInputThreadRecieve);
	}
}

int main(void)
{
	// Announcemetn
	xil_printf("Hello from FreeRTOS Example\r\n");

	//Initialize the HW
	prvSetupHardware();

	//Creatye Semaphore
	vSemaphoreCreateBinary(binary_sem);

	/* Create the queue */
	xQueue = xQueueCreate( mainQUEUE_LENGTH, sizeof( unsigned long ) );

	/* Sanity check that the queue was created. */
	configASSERT( xQueue );

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
const unsigned char ucSetToOutput = 0U;
const unsigned char ucSetToInput = 0xFFU;

	xil_printf("Initializing GPIO's\r\n");

	/* Initialize the GPIO for the LEDs. */
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
			XGpio_InterruptGlobalEnable( &xInputGPIOInstance );
		}
	}

	configASSERT( ( xStatus == pdPASS ) );
}

