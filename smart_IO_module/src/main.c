/*
 * Bernardo Tavares 84712 e João Rodrigues 89029, 2023/03
 * Zephyr: Simple thread and Digital Input Interrupt example
 * 
 *  Button 1 generates an interrupt that toggles a global var
 *  A Periodic task updates LED status according to global var
 *  Also shows how a single callback can handle multiple gpio interrupts
 *
 * Base documentation:
 *  Zephyr kernel:  
 *      https://developer.nordicsemi.com/nRF_Connect_SDK/doc/latest/zephyr/kernel/services/index.html#kernel-services
 *      
 *  DeviceTree 
 *      Board DTS can be found in BUILD_DIR/zephyr/zephyr.dts
 *      https://developer.nordicsemi.com/nRF_Connect_SDK/doc/latest/zephyr/guides/dts/api-usage.html#dt-from-c  
 *
 *      HW info
 *      https://infocenter.nordicsemi.com/topic/struct_nrf52/struct/nrf52840.html
 *      Section: nRF52840 Product Specification -> Peripherals -> GPIO / GPIOTE
 * 
 *      Board specific HW info can be found in the nRF52840_DK_User_Guide_20201203. I/O pins available at pg 27
 * 
 * 
 */
#include <zephyr/kernel.h>          /* for kernel functions*/
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <zephyr/timing/timing.h>   /* for timing services */

#include <zephyr/types.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/util.h>
#include <inttypes.h>

/* Size of stack area used by each thread (can be thread specific, if necessary)*/
#define STACK_SIZE 1024

/* Thread scheduling priority */
#define thread_temperature_prio 3
#define thread_button_prio 3
#define thread_led_prio 3
#define thread_cmd_prio 3
#define thread_print_prio 2

/* Therad periodicity (in ms)*/
volatile int thread_temperature_period = 20;
volatile int64_t release_time_temperature = 0;
volatile int thread_button_period = 20;
volatile int64_t release_time_button = 0;
volatile int thread_led_period = 20;
volatile int64_t release_time_led = 0;
#define thread_print_period 300
#define thread_cmd_period 20

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS 1000

/* Define the size of the UART receive buffer */
#define RECEIVE_BUFF_SIZE 10

/* Define the receiving timeout period of UART */
#define RECEIVE_TIMEOUT 100

/* I2C node identifier*/
#define I2C_NODE DT_NODELABEL(tc74)

/* UART node identifier*/
#define UART_NODE DT_NODELABEL(uart0)


/* Get node IDs. Refer to dts file and HW documentation*/
#define LED0_NID DT_NODELABEL(led0) 
#define LED1_NID DT_NODELABEL(led1)
#define LED2_NID DT_NODELABEL(led2)
#define LED3_NID DT_NODELABEL(led3)  
#define BUT0_NID DT_NODELABEL(button0) 
#define BUT1_NID DT_NODELABEL(button1) 
#define BUT2_NID DT_NODELABEL(button2) 
#define BUT3_NID DT_NODELABEL(button3) 

/* Define for filtering commands */
#define MAX_CMDSTRING_SIZE 10   /* Maximum size of the command string */ 
#define SOF_SYM '#'             /* Start of Frame Symbol */
#define EOF_SYM '!'             /* End of Frame Symbol */
#define EXIT_SUCCESS    0;      /* Successfull exit */
#define EMPTY_STRING   -1;      /* String is empty */
#define STRING_FULL    -1;      /* String is full */
#define CMD_NOT_FOUND  -2;      /* Invalid CMD */
#define WRONG_STR_FORMAT -4;    /* Wrong string format*/

/* sensor config register*/
#define config  0x00

/* Create thread stack space */
K_THREAD_STACK_DEFINE(thread_temperature_stack, STACK_SIZE);
K_THREAD_STACK_DEFINE(thread_button_stack, STACK_SIZE);
K_THREAD_STACK_DEFINE(thread_led_stack, STACK_SIZE);
K_THREAD_STACK_DEFINE(thread_print_stack, STACK_SIZE);
K_THREAD_STACK_DEFINE(thread_cmd_stack, STACK_SIZE);
  
/* Create variables for thread data */
struct k_thread thread_temperature_data;
struct k_thread thread_button_data;
struct k_thread thread_led_data;
struct k_thread thread_print_data;
struct k_thread thread_cmd_data;

/* Create task IDs */
k_tid_t thread_temperature_tid;
k_tid_t thread_button_tid;
k_tid_t thread_led_tid;
k_tid_t thread_print_tid;
k_tid_t thread_cmd_tid;

/* Thread code prototypes */
void thread_temperature_code(void *argA, void *argB, void *argC);
void thread_button_code(void *argA, void *argB, void *argC);
void thread_led_code(void *argA, void *argB, void *argC);
void thread_print_code(void *argA, void *argB, void *argC);
void thread_cmd_code(void *argA, void *argB, void *argC);

/* I2C struct*/
static const struct i2c_dt_spec dev_i2c = I2C_DT_SPEC_GET(I2C_NODE);

/* Get the device pointer of the UART hardware */
const struct device *uart = DEVICE_DT_GET(UART_NODE);

/* Get gpio device structures */
const struct gpio_dt_spec led0_dev = GPIO_DT_SPEC_GET(LED0_NID,gpios);
const struct gpio_dt_spec led1_dev = GPIO_DT_SPEC_GET(LED1_NID,gpios);
const struct gpio_dt_spec led2_dev = GPIO_DT_SPEC_GET(LED2_NID,gpios);
const struct gpio_dt_spec led3_dev = GPIO_DT_SPEC_GET(LED3_NID,gpios);

const struct gpio_dt_spec but0_dev = GPIO_DT_SPEC_GET(BUT0_NID,gpios);
const struct gpio_dt_spec but1_dev = GPIO_DT_SPEC_GET(BUT1_NID,gpios);
const struct gpio_dt_spec but2_dev = GPIO_DT_SPEC_GET(BUT2_NID,gpios);
const struct gpio_dt_spec but3_dev = GPIO_DT_SPEC_GET(BUT3_NID,gpios);

/* Variables to use when a button is pressed */
volatile int But0 = 0;      
volatile int But1 = 0;      
volatile int But2 = 0;      
volatile int But3 = 0;  

volatile int led0stat = 0; /* Led status variable. Updated by the callback function */
volatile int led1stat = 0; /* Led status variable. Updated by the callback function */
volatile int led2stat = 0; /* Led status variable. Updated by the callback function */
volatile int led3stat = 0; /* Led status variable. Updated by the callback function */
volatile int led0temp =0,led1temp=0,led2temp=0,led3temp=0;

int ret;
volatile int cmd = 0;
volatile int res = 1;

/* Temperature */
volatile int8_t temp;
volatile int update_temperature_period = 0;
volatile int update_button_period = 0;
volatile int update_led_period = 0;

/* Define the transmission buffer, which is a buffer to hold the data to be sent over UART */
static uint8_t tx_buf[] =   {""};

/* STEP 10.1.2 - Define the receive buffer */
static uint8_t rx_buf[RECEIVE_BUFF_SIZE] = {0};

/* String full*/
volatile int strFull = 0;

/* Internal variables */

static char cmdString[RECEIVE_BUFF_SIZE];

static unsigned char cmdStringLen = 0; 

int SOF_C = 0;

int EOF_C = 0;

/* Functions prototypes */
void resetCmdString(void);
int cmdProcessor(void);
int newCmdChar(unsigned char newChar);
static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data);

/* Main function */
void main(void) {
    /* Check device status */  

    /* Leds */
	if (!device_is_ready(led0_dev.port))  
	{
        printk("Fatal error: led1 device not ready!");
		return;
	}

    if (!device_is_ready(led1_dev.port))  
	{
        printk("Fatal error: led2 device not ready!");
		return;
	}

    if (!device_is_ready(led2_dev.port))  
	{
        printk("Fatal error: led3 device not ready!");
		return;
	}

    if (!device_is_ready(led3_dev.port))  
	{
        printk("Fatal error: led4 device not ready!");
		return;
	}
    
    /* Buttons */
    if (!device_is_ready(but0_dev.port))  
	{
        printk("Fatal error: but0 device not ready!");
		return;
	}

    if (!device_is_ready(but1_dev.port))  
	{
        printk("Fatal error: but1 device not ready!");
		return;
	}

    if (!device_is_ready(but2_dev.port))  
	{
        printk("Fatal error: but2 device not ready!");
        return;
	}

    if (!device_is_ready(but3_dev.port))  
	{
        printk("Fatal error: but3 device not ready!");
		return;
	}

    /* UART */
    if (!device_is_ready(uart)) {
        printk("UART device not ready\r\n");
        return;
    }

    /* I2C */

    if (!device_is_ready(dev_i2c.bus)) {
        printk("I2C bus %s is not ready!\n\r",dev_i2c.bus->name);
        return;
    }
    
    
    /* Configure PINS */
    /* Leds */

    ret = gpio_pin_configure_dt(&led0_dev, led0_dev.dt_flags | GPIO_OUTPUT_ACTIVE);
    if (ret < 0) {
        printk("Error %d: Failed to configure LED 0 \n\r", ret);
	    return;
    }

	ret = gpio_pin_configure_dt(&led1_dev, led1_dev.dt_flags | GPIO_OUTPUT_ACTIVE);
    if (ret < 0) {
        printk("Error %d: Failed to configure LED 1 \n\r", ret);
	    return;
    }

	ret = gpio_pin_configure_dt(&led2_dev, led2_dev.dt_flags | GPIO_OUTPUT_ACTIVE);
    if (ret < 0) {
        printk("Error %d: Failed to configure LED 2 \n\r", ret);
	    return;
    }

	ret = gpio_pin_configure_dt(&led3_dev, led3_dev.dt_flags | GPIO_OUTPUT_ACTIVE);
    if (ret < 0) {
        printk("Error %d: Failed to configure LED 3 \n\r", ret);
	    return;
    }

    /* Buttons */

    ret = gpio_pin_configure_dt(&but0_dev, GPIO_INPUT | GPIO_PULL_UP);
    if (ret < 0) {
        printk("Error %d: Failed to configure BUT 0 \n\r", ret);
	    return;
    }

    ret = gpio_pin_configure_dt(&but1_dev, GPIO_INPUT | GPIO_PULL_UP);
    if (ret < 0) {
        printk("Error %d: Failed to configure BUT 1 \n\r", ret);
	    return;
    }

    ret = gpio_pin_configure_dt(&but2_dev, GPIO_INPUT | GPIO_PULL_UP);
    if (ret < 0) {
        printk("Error %d: Failed to configure BUT 2 \n\r", ret);
	    return;
    }

    ret = gpio_pin_configure_dt(&but3_dev, GPIO_INPUT | GPIO_PULL_UP);
    if (ret < 0) {
        printk("Error %d: Failed to configure BUT 3 \n\r", ret);
	    return;
    }

    /* UART */

    ret = uart_callback_set(uart, uart_cb, NULL);
    if (ret) {
		return;
	}

    /* Send the data over UART by calling uart_tx() */
	ret = uart_tx(uart, tx_buf, sizeof(tx_buf), SYS_FOREVER_MS);
    if (ret) {
		return;
	}

    /* Start receiving by calling uart_rx_enable() and pass it the address of the receive  buffer */
	ret = uart_rx_enable(uart ,rx_buf,sizeof rx_buf,RECEIVE_TIMEOUT);
	if (ret) {
		return;
	}

    /* I2C */

    ret = i2c_write_dt(&dev_i2c, config, sizeof(config));
    if (ret != 0) {
        printk("Failed to write to I2C device address %x at Reg. %x \n",dev_i2c.addr,config);
    }

    timing_init();
    timing_start();

    /* Then create the task */
    thread_temperature_tid = k_thread_create(&thread_temperature_data, thread_temperature_stack,
        K_THREAD_STACK_SIZEOF(thread_temperature_stack), thread_temperature_code,
        NULL, NULL, NULL, thread_temperature_prio, 0, K_NO_WAIT);

    thread_button_tid = k_thread_create(&thread_button_data, thread_button_stack,
        K_THREAD_STACK_SIZEOF(thread_button_stack), thread_button_code,
        NULL, NULL, NULL, thread_button_prio, 0, K_NO_WAIT);

    thread_led_tid = k_thread_create(&thread_led_data, thread_led_stack,
        K_THREAD_STACK_SIZEOF(thread_led_stack), thread_led_code,
        NULL, NULL, NULL, thread_led_prio, 0, K_NO_WAIT);
	
	thread_print_tid = k_thread_create(&thread_print_data, thread_print_stack,
        K_THREAD_STACK_SIZEOF(thread_print_stack), thread_print_code,
        NULL, NULL, NULL, thread_print_prio, 0, K_NO_WAIT);
    
    thread_cmd_tid = k_thread_create(&thread_cmd_data, thread_cmd_stack,
        K_THREAD_STACK_SIZEOF(thread_cmd_stack), thread_cmd_code,
        NULL, NULL, NULL, thread_cmd_prio, 0, K_NO_WAIT);

        resetCmdString();
    return;

}

/* ************************************************************ */
/* Processes the chars received so far looking for commands     */
/* Returns:                                                     */
/*  	 0: if a valid command was found and executed           */
/* 	-1: if empty string or incomplete command found             */
/* 	-2: if an invalid command was found                         */
/* 	-3: if a CS error is detected (command not executed)        */
/* 	-4: if string format is wrong                               */
/* ************************************************************ */

int cmdProcessor(void) {

	int i;
    for(i=0; i < cmdStringLen; i++) {
        if(cmdString[i] == '#') {
			SOF_C ++;
		}
		else if(cmdString[i] == '!') {
			EOF_C ++;
        }
	}
    if((SOF_C != 1) || (EOF_C != 1)) {
		SOF_C = 0;
		EOF_C = 0;
	    return WRONG_STR_FORMAT;
	}

	/* Detect empty cmd string */
	if(cmdStringLen == 0) {
		return EMPTY_STRING;
    } 

	/* Find index of SOF */
	for(i=0; i < cmdStringLen; i++) {
		if(cmdString[i] == SOF_SYM) {
			break;
		}
	}											

	/* If a SOF was found look for commands */
	if(i < cmdStringLen) {
		if(cmdString[i+1] == 'L') { /* L command detected */

            if((cmdString[i+4] != EOF_SYM) || ((i+4) > cmdStringLen)) { /*Detect EOF symbol*/
				return WRONG_STR_FORMAT;	
			}

            switch(cmdString[i+2]) {
                case('1'):
                    led0temp = (cmdString[i+3]-'0');
                break;

                case('2'):
                    led1temp = (cmdString[i+3]-'0');
                break;

                case('3'):
                    led2temp = (cmdString[i+3]-'0');
                break;

                case('4'):
                    led3temp = (cmdString[i+3]-'0');
                break;
                
                default:
                return WRONG_STR_FORMAT;
            }
            return EXIT_SUCCESS;

        }else if(cmdString[i+1] == 'T') { /* T command detected */

			if(cmdString[i+7] != EOF_SYM)  {/*Detect EOF symbol*/
					return WRONG_STR_FORMAT;	
			}
            switch(cmdString[i+2]) {
                case('1'):
                    thread_temperature_period = (cmdString[i+3] - '0') * 1000 + (cmdString[i+4] - '0') * 100 + (cmdString[i+5] - '0') * 10 + (cmdString[i+6] - '0') * 1;
                    update_temperature_period = 1;
                break;

                case('2'):
                    thread_button_period = (cmdString[i+3] - '0') * 1000 + (cmdString[i+4] - '0') * 100 + (cmdString[i+5] - '0') * 10 + (cmdString[i+6] - '0') * 1;                    
                    update_button_period = 1;
                break;

                case('3'):
                    thread_led_period = (cmdString[i+3] - '0') * 1000 + (cmdString[i+4] - '0') * 100 + (cmdString[i+5] - '0') * 10 + (cmdString[i+6] - '0') * 1;
                    update_led_period = 1;
                break;

                default:
                    return WRONG_STR_FORMAT;                        
                }
			return EXIT_SUCCESS;
		}else {
			return CMD_NOT_FOUND;
		}		
	}
	/* cmd string not null and SOF not found */
	return WRONG_STR_FORMAT;
}

/* ******************************** */
/* Adds a char to the cmd string 	*/
/* Returns: 				        */
/*  	 0: if success 		        */
/* 		-1: if cmd string full 	    */
/* ******************************** */
int newCmdChar(unsigned char newChar) {
	/* If cmd string not full add char to it */
	if (cmdStringLen < MAX_CMDSTRING_SIZE) {
		cmdString[cmdStringLen] = newChar;
		cmdStringLen ++;
		return EXIT_SUCCESS;
	}
	/* If cmd string full return error */
	return STRING_FULL;
}

/* Resets the command string */  
void resetCmdString(void) {
	cmdStringLen = 0;
	SOF_C = 0;
	EOF_C = 0;	
    strFull = 0;	
}

static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data) {
	
    switch (evt->type) {

	case UART_RX_RDY:
        if(evt->data.rx.len > 0){
            if(evt->data.rx.buf[evt->data.rx.offset] == '\r') {
                cmd = 1;
                break;
            }
            res = newCmdChar(evt->data.rx.buf[evt->data.rx.offset]);
            if(res != 0) {
                strFull = 1;
            }
        }
        break;

	case UART_RX_DISABLED:
		uart_rx_enable(dev ,rx_buf,sizeof rx_buf,RECEIVE_TIMEOUT);
		break;
		
	default:
		break;
    }
}

/* Thread temperature code implementation */
void thread_temperature_code(void *argA , void *argB, void *argC)
{
    /* Local vars */
    int64_t fin_time=0;     /* Timing variables to control task periodicity */  
    
    /* Task init code */
    printk("Thread temperature init (periodic)\n");

    /* Compute next release instant */
    release_time_temperature = k_uptime_get() + thread_temperature_period;

    /* Thread loop */
    while(1) {		
		
        if(update_temperature_period){
            release_time_temperature = k_uptime_get() + thread_temperature_period;
            update_temperature_period = 0;
        }

		ret = i2c_read_dt(&dev_i2c, &temp, sizeof(temp));
		if(ret != 0){
			printk("Failed to write/read I2C device address %x at Reg. %x \r\n", dev_i2c.addr,config);
		}

		if(temp >= 128){
			temp = 128 - temp;
		}

        /* Wait for next release instant */ 
        fin_time = k_uptime_get();
        if( fin_time < release_time_temperature) {
            k_msleep(release_time_temperature - fin_time);
            release_time_temperature += thread_temperature_period;

        }
    }

    /* Stop timing functions */
    timing_stop();
}

/* Thread button code implementation */
void thread_button_code(void *argA , void *argB, void *argC)
{
    /* Local vars */
    int64_t fin_time=0;     /* Timing variables to control task periodicity */  
        
    /* Task init code */
    printk("Thread button init (periodic)\n");

    /* Compute next release instant */
    release_time_button = k_uptime_get() + thread_button_period;

    /* Thread loop */
    while(1) {        
       
        if(update_button_period){
            release_time_button = k_uptime_get() + thread_button_period;
            update_button_period = 0;
        }

        But0 = gpio_pin_get_dt(&but0_dev);
        But1 = gpio_pin_get_dt(&but1_dev);
        But2 = gpio_pin_get_dt(&but2_dev);
        But3 = gpio_pin_get_dt(&but3_dev);

        /* Wait for next release instant */ 
        fin_time = k_uptime_get();
        if( fin_time < release_time_button) {
            k_msleep(release_time_button - fin_time);
            release_time_button += thread_button_period;
        }
    }
    /* Stop timing functions */
    timing_stop();
}

/* Thread led code implementation */
void thread_led_code(void *argA , void *argB, void *argC)
{
    /* Local vars */
    int64_t fin_time=0;     /* Timing variables to control task periodicity */    
        
    /* Task init code */
    printk("Thread led init (periodic)\n");

    /* Compute next release instant */
    release_time_led = k_uptime_get() + thread_led_period;

    /* Thread loop */
    while(1) {     

        if(update_led_period){
            release_time_led = k_uptime_get() + thread_led_period;
            update_led_period = 0;
        }
        led0stat = led0temp;
        led1stat = led1temp;
        led2stat = led2temp;
        led3stat = led3temp;
       /* checks the value of each led in the table and sets it*/
        gpio_pin_set_dt(&led0_dev,led0stat);
        gpio_pin_set_dt(&led1_dev,led1stat);
        gpio_pin_set_dt(&led2_dev,led2stat);
        gpio_pin_set_dt(&led3_dev,led3stat);

        /* Wait for next release instant */ 
        fin_time = k_uptime_get();
        if( fin_time < release_time_led) {
            k_msleep(release_time_led - fin_time);
            release_time_led += thread_led_period;
        }
    }
    /* Stop timing functions */
    timing_stop();
}

/* Thread cmd code implementation */
void thread_cmd_code(void *argA , void *argB, void *argC)
{
    /* Local vars */
    int64_t fin_time=0, release_time = 0;     /* Timing variables to control task periodicity */    
        
    /* Task init code */
    printk("Thread cmd init (periodic)\n");

    /* Compute next release instant */
    release_time = k_uptime_get() + thread_cmd_period;

    /* Thread loop */
    while(1) {        
        if(cmd){
            res = cmdProcessor();
            printk("\n\rcmdProcessor output: %d\n\r", res);
            resetCmdString();
            cmd=0;
        }

        /* Wait for next release instant */ 
        fin_time = k_uptime_get();
        if( fin_time < release_time) {
            k_msleep(release_time - fin_time);
            release_time += thread_cmd_period;
        }
    }

    /* Stop timing functions */
    timing_stop();
}

/* Thread print code implementation */
void thread_print_code(void *argA , void *argB, void *argC)
{
    /* Local vars */
    int64_t fin_time=0, release_time=0;     /* Timing variables to control task periodicity */    
        
    /* Task init code */
    printk("Thread led init (periodic)\n");

    /* Compute next release instant */
    release_time = k_uptime_get() + thread_print_period;

    /* Thread loop */
    while(1) {    
        printk("\033[2J\033[H");  
       /* Print menu */       
	    printk("#######     Menu     #######\n\n\r   Button state (0-ON  1-OFF):\n\r   Button 1 : %d   Button 2 : %d   Button 3 : %d   Button 4 : %d\n\n\r",But0,But1,But2,But3);
	    printk("   Led state(1-ON  0-OFF):\n\r   Led 1 : %d   Led 2 : %d   Led 3 : %d   Led 4 : %d\n\n\r",led0stat,led1stat,led2stat,led3stat);
		printk("   Temperature : %d C\n\r",temp);
        printk("   Command : ");
        for(int i = 0; i < cmdStringLen; i++) {
            printk("%c",cmdString[i]);
        }
        if(strFull) {
            printk("        String is full! Press enter!");
        }
        /* Wait for next release instant */ 
        fin_time = k_uptime_get();
        if( fin_time < release_time) {
            k_msleep(release_time - fin_time);
            release_time += thread_print_period;

        }
    }

    /* Stop timing functions */
    timing_stop();
}