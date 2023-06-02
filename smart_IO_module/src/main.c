/*
 * Paulo Pedreiras, 2023/03
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
#include <zephyr/sys/util.h>
#include <inttypes.h>

/* Size of stack area used by each thread (can be thread specific, if necessary)*/
#define STACK_SIZE 1024

/* Thread scheduling priority */
#define thread_temperature_prio 3
#define thread_button_prio 3
#define thread_led_prio 3
#define thread_print_prio 3

/* Therad periodicity (in ms)*/
#define thread_temperature_period 20
#define thread_button_period 20
#define thread_led_period 20

/* I2C node identifier*/
#define I2C_NODE DT_NODELABEL(tempsensor)


/* Create thread stack space */
K_THREAD_STACK_DEFINE(thread_temperature_stack, STACK_SIZE);
K_THREAD_STACK_DEFINE(thread_button_stack, STACK_SIZE);
K_THREAD_STACK_DEFINE(thread_led_stack, STACK_SIZE);
  
/* Create variables for thread data */
struct k_thread thread_temperature_data;
struct k_thread thread_button_data;
struct k_thread thread_led_data;

/* Create task IDs */
k_tid_t thread_temperature_tid;
k_tid_t thread_button_tid;
k_tid_t thread_led_tid;

/* Thread code prototypes */
void thread_temperature_code(void *, void *, void *);
void thread_button_code(void *, void *, void *);
void thread_led_code(void *, void *, void *);

/* I2C struct*/
static const struct i2c_dt_spec dev_i2c = I2C_DT_SPEC_GET(I2C_NODE);

/* sensor config register*/
uint8_t config = 0x00;

/* Get node IDs. Refer to dts file and HW documentation*/
#define LED0_NID DT_NODELABEL(led0) 
#define LED1_NID DT_NODELABEL(led1)
#define LED2_NID DT_NODELABEL(led2)
#define LED3_NID DT_NODELABEL(led3)  
#define BUT0_NID DT_NODELABEL(button0) 
#define BUT1_NID DT_NODELABEL(button1) 
#define BUT2_NID DT_NODELABEL(button2) 
#define BUT3_NID DT_NODELABEL(button3) 

/* Get gpio device structures */
const struct gpio_dt_spec led0_dev = GPIO_DT_SPEC_GET(LED0_NID,gpios);
const struct gpio_dt_spec led1_dev = GPIO_DT_SPEC_GET(LED1_NID,gpios);
const struct gpio_dt_spec led2_dev = GPIO_DT_SPEC_GET(LED2_NID,gpios);
const struct gpio_dt_spec led3_dev = GPIO_DT_SPEC_GET(LED3_NID,gpios);

const struct gpio_dt_spec but0_dev = GPIO_DT_SPEC_GET(BUT0_NID,gpios);
const struct gpio_dt_spec but1_dev = GPIO_DT_SPEC_GET(BUT1_NID,gpios);
const struct gpio_dt_spec but2_dev = GPIO_DT_SPEC_GET(BUT2_NID,gpios);
const struct gpio_dt_spec but3_dev = GPIO_DT_SPEC_GET(BUT3_NID,gpios);

/* Int related declarations */
//static struct gpio_callback but_cb_data; /* Callback structure */

/* Callback function and variables*/
/* Variables to use when a button is pressed */
volatile int But0 = 0;      
volatile int But1 = 0;      
volatile int But2 = 0;      
volatile int But3 = 0;  

volatile int led0stat = 0; /* Led status variable. Updated by the callback function */
volatile int led1stat = 0; /* Led status variable. Updated by the callback function */
volatile int led2stat = 0; /* Led status variable. Updated by the callback function */
volatile int led3stat = 0; /* Led status variable. Updated by the callback function */

/* Main function */
void main(void) {
    int ret;

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

    /* I2C */

    ret = i2c_write_dt(&dev_i2c, config, sizeof(config));
    if (ret != 0) {
        printk("Failed to write to I2C device address %x at Reg. %x \n",dev_i2c.addr,config);
    }

    // /* Set interrupt HW - which pin and event generate interrupt */
    // ret = gpio_pin_interrupt_configure_dt(&but0_dev, GPIO_INT_EDGE_TO_ACTIVE);
    // if (ret != 0) {
	//     printk("Error %d: failed to configure interrupt on BUT0 pin \n\r", ret);
	//     return;
    // }

    // /* Set interrupt HW - which pin and event generate interrupt */
    // ret = gpio_pin_interrupt_configure_dt(&but1_dev, GPIO_INT_EDGE_TO_ACTIVE);
    // if (ret != 0) {
	//     printk("Error %d: failed to configure interrupt on BUT1 pin \n\r", ret);
	//     return;
    // }

    // /* Set interrupt HW - which pin and event generate interrupt */
    // ret = gpio_pin_interrupt_configure_dt(&but2_dev, GPIO_INT_EDGE_TO_ACTIVE);
    // if (ret != 0) {
	//     printk("Error %d: failed to configure interrupt on BUT2 pin \n\r", ret);
	//     return;
    // }

    // /* Set interrupt HW - which pin and event generate interrupt */
    // ret = gpio_pin_interrupt_configure_dt(&but3_dev, GPIO_INT_EDGE_TO_ACTIVE);
    // if (ret != 0) {
	//     printk("Error %d: failed to configure interrupt on BUT3 pin \n\r", ret);
	//     return;
    // }
    
    // /* Set callback */
    // gpio_init_callback(&but_cb_data, butpress_cbfunction, BIT(but0_dev.pin)| BIT(but1_dev.pin)| BIT(but2_dev.pin) | BIT(but3_dev.pin));
    
    // gpio_add_callback(but0_dev.port, &but_cb_data);
    // gpio_add_callback(but1_dev.port, &but_cb_data);
    // gpio_add_callback(but2_dev.port, &but_cb_data);
    // gpio_add_callback(but3_dev.port, &but_cb_data);

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

    return;

} 

// void butpress_cbfunction(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    
//     /* Test each button ...*/
//     if(BIT(but0_dev.pin) & pins) {
//         /* Update global var*/
//         But1 = !But1;
//     }

//     if(BIT(but1_dev.pin) & pins) {
//         /* Update global var*/        
//         But2 = !But2;
//     }

//     if(BIT(but2_dev.pin) & pins) {
//         /* Update global var*/        
//         But3 = !But3;
//     }

//     if(BIT(but3_dev.pin) & pins) {
//         /* Update global var*/        
//         But4 = !But4;
//     }

// }

/* Thread temperature code implementation */
void thread_temperature_code(void *argA , void *argB, void *argC)
{
    /* Local vars */
    int64_t fin_time=0, release_time=0;     /* Timing variables to control task periodicity */    
        
    /* Task init code */
    printk("Thread temperature init (periodic)\n");

    /* Compute next release instant */
    release_time = k_uptime_get() + thread_temperature_period;

    /* Thread loop */
    while(1) {        
       


        /* Wait for next release instant */ 
        fin_time = k_uptime_get();
        if( fin_time < release_time) {
            k_msleep(release_time - fin_time);
            release_time += thread_temperature_period;

        }
    }

    /* Stop timing functions */
    timing_stop();
}

/* Thread button code implementation */
void thread_button_code(void *argA , void *argB, void *argC)
{
    /* Local vars */
    int64_t fin_time=0, release_time=0;     /* Timing variables to control task periodicity */  
    int temp0, temp1, temp2, temp3;
        
    /* Task init code */
    printk("Thread button init (periodic)\n");

    /* Compute next release instant */
    release_time = k_uptime_get() + thread_button_period;

    /* Thread loop */
    while(1) {        
       
       temp0 = gpio_pin_get_dt(&but0_dev);
       temp1 = gpio_pin_get_dt(&but1_dev);
       temp2 = gpio_pin_get_dt(&but2_dev);
       temp3 = gpio_pin_get_dt(&but3_dev);

       if(!temp0){
        But0 = 1;
       } else {
        But0 = 0;
       }

       if(!temp1){
        But1 = 1;
       } else {
        But1 = 0;
       }

       if(!temp2){
        But2 = 1;
       } else {
        But2 = 0;
       }

       if(!temp3){
        But3 = 1;
       } else {
        But3 = 0;
       }

        /* Wait for next release instant */ 
        fin_time = k_uptime_get();
        if( fin_time < release_time) {
            k_msleep(release_time - fin_time);
            release_time += thread_temperature_period;

        }
    }

    /* Stop timing functions */
    timing_stop();
}

/* Thread led code implementation */
void thread_led_code(void *argA , void *argB, void *argC)
{
    /* Local vars */
    int64_t fin_time=0, release_time=0;     /* Timing variables to control task periodicity */    
        
    /* Task init code */
    printk("Thread led init (periodic)\n");

    /* Compute next release instant */
    release_time = k_uptime_get() + thread_led_period;

    /* Thread loop */
    while(1) {        
       /* checks the value of each led in the table and sets it*/
        gpio_pin_set_dt(&led0_dev,led0stat);
        gpio_pin_set_dt(&led1_dev,led1stat);
        gpio_pin_set_dt(&led2_dev,led2stat);
        gpio_pin_set_dt(&led3_dev,led3stat);

        /* Wait for next release instant */ 
        fin_time = k_uptime_get();
        if( fin_time < release_time) {
            k_msleep(release_time - fin_time);
            release_time += thread_temperature_period;

        }
    }

    /* Stop timing functions */
    timing_stop();
}
