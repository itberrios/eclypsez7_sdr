/******************************************************************************
* Copyright (C) 2023 Advanced Micro Devices, Inc. All Rights Reserved.
* SPDX-License-Identifier: MIT
******************************************************************************/
/*
 * helloworld.c: simple test application
 *
 * This application configures UART 16550 to baud rate 9600.
 * PS7 UART (Zynq) is not initialized by this application, since
 * bootrom/bsp configures it to baud rate 115200
 *
 * ------------------------------------------------
 * | UART TYPE   BAUD RATE                        |
 * ------------------------------------------------
 *   uartns550   9600
 *   uartlite    Configurable only in HW design
 *   ps7_uart    115200 (configured by bootrom/bsp)
 */

#include <stdio.h>
#include "platform.h"
#include "xil_printf.h"

#include "xparameters.h" // holds register definitions
#include "xgpio.h"       // gpio control
#include "sleep.h"       

XGpio_Config *gpio_config; // pointer to config struct
XGpio gpio; // struct for actual gpio data

void gpio_init(void)
{
    int status = XGpio_Initialize(&gpio, XPAR_AXI_GPIO_0_BASEADDR);
    
    if (status == XST_SUCCESS) {
        xil_printf("AXI GPIO 0 INIT SUCESSEFUL\n\r");
    } else if (status == XST_DEVICE_NOT_FOUND) {
        xil_printf("AXI GPIO 0 INIT FAILED: DEVICE NOT FOUND\n\r");
    } else {
        xil_printf("AXI GPIO 0 INIT FAILED: UNKOWN REASON\n\r");
    }
    
}

// ld_idx = 0 or 1, this is a blockin op
void blink_cycle(size_t ld_idx)
{
    // start value for loop index
    size_t lstart = (ld_idx == 0) ? 0 : 3;
    
    for (size_t i = lstart; i < (lstart + 3); i++)
    {   
        XGpio_DiscreteWrite(&gpio, 2, 1 << i);
        sleep(1);
    }

    // ensure its reset
    XGpio_DiscreteWrite(&gpio, 2, 0);
    
}

u32 gpio1_data;

int main()
{
    init_platform();
    gpio_init();

    // set data directions (input / output)
    XGpio_SetDataDirection(&gpio, 1, 0x0000); // set all bits low for output only
    XGpio_SetDataDirection(&gpio, 2, 0x003F); // set first 6 bits High for input pnly

    while(1) 
    {
        gpio1_data = XGpio_DiscreteRead(&gpio, 1);

        if (gpio1_data == 1) {
            blink_cycle(0); // blink led 0
        } else if (gpio1_data == 2) {
            blink_cycle(1); // blink led 1
        }
        
    }
 
    cleanup_platform();
    return 0;
}