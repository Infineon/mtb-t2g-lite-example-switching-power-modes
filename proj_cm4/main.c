/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for CM4 in TRAVEO T2G MCU switching between 
*              power modes application for ModusToolbox, and demonstrates how 
*              to transition TRAVEO T2G MCU to the following power states:
*              - Power states - Active / Sleep / Deep Sleep / Hibernate
*
* Related Document: See README.md
*
*
*******************************************************************************
* Copyright 2020-2023, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"


/*******************************************************************************
* Macros
********************************************************************************/
/* Constants to define LONG and SHORT presses on User Button (x10 = ms) */
#define QUICK_PRESS_COUNT       2u      /* 20 ms < press < 200 ms */
#define SHORT_PRESS_COUNT       20u     /* 200 ms < press < 2 sec */
#define LONG_PRESS_COUNT        200u    /* press > 2 sec */

/* PWM LED frequency constants (in Hz) */
#define PWM_FREQ_HZ             3
#define PWM_DIM_FREQ_HZ         100

/* PWM Duty cycles (Active Low, in %) */
#define PWM_50P_DUTY_CYCLE      50.0f
#define PWM_10P_DUTY_CYCLE      90.0f

/* Glitch delays */
#define SHORT_GLITCH_DELAY_MS   10u     /* in ms */
#define LONG_GLITCH_DELAY_MS    200u    /* in ms */

typedef enum
{
    SWITCH_NO_EVENT     = 0u,
    SWITCH_QUICK_PRESS  = 1u,
    SWITCH_SHORT_PRESS  = 2u,
    SWITCH_LONG_PRESS   = 3u,
} en_switch_event_t;

/*****************************************************************************
* Function Prototypes
********************************************************************************/
en_switch_event_t get_switch_event(void);
void handle_error(void);
/* Power callbacks */
bool pwm_power_callback(cyhal_syspm_callback_state_t state, cyhal_syspm_callback_mode_t mode, void *arg);

/*******************************************************************************
* Global Variables
********************************************************************************/
/* HAL Objects */
cyhal_pwm_t pwm;


/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* This is the main function for the CM7_0. It does...
*    1. Initialize the UART block for uart logging.
*    2. Initialize the PWM block that controls the LED brightness.
*    3. Register power management callbacks.
*    Do Forever loop:
*    4. Check if User button was pressed and for how long.
*    5. If quickly pressed, go to Sleep mode.
*    6. If short pressed, go to DeepSleep mode.
*    7. If long pressed, go to Hibernate mode.
*
* Parameters:
*  void
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    /* API return code */
    cy_rslt_t result;

    /* Initialize the device and board peripherals */
    result = cybsp_init() ;
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Enable global interrupts */
    __enable_irq();

    /* Initialize retarget-io for uart logging */
    result = cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, 
                                CY_RETARGET_IO_BAUDRATE);
    if (result != CY_RSLT_SUCCESS)
    {
        handle_error();
    }

    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    printf("\x1b[2J\x1b[;H");
    printf("********************************************************************************\r\n");
    printf("TRAVEO T2G MCU: Switching between power modes\r\n");
    printf("********************************************************************************\r\n");
    printf("Quick press 'SW2' key to Sleep mode, short press 'SW2' key to DeepSleep mode.\r\n");
    printf("Long press 'SW2' key to Hibernate mode.\r\n\r\n");

    /* Initialize the User Button */
    cyhal_gpio_init(CYBSP_USER_BTN, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_PULLUP, CYBSP_BTN_OFF);
    /* Enable the GPIO interrupt to wake-up the device */
    cyhal_gpio_enable_event(CYBSP_USER_BTN, CYHAL_GPIO_IRQ_FALL, CYHAL_ISR_PRIORITY_DEFAULT, true);

    /* Initialize the PWM to control LED brightness */
    cyhal_pwm_init(&pwm, CYBSP_USER_LED, NULL);
    cyhal_pwm_set_duty_cycle(&pwm, PWM_50P_DUTY_CYCLE, PWM_FREQ_HZ);
    cyhal_pwm_start(&pwm);

    /* Check the reset reason */
    if(CY_SYSLIB_RESET_HIB_WAKEUP == (Cy_SysLib_GetResetReason() & CY_SYSLIB_RESET_HIB_WAKEUP))
    {
        /* Wait a bit to avoid glitches from the button press */
        cyhal_system_delay_ms(LONG_GLITCH_DELAY_MS);
        /* The reset has occurred on a wakeup from Hibernate power mode */
        printf("Wake up from the Hibernate mode\r\n");
    }

    /* Callback declaration for Power Modes */
    cyhal_syspm_callback_data_t pwm_callback = {pwm_power_callback,             /* Callback function */
                                               (cyhal_syspm_callback_state_t)
                                               (CYHAL_SYSPM_CB_CPU_SLEEP |
                                                CYHAL_SYSPM_CB_CPU_DEEPSLEEP |
                                                CYHAL_SYSPM_CB_SYSTEM_HIBERNATE), /* Power States supported */
                                               (cyhal_syspm_callback_mode_t)
                                               (CYHAL_SYSPM_CHECK_FAIL),        /* Modes to ignore */
                                                NULL,                           /* Callback Argument */
                                                NULL};                          /* For internal use */
    /* Initialize the System Power Management */
    cyhal_syspm_init();
    /* Power Management Callback registration */
    cyhal_syspm_register_callback(&pwm_callback);
    
    for(;;)
    {
        switch (get_switch_event())
        {
            case SWITCH_QUICK_PRESS:
                /* Print out the information, wait a bit to UART output */
                printf("Go to Sleep mode.\r\n");
                cyhal_system_delay_ms(SHORT_GLITCH_DELAY_MS);

                /* Go to sleep */
                cyhal_syspm_sleep();

                printf("Wake up from Sleep mode.\r\n");
                /* Wait a bit to avoid glitches from the button press */
                cyhal_system_delay_ms(LONG_GLITCH_DELAY_MS);
                break;
            
            case SWITCH_SHORT_PRESS:
                /* Print out the information, wait a bit to UART output */
                printf("Go to DeepSleep mode.\r\n");
                cyhal_system_delay_ms(SHORT_GLITCH_DELAY_MS);

                /* Go to deep sleep */
                cyhal_syspm_deepsleep();

                printf("Wake up from DeepSleep mode.\r\n");
                /* Wait a bit to avoid glitches from the button press */
                cyhal_system_delay_ms(LONG_GLITCH_DELAY_MS);
                break;

            case SWITCH_LONG_PRESS:
                /* Print out the information, wait a bit to UART output */
                printf("Go to Hibernate mode.\r\n");
                cyhal_system_delay_ms(SHORT_GLITCH_DELAY_MS);

                /* Go to hibernate and Configure a low logic level for the first wakeup-pin */
                cyhal_syspm_hibernate(CYHAL_SYSPM_HIBERNATE_PINA_LOW);
                break;

            default:
                break;
        }
    }
}

/*******************************************************************************
* Function Name: get_switch_event
****************************************************************************//**
* Summary:
*  Returns how the User button was pressed:
*  - SWITCH_NO_EVENT: No press 
*  - SWITCH_QUICK_PRESS: Very quick press
*  - SWITCH_SHORT_PRESS: Short press was detected
*  - SWITCH_LONG_PRESS: Long press was detected
*
* Return:
*  Switch event that occurred, if any. 
*
*******************************************************************************/
en_switch_event_t get_switch_event(void)
{
    en_switch_event_t event = SWITCH_NO_EVENT;
    uint32_t pressCount = 0;

    /* Check if User button is pressed */
    while (cyhal_gpio_read(CYBSP_USER_BTN) == CYBSP_BTN_PRESSED)
    {
        /* Wait for 10 ms */
        cyhal_system_delay_ms(10);

        /* Increment counter. Each count represents 10 ms */
        pressCount++;
    }

    /* Check for how long the button was pressed */
    if (pressCount > LONG_PRESS_COUNT)
    {
        event = SWITCH_LONG_PRESS;
    }
    else if (pressCount > SHORT_PRESS_COUNT)
    {
        event = SWITCH_SHORT_PRESS;
    }
    else if (pressCount > QUICK_PRESS_COUNT)
    {
        event = SWITCH_QUICK_PRESS;
    }

    /* Add a delay to avoid glitches */
    cyhal_system_delay_ms(SHORT_GLITCH_DELAY_MS);

    return event;
}

/*******************************************************************************
* Function Name: pwm_power_callback
********************************************************************************
* Summary:
*  Callback implementation for the PWM block. It changes the blinking pattern
*  based on the power state and MCU state.
*
* Parameters:
*  state - state the system or CPU is being transitioned into
*  mode  - callback mode
*  arg   - user argument (not used)
*
* Return:
*  Always true
*
*******************************************************************************/
bool pwm_power_callback(cyhal_syspm_callback_state_t state, cyhal_syspm_callback_mode_t mode, void *arg)
{
    (void) arg;

    /* Stop the PWM before applying any changes */
    cyhal_pwm_stop(&pwm);

    if (mode == CYHAL_SYSPM_BEFORE_TRANSITION)
    {
        if (state == CYHAL_SYSPM_CB_CPU_SLEEP)
        {
            /* Before going to Sleep Mode, set LED brightness to 10% */
            cyhal_pwm_set_duty_cycle(&pwm, PWM_10P_DUTY_CYCLE, PWM_DIM_FREQ_HZ);
            /* Restart the PWM */
            cyhal_pwm_start(&pwm);
        }
    }
    else if (mode == CYHAL_SYSPM_AFTER_TRANSITION)
    {
        switch (state)
        {
            case CYHAL_SYSPM_CB_CPU_SLEEP:
            case CYHAL_SYSPM_CB_CPU_DEEPSLEEP:
                /* After waking up, set the blink pattern */
                cyhal_pwm_set_duty_cycle(&pwm, PWM_50P_DUTY_CYCLE, PWM_FREQ_HZ);
                break;

            default:
                break;
        }

        /* Restart the PWM */
        cyhal_pwm_start(&pwm);
    }

    return true;
}

/*******************************************************************************
* Function Name: handle_error
********************************************************************************
* Summary:
* User defined error handling function
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
void handle_error(void)
{
    /* Disable all interrupts. */
    __disable_irq();

    CY_ASSERT(0);
}

/* [] END OF FILE */
