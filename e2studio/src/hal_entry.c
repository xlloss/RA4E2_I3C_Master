/***********************************************************************************************************************
 * File Name    : hal_entry.c
 * Description  : Contains data structures and functions used in hal_entry.c.
 **********************************************************************************************************************/
/***********************************************************************************************************************
 * DISCLAIMER
 * This software is supplied by Renesas Electronics Corporation and is only intended for use with Renesas products. No
 * other uses are authorized. This software is owned by Renesas Electronics Corporation and is protected under all
 * applicable laws, including copyright laws.
 * THIS SOFTWARE IS PROVIDED "AS IS" AND RENESAS MAKES NO WARRANTIES REGARDING
 * THIS SOFTWARE, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING BUT NOT LIMITED TO WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT. ALL SUCH WARRANTIES ARE EXPRESSLY DISCLAIMED. TO THE MAXIMUM
 * EXTENT PERMITTED NOT PROHIBITED BY LAW, NEITHER RENESAS ELECTRONICS CORPORATION NOR ANY OF ITS AFFILIATED COMPANIES
 * SHALL BE LIABLE FOR ANY DIRECT, INDIRECT, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR ANY REASON RELATED TO THIS
 * SOFTWARE, EVEN IF RENESAS OR ITS AFFILIATES HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
 * Renesas reserves the right, without notice, to make changes to this software and to discontinue the availability of
 * this software. By using this software, you agree to the additional terms and conditions found by accessing the
 * following link:
 * http://www.renesas.com/disclaimer
 *
 * Copyright (C) 2020 Renesas Electronics Corporation. All rights reserved.
 ***********************************************************************************************************************/

#include "common_utils.h"
#include "i3c_master_ep.h"

/*******************************************************************************************************************//**
 * @addtogroup i3c_master_ep
 * @{
 **********************************************************************************************************************/

FSP_CPP_HEADER
void R_BSP_WarmStart(bsp_warm_start_event_t event);
FSP_CPP_FOOTER

/* Global Variables */
static volatile uint32_t g_i3c_event_count[I3C_EVENT_INTERNAL_ERROR + ONE];
static volatile uint32_t g_i3c_event_status = RESET_VALUE;

/* Configure the information for the slave device. */
static i3c_device_cfg_t master_device_cfg =
{
    /* This is the Static I3C / I2C Legacy address defined by the device manufacturer. */
    .static_address  = I3C_MASTER_DEVICE_STATIC_ADDRESS,
    /* The dynamic address will be automatically updated when the master configures this device using CCC ENTDAA. */
    .dynamic_address = I3C_MASTER_DEVICE_DYNAMIC_ADDRESS
};

/* I3C bus/device management */
static i3c_device_table_cfg_t       g_device_table_cfg;

static volatile bool b_process_timeout = false;


/* Private function declarations.*/
static uint32_t i3c_app_event_notify(uint32_t set_event_flag_value, uint32_t timout);
static fsp_err_t start_timeout_timer_with_defined_ms(uint32_t timeout_ms);
static void agt_deinit(void);

void i3c_legacy_i2_init();
int i3c_read(uint8_t *p_read_data, uint8_t len, uint8_t enable_resatrt);
int i3c_write(uint8_t *p_write_data, uint8_t len, uint8_t enable_resatrt);
void i3c_deinit(void);

void i3c_legacy_i2_init()
{
    fsp_err_t err = FSP_SUCCESS;

    /* Initializes the I3C module. */
    err = R_I3C_Open(&g_i3c0_ctrl, &g_i3c0_cfg);
    if (FSP_SUCCESS != err) {
        APP_ERR_PRINT ("\r\nERROR : R_I3C_Open API FAILED \r\n");
        /* de-initialize the opened AGT timer module.*/
        agt_deinit();
        APP_ERR_TRAP(err);
    }
    APP_PRINT("\r\nINFO : I3C Initialized successfully in master mode.\r\n");

    /* Set the device configuration for this device. */
    err = R_I3C_DeviceCfgSet(&g_i3c0_ctrl, &master_device_cfg);
    if (FSP_SUCCESS != err) {
        APP_ERR_PRINT ("\r\nERROR : R_I3C_DeviceCfgSet API FAILED \r\n");
        /* de-initialize the opened I3C and AGT timer module.*/
        i3c_deinit();
        agt_deinit();
        APP_ERR_TRAP(err);
    }

    /* Set the I3C devices information through device table entries */
    memset(&g_device_table_cfg, RESET_VALUE, sizeof(i3c_device_table_cfg_t));

    g_device_table_cfg.dynamic_address = 0;
    g_device_table_cfg.static_address = EEPROM_I2C_ADDRESS;
    g_device_table_cfg.device_protocol = I3C_DEVICE_PROTOCOL_I2C;
    g_device_table_cfg.ibi_accept = false;
    g_device_table_cfg.ibi_payload = false;
    g_device_table_cfg.master_request_accept = false;

    err = R_I3C_MasterDeviceTableSet(&g_i3c0_ctrl, RESET_VALUE, &g_device_table_cfg);
    if (FSP_SUCCESS != err) {
        APP_ERR_PRINT ("\r\nERROR : R_I3C_MasterDeviceTableSet API FAILED \r\n");
        /* de-initialize the opened I3C and AGT timer module.*/
        i3c_deinit();
        agt_deinit();
        APP_ERR_TRAP(err);
    }

    /* Enable I3C Mode. */
    err = R_I3C_Enable(&g_i3c0_ctrl);
    if (FSP_SUCCESS != err) {
        APP_ERR_PRINT ("\r\nERROR : R_I3C_Enable API FAILED \r\n");
        /* de-initialize the opened I3C and AGT timer module.*/
        i3c_deinit();
        agt_deinit();
        APP_ERR_TRAP(err);
    }

    /* Select the configured device for the following operations. */
    err = R_I3C_DeviceSelect(&g_i3c0_ctrl, 0, I3C_BITRATE_MODE_I2C_STDBR);
    if (FSP_SUCCESS != err) {
        APP_ERR_PRINT ("\r\nERROR : R_I3C_DeviceSelect API FAILED \r\n");
        /* de-initialize the opened I3C and AGT timer module.*/
        i3c_deinit();
        agt_deinit();
        APP_ERR_TRAP(err);
    }

}

void i3c_deinit(void)
{
    fsp_err_t err = FSP_SUCCESS;

    err = R_I3C_Close(&g_i3c0_ctrl);
    if (FSP_SUCCESS != err) {
        /* I3C Close failure message */
        APP_ERR_PRINT("\r\nERROR : R_I3C_Close API FAILED.\r\n");
    }
}

int i3c_write(uint8_t *p_write_data, uint8_t len, uint8_t enable_resatrt)
{
    fsp_err_t err = FSP_SUCCESS;
    uint32_t status = RESET_VALUE;

    err = R_I3C_Write(&g_i3c0_ctrl, p_write_data, len, enable_resatrt);
    if (FSP_SUCCESS != err) {
        APP_ERR_PRINT ("\r\nERROR : R_I3C_Write API FAILED \r\n");
        /* de-initialize the opened I3C and AGT timer module.*/
        i3c_deinit();
        agt_deinit();
        APP_ERR_TRAP(err);
    }

    APP_PRINT("\rwait for write complete event\r\n");
    status = i3c_app_event_notify(I3C_EVENT_FLAG_WRITE_COMPLETE, WAIT_TIME);
    if (RESET_VALUE == status) {
        APP_ERR_PRINT ("\r\nERROR : Requested event not received with in specified timeout.\r\n");
        return FSP_ERR_TIMEOUT;
    }

    APP_PRINT("\rRITE Complete Get\r\n");
    return FSP_SUCCESS;
}


int i3c_read(uint8_t *p_read_data, uint8_t len, uint8_t enable_resatrt)
{
    fsp_err_t err = FSP_SUCCESS;
    uint32_t status = RESET_VALUE;

    err = R_I3C_Read(&g_i3c0_ctrl, p_read_data, len, enable_resatrt);
    if (FSP_SUCCESS != err) {
        APP_ERR_PRINT ("\r\nERROR : R_I3C_Read API FAILED \r\n");
        i3c_deinit();
        agt_deinit();
        APP_ERR_TRAP(err);
    }

    status = i3c_app_event_notify(I3C_EVENT_FLAG_READ_COMPLETE, WAIT_TIME);
    if (RESET_VALUE == status) {
        APP_ERR_PRINT ("\r\nERROR : Requested event not received with in specified timeout.\r\n");
        return FSP_ERR_TIMEOUT;
    }

    return FSP_SUCCESS;
}

void hal_entry(void)
{
    fsp_err_t err = FSP_SUCCESS;
    fsp_pack_version_t version = {RESET_VALUE};
    unsigned char input_data[BUFFER_SIZE_DOWN] = {RESET_VALUE};
    uint8_t converted_rtt_input = RESET_VALUE;
    uint8_t p_read_data[1] = {0};
    uint8_t p_write_data[2] = {0x00, 0x65};

    R_FSP_VersionGet(&version);
    APP_PRINT(BANNER_INFO, EP_VERSION, version.version_id_b.major,
                version.version_id_b.minor, version.version_id_b.patch );
    APP_PRINT(EP_INFO);

    /* Initialize AGT driver */
    err = R_AGT_Open(&g_timeout_timer_ctrl, &g_timeout_timer_cfg);
    if (FSP_SUCCESS != err) {
        APP_ERR_PRINT ("\r\nERROR : R_AGT_Open API FAILED \r\n");
        APP_ERR_TRAP(err);
    }

    i3c_legacy_i2_init();

    while (true) {
        if (APP_CHECK_DATA) {
            memset(&input_data[RESET_VALUE], NULL_CHAR, BUFFER_SIZE_DOWN);
            converted_rtt_input = RESET_VALUE;

            APP_READ (input_data);
            converted_rtt_input = (uint8_t)atoi((char *)input_data);
            APP_PRINT("\r\nINFO : converted_rtt_input %d\r\n", converted_rtt_input);
            switch (converted_rtt_input) {
                case 1:
                    APP_PRINT("\r\nINFO : Write EEPROM Address 0, Vaule 0x33\r\n");
                    err = i3c_write(p_write_data, 2, 0);
                    if (err != FSP_SUCCESS)
                        APP_PRINT("\r\nINFO : i3c_write fail\r\n");

                    break;

                case 2:
                    /* dummy write */
                    APP_PRINT("\r\nINFO : Dummy Write EEPROM Address 0\r\n");
                    err = i3c_write(p_write_data, 1, 0);
                    if (err != FSP_SUCCESS)
                        APP_PRINT("\r\nINFO : i3c_write fail\r\n");
                    break;

                case 3:
                    err = i3c_read(p_read_data, 1, 0);
                    if (err != FSP_SUCCESS) {
                        APP_PRINT("\r\nINFO : i3c_write fail\r\n");
                    } else {
                        APP_PRINT ("p_read_data 0x%x\r\n", (uint8_t)p_read_data[0]);
                        APP_PRINT("\r\nINFO : Read EEPROM Address 0, Value = 0x%x\r\n",
                            p_read_data[0]);
                    }

                    break;

                default:
                break;

            }
        }
    }
}

/*******************************************************************************************************************//**
 * This function is called at various points during the startup process.  This implementation uses the event that is
 * called right before main() to set up the pins.
 *
 * @param[in]  event    Where at in the start up process the code is currently at
 **********************************************************************************************************************/
void R_BSP_WarmStart(bsp_warm_start_event_t event)
{
    if (BSP_WARM_START_RESET == event)
    {
#if BSP_FEATURE_FLASH_LP_VERSION != 0

        /* Enable reading from data flash. */
        R_FACI_LP->DFLCTL = 1U;

        /* Would normally have to wait tDSTOP(6us) for data flash recovery. Placing the enable here, before clock and
         * C runtime initialization, should negate the need for a delay since the initialization will typically take more than 6us. */
#endif
    }

    if (BSP_WARM_START_POST_C == event)
    {
        /* C runtime environment and system clocks are setup. */

        /* Configure pins. */
        R_IOPORT_Open (&g_ioport_ctrl, g_ioport.p_cfg);
    }
}


/*******************************************************************************************************************//**
 * @brief This function is callback for i3c.
 *
 * @param[IN] p_args
 **********************************************************************************************************************/
void i2c_legacy_basic_callback (i3c_callback_args_t const * const p_args)
{
    /* update the event in global array and this will be used in i3c_app_event_notify function.*/
    g_i3c_event_status = p_args->event_status;
    g_i3c_event_count[p_args->event]++;

    switch (p_args->event)
    {
        case I3C_EVENT_WRITE_COMPLETE:
        {
            break;
        }
        case I3C_EVENT_READ_COMPLETE:
        {
            break;
        }
        default:
        {
            break;
        }
    }
}

/*******************************************************************************************************************//**
 * @brief This function starts the timer and wait for the event set in the i3c callback till specified timeout.
 * @param[IN]   set_event_flag_value  requested event flag
 * @param[IN]   timeout               specified timeout
 * @retval      on successful operation, returns i3c event flag value.
 **********************************************************************************************************************/
static uint32_t i3c_app_event_notify(uint32_t set_event_flag_value, uint32_t timeout)
{
    fsp_err_t       err = FSP_SUCCESS;
    uint32_t        get_event_flag_value = RESET_VALUE;
    /* Reset the timeout flag. */
    b_process_timeout = false;

    /* start the timer.*/
    err = start_timeout_timer_with_defined_ms(timeout);
    if(FSP_SUCCESS != err)
    {
        APP_ERR_PRINT("\r\nERROR : start_timeout_timer_with_defined_ms function failed.\r\n");
        /* de-initialize the opened I3C and AGT timer module.*/
        i3c_deinit();
        agt_deinit();
        APP_ERR_TRAP(err);
    }

    /* wait for the event set in the i3c callback till specified timeout.*/
    while (!b_process_timeout)
    {
        /* process for all i3c events.*/
        for(uint8_t cnt = RESET_VALUE; cnt < (I3C_EVENT_INTERNAL_ERROR+ONE); cnt++)
        {
            /* check for callback event.*/
            if(g_i3c_event_count[cnt] > RESET_VALUE)
            {
                /* store the event in local variable.*/
                get_event_flag_value |= (uint32_t)(0x1 << cnt);
                g_i3c_event_count[cnt] -= ONE;
            }
        }

        /* check for event received from i3c callback function is similar to event which user wants.*/
        get_event_flag_value = (set_event_flag_value & get_event_flag_value);
        if(get_event_flag_value)
        {
            g_i3c_event_status = RESET_VALUE;
            return get_event_flag_value;
        }
    }
    return 0;
}

/* timer related functions */
static uint32_t timeout_value_in_ms = RESET_VALUE;

/*******************************************************************************************************************//**
 * @brief This function is callback for periodic mode timer and stops AGT0 timer in Periodic mode.
 *
 * @param[in] (timer_callback_args_t *) p_args
 **********************************************************************************************************************/
void g_timeout_timer_callback(timer_callback_args_t *p_args)
{
    FSP_PARAMETER_NOT_USED(p_args);

    /* check if specified timeout is zero.*/
    if(RESET_VALUE == --timeout_value_in_ms)
    {
        /* set the timeout flag.*/
        b_process_timeout = true;
        /* stop AGT timer.*/
        R_AGT_Stop(&g_timeout_timer_ctrl);
    }
}

/*******************************************************************************************************************//**************
 * @brief       This function Resets the counter value and start the AGT timer.
 * @param[IN]   timeout_ms
 * @retval      FSP_SUCCESS or Any Other Error code apart from FSP_SUCCESS upon unsuccessful start_timeout_timer_with_defined_ms.
 ***********************************************************************************************************************************/
static fsp_err_t start_timeout_timer_with_defined_ms(uint32_t timeout_ms)
{
    fsp_err_t err = FSP_SUCCESS;

    /* update the specified timeout into a global variable and this will be checked in timer callback.*/
    timeout_value_in_ms = timeout_ms;
    /* Resets the counter value.*/
    err = R_AGT_Reset(&g_timeout_timer_ctrl);
    if (FSP_SUCCESS != err)
    {
        APP_ERR_PRINT ("\r\nERROR : R_AGT_Reset API FAILED \r\n");
        return err;
    }

    /* start the AGT timer.*/
    err = R_AGT_Start(&g_timeout_timer_ctrl);
    if (FSP_SUCCESS != err)
    {
        APP_ERR_PRINT ("\r\nERROR : R_AGT_Start API FAILED \r\n");
    }
    return err;
}

/*******************************************************************************************************************//**
 * @brief This function closes opened AGT module before the project ends up in an Error Trap.
 * @param[IN]   None
 * @retval      None
 **********************************************************************************************************************/
void agt_deinit(void)
{
    fsp_err_t err = FSP_SUCCESS;

    /* Close AGT0 module */
    err = R_AGT_Close(&g_timeout_timer_ctrl);
    /* handle error */
    if (FSP_SUCCESS != err)
    {
        /* AGT0 Close failure message */
        APP_ERR_PRINT("\r\nERROR : R_AGT_Close API FAILED.\r\n");
    }
}

/*******************************************************************************************************************//**
 * @} (end addtogroup i3c_master_ep)
 **********************************************************************************************************************/
