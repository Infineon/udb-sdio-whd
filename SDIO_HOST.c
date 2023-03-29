/***********************************************************************************************//**
 * \file SDIO_HOST.c
 *
 * \brief
 *  This file provides the source code to the API for the UDB based SDIO driver.
 *
 ***************************************************************************************************
 * \copyright
 * Copyright 2016-2022 Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 **************************************************************************************************/

#include "SDIO_HOST.h"
#include "cy_utils.h"
#include "cy_gpio.h"
#include "cybsp.h"
#if defined(CYHAL_UDB_SDIO)
#include "cyhal.h"
#if defined(_CYHAL_DRIVER_AVAILABLE_IRQ) && (_CYHAL_DRIVER_AVAILABLE_IRQ)
#include "cyhal_irq_impl.h"
#endif /* defined(_CYHAL_DRIVER_AVAILABLE_IRQ) && (_CYHAL_DRIVER_AVAILABLE_IRQ) */


#if defined(__cplusplus)
extern "C" {
#endif

#if defined(CY_RTOS_AWARE) || defined(COMPONENT_RTOS_AWARE)

#include "cyabs_rtos.h"

#define NEVER_TIMEOUT ( (uint32_t)0xffffffffUL )
static cy_semaphore_t _udb_sdio_transfer_finished_semaphore;
static bool           _udb_sdio_sema_initialized = false;
#endif

// Backup struct used to store and restore non retention UDB registers
typedef struct
{
    uint32_t CY_SDIO_UDB_WRKMULT_CTL_0;
    uint32_t CY_SDIO_UDB_WRKMULT_CTL_1;
    uint32_t CY_SDIO_UDB_WRKMULT_CTL_2;
    uint32_t CY_SDIO_UDB_WRKMULT_CTL_3;
} stc_sdio_backup_regs_t;

// Globals Needed for DMA
// DMA channel structures
static cy_stc_dma_channel_config_t _udb_sdio_channel_config_resp;
static cy_stc_dma_channel_config_t _udb_sdio_channel_config_cmd;
static cy_stc_dma_channel_config_t _udb_sdio_channel_config_write;
static cy_stc_dma_channel_config_t _udb_sdio_channel_config_read;

// DMA Descriptor structures
static cy_stc_dma_descriptor_t _udb_sdio_desr_resp;
static cy_stc_dma_descriptor_t _udb_sdio_desr_cmd;
static cy_stc_dma_descriptor_t _udb_sdio_desr_read0;
static cy_stc_dma_descriptor_t _udb_sdio_desr_read1;
static cy_stc_dma_descriptor_t _udb_sdio_desr_write0;
static cy_stc_dma_descriptor_t _udb_sdio_desr_write1;

// Global structure used for data keeping
static stc_sdio_gInternalData_t _udb_sdio_gstc_internal_data;

// Global CRC table
static uint8_t _udb_sdio_crc_table[256];

// Global values used for DMA interrupt
static uint32_t _udb_sdio_y_count_remainder;
static uint32_t _udb_sdio_y_counts;

// Global value for card interrupt
static uint8_t _udb_sdio_pfn_card_int_count = 0;

// Global structure to store UDB registers
static stc_sdio_backup_regs_t _udb_sdio_regs;

static uint32_t _udb_sdio_udb_initialized = 0;

#define DMA_INSTANCES   (4u)
#if (CYHAL_API_VERSION >= 2)
static cyhal_clock_t _udb_sdio_rsc_clk = { CYHAL_CLOCK_BLOCK_PERIPHERAL_8BIT, 0, false, NULL };
#else
static cyhal_resource_inst_t _udb_sdio_rsc_inst_clk =
    { CYHAL_RSC_CLOCK, CYHAL_CLOCK_BLOCK_PERIPHERAL_8BIT, 0 };
static bool _udb_sdio_reserved_clock = false;
#endif
static const cyhal_resource_inst_t _udb_sdio_rec_dmas[DMA_INSTANCES] =
{
    { CYHAL_RSC_DW, 0, 0 },
    { CYHAL_RSC_DW, 0, 1 },
    { CYHAL_RSC_DW, 1, 1 },
    { CYHAL_RSC_DW, 1, 3 },
};
static uint8_t _udb_sdio_reserved_dmas = 0;

// Deep Sleep Mode API Support
static void SDIO_SaveConfig(void);
static void SDIO_RestoreConfig(void);

/***************************************************************************************************
 * Function Name: SDIO_ReserveResources
 ***********************************************************************************************//**
 *
 * Allows reserving some of the internal resources that be allocated to other purposes in the
 * application. This includes Clocks and DMA instances. Calling this is purely optional. If not
 * called, the resources will be reserved by SDIO_Init. If called, SDIO_Init will bypass the
 * reservations to avoid doing it multiple times.
 *
 * \return
 * CY_RSLT_SUCCESS if the reservation was successful, else an error for what went wrong
 *
 **************************************************************************************************/
cy_rslt_t SDIO_ReserveResources()
{
    cy_rslt_t ret = CY_RSLT_SUCCESS;
    #if (CYHAL_API_VERSION >= 2)
    if (!_udb_sdio_rsc_clk.reserved)
    {
        ret = cyhal_clock_reserve(&_udb_sdio_rsc_clk, &_udb_sdio_rsc_clk);
    }
    #else
    if (!_udb_sdio_reserved_clock)
    {
        ret = cyhal_hwmgr_reserve(&_udb_sdio_rsc_inst_clk);
        _udb_sdio_reserved_clock = (CY_RSLT_SUCCESS == ret);
    }
    cyhal_clock_t _udb_sdio_rsc_clk = { 0, 0, CYHAL_CLOCK_BLOCK_PERIPHERAL_8BIT, 0, true };
    #endif // if (CYHAL_API_VERSION >= 2)
    if (CY_RSLT_SUCCESS == ret)
    {
        /* Assign clock divider */
        ret = _cyhal_utils_peri_pclk_set_divider(PCLK_UDB_CLOCKS0, &_udb_sdio_rsc_clk, 0U);

        if (CY_RSLT_SUCCESS == ret)
        {
            ret = _cyhal_utils_peri_pclk_enable_divider(PCLK_UDB_CLOCKS0, &_udb_sdio_rsc_clk);
        }

        if (CY_RSLT_SUCCESS == ret)
        {
            ret = _cyhal_utils_peri_pclk_assign_divider(PCLK_UDB_CLOCKS0, &_udb_sdio_rsc_clk);
        }
    }

    while (CY_RSLT_SUCCESS == ret && _udb_sdio_reserved_dmas < DMA_INSTANCES)
    {
        ret = cyhal_hwmgr_reserve(&_udb_sdio_rec_dmas[_udb_sdio_reserved_dmas]);
        if (CY_RSLT_SUCCESS == ret)
        {
            _udb_sdio_reserved_dmas++;
        }
    }

    return ret;
}


/***************************************************************************************************
 * Function Name: SDIO_FreeResources
 ***********************************************************************************************//**
 *
 * Frees all 'sharable' clock/DMA resources that were allocated by SDIO_ReserveResources. If the
 * resources were already freed, this will do nothing.
 *
 **************************************************************************************************/
void SDIO_FreeResources()
{
    #if (CYHAL_API_VERSION >= 2)
    if (_udb_sdio_rsc_clk.reserved)
    {
        cyhal_clock_free(&_udb_sdio_rsc_clk);
    }
    #else
    if (_udb_sdio_reserved_clock)
    {
        cyhal_hwmgr_free(&_udb_sdio_rsc_inst_clk);
        _udb_sdio_reserved_clock = false;
    }
    #endif

    while (_udb_sdio_reserved_dmas > 0)
    {
        _udb_sdio_reserved_dmas--;
        cyhal_hwmgr_free(&_udb_sdio_rec_dmas[_udb_sdio_reserved_dmas]);
    }
}


/***************************************************************************************************
 * Function Name: SDIO_DeepSleepCallback
 ***********************************************************************************************//**
 *
 * Callback executed during Deep Sleep entry/exit
 *
 * \param params
 * Pointer to structure that holds callback parameters for this driver.
 *
 * \param mode
 * The state transition mode that is currently happening.
 *
 * \note
 * Saves/Restores SDIO UDB registers
 *
 * \return
 * CY_SYSPM_SUCCESS if the transition was successful, otherwise CY_SYSPM_FAIL
 *
 **************************************************************************************************/
cy_en_syspm_status_t SDIO_DeepSleepCallback(cy_stc_syspm_callback_params_t* params,
                                            cy_en_syspm_callback_mode_t mode)
{
    CY_UNUSED_PARAMETER(params);
    cy_en_syspm_status_t status = CY_SYSPM_FAIL;

    switch (mode)
    {
        case CY_SYSPM_CHECK_READY:
        case CY_SYSPM_CHECK_FAIL:
            status = CY_SYSPM_SUCCESS;
            break;

        case CY_SYSPM_BEFORE_TRANSITION:
            SDIO_SaveConfig();
            status = CY_SYSPM_SUCCESS;
            break;

        case CY_SYSPM_AFTER_TRANSITION:
            SDIO_RestoreConfig();
            status = CY_SYSPM_SUCCESS;
            break;

        default:
            CY_ASSERT(false);
            break;
    }

    return status;
}


/***************************************************************************************************
 * Function Name: SDIO_Init
 ***********************************************************************************************//**
 *
 * Initializes the SDIO hardware
 *
 * \param pfuCb
 * Pointer to structure that holds pointers to callback function
 * see \ref stc_sdio_irq_cb_t.
 *
 * \note
 * Sets SD Clock Frequency to 400 kHz
 **************************************************************************************************/
void SDIO_Init(stc_sdio_irq_cb_t* pfuCb)
{
    if (!_udb_sdio_udb_initialized)
    {
        _udb_sdio_udb_initialized = 1;
        SDIO_Host_Config_TriggerMuxes();
        SDIO_Host_Config_UDBs();
    }

    // Set Number of Blocks to 1 initially, this will be updated later
    SDIO_SetNumBlocks(1);

    // Enable SDIO ISR
    #if defined(_CYHAL_DRIVER_AVAILABLE_IRQ) && (_CYHAL_DRIVER_AVAILABLE_IRQ)
    _cyhal_irq_enable((_cyhal_system_irq_t)SDIO_HOST_sdio_int__INTC_NUMBER);
    #else
    NVIC_EnableIRQ((IRQn_Type)SDIO_HOST_sdio_int__INTC_NUMBER);
    #endif

    // Enable the Status Reg to generate an interrupt
    SDIO_STATUS_AUX_CTL |= (0x10);

    // Set the priority of DW0, DW1, M4 and M0. DW1 should have highest
    // First clear priority of all
    (*(reg32*)CYREG_PROT_SMPU_MS0_CTL)  &= ~0x0300;
    (*(reg32*)CYREG_PROT_SMPU_MS2_CTL)  &= ~0x0300;
    (*(reg32*)CYREG_PROT_SMPU_MS3_CTL)  &= ~0x0300;
    (*(reg32*)CYREG_PROT_SMPU_MS14_CTL) &= ~0x0300;

    // Next set priority DW1 =  0, DW0 = 1, M4 = 2, M0 =3
    (*(reg32*)CYREG_PROT_SMPU_MS2_CTL)  |= 0x0100;
    (*(reg32*)CYREG_PROT_SMPU_MS0_CTL)  |= 0x0200;
    (*(reg32*)CYREG_PROT_SMPU_MS14_CTL) |= 0x0200;

    // Setup callback for card interrupt
    _udb_sdio_gstc_internal_data.pstcCallBacks.pfnCardIntCb = pfuCb->pfnCardIntCb;

    // Setup the DMA channels
    SDIO_SetupDMA();

    // Initialize CRC
    SDIO_Crc7Init();

    // Enable all the bit counters
    SDIO_CMD_BIT_CNT_CONTROL_REG   |=  SDIO_ENABLE_CNT;
    SDIO_WRITE_CRC_CNT_CONTROL_REG |=  SDIO_ENABLE_CNT;
    SDIO_CRC_BIT_CNT_CONTROL_REG   |=  SDIO_ENABLE_CNT;
    SDIO_BYTE_CNT_CONTROL_REG      |=  SDIO_ENABLE_CNT;

    // Set block byte count to 64, this will be changed later
    SDIO_SetBlockSize(64);

    // Set the read and write FIFOs to use the half full status
    (*(reg32*)SDIO_HOST_bSDIO_Write_DP__DP_AUX_CTL_REG) |= 0x0c;
    (*(reg32*)SDIO_HOST_bSDIO_Read_DP__DP_AUX_CTL_REG)  |= 0x0c;

    // Set clock to 400k, and enable it
    SDIO_SetSdClkFrequency(400000);
    SDIO_EnableIntClock();
    SDIO_EnableSdClk();
}


/***************************************************************************************************
 * Function Name: SDIO_SendCommand
 ***********************************************************************************************//**
 *
 * Send an SDIO command, don't wait for it to finish.
 *
 * \param pstcCmdConfig
 * Command configuration structure. See \ref stc_sdio_cmd_config_t.
 *
 **************************************************************************************************/
void SDIO_SendCommand(stc_sdio_cmd_config_t* pstcCmdConfig)
{
    // buffer to hold command data
    static uint8_t u8cmdBuf[6];

    // Populate buffer
    // Element 0 is the Most Significant Byte
    u8cmdBuf[0] = SDIO_HOST_DIR | pstcCmdConfig->u8CmdIndex;
    u8cmdBuf[1] = (uint8_t)((pstcCmdConfig->u32Argument & 0xff000000)>>24);
    u8cmdBuf[2] = (uint8_t)((pstcCmdConfig->u32Argument & 0x00ff0000)>>16);
    u8cmdBuf[3] = (uint8_t)((pstcCmdConfig->u32Argument & 0x0000ff00)>>8);
    u8cmdBuf[4] = (uint8_t)((pstcCmdConfig->u32Argument & 0x000000ff));

    // calculate the CRC of above data
    u8cmdBuf[5] = SDIO_CalculateCrc7(u8cmdBuf, 5);
    // Shift it up by 1 as the CRC takes the upper 7 bits of the last byte of the cmd
    u8cmdBuf[5] = u8cmdBuf[5] << 1;
    // Add on the end bit
    u8cmdBuf[5] = u8cmdBuf[5] | SDIO_CMD_END_BIT;

    // Load the first byte into A0
    SDIO_CMD_COMMAND_A0_REG = u8cmdBuf[0];

    // If a response is expected setup DMA to receive the response
    if (pstcCmdConfig->bResponseRequired == true)
    {
        // Clear the flag in hardware that says skip response
        SDIO_CONTROL_REG &= ~SDIO_CTRL_SKIP_RESPONSE;

        // Set the destination address
        _udb_sdio_desr_resp.dst = (uint32_t)(pstcCmdConfig->pu8ResponseBuf);

        // Initialize the channel with the descriptor
        Cy_DMA_Channel_SetDescriptor(SDIO_HOST_Resp_DMA_HW, SDIO_HOST_Resp_DMA_DW_CHANNEL,
                                     &_udb_sdio_desr_resp);

        // Enable the channel
        Cy_DMA_Channel_Enable(SDIO_HOST_Resp_DMA_HW, SDIO_HOST_Resp_DMA_DW_CHANNEL);
    }
    else
    {
        // Set the skip flag
        SDIO_CONTROL_REG |= SDIO_CTRL_SKIP_RESPONSE;
    }

    // Setup the Command DMA
    // Set the source address
    _udb_sdio_desr_cmd.src = (uint32_t)(&u8cmdBuf[1]);

    // Initialize the channel with the descriptor
    Cy_DMA_Channel_SetDescriptor(SDIO_HOST_CMD_DMA_HW, SDIO_HOST_CMD_DMA_DW_CHANNEL,
                                 &_udb_sdio_desr_cmd);

    // Enable the channel
    Cy_DMA_Channel_Enable(SDIO_HOST_CMD_DMA_HW, SDIO_HOST_CMD_DMA_DW_CHANNEL);
}


/***************************************************************************************************
 * Function Name: SDIO_GetResponse
 ***********************************************************************************************//**
 *
 * Takes a 6 byte response buffer, and extracts the 32 bit response, also checks
 * for index errors, CRC errors, and end bit errors.
 *
 * \param bCmdIndexCheck
 * If True check for index errors
 *
 * \param bCmdCrcCheck
 * If True check for CRC errors
 *
 * \param u8cmdIdx
 * Command index, used for checking the index error
 *
 * \param pu32Response
 * location to store 32 bit response
 *
 * \param pu8ResponseBuf
 * buffer that holds the 6 bytes of response data
 *
 * \return
 * \ref en_sdio_result_t
 *
 **************************************************************************************************/
en_sdio_result_t SDIO_GetResponse(uint8_t bCmdIndexCheck, uint8_t bCmdCrcCheck, uint8_t u8cmdIdx,
                                  uint32_t* pu32Response, uint8_t* pu8ResponseBuf)
{
    // Function return
    en_sdio_result_t enRet = Error;
    // variable to hold temporary CRC
    uint8_t u8TmpCrc;
    // temporary response
    uint32_t u32TmpResponse;

    // Zero out the pu32Response
    *pu32Response = 0;

    // Check if the CRC needs to be checked
    if (bCmdCrcCheck)
    {
        // Calculate the CRC
        u8TmpCrc = SDIO_CalculateCrc7(pu8ResponseBuf, 5);

        // Shift calculated CRC up by one bit to match bit position of CRC
        u8TmpCrc = u8TmpCrc << 1;

        // Compare calculated CRC with received CRC
        if ((u8TmpCrc & 0xfe) != (pu8ResponseBuf[5] & 0xfe))
        {
            enRet |= CommandCrcError;
        }
    }

    // Check if the index needs to be checked
    if (bCmdIndexCheck)
    {
        // The index resides in the lower 6 bits of the 1st byte of the response
        if ((u8cmdIdx != (pu8ResponseBuf[0] & 0x3f)))
        {
            enRet |= CommandIdxError;
        }
    }

    // Check the end bit
    if (!(pu8ResponseBuf[5] & 0x01))
    {
        enRet |= CommandEndError;
    }

    if (enRet == Error)
    {
        // If we get here then there were no errors with the command populate the response
        u32TmpResponse  = pu8ResponseBuf[1];
        u32TmpResponse  = u32TmpResponse << 8;
        u32TmpResponse |= pu8ResponseBuf[2];
        u32TmpResponse  = u32TmpResponse << 8;
        u32TmpResponse |= pu8ResponseBuf[3];
        u32TmpResponse  = u32TmpResponse << 8;
        u32TmpResponse |= pu8ResponseBuf[4];

        *pu32Response = u32TmpResponse;

        enRet = Ok;
    }

    return enRet;
}


/***************************************************************************************************
 * Function Name: SDIO_InitDataTransfer
 ***********************************************************************************************//**
 *
 * Configure the data channel for a data transfer. For a write this doesn't start
 * the write, that must be done separately after the response is received.
 *
 * \param pstcDataConfig
 * Data configuration structure. See \ref stc_sdio_data_config_t
 *
 *
 **************************************************************************************************/
void SDIO_InitDataTransfer(stc_sdio_data_config_t* pstcDataConfig)
{
    // hold size of entire transfer
    uint32_t dataSize;

    // calculate how many bytes are going to be sent
    dataSize = pstcDataConfig->u16BlockSize * pstcDataConfig->u16BlockCount;

    // Set the block size and number of blocks
    SDIO_SetBlockSize(pstcDataConfig->u16BlockSize);
    SDIO_SetNumBlocks((pstcDataConfig->u16BlockCount) - 1);

    // If we are reading data setup the DMA to receive read data
    if (pstcDataConfig->bRead == true)
    {
        // First disable the write channel
        Cy_DMA_Channel_Disable(SDIO_HOST_Write_DMA_HW, SDIO_HOST_Write_DMA_DW_CHANNEL);

        // Clear any pending interrupts in the DMA
        Cy_DMA_Channel_ClearInterrupt(SDIO_HOST_Read_DMA_HW, SDIO_HOST_Read_DMA_DW_CHANNEL);

        #if defined(_CYHAL_DRIVER_AVAILABLE_IRQ) && (_CYHAL_DRIVER_AVAILABLE_IRQ)
        _cyhal_irq_clear_pending((_cyhal_system_irq_t)SDIO_HOST_Read_Int_INTC_NUMBER);
        #else
        NVIC_ClearPendingIRQ((IRQn_Type)SDIO_HOST_Read_Int_INTC_NUMBER);
        #endif

        // setup the destination addresses
        _udb_sdio_desr_read0.dst = (uint32_t)(pstcDataConfig->pu8Data);
        _udb_sdio_desr_read1.dst = (uint32_t)((pstcDataConfig->pu8Data) + 1024);

        // Setup the X control to transfer two 16 bit elements per transfer for a total of 4 bytes
        // Remember X increment is in terms of data element size which is 16, thus why it is 1
        _udb_sdio_desr_read0.xCtl = _VAL2FLD(CY_DMA_CTL_COUNT, 1) |
                                    _VAL2FLD(CY_DMA_CTL_DST_INCR, 1);
        _udb_sdio_desr_read1.xCtl = _VAL2FLD(CY_DMA_CTL_COUNT, 1) |
                                    _VAL2FLD(CY_DMA_CTL_DST_INCR, 1);

        // The X Loop will always transfer 4 bytes. The FIFO will only trigger the
        // DMA when it has 4 bytes to send (2 in each F0 and F1). There is a possibility
        // that there could be 3,2,or 1 bytes still in the FIFOs. To solve this the DMA
        // will be SW triggered when hardware indicates all bytes have been received.
        // This leads to an extra 1, 2 or 3 bytes being received. So the RX buffer needs to
        // be at least 3 bytes bigger than the data size.
        //
        // Since the X loop is setup to 4, the maximum number of Y loop is 256 so one
        // descriptor can transfer 1024 bytes. Two descriptors can transfer 2048 bytes.
        // Since we don't know the maximum number of bytes to read only two descriptors will
        // be used. If more than 2048 bytes need to be read then and interrupt will be enabled
        // The descriptor that is not currently running will be updated in the ISR to receive
        // more data.
        //
        // So there are three conditions to check:
        // 1) Are we sending less than or equal to 1024 bytes if so use one descriptor
        // 2) Are we sending greater than 1024, but less than or equal to 2048, use two descriptors
        // 3) Greater than 2048, use two descriptors and the ISR

        if (dataSize <= 1024)
        {
            // Setup one descriptor
            // Y Increment is 2 because the X is transfer 2 data elements (which are 16 bits)
            _udb_sdio_desr_read0.yCtl = _VAL2FLD(CY_DMA_CTL_COUNT, (dataSize - 1) / 4) |
                                        _VAL2FLD(CY_DMA_CTL_DST_INCR, 2);

            // Setup descriptor 0 to point to nothing and disable
            _udb_sdio_desr_read0.nextPtr = 0;
            _udb_sdio_desr_read0.ctl    |= 0x01000000;

            // Disable Interrupt
            #if defined(_CYHAL_DRIVER_AVAILABLE_IRQ) && (_CYHAL_DRIVER_AVAILABLE_IRQ)
            _cyhal_irq_disable((_cyhal_system_irq_t)SDIO_HOST_Read_Int_INTC_NUMBER);
            #else
            NVIC_DisableIRQ((IRQn_Type)SDIO_HOST_Read_Int_INTC_NUMBER);
            #endif
        }
        else if (dataSize <= 2048)
        {
            // setup the first descriptor for 1024, then setup 2nd descriptor for remainder

            _udb_sdio_desr_read0.yCtl = _VAL2FLD(CY_DMA_CTL_COUNT, 255) |
                                        _VAL2FLD(CY_DMA_CTL_DST_INCR, 2);
            _udb_sdio_desr_read1.yCtl = _VAL2FLD(CY_DMA_CTL_COUNT, (dataSize - 1025) / 4) |
                                        _VAL2FLD(CY_DMA_CTL_DST_INCR, 2);


            // Setup descriptor 0 to point to descriptor 1
            _udb_sdio_desr_read0.nextPtr = (uint32_t)(&_udb_sdio_desr_read1);
            // Setup descriptor 1 to point to nothing and disable
            _udb_sdio_desr_read1.nextPtr = 0;

            // Don't disable after first descriptor
            _udb_sdio_desr_read0.ctl &= ~0x01000000;
            // Disable after second descriptor
            _udb_sdio_desr_read1.ctl |= 0x01000000;

            // Disable Interrupt
            #if defined(_CYHAL_DRIVER_AVAILABLE_IRQ) && (_CYHAL_DRIVER_AVAILABLE_IRQ)
            _cyhal_irq_disable((_cyhal_system_irq_t)SDIO_HOST_Read_Int_INTC_NUMBER);
            #else
            NVIC_DisableIRQ((IRQn_Type)SDIO_HOST_Read_Int_INTC_NUMBER);
            #endif
        }
        else // dataSize must be greater than 2048
        {
            // These are for the ISR, Need to figure out how many "descriptors"
            // need to run, and the yCount for last descriptor.
            // Example: dataSize = 2080
            // _udb_sdio_y_counts = 2, _udb_sdio_y_count_remainder = 7 (send 8 more set of 4)
            _udb_sdio_y_counts = (dataSize / 1024);

            // the Ycount register is a +1 register meaning 0 = 1. I However, need to know when
            // there is no remainder so I increase the value to make sure there is a remainder and
            // decrement in the ISR
            _udb_sdio_y_count_remainder = (((dataSize - (_udb_sdio_y_counts * 1024)) + 3) / 4);

            // Setup the Y Ctrl for both descriptors
            _udb_sdio_desr_read0.yCtl = _VAL2FLD(CY_DMA_CTL_COUNT, 255) |
                                        _VAL2FLD(CY_DMA_CTL_DST_INCR, 2);
            _udb_sdio_desr_read1.yCtl = _VAL2FLD(CY_DMA_CTL_COUNT, 255) |
                                        _VAL2FLD(CY_DMA_CTL_DST_INCR, 2);

            // Setup descriptor 0 to point to descriptor 1
            _udb_sdio_desr_read0.nextPtr = (uint32_t)(&_udb_sdio_desr_read1);
            // Setup descriptor 1 to point to descriptor 0
            _udb_sdio_desr_read1.nextPtr = (uint32_t)(&_udb_sdio_desr_read0);

            // Don't disable the channel on completion of descriptor
            _udb_sdio_desr_read0.ctl &= ~0x01000000;
            _udb_sdio_desr_read1.ctl &= ~0x01000000;

            // Decrement _udb_sdio_y_counts by 2 since we already have 2 descriptors setup
            _udb_sdio_y_counts -= 2;

            // Enable DMA interrupt
            #if defined(_CYHAL_DRIVER_AVAILABLE_IRQ) && (_CYHAL_DRIVER_AVAILABLE_IRQ)
            _cyhal_irq_enable((_cyhal_system_irq_t)SDIO_HOST_Read_Int_INTC_NUMBER);
            #else
            NVIC_EnableIRQ((IRQn_Type)SDIO_HOST_Read_Int_INTC_NUMBER);
            #endif
        }

        // Initialize the channel with the first descriptor
        Cy_DMA_Channel_SetDescriptor(SDIO_HOST_Read_DMA_HW, SDIO_HOST_Read_DMA_DW_CHANNEL,
                                     &_udb_sdio_desr_read0);

        // Enable the channel
        Cy_DMA_Channel_Enable(SDIO_HOST_Read_DMA_HW, SDIO_HOST_Read_DMA_DW_CHANNEL);

        // Set the flag in the control register to enable the read
        SDIO_CONTROL_REG |= SDIO_CTRL_ENABLE_READ;
    }
    // Otherwise it is a write
    else
    {
        // First disable the Read channel
        Cy_DMA_Channel_Disable(SDIO_HOST_Read_DMA_HW, SDIO_HOST_Read_DMA_DW_CHANNEL);

        // Clear any pending interrupts in the DMA
        Cy_DMA_Channel_ClearInterrupt(SDIO_HOST_Write_DMA_HW, SDIO_HOST_Write_DMA_DW_CHANNEL);

        #if defined(_CYHAL_DRIVER_AVAILABLE_IRQ) && (_CYHAL_DRIVER_AVAILABLE_IRQ)
        _cyhal_irq_clear_pending((_cyhal_system_irq_t)SDIO_HOST_Write_Int_INTC_NUMBER);
        #else
        NVIC_ClearPendingIRQ((IRQn_Type)SDIO_HOST_Write_Int_INTC_NUMBER);
        #endif

        // setup the SRC addresses
        _udb_sdio_desr_write0.src = (uint32_t)(pstcDataConfig->pu8Data);
        _udb_sdio_desr_write1.src = (uint32_t)((pstcDataConfig->pu8Data) + 1024);


        // Setup the X control to transfer two 16 bit elements per transfer for a total of 4 bytes
        // Remember X increment is in terms of data element size which is 16, thus why it is 1
        _udb_sdio_desr_write0.xCtl = _VAL2FLD(CY_DMA_CTL_COUNT, 1) |
                                     _VAL2FLD(CY_DMA_CTL_SRC_INCR, 1);
        _udb_sdio_desr_write1.xCtl = _VAL2FLD(CY_DMA_CTL_COUNT, 1) |
                                     _VAL2FLD(CY_DMA_CTL_SRC_INCR, 1);

        if (dataSize <= 1024)
        {
            // Setup one descriptor
            // Y Increment is 2 because the X is transfer 2 data elements (which are 16 bits)
            _udb_sdio_desr_write0.yCtl = _VAL2FLD(CY_DMA_CTL_COUNT, (dataSize - 1) / 4) |
                                         _VAL2FLD(CY_DMA_CTL_SRC_INCR, 2);

            // Setup descriptor 0 to point to nothing and disable
            _udb_sdio_desr_write0.nextPtr = 0;
            _udb_sdio_desr_write0.ctl    |= 0x01000000;

            // Disable Interrupt
            #if defined(_CYHAL_DRIVER_AVAILABLE_IRQ) && (_CYHAL_DRIVER_AVAILABLE_IRQ)
            _cyhal_irq_disable((_cyhal_system_irq_t)SDIO_HOST_Write_Int_INTC_NUMBER);
            #else
            NVIC_DisableIRQ((IRQn_Type)SDIO_HOST_Write_Int_INTC_NUMBER);
            #endif
        }
        else if (dataSize <= 2048)
        {
            // setup the first descriptor for 1024, then setup 2nd descriptor for remainder

            _udb_sdio_desr_write0.yCtl = _VAL2FLD(CY_DMA_CTL_COUNT, 255) |
                                         _VAL2FLD(CY_DMA_CTL_SRC_INCR, 2);
            _udb_sdio_desr_write1.yCtl = _VAL2FLD(CY_DMA_CTL_COUNT, (dataSize - 1025) / 4) |
                                         _VAL2FLD(CY_DMA_CTL_SRC_INCR, 2);


            // Setup descriptor 0 to point to descriptor 1
            _udb_sdio_desr_write0.nextPtr = (uint32_t)(&_udb_sdio_desr_write1);
            // Setup descriptor 1 to point to nothing and disable
            _udb_sdio_desr_write1.nextPtr = 0;

            // Don't disable after first descriptor
            _udb_sdio_desr_write0.ctl &= ~0x01000000;
            // Disable after second descriptor
            _udb_sdio_desr_write1.ctl |= 0x01000000;

            // Disable Interrupt
            #if defined(_CYHAL_DRIVER_AVAILABLE_IRQ) && (_CYHAL_DRIVER_AVAILABLE_IRQ)
            _cyhal_irq_disable((_cyhal_system_irq_t)SDIO_HOST_Write_Int_INTC_NUMBER);
            #else
            NVIC_DisableIRQ((IRQn_Type)SDIO_HOST_Write_Int_INTC_NUMBER);
            #endif
        }
        else // dataSize must be greater than 2048
        {
            // These are for the ISR, Need to figure out how many "descriptors"
            // need to run, and the yCount for last descriptor.
            // Example: dataSize = 2080
            // _udb_sdio_y_counts = 2, _udb_sdio_y_count_remainder = 7 (send 8 more set of 4)
            _udb_sdio_y_counts = (dataSize / 1024);

            // the Ycount register is a +1 register meaning 0 = 1. I However, need to know when
            // there is no remainder so I increase the value to make sure there is a remainder and
            // decrement in the ISR
            _udb_sdio_y_count_remainder = (((dataSize - (_udb_sdio_y_counts * 1024)) + 3) / 4);

            // Setup the Y Ctrl for both descriptors
            _udb_sdio_desr_write0.yCtl = _VAL2FLD(CY_DMA_CTL_COUNT, 255) |
                                         _VAL2FLD(CY_DMA_CTL_SRC_INCR, 2);
            _udb_sdio_desr_write1.yCtl = _VAL2FLD(CY_DMA_CTL_COUNT, 255)  |
                                         _VAL2FLD(CY_DMA_CTL_SRC_INCR, 2);

            // Setup descriptor 0 to point to descriptor 1
            _udb_sdio_desr_write0.nextPtr = (uint32_t)(&_udb_sdio_desr_write1);
            // Setup descriptor 1 to point to descriptor 0
            _udb_sdio_desr_write1.nextPtr = (uint32_t)(&_udb_sdio_desr_write0);

            // Don't disable the channel on completion of descriptor
            _udb_sdio_desr_write0.ctl &= ~0x01000000;
            _udb_sdio_desr_write1.ctl &= ~0x01000000;

            // Decrement _udb_sdio_y_counts by 2 since we already have 2 descriptors setup
            _udb_sdio_y_counts -= 2;

            // Enable DMA interrupt
            #if defined(_CYHAL_DRIVER_AVAILABLE_IRQ) && (_CYHAL_DRIVER_AVAILABLE_IRQ)
            _cyhal_irq_enable((_cyhal_system_irq_t)SDIO_HOST_Write_Int_INTC_NUMBER);
            #else
            NVIC_EnableIRQ((IRQn_Type)SDIO_HOST_Write_Int_INTC_NUMBER);
            #endif
        }

        // Initialize the channel with the first descriptor
        Cy_DMA_Channel_SetDescriptor(SDIO_HOST_Write_DMA_HW, SDIO_HOST_Write_DMA_DW_CHANNEL,
                                     &_udb_sdio_desr_write0);
    }
}


/***************************************************************************************************
 * Function Name: SDIO_SendCommandAndWait
 ***********************************************************************************************//**
 *
 * This function sends a command on the command channel and waits for that
 * command to finish before returning. If a Command 53 is issued this function
 * will handle all of the data transfer and wait to return until it is done.
 *
 * \param pstcCmd
 * Pointer command configuration structure see \ref stc_sdio_cmd_t.
 *
 * \return
 * \ref en_sdio_result_t
 *
 **************************************************************************************************/
en_sdio_result_t SDIO_SendCommandAndWait(stc_sdio_cmd_t* pstcCmd)
{
    // Store the command and data configurations
    stc_sdio_cmd_config_t  stcCmdConfig;
    stc_sdio_data_config_t stcDataConfig;

    uint32_t u32CmdTimeout = 0;

    // Returns from various function calls
    en_sdio_result_t enRet    = Ok;
    en_sdio_result_t enRetTmp = Ok;

    // Hold value of if these checks are needed
    uint8_t        bCmdIndexCheck;
    uint8_t        bCmdCrcCheck;
    static uint8_t u8responseBuf[6];

    // Clear statuses
    _udb_sdio_gstc_internal_data.stcEvents.u8CmdComplete   = 0;
    _udb_sdio_gstc_internal_data.stcEvents.u8TransComplete = 0;
    _udb_sdio_gstc_internal_data.stcEvents.u8CRCError      = 0;

    // Setup the command configuration
    stcCmdConfig.u8CmdIndex  = (uint8_t)pstcCmd->u32CmdIdx;
    stcCmdConfig.u32Argument =  pstcCmd->u32Arg;

    #if defined(CY_RTOS_AWARE) || defined(COMPONENT_RTOS_AWARE)

    cy_rslt_t result;

    // Initialize the semaphore. This is not done in init because init is called * in interrupt
    // thread. cy_rtos_init_semaphore call is prohibited in * interrupt thread.
    if (!_udb_sdio_sema_initialized)
    {
        cy_rtos_init_semaphore(&_udb_sdio_transfer_finished_semaphore, 1, 0);
        _udb_sdio_sema_initialized = true;
    }
    #else // if defined(CY_RTOS_AWARE) || defined(COMPONENT_RTOS_AWARE)

    // Variable used for holding timeout value
    uint32_t u32Timeout = 0;
    #endif // if defined(CY_RTOS_AWARE) || defined(COMPONENT_RTOS_AWARE)

    // Determine the type of response and if we need to do any checks
    // Command 0 and 8 have no response, so don't wait for one
    if ((pstcCmd->u32CmdIdx == 0) || (pstcCmd->u32CmdIdx == 8))
    {
        bCmdIndexCheck                 = false;
        bCmdCrcCheck                   = false;
        stcCmdConfig.bResponseRequired = false;
        stcCmdConfig.pu8ResponseBuf    = NULL;
    }
    // Command 5's response doesn't have a CRC or index, so don't check
    else if (pstcCmd->u32CmdIdx == 5)
    {
        bCmdIndexCheck                 = false;
        bCmdCrcCheck                   = false;
        stcCmdConfig.bResponseRequired = true;
        stcCmdConfig.pu8ResponseBuf    = u8responseBuf;
    }
    // Otherwise check everything
    else
    {
        bCmdIndexCheck                 = true;
        bCmdCrcCheck                   = true;
        stcCmdConfig.bResponseRequired = true;
        stcCmdConfig.pu8ResponseBuf    = u8responseBuf;
    }

    // Check if the command is 53, if it is then setup the data transfer
    if (pstcCmd->u32CmdIdx == 53)
    {
        // Set the number of blocks in the global struct
        stcDataConfig.u16BlockCount = (uint16_t)pstcCmd->u16BlockCnt;
        // Set the size of the data transfer
        stcDataConfig.u16BlockSize = (uint16_t)pstcCmd->u16BlockSize;
        // Set the direction are we reading or writing
        stcDataConfig.bRead = pstcCmd->bRead;
        // Set the pointer for the data
        stcDataConfig.pu8Data = pstcCmd->pu8Data;

        // Check DAT[0] to ensure it isn't low, if it is wait
        uint32_t count = 0;
        while (0UL ==
               Cy_GPIO_Read(Cy_GPIO_PortToAddr(CYHAL_GET_PORT(CYBSP_WIFI_SDIO_D0)),
                            CYHAL_GET_PIN(CYBSP_WIFI_SDIO_D0)) && count < SDIO_DAT_BUSY_TIMEOUT_MS)
        {
            #if defined(CY_RTOS_AWARE) || defined(COMPONENT_RTOS_AWARE)
            cy_rtos_delay_milliseconds(1);
            #else
            Cy_SysLib_Delay(1);
            #endif
            count++;
        }
        if (count >= SDIO_DAT_BUSY_TIMEOUT_MS)
        {
            enRet |= DataTimeout;
        }
        else
        {
            // Get the data Transfer Ready
            SDIO_InitDataTransfer(&stcDataConfig);

            // Set bit saying this was a CMD_53
            SDIO_CONTROL_REG |= SDIO_CTRL_ENABLE_INT;
        }
    }

    if (enRet == Ok)
    {
        // Send the command
        SDIO_SendCommand(&stcCmdConfig);

        // Wait for the command to finish
        do
        {
            u32CmdTimeout++;
            enRetTmp = SDIO_CheckForEvent(SdCmdEventCmdDone);
        } while ((enRetTmp != Ok) && (u32CmdTimeout < SDIO_CMD_TIMEOUT));


        if (u32CmdTimeout == SDIO_CMD_TIMEOUT)
        {
            enRet |= CMDTimeout;
        }
        else // CMD Passed
        {
            // If a response is expected check it
            if (stcCmdConfig.bResponseRequired == true)
            {
                enRetTmp = SDIO_GetResponse(bCmdCrcCheck, bCmdIndexCheck,
                                            (uint8_t)pstcCmd->u32CmdIdx, pstcCmd->pu32Response,
                                            u8responseBuf);
                if (enRetTmp != Ok)
                {
                    enRet |= enRetTmp;
                }
                else  // Response good
                {
                    // if it was command 53, check the response to ensure there was no error
                    if ((pstcCmd->u32CmdIdx) == 53)
                    {
                        // Make sure none of the error bits are set
                        if (*(pstcCmd->pu32Response) & 0x0000cf00)
                        {
                            enRet |= ResponseFlagError;
                        }
                        else // CMD53 Response good
                        {
                            // If it was command 53 and it was a write enable the write
                            if ((pstcCmd->bRead == false) && (enRet == Ok))
                            {
                                Cy_DMA_Channel_Disable(SDIO_HOST_Resp_DMA_HW,
                                                       SDIO_HOST_Resp_DMA_DW_CHANNEL);
                                Cy_DMA_Channel_Disable(SDIO_HOST_CMD_DMA_HW,
                                                       SDIO_HOST_CMD_DMA_DW_CHANNEL);
                                Cy_DMA_Channel_Disable(SDIO_HOST_Read_DMA_HW,
                                                       SDIO_HOST_Read_DMA_DW_CHANNEL);

                                // Set the flag in the control register to enable the write
                                Cy_DMA_Channel_Enable(SDIO_HOST_Write_DMA_HW,
                                                      SDIO_HOST_Write_DMA_DW_CHANNEL);
                                // Enable the channel
                                Cy_SysLib_DelayCycles(35);
                                SDIO_CONTROL_REG |= SDIO_CTRL_ENABLE_WRITE;
                            }

                            #if defined(CY_RTOS_AWARE) || defined(COMPONENT_RTOS_AWARE)
                            // Wait for the transfer to finish.
                            //  Acquire semaphore and wait until it will be released
                            //  in SDIO_IRQ:
                            //  1. _udb_sdio_transfer_finished_semaphore count is equal to
                            //     zero. cy_rtos_get_semaphore waits until semaphore
                            //     count is increased by cy_rtos_set_semaphore() in
                            //     SDIO_IRQ.
                            //  2. The cy_rtos_set_semaphore() increases
                            //     _udb_sdio_transfer_finished_semaphore count.
                            //  3. The cy_rtos_get_semaphore() function decreases
                            //     _udb_sdio_transfer_finished_semaphore back to zero
                            //     and exit. Or timeout occurs
                            result =
                                cy_rtos_get_semaphore(&_udb_sdio_transfer_finished_semaphore, 10,
                                                      false);

                            enRetTmp = SDIO_CheckForEvent(SdCmdEventTransferDone);

                            if (result != CY_RSLT_SUCCESS)
                            #else // if defined(CY_RTOS_AWARE) || defined(COMPONENT_RTOS_AWARE)
                            // Wait for the transfer to finish
                            do
                            {
                                u32Timeout++;
                                enRetTmp = SDIO_CheckForEvent(SdCmdEventTransferDone);
                            } while (!((enRetTmp == Ok) || (enRetTmp == DataCrcError) ||
                                       (u32Timeout >= SDIO_DAT_TIMEOUT)));

                            if (u32Timeout == SDIO_DAT_TIMEOUT)
                            #endif // if defined(CY_RTOS_AWARE) || defined(COMPONENT_RTOS_AWARE)
                            {
                                enRet |= DataTimeout;
                            }

                            // if it was a read it is possible there is still extra data hanging
                            // out, trigger the DMA again. This can result in extra data being
                            // transferred so the read buffer should be 3 bytes bigger than needed
                            if (pstcCmd->bRead == true)
                            {
                                Cy_TrigMux_SwTrigger((uint32_t)SDIO_HOST_Read_DMA_DW__TR_IN, 2);
                            }

                            if (enRetTmp == DataCrcError)
                            {
                                enRet |= DataCrcError;
                            }
                        }// CMD53 response good
                    }// Not a CMD53
                } // Response Good
            } // No Response Required, thus no CMD53
        } // CMD Passed
    } // Timeout error

    #if !defined(CY_RTOS_AWARE) && !defined(COMPONENT_RTOS_AWARE)
    u32Timeout = 0;
    #endif

    // If there were any errors then set general error flag
    if (enRet != Ok)
    {
        enRet |= Error;
    }

    // reset CmdTimeout value
    u32CmdTimeout = 0;

    // Always Reset on exit to clean up
    Cy_DMA_Channel_Disable(SDIO_HOST_Resp_DMA_HW, SDIO_HOST_Resp_DMA_DW_CHANNEL);
    Cy_DMA_Channel_Disable(SDIO_HOST_CMD_DMA_HW, SDIO_HOST_CMD_DMA_DW_CHANNEL);
    Cy_DMA_Channel_Disable(SDIO_HOST_Write_DMA_HW, SDIO_HOST_Write_DMA_DW_CHANNEL);
    Cy_DMA_Channel_Disable(SDIO_HOST_Read_DMA_HW, SDIO_HOST_Read_DMA_DW_CHANNEL);
    // No longer a CMD_53
    SDIO_CONTROL_REG &= ~(SDIO_CTRL_ENABLE_INT | SDIO_CTRL_ENABLE_WRITE | SDIO_CTRL_ENABLE_READ);
    SDIO_Reset();

    return enRet;
}


/***************************************************************************************************
 * Function Name: SDIO_CheckForEvent
 ***********************************************************************************************//**
 *
 * Checks to see if a specific event has occurred such a command complete or
 * transfer complete.
 *
 * \param enEventType
 * The type of event to check for. See \ref en_sdio_event_t.
 *
 * \return
 * \ref en_sdio_result_t
 *
 **************************************************************************************************/
en_sdio_result_t SDIO_CheckForEvent(en_sdio_event_t enEventType)
{
    en_sdio_result_t enRet = Error;

    // Disable Interrupts while modifying the global
    #if defined(_CYHAL_DRIVER_AVAILABLE_IRQ) && (_CYHAL_DRIVER_AVAILABLE_IRQ)
    _cyhal_irq_disable((_cyhal_system_irq_t)SDIO_HOST_sdio_int__INTC_NUMBER);
    #else
    NVIC_DisableIRQ((IRQn_Type)SDIO_HOST_sdio_int__INTC_NUMBER);
    #endif

    // Switch the event to check
    switch (enEventType)
    {
        // If the command is done clear the flag
        case SdCmdEventCmdDone:
            if (_udb_sdio_gstc_internal_data.stcEvents.u8CmdComplete > 0)
            {
                _udb_sdio_gstc_internal_data.stcEvents.u8CmdComplete = 0;
                enRet                                    = Ok;
            }
            break;

        // If the transfer is done check for CRC Error and clear the flag
        case SdCmdEventTransferDone:
            if (_udb_sdio_gstc_internal_data.stcEvents.u8TransComplete > 0)
            {
                _udb_sdio_gstc_internal_data.stcEvents.u8TransComplete = 0;
                enRet                                      = Ok;
            }
            // Check for CRC error and set flags
            if (_udb_sdio_gstc_internal_data.stcEvents.u8CRCError > 0)
            {
                enRet                                 = DataCrcError;
                _udb_sdio_gstc_internal_data.stcEvents.u8CRCError = 0;
            }
            break;
    }

    // Re-enable Interrupts
    #if defined(_CYHAL_DRIVER_AVAILABLE_IRQ) && (_CYHAL_DRIVER_AVAILABLE_IRQ)
    _cyhal_irq_enable((_cyhal_system_irq_t)SDIO_HOST_sdio_int__INTC_NUMBER);
    #else
    NVIC_EnableIRQ((IRQn_Type)SDIO_HOST_sdio_int__INTC_NUMBER);
    #endif
    return enRet;
}


/***************************************************************************************************
 * Function Name: SDIO_CalculateCrc7
 ***********************************************************************************************//**
 *
 * Calculate the 7 bit CRC for the command channel
 *
 * \param pu8Data
 * Data to calculate CRC on
 *
 * \param u8Size
 * Number of bytes to calculate CRC on
 *
 * \return
 * CRC
 *
 * \note
 * This code was copied from
 * http://www.barrgroup.com/Embedded-Systems/How-To/CRC-Calculation-C-Code
 *
 **************************************************************************************************/
uint8_t SDIO_CalculateCrc7(uint8_t* pu8Data, uint8_t u8Size)
{
    uint8_t  data;
    uint8_t  remainder = 0;
    uint32_t byte;

    for (byte = 0; byte < u8Size; ++byte)
    {
        data      = pu8Data[byte] ^ remainder;
        remainder = _udb_sdio_crc_table[data] ^ (remainder << 8);
    }

    return (remainder>>1);
}


/***************************************************************************************************
 * Function Name: SDIO_Crc7Init
 ***********************************************************************************************//**
 *
 * Initialize 7-bit CRC Table
 *
 * \note
 * This code was copied from
 * http://www.barrgroup.com/Embedded-Systems/How-To/CRC-Calculation-C-Code
 *
 **************************************************************************************************/
void SDIO_Crc7Init(void)
{
    uint8_t  remainder;
    uint8_t  bit;
    uint32_t dividend;

    for (dividend = 0; dividend < 256; ++dividend)
    {
        remainder = dividend;

        for (bit = 8; bit > 0; --bit)
        {
            if (remainder & SDIO_CRC_UPPER_BIT)
            {
                remainder = (remainder << 1) ^ SDIO_CRC7_POLY;
            }
            else
            {
                remainder = (remainder << 1);
            }
        }

        _udb_sdio_crc_table[dividend] = (remainder);
    }
}


/***************************************************************************************************
 * Function Name: SDIO_SetBlockSize
 ***********************************************************************************************//**
 *
 * Sets the size of each block
 *
 * \param u8ByteCount
 * Size of the block
 *
 **************************************************************************************************/
void SDIO_SetBlockSize(uint8_t u8ByteCount)
{
    SDIO_BYTE_COUNT_REG = u8ByteCount;
}


/***************************************************************************************************
 * Function Name: SDIO_SetNumBlocks
 ***********************************************************************************************//**
 *
 * Sets the number of blocks to send
 *
 * \param u8BlockCount
 * Size of the block
 *
 **************************************************************************************************/
void SDIO_SetNumBlocks(uint8_t u8BlockCount)
{
    SDIO_DATA_BLOCK_COUNTER_A0_REG = u8BlockCount;
    SDIO_DATA_BLOCK_COUNTER_D0_REG = u8BlockCount;
    // The one is used so that we can do 256 bytes
    SDIO_DATA_BLOCK_COUNTER_A1_REG = 1;
    SDIO_DATA_BLOCK_COUNTER_D1_REG = 1;
}


/***************************************************************************************************
 * Function Name: SDIO_EnableIntClock
 ***********************************************************************************************//**
 *
 * Enable Internal clock for the block
 *
 **************************************************************************************************/
void SDIO_EnableIntClock(void)
{
    SDIO_CONTROL_REG |= SDIO_CTRL_INT_CLK;
    Cy_SysClk_PeriphEnableDivider(SDIO_HOST_Internal_Clock_DIV_TYPE,
                                  SDIO_HOST_Internal_Clock_DIV_NUM);
}


/***************************************************************************************************
 * Function Name: SDIO_DisableIntClock
 ***********************************************************************************************//**
 *
 * Enable Disable clock for the block
 *
 **************************************************************************************************/
void SDIO_DisableIntClock(void)
{
    SDIO_CONTROL_REG &= ~SDIO_CTRL_INT_CLK;
    Cy_SysClk_PeriphDisableDivider(SDIO_HOST_Internal_Clock_DIV_TYPE,
                                   SDIO_HOST_Internal_Clock_DIV_NUM);
}


/***************************************************************************************************
 * Function Name: SDIO_EnableSdClk
 ***********************************************************************************************//**
 *
 * Enable SD Clock out to pin
 *
 **************************************************************************************************/
void SDIO_EnableSdClk(void)
{
    SDIO_CONTROL_REG |= SDIO_CTRL_SD_CLK;
}


/***************************************************************************************************
 * Function Name: SDIO_DisableSdClk
 ***********************************************************************************************//**
 *
 * Disable SD Clock out to the pin
 *
 **************************************************************************************************/
void SDIO_DisableSdClk(void)
{
    SDIO_CONTROL_REG &= ~SDIO_CTRL_SD_CLK;
}


/***************************************************************************************************
 * Function Name: SDIO_SetSdClkFrequency
 ***********************************************************************************************//**
 *
 * Sets the frequency of the SD Clock
 *
 * \param u32SdClkFreqHz
 * Frequency of SD Clock in Hz.
 *
 * \note
 * Only an integer divider is used, so the desired frequency may not be meet
 **************************************************************************************************/
void SDIO_SetSdClkFrequency(uint32_t u32SdClkFreqHz)
{
    uint16_t u16Div;
    /*
     * The UDB SDIO implemenation has a extra divider internally that divides the input clock to the
     * UDB
     * by 2. The desired clock frequency is hence intentionally multiplied by 2 in order to get the
     * required
     * SDIO operating frequency.
     */
    u16Div = Cy_SysClk_ClkPeriGetFrequency() / (2 * u32SdClkFreqHz);
    Cy_SysClk_PeriphSetDivider(SDIO_HOST_Internal_Clock_DIV_TYPE, SDIO_HOST_Internal_Clock_DIV_NUM,
                               (u16Div-1));
}


/***************************************************************************************************
 * Function Name: SDIO_SetupDMA
 ***********************************************************************************************//**
 *
 * Configures the DMA for the SDIO block
 *
 **************************************************************************************************/
void SDIO_SetupDMA(void)
{
    // Set the number of bytes to send
    SDIO_HOST_CMD_DMA_CMD_DMA_Desc_config.xCount = (SDIO_NUM_RESP_BYTES - 1);
    // Set the destination address
    SDIO_HOST_CMD_DMA_CMD_DMA_Desc_config.dstAddress = (void*)SDIO_CMD_COMMAND_PTR;

    // Initialize descriptor for cmd channel
    Cy_DMA_Descriptor_Init(&_udb_sdio_desr_cmd, &SDIO_HOST_CMD_DMA_CMD_DMA_Desc_config);

    // Set flag to disable descriptor when done
    _udb_sdio_desr_cmd.ctl |= 0x01000000;

    // Configure channel
    // CMD channel can be preempted, and has lower priority
    _udb_sdio_channel_config_cmd.descriptor  = &_udb_sdio_desr_cmd;
    _udb_sdio_channel_config_cmd.preemptable = 1;
    _udb_sdio_channel_config_cmd.priority    = 1;
    _udb_sdio_channel_config_cmd.enable      = 0u;

    // Configure Channel with initial Settings
    Cy_DMA_Channel_Init(SDIO_HOST_CMD_DMA_HW, SDIO_HOST_CMD_DMA_DW_CHANNEL,
                        &_udb_sdio_channel_config_cmd);

    // Enable DMA block
    Cy_DMA_Enable(SDIO_HOST_CMD_DMA_HW);

    // Set the number of bytes to receive
    SDIO_HOST_Resp_DMA_Resp_DMA_Desc_config.xCount = SDIO_NUM_RESP_BYTES;
    // Set the source address
    SDIO_HOST_Resp_DMA_Resp_DMA_Desc_config.srcAddress = (void*)SDIO_CMD_RESPONSE_PTR;

    // Initialize descriptor for response channel
    Cy_DMA_Descriptor_Init(&_udb_sdio_desr_resp, &SDIO_HOST_Resp_DMA_Resp_DMA_Desc_config);

    // Set flag to disable descriptor when done
    _udb_sdio_desr_resp.ctl |= 0x01000000;

    // Configure channel
    // response channel can be preempted, and has lower priority
    _udb_sdio_channel_config_resp.descriptor  = &_udb_sdio_desr_resp;
    _udb_sdio_channel_config_resp.preemptable = 1;
    _udb_sdio_channel_config_resp.priority    = 1;
    _udb_sdio_channel_config_resp.enable      = 0u;

    // Configure Channel with initial Settings
    Cy_DMA_Channel_Init(SDIO_HOST_Resp_DMA_HW, SDIO_HOST_Resp_DMA_DW_CHANNEL,
                        &_udb_sdio_channel_config_resp);
    // Enable DMA block
    Cy_DMA_Enable(SDIO_HOST_Resp_DMA_HW);

    // Set the destination address
    SDIO_HOST_Write_DMA_Write_DMA_Desc_config.dstAddress = (void*)SDIO_DAT_WRITE_PTR;

    // Initialize descriptor for write channel
    Cy_DMA_Descriptor_Init(&_udb_sdio_desr_write0, &SDIO_HOST_Write_DMA_Write_DMA_Desc_config);
    Cy_DMA_Descriptor_Init(&_udb_sdio_desr_write1, &SDIO_HOST_Write_DMA_Write_DMA_Desc_config);

    // Configure channel
    // write channel cannot be preempted, and has highest priority
    _udb_sdio_channel_config_write.descriptor  = &_udb_sdio_desr_write0;
    _udb_sdio_channel_config_write.preemptable = 0;
    _udb_sdio_channel_config_write.priority    = 0;
    _udb_sdio_channel_config_write.enable      = 0u;

    // Configure Channel with initial Settings
    Cy_DMA_Channel_Init(SDIO_HOST_Write_DMA_HW, SDIO_HOST_Write_DMA_DW_CHANNEL,
                        &_udb_sdio_channel_config_write);

    // Enable the interrupt
    Cy_DMA_Channel_SetInterruptMask(SDIO_HOST_Write_DMA_HW, SDIO_HOST_Write_DMA_DW_CHANNEL,
                                    CY_DMA_INTR_MASK);

    // Enable DMA block
    Cy_DMA_Enable(SDIO_HOST_Write_DMA_HW);

    // Set the source address
    SDIO_HOST_Read_DMA_Read_DMA_Desc_config.srcAddress = (void*)SDIO_DAT_READ_PTR;
    // Initialize descriptor for read channel
    Cy_DMA_Descriptor_Init(&_udb_sdio_desr_read0, &SDIO_HOST_Read_DMA_Read_DMA_Desc_config);
    Cy_DMA_Descriptor_Init(&_udb_sdio_desr_read1, &SDIO_HOST_Read_DMA_Read_DMA_Desc_config);

    // Configure channel
    // read channel cannot be preempted, and has highest priority
    _udb_sdio_channel_config_read.descriptor  = &_udb_sdio_desr_read0;
    _udb_sdio_channel_config_read.preemptable = 0;
    _udb_sdio_channel_config_read.priority    = 0;
    _udb_sdio_channel_config_read.enable      = 0u;

    // Configure Channel with initial Settings
    Cy_DMA_Channel_Init(SDIO_HOST_Read_DMA_HW, SDIO_HOST_Read_DMA_DW_CHANNEL,
                        &_udb_sdio_channel_config_read);

    // Enable the interrupt
    Cy_DMA_Channel_SetInterruptMask(SDIO_HOST_Read_DMA_HW, SDIO_HOST_Read_DMA_DW_CHANNEL,
                                    CY_DMA_INTR_MASK);

    // Enable DMA block
    Cy_DMA_Enable(SDIO_HOST_Read_DMA_HW);
}


/***************************************************************************************************
 * Function Name: SDIO_Reset
 ***********************************************************************************************//**
 *
 * Reset the SDIO interface
 *
 **************************************************************************************************/
void SDIO_Reset(void)
{
    // Control register is in pulse mode, so this just pulses the reset
    SDIO_CONTROL_REG |= (SDIO_CTRL_RESET_DP);
}


/***************************************************************************************************
 * Function Name: SDIO_EnableChipInt
 ***********************************************************************************************//**
 *
 * Enables the SDIO Chip Int by setting the mask bit
 *
 **************************************************************************************************/
void SDIO_EnableChipInt(void)
{
    SDIO_STATUS_INT_MSK |= SDIO_STS_CARD_INT;
}


/***************************************************************************************************
 * Function Name: SDIO_DisableChipInt
 ***********************************************************************************************//**
 *
 * Enables the SDIO Chip Int by setting the mask bit
 *
 **************************************************************************************************/
void SDIO_DisableChipInt(void)
{
    SDIO_STATUS_INT_MSK &= ~SDIO_STS_CARD_INT;
}


/***************************************************************************************************
 * Function Name: SDIO_IRQ
 ***********************************************************************************************//**
 *
 * SDIO interrupt, checks for events, and calls callbacks
 *
 **************************************************************************************************/
void SDIO_IRQ(void)
{
    uint8_t u8Status;

    // First read the status register
    u8Status = SDIO_STATUS_REG;

    // Check card interrupt
    if (u8Status & SDIO_STS_CARD_INT)
    {
        _udb_sdio_pfn_card_int_count++;
    }

    // Execute card interrupt callback if neccesary
    if (0 != _udb_sdio_pfn_card_int_count)
    {
        if (NULL != _udb_sdio_gstc_internal_data.pstcCallBacks.pfnCardIntCb)
        {
            _udb_sdio_gstc_internal_data.pstcCallBacks.pfnCardIntCb();
        }
        _udb_sdio_pfn_card_int_count--;
    }

    // If the command is complete set the flag
    if (u8Status & SDIO_STS_CMD_DONE)
    {
        _udb_sdio_gstc_internal_data.stcEvents.u8CmdComplete++;
    }

    // Check if a write is complete
    if (u8Status & SDIO_STS_WRITE_DONE)
    {
        // Clear the Write flag and CMD53 flag
        SDIO_CONTROL_REG &= ~(SDIO_CTRL_ENABLE_WRITE | SDIO_CTRL_ENABLE_INT);

        // Check if the CRC status return was bad
        if (u8Status & SDIO_STS_CRC_ERR)
        {
            // CRC was bad, set the flag
            _udb_sdio_gstc_internal_data.stcEvents.u8CRCError++;
        }

        // Set the done flag

        #if defined(CY_RTOS_AWARE) || defined(COMPONENT_RTOS_AWARE)
        cy_rtos_set_semaphore(&_udb_sdio_transfer_finished_semaphore, true);
        #else
        _udb_sdio_gstc_internal_data.stcEvents.u8TransComplete++;
        #endif
    }

    // Check if a read is complete
    if (u8Status & SDIO_STS_READ_DONE)
    {
        // Clear the read flag
        SDIO_CONTROL_REG &= ~(SDIO_CTRL_ENABLE_READ| SDIO_CTRL_ENABLE_INT);

        // Check the CRC
        if (u8Status & SDIO_STS_CRC_ERR)
        {
            // CRC was bad, set the flag
            _udb_sdio_gstc_internal_data.stcEvents.u8CRCError++;
        }
        // Okay we're done so set the done flag
        #if defined(CY_RTOS_AWARE) || defined(COMPONENT_RTOS_AWARE)
        cy_rtos_set_semaphore(&_udb_sdio_transfer_finished_semaphore, true);
        #else
        _udb_sdio_gstc_internal_data.stcEvents.u8TransComplete++;
        #endif
    }

    #if defined(_CYHAL_DRIVER_AVAILABLE_IRQ) && (_CYHAL_DRIVER_AVAILABLE_IRQ)
    _cyhal_irq_clear_pending((_cyhal_system_irq_t)SDIO_HOST_sdio_int__INTC_NUMBER);
    #else
    NVIC_ClearPendingIRQ((IRQn_Type)SDIO_HOST_sdio_int__INTC_NUMBER);
    #endif
}


/***************************************************************************************************
 * Function Name: SDIO_READ_DMA_IRQ
 ***********************************************************************************************//**
 *
 * SDIO DMA Read interrupt, checks counts and toggles to other descriptor if
 * needed
 *
 **************************************************************************************************/
void SDIO_READ_DMA_IRQ(void)
{
    // Shouldn't have to change anything unless it is the last descriptor

    // If the current descriptor is 0, then change descriptor 1
    if (Cy_DMA_Channel_GetCurrentDescriptor(SDIO_HOST_Read_DMA_HW,
                                            SDIO_HOST_Read_DMA_DW_CHANNEL) == &_udb_sdio_desr_read0)
    {
        // We need to increment the destination address every time
        _udb_sdio_desr_read1.dst += 2048;

        // If this is the last descriptor
        if ((_udb_sdio_y_counts == 1) && (_udb_sdio_y_count_remainder == 0))
        {
            // In this case all we need to change is the next descriptor and disable
            _udb_sdio_desr_read1.nextPtr = 0;
            _udb_sdio_desr_read1.ctl    |= 0x01000000;
            #if defined(_CYHAL_DRIVER_AVAILABLE_IRQ) && (_CYHAL_DRIVER_AVAILABLE_IRQ)
            _cyhal_irq_disable((_cyhal_system_irq_t)SDIO_HOST_Read_Int_INTC_NUMBER);
            #else
            NVIC_DisableIRQ((IRQn_Type)SDIO_HOST_Read_Int_INTC_NUMBER);
            #endif
        }
        else if ((_udb_sdio_y_counts == 0) && (_udb_sdio_y_count_remainder > 0))
        {
            // change next descriptor, and disable
            _udb_sdio_desr_read1.nextPtr = 0;
            _udb_sdio_desr_read1.ctl    |= 0x01000000;
            // Also change the yCount
            _udb_sdio_desr_read1.yCtl =
                _VAL2FLD(CY_DMA_CTL_COUNT, (_udb_sdio_y_count_remainder-1)) |
                _VAL2FLD(CY_DMA_CTL_DST_INCR, 2);
            #if defined(_CYHAL_DRIVER_AVAILABLE_IRQ) && (_CYHAL_DRIVER_AVAILABLE_IRQ)
            _cyhal_irq_disable((_cyhal_system_irq_t)SDIO_HOST_Read_Int_INTC_NUMBER);
            #else
            NVIC_DisableIRQ((IRQn_Type)SDIO_HOST_Read_Int_INTC_NUMBER);
            #endif
        }
    }

    // If the current descriptor is 1, then change descriptor 0
    if (Cy_DMA_Channel_GetCurrentDescriptor(SDIO_HOST_Read_DMA_HW,
                                            SDIO_HOST_Read_DMA_DW_CHANNEL) == &_udb_sdio_desr_read1)
    {
        // We need to increment the destination address everytime
        _udb_sdio_desr_read0.dst += 2048;

        // If this is the last descriptor
        if ((_udb_sdio_y_counts == 1) && (_udb_sdio_y_count_remainder == 0))
        {
            // In this case all we need to change is the next descriptor and disable
            _udb_sdio_desr_read0.nextPtr = 0;
            _udb_sdio_desr_read0.ctl    |= 0x01000000;
            #if defined(_CYHAL_DRIVER_AVAILABLE_IRQ) && (_CYHAL_DRIVER_AVAILABLE_IRQ)
            _cyhal_irq_disable((_cyhal_system_irq_t)SDIO_HOST_Read_Int_INTC_NUMBER);
            #else
            NVIC_DisableIRQ((IRQn_Type)SDIO_HOST_Read_Int_INTC_NUMBER);
            #endif
        }
        else if ((_udb_sdio_y_counts == 0) && (_udb_sdio_y_count_remainder > 0))
        {
            // change next descriptor, and disable
            _udb_sdio_desr_read0.nextPtr = 0;
            _udb_sdio_desr_read0.ctl    |= 0x01000000;
            // Also change the yCount
            _udb_sdio_desr_read0.yCtl =
                _VAL2FLD(CY_DMA_CTL_COUNT, (_udb_sdio_y_count_remainder-1)) |
                _VAL2FLD(CY_DMA_CTL_DST_INCR, 2);
            #if defined(_CYHAL_DRIVER_AVAILABLE_IRQ) && (_CYHAL_DRIVER_AVAILABLE_IRQ)
            _cyhal_irq_disable((_cyhal_system_irq_t)SDIO_HOST_Read_Int_INTC_NUMBER);
            #else
            NVIC_DisableIRQ((IRQn_Type)SDIO_HOST_Read_Int_INTC_NUMBER);
            #endif
        }
    }

    // Clear the interrupt
    Cy_DMA_Channel_ClearInterrupt(SDIO_HOST_Read_DMA_HW, SDIO_HOST_Read_DMA_DW_CHANNEL);
    // decrement y counts
    _udb_sdio_y_counts--;
}


/***************************************************************************************************
 * Function Name: SDIO_WRITE_DMA_IRQ
 ***********************************************************************************************//**
 *
 * SDIO DMA Write interrupt, checks counts and toggles to other descriptor if
 * needed
 *
 **************************************************************************************************/
void SDIO_WRITE_DMA_IRQ(void)
{
    // We shouldn't have to change anything unless it is the last descriptor

    // If the current descriptor is 0, then change descriptor 1
    if (Cy_DMA_Channel_GetCurrentDescriptor(SDIO_HOST_Write_DMA_HW,
                                            SDIO_HOST_Write_DMA_DW_CHANNEL) ==
        &_udb_sdio_desr_write0)
    {
        // We also need to increment the destination address every-time
        _udb_sdio_desr_write1.src += 2048;

        // If this is the last descriptor
        if ((_udb_sdio_y_counts == 1) && (_udb_sdio_y_count_remainder == 0))
        {
            // In this case all we need to change is the next descriptor and disable
            _udb_sdio_desr_write1.nextPtr = 0;
            _udb_sdio_desr_write1.ctl    |= 0x01000000;
            #if defined(_CYHAL_DRIVER_AVAILABLE_IRQ) && (_CYHAL_DRIVER_AVAILABLE_IRQ)
            _cyhal_irq_disable((_cyhal_system_irq_t)SDIO_HOST_Write_Int_INTC_NUMBER);
            #else
            NVIC_DisableIRQ((IRQn_Type)SDIO_HOST_Write_Int_INTC_NUMBER);
            #endif
        }
        else if ((_udb_sdio_y_counts == 0) && (_udb_sdio_y_count_remainder > 0))
        {
            // change next descriptor, and disable
            _udb_sdio_desr_write1.nextPtr = 0;
            _udb_sdio_desr_write1.ctl    |= 0x01000000;
            // Also change the yCount
            _udb_sdio_desr_write1.yCtl =
                _VAL2FLD(CY_DMA_CTL_COUNT, (_udb_sdio_y_count_remainder -1)) |
                _VAL2FLD(CY_DMA_CTL_SRC_INCR, 2);
            #if defined(_CYHAL_DRIVER_AVAILABLE_IRQ) && (_CYHAL_DRIVER_AVAILABLE_IRQ)
            _cyhal_irq_disable((_cyhal_system_irq_t)SDIO_HOST_Write_Int_INTC_NUMBER);
            #else
            NVIC_DisableIRQ((IRQn_Type)SDIO_HOST_Write_Int_INTC_NUMBER);
            #endif
        }
    }

    // If the current descriptor is 1, then change descriptor 0
    if (Cy_DMA_Channel_GetCurrentDescriptor(SDIO_HOST_Write_DMA_HW,
                                            SDIO_HOST_Write_DMA_DW_CHANNEL) ==
        &_udb_sdio_desr_write1)
    {
        // We also need to increment the destination address
        _udb_sdio_desr_write0.src += 2048;
        // If this is the last descriptor
        if ((_udb_sdio_y_counts == 1) && (_udb_sdio_y_count_remainder == 0))
        {
            // In this case all we need to change is the next descriptor and disable
            _udb_sdio_desr_write0.nextPtr = 0;
            _udb_sdio_desr_write0.ctl    |= 0x01000000;
            #if defined(_CYHAL_DRIVER_AVAILABLE_IRQ) && (_CYHAL_DRIVER_AVAILABLE_IRQ)
            _cyhal_irq_disable((_cyhal_system_irq_t)SDIO_HOST_Write_Int_INTC_NUMBER);
            #else
            NVIC_DisableIRQ((IRQn_Type)SDIO_HOST_Write_Int_INTC_NUMBER);
            #endif
        }
        else if ((_udb_sdio_y_counts == 0) && (_udb_sdio_y_count_remainder > 0))
        {
            // change next descriptor, and disable
            _udb_sdio_desr_write0.nextPtr = 0;
            _udb_sdio_desr_write0.ctl    |= 0x01000000;
            // Also change the yCount
            _udb_sdio_desr_write0.yCtl =
                _VAL2FLD(CY_DMA_CTL_COUNT, (_udb_sdio_y_count_remainder -1)) |
                _VAL2FLD(CY_DMA_CTL_SRC_INCR, 2);
            #if defined(_CYHAL_DRIVER_AVAILABLE_IRQ) && (_CYHAL_DRIVER_AVAILABLE_IRQ)
            _cyhal_irq_disable((_cyhal_system_irq_t)SDIO_HOST_Write_Int_INTC_NUMBER);
            #else
            NVIC_DisableIRQ((IRQn_Type)SDIO_HOST_Write_Int_INTC_NUMBER);
            #endif
        }
    }

    // Clear the interrupt
    Cy_DMA_Channel_ClearInterrupt(SDIO_HOST_Write_DMA_HW, SDIO_HOST_Write_DMA_DW_CHANNEL);
    _udb_sdio_y_counts--;
}


/***************************************************************************************************
 * Function Name: SDIO_Free
 ***********************************************************************************************//**
 *
 * Frees any system resources that were allocated by the SDIO driver.
 *
 **************************************************************************************************/
void SDIO_Free(void)
{
    SDIO_FreeResources();
    #if defined(CY_RTOS_AWARE) || defined(COMPONENT_RTOS_AWARE)
    if (_udb_sdio_sema_initialized)
    {
        cy_rtos_deinit_semaphore(&_udb_sdio_transfer_finished_semaphore);
        _udb_sdio_sema_initialized = false;
    }
    #endif
}


/*******************************************************************************
* Function Name: SDIO_SaveConfig
********************************************************************************
*
* Saves the user configuration of the SDIO UDB non-retention registers. Call the
* SDIO_SaveConfig() function before the Cy_SysPm_CpuEnterDeepSleep() function.
*
*******************************************************************************/
static void SDIO_SaveConfig(void)
{
    _udb_sdio_regs.CY_SDIO_UDB_WRKMULT_CTL_0 = UDB->WRKMULT.CTL[0];
    _udb_sdio_regs.CY_SDIO_UDB_WRKMULT_CTL_1 = UDB->WRKMULT.CTL[1];
    _udb_sdio_regs.CY_SDIO_UDB_WRKMULT_CTL_2 = UDB->WRKMULT.CTL[2];
    _udb_sdio_regs.CY_SDIO_UDB_WRKMULT_CTL_3 = UDB->WRKMULT.CTL[3];
}


/*******************************************************************************
* Function Name: SDIO_RestoreConfig
********************************************************************************
*
* Restores the user configuration of the SDIO UDB non-retention registers. Call
* the SDIO_Wakeup() function after the Cy_SysPm_CpuEnterDeepSleep() function.
*
*******************************************************************************/
static void SDIO_RestoreConfig(void)
{
    UDB->WRKMULT.CTL[0] = _udb_sdio_regs.CY_SDIO_UDB_WRKMULT_CTL_0;
    UDB->WRKMULT.CTL[1] = _udb_sdio_regs.CY_SDIO_UDB_WRKMULT_CTL_1;
    UDB->WRKMULT.CTL[2] = _udb_sdio_regs.CY_SDIO_UDB_WRKMULT_CTL_2;
    UDB->WRKMULT.CTL[3] = _udb_sdio_regs.CY_SDIO_UDB_WRKMULT_CTL_3;
}


#if defined(__cplusplus)
}
#endif

#endif // defined(CYHAL_UDB_SDIO)
