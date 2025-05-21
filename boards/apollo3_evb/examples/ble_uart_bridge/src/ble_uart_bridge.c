#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"

#include "wsf_types.h"
#include "wsf_os.h"
#include "wsf_timer.h"
#include "wsf_buf.h"

#include "hci_api.h"
#include "dm_api.h"
#include "att_api.h"
#include "app_api.h"
#include "svc_core.h"
#include "svc_dis.h"
#include "amdtps_api.h"
#include "svc_amdtp.h"

// UART handle
static void *g_pvUART;

// Simple buffer for UART reception
static uint8_t g_uartByte;

static void amdtp_uart_recv_cb(uint8_t *buf, uint16_t len)
{
    am_hal_uart_transfer_t xfer = {
        .ui32Direction = AM_HAL_UART_WRITE,
        .pui8Data = buf,
        .ui32NumBytes = len,
        .ui32TimeoutMs = AM_HAL_UART_WAIT_FOREVER,
        .pui32BytesTransferred = NULL,
    };
    am_hal_uart_transfer(g_pvUART, &xfer);
}

static void amdtp_uart_tx_cb(eAmdtpStatus_t status)
{
    // optional status handling
    (void)status;
}

// Buffer memory for WSF
#define WSF_BUF_POOLS 4
uint32_t g_pui32BufMem[3350/4 + 32];

static const wsfBufPoolDesc_t g_psPoolDescriptors[WSF_BUF_POOLS] =
{
    { 16,  8 },
    { 32, 4 },
    { 64, 4 },
    { 256, 4 }
};

static void exactle_stack_init(void)
{
    wsfHandlerId_t handlerId;
    uint16_t wsfBufMemLen;

    WsfOsInit();
    WsfTimerInit();

    wsfBufMemLen = WsfBufInit(sizeof(g_pui32BufMem), (uint8_t*)g_pui32BufMem,
                              WSF_BUF_POOLS, g_psPoolDescriptors);
    if (wsfBufMemLen > sizeof(g_pui32BufMem))
    {
        am_util_stdio_printf("WSF buffer too small\n");
    }

    SecInit();
    SecAesInit();
    SecCmacInit();
    SecEccInit();

    handlerId = WsfOsSetNextHandler(HciHandler);
    HciHandlerInit(handlerId);

    handlerId = WsfOsSetNextHandler(DmHandler);
    DmDevVsInit(0);
    DmAdvInit();
    DmConnInit();
    DmConnSlaveInit();
    DmSecInit();
    DmSecLescInit();
    DmPrivInit();
    DmHandlerInit(handlerId);

    handlerId = WsfOsSetNextHandler(L2cSlaveHandler);
    L2cSlaveHandlerInit(handlerId);
    L2cInit();
    L2cSlaveInit();

    handlerId = WsfOsSetNextHandler(AttHandler);
    AttHandlerInit(handlerId);
    AttsInit();
    AttsIndInit();
    AttcInit();

    handlerId = WsfOsSetNextHandler(SmpHandler);
    SmpHandlerInit(handlerId);
    SmprInit();
    SmprScInit();
    HciSetMaxRxAclLen(251);

    handlerId = WsfOsSetNextHandler(AppHandler);
    AppHandlerInit(handlerId);

    handlerId = WsfOsSetNextHandler(AmdtpHandler);
    AmdtpHandlerInit(handlerId);

    handlerId = WsfOsSetNextHandler(HciDrvHandler);
    HciDrvHandlerInit(handlerId);
}

int main(void)
{
    am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_SYSCLK_MAX, 0);
    am_hal_cachectrl_config(&am_hal_cachectrl_defaults);
    am_hal_cachectrl_enable();
    am_bsp_itm_printf_enable();

    // UART config
    am_hal_uart_config_t uartCfg = {
        .ui32BaudRate = 115200,
        .ui32DataBits = AM_HAL_UART_DATA_BITS_8,
        .ui32Parity = AM_HAL_UART_PARITY_NONE,
        .ui32StopBits = AM_HAL_UART_ONE_STOP_BIT,
        .ui32FlowControl = AM_HAL_UART_FLOW_CTRL_NONE,
        .ui32FifoLevels = (AM_HAL_UART_TX_FIFO_1_2 | AM_HAL_UART_RX_FIFO_1_2),
        .pui8TxBuffer = NULL,
        .ui32TxBufferSize = 0,
        .pui8RxBuffer = NULL,
        .ui32RxBufferSize = 0,
    };

    am_hal_uart_initialize(0, &g_pvUART);
    am_hal_uart_power_control(g_pvUART, AM_HAL_SYSCTRL_WAKE, false);
    am_hal_uart_configure(g_pvUART, &uartCfg);
    am_hal_gpio_pinconfig(AM_BSP_GPIO_COM_UART_TX, g_AM_BSP_GPIO_COM_UART_TX);
    am_hal_gpio_pinconfig(AM_BSP_GPIO_COM_UART_RX, g_AM_BSP_GPIO_COM_UART_RX);

    // Initialize BLE stack and AMDTP
    exactle_stack_init();
    AmdtpStart();

    while (1)
    {
        // Process BLE events
        wsfOsDispatcher();

        // Poll UART for one byte
        uint32_t rxed = 0;
        am_hal_uart_transfer_t read = {
            .ui32Direction = AM_HAL_UART_READ,
            .pui8Data = &g_uartByte,
            .ui32NumBytes = 1,
            .ui32TimeoutMs = 0,
            .pui32BytesTransferred = &rxed,
        };
        if (am_hal_uart_transfer(g_pvUART, &read) == AM_HAL_STATUS_SUCCESS && rxed)
        {
            AmdtpsSendPacket(AMDTP_PKT_TYPE_DATA, false, false, &g_uartByte, 1);
        }
    }
}
