#include "cy_device.h"
#include "cy_gpio.h"
#include "cy_retarget_io.h"
#include "cy_sysclk.h"
#include "cy_syspm.h"
#include "cy_tcpwm_counter.h"
#include "cy_utils.h"
#include "cybsp.h"
#include "cyhal.h"
#include "psoc6_04_config.h"
#include <cy8c6244lqi_s4d92.h>
#include <cy_dma.h>
#include <cy_tcpwm_pwm.h>
#include <cyhal_hw_types.h>
#include <stdint.h>
#include <string.h>

/* Util Macros */
#define HANDLE_STATUS_MAIN_LOOP(msg, err)                                      \
    do {                                                                       \
        if (err) {                                                             \
            printf("error msg: %s error code: %d ", msg, (int)err);            \
            while (1)                                                          \
                ;                                                              \
        }                                                                      \
    } while (0);

/* Util Macros */
#define HANDLE_STATUS_ISR(msg, err)                                            \
    do {                                                                       \
        if (err) {                                                             \
            printf("error msg: %s error code: %d ", msg, (int)err);            \
            isr_status = err;                                                  \
            return;                                                            \
        }                                                                      \
    } while (0);

#define USEC_PER_SEC (1000U * 1000U)

/* DMA macros */
#define DMA_INST DW0
#define DMA_CHANNEL 0
#define DMA_ISR cpuss_interrupts_dw0_0_IRQn
#define DMA_GROUP 0
#define DMA_SRC_ADDR                                                           \
    (&(((TCPWM_V2_Type *)TCPWM0)                                               \
           ->GRP[0]                                                            \
           .CNT[0]                                                             \
           .COUNTER)) /* The DMA transfers the current tick count into a       \
                         buffer to record the time state of gpio changed */

/* Trigmux Macros */
#define IN_TRIGGER TRIG_IN_MUX_0_HSIOM_TR_OUT10
#define OUT_TRIGGER TRIG_OUT_MUX_0_PDMA0_TR_IN0

/* Edge recordings macros */
#define NUM_EDGES_PER_BIT (2U)
#define MAX_BITS_PER_ADDR (16U)
#define MAX_EDGES_PER_ADDR (MAX_BITS_PER_CMD * NUM_EDGES_PER_BIT)
#define MAX_BITS_PER_CMD (16U)
#define MAX_EDGES_PER_CMD (MAX_BITS_PER_CMD * NUM_EDGES_PER_BIT)
#define START_SYMBOL_EDGES (2U)
#define STOP_SYMBOL_EDGES (2U)
#define MAX_EDGES_PER_SEQ                                                      \
    (STOP_SYMBOL_EDGES + MAX_EDGES_PER_ADDR + MAX_EDGES_PER_CMD +              \
     STOP_SYMBOL_EDGES)
#define EDGE_DETECTED_TICKS_DMA_START                                          \
    (&edge_detected_ticks[1]) /* offsetting the start by one because first     \
           recording is inserting via gpio interrupt, not DMA */
#define EDGE_DETECTED_TICKS_DMA_NUM_TRANSFERS                                  \
    (MAX_EDGES_PER_SEQ -                                                       \
     1U) /* subtract 1 because the first recording is done in gpio isr*/

/* GPIO macros */
#define GPIO_PORT GPIO_PRT5
#define GPIO_PIN (0U)
#define GPIO_INTERRUPT_MODE P5_0_GPIO
#define GPIO_DMA_MODE P5_0_PERI_TR_IO_INPUT10
#define GPIO_ISR ioss_interrupts_gpio_5_IRQn

/* Timer macros */
#define PERI_CLK_TO_TCPWM_DIV (1U)
#define MAX_MS_BTW_EDGE                                                        \
    (9U) /* The largest gap is during 9ms start condition. This is             \
            significantly larger than next largest whch is 4.5ms during addr   \
            and cmd phase but it is better to overestimate as not to cutoff    \
            recording*/
#define SEQUENCE_MAX_USECS                                                     \
    ((MAX_MS_BTW_EDGE * USEC_PER_SEC) * EDGE_DETECTED_TICKS_DMA_NUM_TRANSFERS)
#define TCPWM_CHAN (0U)
#define TCPWM_DIV_NUM (1U)
#define TCPWM_CLOCK PCLK_TCPWM0_CLOCKS0
#define TCPWM_INST TCPWM0

/* Error globals */
int isr_status = 0;

/* Edge detected definitions */
uint32_t edge_detected_ticks[MAX_EDGES_PER_SEQ];

/*DMA structs definition */
cy_stc_dma_descriptor_t desc;
cy_stc_dma_descriptor_config_t desc_config = {
    .retrigger = CY_DMA_WAIT_FOR_REACT,
    .interruptType = CY_DMA_DESCR,
    .triggerOutType = CY_DMA_DESCR,
    .channelState = CY_DMA_CHANNEL_ENABLED,
    .triggerInType = CY_DMA_1ELEMENT,
    .dataSize = CY_DMA_WORD,
    .srcTransferSize = CY_DMA_TRANSFER_SIZE_DATA,
    .dstTransferSize = CY_DMA_TRANSFER_SIZE_DATA,
    .descriptorType = CY_DMA_1D_TRANSFER,
    .srcAddress = (void *)DMA_SRC_ADDR,
    .dstAddress = (void *)EDGE_DETECTED_TICKS_DMA_START,
    .srcXincrement = 0,
    .dstXincrement = 1,
    .xCount = EDGE_DETECTED_TICKS_DMA_NUM_TRANSFERS,
    .srcYincrement = 1,
    .dstYincrement = 1,
    .yCount = 1,
    .nextDescriptor = NULL,
};

const cy_stc_dma_channel_config_t channel_config = {
    .descriptor = &desc,
    .preemptable = false,
    .priority = 3,
    .enable = true,
    .bufferable = false,
};

cy_stc_sysint_t dma_int_config = {
    .intrPriority = 7,
    .intrSrc = DMA_ISR,
};

/* GPIO definitions */
const cy_stc_gpio_pin_config_t gpio_config_default = {
    .outVal = 1,
    .driveMode = CY_GPIO_DM_OD_DRIVESLOW,
    .intEdge = CY_GPIO_INTR_DISABLE,
    .intMask = 0UL,
    .vtrip = CY_GPIO_VTRIP_CMOS,
    .slewRate = CY_GPIO_SLEW_FAST,
    .driveSel = CY_GPIO_DRIVE_1_2,
    .vregEn = 0UL,
    .ibufMode = 0UL,
    .vtripSel = 0UL,
    .vrefSel = 0UL,
    .vohSel = 0UL,
};

cy_stc_sysint_t gpio_int_config = {.intrSrc = GPIO_ISR, .intrPriority = 7UL};

/* Timer definitions*/
static const cy_stc_tcpwm_counter_config_t counter_config = {
    // TODO: setup timeout handler
    .clockPrescaler = CY_TCPWM_COUNTER_PRESCALER_DIVBY_1,
    .runMode = CY_TCPWM_COUNTER_CONTINUOUS,
    .countDirection = CY_TCPWM_COUNTER_COUNT_UP,
    .compareOrCapture = CY_TCPWM_COUNTER_MODE_CAPTURE,
    .enableCompareSwap = false,
    .interruptSources = CY_TCPWM_INT_NONE,
    .captureInputMode = CY_TCPWM_INPUT_LEVEL,
    .captureInput = CY_TCPWM_INPUT_0,
    .reloadInputMode = CY_TCPWM_INPUT_LEVEL,
    .reloadInput = CY_TCPWM_INPUT_0,
    .startInputMode = CY_TCPWM_INPUT_LEVEL,
    .startInput = CY_TCPWM_INPUT_0,
    .stopInputMode = CY_TCPWM_INPUT_LEVEL,
    .stopInput = CY_TCPWM_INPUT_0,
    .countInputMode = CY_TCPWM_INPUT_LEVEL,
    .countInput = CY_TCPWM_INPUT_1,
    .capture1InputMode = CY_TCPWM_INPUT_LEVEL,
    .capture1Input = CY_TCPWM_INPUT_0,
    .enableCompare1Swap = false,
    .trigger0Event = CY_TCPWM_CNT_TRIGGER_ON_DISABLED,
    .trigger1Event = CY_TCPWM_CNT_TRIGGER_ON_DISABLED,
};

/* Func prototypes*/
uint64_t ticks_to_usecs(uint64_t ticks);
uint64_t usecs_to_ticks(uint64_t usecs);
void dma_complete();
void gpio_first_edge();
int setup_gpio_dma();
void init_timer();
int start_timer();
int setup_gpio_interrupt();
int setup_gpio_dma();
int setup_dma();
void disable_timer();
void disable_dma();

/* Util funcs */
uint64_t ticks_to_usecs(uint64_t ticks) {
    uint32_t clock_frequency_hz =
        Cy_SysClk_ClkPeriGetFrequency() / PERI_CLK_TO_TCPWM_DIV;
    if (clock_frequency_hz > USEC_PER_SEC) {
        return ticks / (clock_frequency_hz / USEC_PER_SEC);
    }

    return (USEC_PER_SEC / clock_frequency_hz) * ticks;
}

uint64_t usecs_to_ticks(uint64_t usecs) {
    uint32_t clock_frequency_hz =
        Cy_SysClk_ClkPeriGetFrequency() / PERI_CLK_TO_TCPWM_DIV;
    if (clock_frequency_hz > USEC_PER_SEC) {
        return (clock_frequency_hz / USEC_PER_SEC) * usecs;
    }

    return usecs * (USEC_PER_SEC / clock_frequency_hz);
}

/* ISRS */

/* Called when dma completes enough transfers, indicating completion of NEC
 * signal */
void dma_complete() {
    printf("dma complete\r\n");
    Cy_DMA_Channel_ClearInterrupt(DMA_INST, DMA_CHANNEL);
    cy_en_dma_intr_cause_t dma_intr_cause =
        Cy_DMA_Channel_GetStatus(DMA_INST, DMA_CHANNEL);
    if (dma_intr_cause != CY_DMA_INTR_CAUSE_COMPLETION) {
        HANDLE_STATUS_ISR("unexpected dma interrupt cause", dma_intr_cause);
    }

    disable_timer();
    disable_dma();
    Cy_SysPm_SleepOnExit(false);
    /* We let the CPU wake up after this to process the timestamps */
};

/* Called when activity detected on GPIO connected to IR reciever, indicating
 * start of signal */
void gpio_first_edge() {
    printf("first edge\r\n");
    int status = 0;
    Cy_GPIO_ClearInterrupt(GPIO_PORT, GPIO_PIN);

    status = setup_gpio_dma();
    HANDLE_STATUS_ISR("error setting up gpio dma", status);

    edge_detected_ticks[0] = 0; /* All subsequent tickstamps will be offsets of
                                   the one that triggered this isr */

    status = start_timer();
    HANDLE_STATUS_ISR("error starting timer", status);

    status = setup_dma();
    HANDLE_STATUS_ISR("error setting up DMA", status);

    /* The cpu can sleep because the rest of the nec sequence is handled by DMA
     */
    Cy_SysPm_CpuSleepOnExit(true);
}

/* Timer funcs */
void init_timer() {
    /* We assign and enable divier to the clock for TCPWM so that it can
     * function */
    Cy_SysClk_PeriphDisableDivider(CY_SYSCLK_DIV_16_BIT, TCPWM_DIV_NUM);
    Cy_SysClk_PeriphAssignDivider(TCPWM_CLOCK, CY_SYSCLK_DIV_16_BIT,
                                  TCPWM_DIV_NUM);
    Cy_SysClk_PeriphSetDivider(CY_SYSCLK_DIV_16_BIT, TCPWM_DIV_NUM,
                               PERI_CLK_TO_TCPWM_DIV - 1U);
    Cy_SysClk_PeriphEnableDivider(CY_SYSCLK_DIV_16_BIT, TCPWM_DIV_NUM);
}

int start_timer() {
    int status = CY_TCPWM_SUCCESS;
    cy_stc_tcpwm_counter_config_t config = counter_config;

    config.period = usecs_to_ticks(SEQUENCE_MAX_USECS);

    status = Cy_TCPWM_Counter_Init(TCPWM_INST, TCPWM_CHAN, &config);
    if (status != CY_TCPWM_SUCCESS) {
        return status;
    }

    Cy_TCPWM_Counter_Enable(TCPWM_INST, TCPWM_CHAN);
    Cy_TCPWM_TriggerStart_Single(TCPWM_INST, TCPWM_CHAN);
    return status;
}

void disable_timer() { Cy_TCPWM_Counter_Disable(TCPWM_INST, TCPWM_CHAN); }

/* GPIO funcs */

int setup_gpio_interrupt() {
    int status = 0;
    cy_stc_gpio_pin_config_t gpio_config_interrupt = gpio_config_default;

    gpio_config_interrupt.hsiom = GPIO_INTERRUPT_MODE;
    status = Cy_GPIO_Pin_Init(GPIO_PORT, GPIO_PIN, &gpio_config_interrupt);
    if (status != CY_GPIO_SUCCESS) {
        return status;
    }

    Cy_GPIO_SetInterruptEdge(GPIO_PORT, GPIO_PIN, CY_GPIO_INTR_FALLING);
    Cy_GPIO_SetInterruptMask(GPIO_PORT, GPIO_PIN, CY_GPIO_INTR_EN_MASK);

    status = Cy_SysInt_Init(&gpio_int_config, gpio_first_edge);
    if (status != CY_SYSINT_SUCCESS) {
        return status;
    }

    NVIC_ClearPendingIRQ(gpio_int_config.intrSrc);
    NVIC_EnableIRQ((IRQn_Type)gpio_int_config.intrSrc);
    return status;
}

int setup_gpio_dma() {
    int status = 0;
    cy_stc_gpio_pin_config_t gpio_config_dma = gpio_config_default;
    gpio_config_dma.hsiom =
        GPIO_DMA_MODE; /* setting GPIO function as a trigger to connect it
                          with DMA transfers */
    status = Cy_GPIO_Pin_Init(GPIO_PORT, GPIO_PIN, &gpio_config_dma);
    if (status != 0) {
        return status;
    }

    Cy_GPIO_SetInterruptMask(GPIO_PORT, GPIO_PIN, CY_GPIO_INTR_EN_MASK);
    return Cy_TrigMux_Connect(IN_TRIGGER, OUT_TRIGGER, false,
                              TRIGGER_TYPE_EDGE);
}

void disable_gpio() { Cy_GPIO_Port_Deinit(GPIO_PORT); }

/* DMA funcs*/

int setup_dma() {
    int status = 0;
    memset(edge_detected_ticks, 0, sizeof(edge_detected_ticks));
    status = Cy_DMA_Descriptor_Init(&desc, &desc_config);
    if (status != 0) {
        return status;
    }

    status = Cy_DMA_Channel_Init(DMA_INST, DMA_CHANNEL, &channel_config);
    if (status != 0) {
        return status;
    }

    Cy_DMA_Channel_SetDescriptor(DMA_INST, DMA_CHANNEL, &desc);

    status = Cy_SysInt_Init(&dma_int_config, &dma_complete);
    if (status != 0) {
        return status;
    }

    NVIC_EnableIRQ(dma_int_config.intrSrc);
    Cy_DMA_Channel_SetInterruptMask(DMA_INST, DMA_CHANNEL, CY_DMA_INTR_MASK);
    Cy_DMA_Channel_Enable(DMA_INST, DMA_CHANNEL);
    Cy_DMA_Enable(DMA_INST);
    return status;
}

void disable_dma() { Cy_DMA_Channel_Disable(DMA_INST, DMA_CHANNEL); }

int main(void) {
    /* Basic HW init */
    cy_rslt_t status;
    status = cybsp_init();
    HANDLE_STATUS_MAIN_LOOP("failed to init hw", status);
    __enable_irq();

    /* Init uart shell */
    status = cy_retarget_io_init_fc(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX,
                                    CYBSP_DEBUG_UART_CTS, CYBSP_DEBUG_UART_RTS,
                                    CY_RETARGET_IO_BAUDRATE);
    HANDLE_STATUS_MAIN_LOOP("error setting debug uart as io", status);

    init_timer();

    /* Init GPIO to detect first falling edge indictating start of nec sequence.
     * The isr setups the DMA transfers for the rest of the sequence */
    status = setup_gpio_interrupt();
    HANDLE_STATUS_MAIN_LOOP("error setting up GPIO", status);

    while (1) {
        /* We can sleep here until we are woken up by  dma_complete isr upon
         * reception of full nec ir sequence*/
        Cy_SysPm_CpuEnterSleep(CY_SYSPM_WAIT_FOR_INTERRUPT);
        /* Check if irq encountered error */
        if (isr_status != 0) {
            disable_dma();
            disable_gpio();
            disable_timer();
            HANDLE_STATUS_MAIN_LOOP("error in isr", isr_status);
        }

        printf("timestamp deltas: \r\n");
        for (int i = 1;
             i < sizeof(edge_detected_ticks) / sizeof(*edge_detected_ticks);
             i++) {
            uint32_t delta_us = ticks_to_usecs(edge_detected_ticks[i] -
                                               edge_detected_ticks[i - 1]);
            printf("ts: %lu hz:%lu \r\n", delta_us, USEC_PER_SEC / delta_us);
        }
        // TODO: implement NEC processing
        setup_gpio_interrupt(); /* setting up the system to repeat this process
                                   on next infrared transfer */
    }
}
