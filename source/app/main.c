#include "app/app_shared.h"

/*******************************************************************************
 * System entry point
 ******************************************************************************/

int main(void)
{
    BOARD_ConfigMPU();
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    SystemCoreClockUpdate();

    USER_LED_INIT(LOGIC_LED_OFF);
    (void)RtcInit();
    ShellCliInit();

    s_canLogQueue = xQueueCreate(CAN_LOG_QUEUE_LENGTH, sizeof(can_log_record_t));
    s_shellTxMutex = xSemaphoreCreateMutex();
    if (s_shellTxMutex == NULL)
    {
        while (1)
        {
        }
    }

    if (xTaskCreate(LedTask, "led", LED_TASK_STACK_WORDS, NULL, LED_TASK_PRIORITY, NULL) != pdPASS)
    {
        while (1)
        {
        }
    }

    if (xTaskCreate(CanTask, "can", CAN_TASK_STACK_WORDS, NULL, CAN_TASK_PRIORITY, NULL) != pdPASS)
    {
        while (1)
        {
        }
    }

    if (s_canLogQueue != NULL)
    {
        if (xTaskCreate(CanLogTask, "can_log", CAN_LOG_TASK_STACK_WORDS, NULL, CAN_LOG_TASK_PRIORITY, NULL) != pdPASS)
        {
            s_canLogQueue = NULL;
        }
    }

    if (xTaskCreate(AppTask, "usb_shell", APP_TASK_STACK_WORDS, NULL, APP_TASK_PRIORITY, NULL) != pdPASS)
    {
        while (1)
        {
        }
    }

    vTaskStartScheduler();

    while (1)
    {
    }
}
