#include "FreeRTOS.h"
#include "task.h"

void vTaskNotifyGiveFromISR(TaskHandle_t task,
                            BaseType_t *higherPriorityTaskWoken)
{
    ARG_UNUSED(task);
    if (higherPriorityTaskWoken != NULL)
    {
        *higherPriorityTaskWoken = pdFALSE;
    }
}

int main(void)
{
    return 0;
}
