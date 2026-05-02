#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

int main(void)
{
    LOG_INF("Zephyr RTOS Booting on Custom Smartwatch...");
    
    while (1) {
        LOG_INF("Main thread alive.");
        k_msleep(1000);
    }
    return 0;
}
