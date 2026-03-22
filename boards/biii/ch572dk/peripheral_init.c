
#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/devicetree.h>

#include <hal/wch/ch5xx/ch5xx.h>

static int peripherals_init(void)
{
	ch5xx_set_debug_enable(true);
	return 0;
}

SYS_INIT(peripherals_init, APPLICATION, CONFIG_KERNEL_INIT_PRIORITY_DEVICE);
