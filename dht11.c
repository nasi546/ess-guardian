#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/device.h>
#include <linux/jiffies.h>

#define DEVICE_NAME "dht11"		// /dev/dht11
#define CLASS_NAME  "dht_device_class"
#define GPIO_PIN	4

MODULE_LICENSE("GPL");
MODULE_AUTHOR("kkk");
MODULE_DESCRIPTION("dht11 driver");

static struct class *dht11_class = NULL;
static dev_t dev_num;
static struct cdev dht11_cdev;

static int wait_pin_status(int level, int time_us)
{
	int counter = 0;
	while(gpio_get_value(GPIO_PIN) != level){
		if(++counter > time_us) return -1;
		udelay(1);
	}
	return counter;
}

static int read_dht11(int *temp, int *humi)
{
	unsigned char data[6] = {0, 0, 0, 0, 0};
	unsigned long flags;
	//int result;

	gpio_direction_output(GPIO_PIN, 0);
	msleep(20);
	gpio_set_value(GPIO_PIN, 1);
	udelay(30);

	gpio_direction_input(GPIO_PIN);
	udelay(2);

	local_irq_save(flags);	// 인터럽트 비활성화
	// ... 타이밍이 중요한 데이터 읽기 루프 ...
	// 1. 센서(DHT11)  응답 확인 : Low(80us) --> High(80us)
	wait_pin_status(0, 100);
	if(wait_pin_status(1, 100) < 0) return -1;
	// 2. wait for the start of the first data bit(low 50us)
	if(wait_pin_status(0, 100) < 0) return -1;

	for(int i = 0; i < 40; i++)
	{
		// wait for high level of data bit
		if(wait_pin_status(1, 100) < 0) return -1;
		udelay(35);		// 0 and 1 decision threahold
		if(gpio_get_value(GPIO_PIN))
		{
			data[i / 8] |= (1 << (7 - (i % 8)));	// set 1
			// wait until high ends
			if(wait_pin_status(0, 100) < 0) return -1;
		}
	}

	local_irq_restore(flags); // 인터럽트 복구

	if(data[4] == ((data[0] + data[1] + data[2] + data[3]) & 0xff))
	{
		*humi = data[0];
		*temp = data[2];
	}
	else return -1;

	return 0;
}

static ssize_t dht11_dev_read(struct file *filep, char *buffer, size_t len, loff_t *offset)
{
	int temp = 0, humi = 0;
	int ret;
	char msg_buff[80];

	if(*offset > 0)
	{
		*offset = 0;
		return 0;
	}

	ret = read_dht11(&temp, &humi);
	if(ret == 0)
		sprintf(msg_buff, "temp: %d c humi: %d %%\n", temp, humi);
	else sprintf(msg_buff, "DHT11 read error !!!! %d\n", ret);

	if(copy_to_user(buffer, msg_buff, strlen(msg_buff)))
		return -1;

	*offset = strlen(msg_buff);

	return strlen(msg_buff);
}

static struct file_operations fops = {
	.read	= dht11_dev_read
};

static int __init dht11_driver_init(void)
{
	int ret;

	printk(KERN_INFO"=== dht11 initializing ===\n");
	// 1. alloc device number
	if((ret = alloc_chrdev_region(&dev_num, 0, 1, DEVICE_NAME)) == -1) {
		printk(KERN_ERR "ERROR: alloc_chrdev_regin ....\n");
		return ret;
	}
	// 2. register char device
	cdev_init(&dht11_cdev, &fops);
	if((ret = cdev_add(&dht11_cdev, dev_num, 1)) == -1) {
		printk(KERN_ERR "ERROR: cdev_add ....\n");
		unregister_chrdev_region(dev_num, 1);
		return ret;
	}
	// 3. create class & create device
	dht11_class = class_create(THIS_MODULE, CLASS_NAME);
	if(IS_ERR(dht11_class)) {
		cdev_del(&dht11_cdev);
		unregister_chrdev_region(dev_num, 1);
		return PTR_ERR(dht11_class);
	}
	device_create(dht11_class, NULL, dev_num, NULL, DEVICE_NAME);
	// 4. request gpio
	// LED output mode
	if(gpio_request(GPIO_PIN, "my_dht11_data_pin") < 0)
	{
		printk(KERN_ERR "ERROR: gpio_request...\n");
		return -1;			
	}
	// set input mode
	// gpio_direction_input(GPIO_PIN);
	// 5. assign gpio to irq
	// interrupt_num_s1 = gpio_to_irq(S1_GPIO);
	// ret = request_irq(interrupt_num_s1,
	//rotary_int_handler, IRQF_TRIGGER_FALLING,
    //                  "my_rotary_irq_S1", NULL);
    // if (ret) {
    //  printk(KERN_ERR "ERROR: request_irq  ........\n");
    //  return -ret;
    //}
	printk(KERN_INFO "DHT11 driver init success ........\n");
	return 0;
}

static void __exit dht11_driver_exit(void)
{
	gpio_free(GPIO_PIN);
	device_destroy(dht11_class, dev_num);
	class_destroy(dht11_class);
	cdev_del(&dht11_cdev);
	unregister_chrdev_region(dev_num, 1);

	printk(KERN_INFO "dht11_driver_exit !!\n");
}
module_init(dht11_driver_init);
module_exit(dht11_driver_exit);

