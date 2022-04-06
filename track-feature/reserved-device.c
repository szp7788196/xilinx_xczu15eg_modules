#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/cdev.h>
#include <linux/ide.h>
#include <linux/poll.h>
#include <linux/fcntl.h>

#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>


#define MAX_TRACK_FEATURE_NUM               128
#define MAX_TRACK_FEATURE_ITEMS_NUM         4096
#define TRACK_FEATURE_ITEMS_NUM             128


struct trackFeatureItem_S {
    unsigned short pos_x;
    unsigned short pos_y;
    unsigned short id;
    unsigned short reserved;

}__attribute__ ((packed));

struct trackFeature_S {
    unsigned short item_num;
    unsigned long time_stamp;
    struct trackFeatureItem_S item[TRACK_FEATURE_ITEMS_NUM];

}__attribute__ ((packed));


#define DRIVER_NAME     "reserved-device"
#define DRIVER_NUMBER   1


struct reserved_device {
	dev_t devid;
	struct cdev cdev;
	struct class *class;
	struct device *device;
	int irq_gpio;
	int irq_num;
	unsigned int mem_start;
	unsigned int mem_end;
	unsigned char  *phys_addr;
	unsigned char  *virt_addr;
	struct fasync_struct *async_queue;	/* fasync_struct结构体 */
};

struct reserved_device *reserved_dev = NULL;

static irqreturn_t reserved_device_irq(int irq, void *reserved_dev)
{
	struct reserved_device *dev = (struct reserved_device *)reserved_dev;
	struct trackFeatureItem_S item;
	struct trackFeatureItem_S item1;
	int i = 0;
	static unsigned long time_stamp = 0;

/***************************************test code*******************************************/
	memset(&item,0,sizeof(struct trackFeatureItem_S));

	time_stamp ++;
	
	memcpy(dev->virt_addr,&time_stamp,8);

	for(i = 0; i < TRACK_FEATURE_ITEMS_NUM; i ++)
	{
		item.id = i;
		item.pos_x = i;
		item.pos_y = i;

		memcpy((dev->virt_addr + 8 + i * 8),&item,sizeof(struct trackFeatureItem_S));

		memcpy(&item1,dev->virt_addr + 8 + i * 8,8);
	}

/*****************************************************************************************/

	if(dev->async_queue)
	{
		kill_fasync(&dev->async_queue, SIGIO, POLL_IN);
	}

	return IRQ_HANDLED;
}

static int reserved_dev_open(struct inode *inode, struct file *filp)
{
	filp->private_data = reserved_dev;

	return 0;
}

/*
 * @description			: 从设备读取数据 
 * @param – filp		: 要打开的设备文件(文件描述符)
 * @param - buf			: 返回给用户空间的数据缓冲区
 * @param - cnt			: 要读取的数据长度
 * @param – off			: 相对于文件首地址的偏移
 * @return				: 读取的字节数，如果为负值，表示读取失败
 */
static ssize_t reserved_dev_read(struct file *filp, char __user *buf,size_t cnt, loff_t *off)
{
	int ret = 0;
	static int write_index = 0;

	ret = copy_to_user(buf, &write_index, cnt);
	if(ret != 0)
	{
		printk(KERN_ERR "Failed to copy data to user buffer\n");

		return -EFAULT;
	}

	return ret;
}

/*
 * @description			: 向设备写数据 
 * @param – filp		: 设备文件，表示打开的文件描述符
 * @param - buf			: 要写给设备写入的数据
 * @param - cnt			: 要写入的数据长度
 * @param - offt		: 相对于文件首地址的偏移
 * @return				: 写入的字节数，如果为负值，表示写入失败
 */
static ssize_t reserved_dev_write(struct file *filp, const char __user *buf,size_t cnt, loff_t *offt)
{
	int ret = 0;
	struct reserved_device *reserved_dev = (struct reserved_device *)filp->private_data;
	char *temp_buf = NULL;

	temp_buf = (char *)kmalloc(cnt * sizeof(char),GFP_KERNEL);

	if(temp_buf == NULL)
	{
		return -1;
	}

	memset(temp_buf,0,cnt);

	ret = copy_from_user(temp_buf, buf, cnt);
	if(ret < 0)
	{
		printk(KERN_ERR "Failed to copy data from user buffer\n");
		return -EFAULT;
	}

	memcpy(reserved_dev->virt_addr,temp_buf,cnt);

	kfree(temp_buf);

	return 0;
}

/*
 * @description		: fasync函数，用于处理异步通知
 * @param - fd		: 文件描述符
 * @param - filp	: 要打开的设备文件(文件描述符)
 * @param - on		: 模式
 * @return			: 负数表示函数执行失败
 */
static int reserved_dev_fasync(int fd, struct file *filp, int on)
{
	struct reserved_device *reserved_dev = (struct reserved_device *)filp->private_data;

	return fasync_helper(fd, filp, on, &reserved_dev->async_queue);
}

static int reserved_dev_mmap(struct file*filp, struct vm_area_struct *vma)
{
	struct reserved_device *reserved_dev = (struct reserved_device *)filp->private_data;
	unsigned long pfn_start = ((reserved_dev->mem_start) >> PAGE_SHIFT) + vma->vm_pgoff;
	unsigned long size = vma->vm_end - vma->vm_start;

	vma->vm_flags |= VM_IO;
	vma->vm_flags |= (VM_DONTEXPAND | VM_DONTDUMP);

	if (remap_pfn_range(vma,vma->vm_start,pfn_start, size, vma->vm_page_prot))
	{
		printk("remap_pfn_range failed\n");
		return  -EAGAIN;
	}

	return 0;
}
/*
 * @description		: 关闭/释放设备
 * @param – filp	: 要关闭的设备文件(文件描述符)
 * @return			: 0 成功;其他 失败
 */
static int reserved_dev_release(struct inode *inode, struct file *filp)
{
	return reserved_dev_fasync(-1, filp, 0);
}

static struct file_operations reserved_dev_fops = {
	.owner  	= THIS_MODULE,
	.open   	= reserved_dev_open,
    .read   	= reserved_dev_read,
	.write  	= reserved_dev_write,
	.fasync		= reserved_dev_fasync,
	.mmap		= reserved_dev_mmap,
	.release	= reserved_dev_release,
};

static int reserved_memory_init(struct platform_device *pdev,struct reserved_device *reserved_dev)
{
	struct device_node *np = NULL;
	struct device *dev = &pdev->dev;
	struct resource r_mem; /* IO mem resources */
	int rc = 0;
	
	/* Get reserved memory region from Device-tree */
	np = of_parse_phandle(dev->of_node, "memory-region", 0);
	if (!np) 
	{
		dev_err(dev, "No %s specified\n", "memory-region");
		goto error1;
	}

	rc = of_address_to_resource(np, 0, &r_mem);
	if (rc) 
	{
		dev_err(dev, "No memory address assigned to the region\n");
		goto error1;
	}

	reserved_dev->mem_start = r_mem.start;
	reserved_dev->mem_end = r_mem.end;
	reserved_dev->phys_addr = (unsigned char *)r_mem.start;

	if (!request_mem_region(reserved_dev->mem_start,reserved_dev->mem_end - reserved_dev->mem_start + 1,DRIVER_NAME)) 
    {
		dev_err(dev, "Couldn't lock device region at %p\n",(void *)(long)reserved_dev->mem_start);
		rc = -EBUSY;
		goto error1;
	}

	reserved_dev->virt_addr = (unsigned char *)memremap(reserved_dev->mem_start, reserved_dev->mem_end - reserved_dev->mem_start + 1,MEMREMAP_WB);
	if (!reserved_dev->virt_addr) 
    {
		dev_err(dev, "reserved-device: Could not allocate iomem\n");
		rc = -EIO;
		goto error2;
	}

	dev_info(dev, "Allocated reserved memory, phys_addr: 0x%08X, virt_addr: 0x%08X\n", 
	        (unsigned char)(long)reserved_dev->phys_addr, (unsigned char)(long)reserved_dev->virt_addr);

	return rc;

error2:
	release_mem_region(reserved_dev->mem_start, reserved_dev->mem_end - reserved_dev->mem_start + 1);
error1:
	kfree(reserved_dev);

	return -1;
}

static int interrupt_init(struct platform_device *pdev,struct reserved_device *reserved_dev)
{
	int ret = 0;
	unsigned long irq_flags;
	struct device *dev = &pdev->dev;

	/* 获取设备树中的key-gpio属性，得到按键的GPIO编号 */
	reserved_dev->irq_gpio = of_get_named_gpio(dev->of_node, "interrupt-gpio", 0);
	if(!gpio_is_valid(reserved_dev->irq_gpio)) 
	{
		printk(KERN_ERR "Failed to get interrupt-gpio\n");
		return -EINVAL;
	}

	/* 获取GPIO对应的中断号 */
	reserved_dev->irq_num = gpio_to_irq(reserved_dev->irq_gpio);
	if (!reserved_dev->irq_num)
	{
		printk(KERN_ERR "Failed to get irq num\n");
		return -EINVAL;
	}

	/* 申请使用GPIO */
	ret = gpio_request(reserved_dev->irq_gpio, "irq gpio");
	if (ret)
	{
		printk(KERN_ERR "Failed to request irq gpio\n");
		return -EINVAL;
	}

	/* 将GPIO设置为输入模式 */
	gpio_direction_input(reserved_dev->irq_gpio);

	/* 获取设备树中指定的中断触发类型 */
	irq_flags = irq_get_trigger_type(reserved_dev->irq_num);
	if (IRQF_TRIGGER_NONE == irq_flags)
	{
		printk(KERN_ERR "Failed to get trigger type, default to IRQF_TRIGGER_FALLING\n");
		irq_flags = IRQF_TRIGGER_FALLING;
	}
	
	/* 申请中断 */
	ret = request_irq(reserved_dev->irq_num, reserved_device_irq, irq_flags, "PS Key0 IRQ", reserved_dev);
	if (ret) 
	{
		printk(KERN_ERR "Failed to request irq\n");
		gpio_free(reserved_dev->irq_gpio);
		return -EINVAL;
	}
		
	return ret;
}

static int reserved_device_probe(struct platform_device *pdev)
{
    int ret = 0;
	struct device *dev = &pdev->dev;

	printk("Device Tree Probing\n");

	reserved_dev = (struct reserved_device *) kmalloc(sizeof(struct reserved_device), GFP_KERNEL);
	if (!reserved_dev) 
    {
		dev_err(dev, "Cound not allocate reserved-device device\n");
		return -ENOMEM;
	}

	dev_set_drvdata(dev, reserved_dev);

	ret = reserved_memory_init(pdev,reserved_dev);
	if(ret == -1)
	{
		dev_err(dev, "Cound not init reserved memory\n");
		goto error0;
	}

	interrupt_init(pdev,reserved_dev);

	ret = alloc_chrdev_region(&reserved_dev->devid, 0, DRIVER_NUMBER, DRIVER_NAME);
	if(ret)
	{
		dev_err(dev, "alloc chrdev region failed.\n");
		return ret;
	}

	reserved_dev->cdev.owner = THIS_MODULE;
	cdev_init(&reserved_dev->cdev, &reserved_dev_fops);

	ret = cdev_add(&reserved_dev->cdev, reserved_dev->devid, DRIVER_NUMBER);
	if(ret)
	{
		dev_err(dev, "cdev add failed.\n");
		goto error2;
	}

	reserved_dev->class = class_create(THIS_MODULE, DRIVER_NAME);
	if (IS_ERR(reserved_dev->class)) 
	{
		dev_err(dev, "class create failed.\n");
		ret = PTR_ERR(reserved_dev->class);
		goto error1;
	}

    reserved_dev->device = device_create(reserved_dev->class, &pdev->dev,reserved_dev->devid, NULL, DRIVER_NAME);
	if (IS_ERR(reserved_dev->device)) 
	{
		dev_err(dev, "device create failed.\n");
		ret = PTR_ERR(reserved_dev->device);
		goto error3;
	}

	return 0;

error3:
	class_destroy(reserved_dev->class);
error2:
	unregister_chrdev_region(reserved_dev->devid, DRIVER_NUMBER);
error1:
	cdev_del(&reserved_dev->cdev);
error0:
	dev_set_drvdata(dev, NULL);

	return -EINVAL;
}

static int reserved_device_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct reserved_device *reserved_dev = dev_get_drvdata(dev);

	gpio_free(reserved_dev->irq_gpio);
	free_irq(reserved_dev->irq_num, NULL);
	memunmap(reserved_dev->virt_addr);
	release_mem_region(reserved_dev->mem_start, reserved_dev->mem_end - reserved_dev->mem_start + 1);
	kfree(reserved_dev);
	dev_set_drvdata(dev, NULL);

	device_destroy(reserved_dev->class, reserved_dev->devid);
	class_destroy(reserved_dev->class);
	cdev_del(&reserved_dev->cdev);
	unregister_chrdev_region(reserved_dev->devid, DRIVER_NUMBER);

	return 0;
}

static struct of_device_id reserved_device_of_match[] = {
	{ .compatible = "eyestar,reserved-device", },
	{ /* end of list */ },
};

MODULE_DEVICE_TABLE(of, reserved_device_of_match);

static struct platform_driver reserved_device_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table	= reserved_device_of_match,
	},
	.probe		= reserved_device_probe,
	.remove		= reserved_device_remove,
};

static int __init reserved_device_init(void)
{
	printk("reserved-device:Hello.\n");
	return platform_driver_register(&reserved_device_driver);
}


static void __exit reserved_device_exit(void)
{
	platform_driver_unregister(&reserved_device_driver);
	printk(KERN_ALERT "reserved-device:Goodbye.\n");
}

module_init(reserved_device_init);
module_exit(reserved_device_exit);

MODULE_AUTHOR("Eyestar, Inc.");
MODULE_DESCRIPTION("Eyestar Reserved device driver");
MODULE_LICENSE("GPL v2");
