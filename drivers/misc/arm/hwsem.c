#include <linux/fs.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/of_address.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/of.h>
#include <linux/cdev.h>

#define DRV_NAME "hwsem"
#define HWSEM_NUM 16

/* HWSEM master ID */
#ifndef HWSEM_MST_ID
#define HWSEM_MST_ID 0xA1B2C3D4
#endif

/* IOCTL commands */
#define HWSEM_RD_MST_ID 1
#define HWSEM_RD_CNT    3
#define HWSEM_ACQUIRE   4
#define HWSEM_RELEASE   5
#define HWSEM_CLEAR     6

struct hwsem {
	struct device *dev;
        struct miscdevice miscdev;
	void __iomem *hwsem_acq;
	void __iomem *hwsem_rel;
	void __iomem *hwsem_rst;
	int irq;
};

static int i = 0;

static long hwsem_ioctl(struct file *f,
		unsigned int cmd, unsigned long arg)
{
        struct hwsem *drvdata = container_of(f->private_data,
                struct hwsem, miscdev);
        ssize_t master_id;

        switch (cmd) {
        case HWSEM_RD_MST_ID:
                return readl(drvdata->hwsem_acq);
        case HWSEM_RD_CNT:
		return readl(drvdata->hwsem_rel);
        case HWSEM_ACQUIRE:
		master_id = readl(drvdata->hwsem_acq);
                writel(master_id, drvdata->hwsem_acq);
                break;
        case HWSEM_RELEASE:
		master_id = readl(drvdata->hwsem_acq);
		writel(master_id, drvdata->hwsem_rel);
                break;
        case HWSEM_CLEAR:
		master_id = readl(drvdata->hwsem_acq);
		writel(master_id, drvdata->hwsem_rst);
                break;
        default:
                break;
        }

        return 0;
}

const static struct file_operations hwsem_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = hwsem_ioctl,
};

static irqreturn_t hwsem_isr(int irq, void *dev_id)
{
	struct hwsem *hwsem_dev_id = dev_id;
        writel(HWSEM_MST_ID, hwsem_dev_id->hwsem_acq);
        return IRQ_HANDLED;
}

static char* hwsem_drv_itoa(int i)
{
    static char str[8];
    sprintf(str, "hwsem%d", i);
    return str;
}

static int hwsem_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct hwsem *hwsem_ptr;
	struct resource *res;
	void __iomem *hwsem_acq;
	void __iomem *hwsem_rel;
	void __iomem *hwsem_rst;
	int ret;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "hwsem_acq");
	hwsem_acq = devm_ioremap_resource(dev, res);
	if (IS_ERR(hwsem_acq))
		return PTR_ERR(hwsem_acq);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "hwsem_rel");
	hwsem_rel = devm_ioremap_resource(dev, res);
	if (IS_ERR(hwsem_rel))
		return PTR_ERR(hwsem_rel);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "hwsem_rst");
	hwsem_rst = devm_ioremap_resource(dev, res);
	if (IS_ERR(hwsem_rst))
		return PTR_ERR(hwsem_rst);

	hwsem_ptr = devm_kzalloc(&pdev->dev, sizeof(struct hwsem), GFP_KERNEL);
	if(!hwsem_ptr)
		return -ENOMEM;

	hwsem_ptr->hwsem_acq = hwsem_acq;
	hwsem_ptr->hwsem_rel = hwsem_rel;
	hwsem_ptr->hwsem_rst = hwsem_rst;
	hwsem_ptr->dev = &pdev->dev;

        hwsem_ptr->irq = platform_get_irq(pdev, 0);
	if(hwsem_ptr->irq < 0) {
		dev_err(hwsem_ptr->dev, "Failed to get IRQ from platform_get_irq()\n");
		return hwsem_ptr->irq;
	}

	ret = devm_request_irq(&pdev->dev, hwsem_ptr->irq, hwsem_isr,
				IRQF_SHARED, "hardware semaphore", hwsem_ptr);

        if(ret) {
		dev_err(hwsem_ptr->dev, "Failed to request IRQ from devm_request_irq()\n");
		return ret;
	}

	platform_set_drvdata(pdev, hwsem_ptr);

	dev_set_drvdata(dev, (void *)hwsem_ptr);
        hwsem_ptr->miscdev.minor = MISC_DYNAMIC_MINOR;
        hwsem_ptr->miscdev.name = hwsem_drv_itoa(i);
        hwsem_ptr->miscdev.fops = &hwsem_fops;
        hwsem_ptr->miscdev.parent = dev;

        ret = misc_register(&hwsem_ptr->miscdev);
	if (ret) {
		dev_err(dev, "Unable to register Hardware Semaphore device\n");
		return ret;
	}

	dev_info(hwsem_ptr->dev, "Alif hardware semaphore registered\n");
        ++i;

	return 0;
}

static int hwsem_remove(struct platform_device *pdev)
{
	struct hwsem *hwsem_ptr = platform_get_drvdata(pdev);

        misc_deregister(&hwsem_ptr->miscdev);
	devm_free_irq(&pdev->dev, hwsem_ptr->irq, hwsem_ptr);
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id hwsem_match[] = {
	{ .compatible = "arm,hwsem" },
	{ }
};

MODULE_DEVICE_TABLE(of, hwsem_match);
#endif

static struct platform_driver hwsem_driver = {
	.driver = {
		.name = DRV_NAME,
		.of_match_table = of_match_ptr(hwsem_match),
	},
	.probe = hwsem_probe,
	.remove = hwsem_remove,
};

module_platform_driver(hwsem_driver)

MODULE_LICENSE("GPLv2");
MODULE_DESCRIPTION("Alif Hardware Semaphore Driver");
MODULE_AUTHOR("@alifsemi.com>");
