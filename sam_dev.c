/*  sam_dev.c - Create an input/output character device */


/* Our own ioctl numbers */
#include "sam_dev.h"

/* Deal with CONFIG_MODVERSIONS */
#if CONFIG_MODVERSIONS==1
#define MODVERSIONS
#include <linux/modversions.h>

#endif

/* Include Headers */

#include <asm/io.h>          /* for ioremap(), ioread(), and iowrite() */
#include <asm/uaccess.h>     /* for get_user and put_user */
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/fs.h>        /* The character device definitions are here */
#include <linux/interrupt.h> /* add interrupt support */
#include <linux/ioport.h>
#include <linux/kernel.h>
#include <linux/amba/bus.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/wait.h>      /* wait_event() for blocking I/O */

/* #define's*/

#define SUCCESS 0

/* Device Declarations */

/* 
 * The name for our device, as it will appear in 
 * /proc/devices 
 */


#define DEVICE_NAME "hwAcc"
#define DEV_IRQ 320
#define DRIVER_NAME "generic_hw_accelerator"

#define MMR_ADDR  0x13002000
#define MMR_SIZE  0x00000000

/* Is the device open right now? Used to prevent 
 * concurent access into the same device */
static int Device_Open = 0;

/* Address of the I/O device */
static void *addr;

static struct resource *mmr_res;

static int intFlag;

static DECLARE_WAIT_QUEUE_HEAD(devQueue);

static uint32_t mmrconfig;

static struct cdev c_dev;
static dev_t first = MKDEV(MAJOR_NUM, 0);
static struct class *cl;
static struct sam_dev *sm;

/* Device struct */
struct hw_acc {
  int irq_num;
};



/* string to match :- of_device_match */
static const struct of_device_id my_of_ids[] = {
        { .compatible = "gem5,io_acc" },
        { }
    };


/* 
 * This function is called whenever a process attempts
 * to open the device file 
 */
static int device_open(struct inode *inode, 
                       struct file *file)
{
  printk ("device_open(%p)\n", file);

  /* We don't want to talk to two processes at the 
   * same time */
  if (Device_Open)
      {
      return -EBUSY;
      }

  /* If this was a process, we would have had to be 
   * more careful here, because one process might have 
   * checked Device_Open right before the other one 
   * tried to increment it. However, we're in the 
   * kernel, so we're protected against context switches.
   *
   * This is NOT the right attitude to take, because we
   * might be running on an SMP box, but we'll deal with
   * SMP in a later chapter.
   */ 

  Device_Open++;

  return SUCCESS;
}


/* This function is called when a process closes the 
 * device file. It doesn't have a return value because 
 * it cannot fail. Regardless of what else happens, you 
 * should always be able to close a device (in 2.0, a 2.2
 * device file could be impossible to close).
 */
static int device_release(struct inode *inode, 
                          struct file *file)
{
  printk ("device_release(%p,%p)\n", inode, file);
 
  /* We're now ready for our next caller */
  Device_Open --;

  return 0;
}

static void mmrWrite(struct file *file, size_t mmrVal)
{
    iowrite32(mmrVal, addr + 4);
}

static void configWrite(struct file *file, size_t val)
{
    iowrite32(val, addr);
}

static uint32_t configRead(struct file *file)
{
    return ioread32(addr);
}

/* This function is called whenever a process tries to 
 * do an ioctl on our device file. We get two extra 
 * parameters (additional to the inode and file 
 * structures, which all device functions get): the number
 * of the ioctl called and the parameter given to the 
 * ioctl function.
 *
 * If the ioctl is write or read/write (meaning output 
 * is returned to the calling process), the ioctl call 
 * returns the output of this function.
 */
int device_ioctl(
    struct file *file,
    unsigned int ioctl_num,/* The number of the ioctl */
    unsigned long ioctl_param) /* The parameter to it */
{
  printk("ioctl() detected with number: %x\n", ioctl_num);
  /* Switch according to the ioctl called */
  switch (ioctl_num) {
    case IOCTL_MMR_WRITE:
      /* Receive a pointer to a message (in user space) 
       * and set that to be the device's message. */ 

      /* Get the parameter given to ioctl by the process */
      printk("Attempting to write to device MMR\n");
      mmrWrite(file, ioctl_param);
      break;

    case IOCTL_BLOCKING_DEV_ACCESS:
      /* Give the current message to the calling 
       * process - the parameter we got is a pointer, 
       * fill it. */
      printk("Attempting to access the device\n");
      intFlag = 0;
      //mmrRead(file);
      configWrite(file, 0x0001);
      printk("Sleeping until device is finished writing back\n");
      wait_event_interruptible(devQueue, intFlag != 0);
      break;
      
    case IOCTL_NONBLOCKING_DEV_ACCESS:
      printk("Attempting to access the device\n");
      intFlag = 0;
      configWrite(file, 0x0001);
      //mmrRead(file);
      break;
      
    case IOCTL_DEV_SYNC:
      printk("Checking to see if device has finished writing back\n");
      wait_event_interruptible(devQueue, intFlag != 0);
      break;
      
    case IOCTL_POLLING_ACCESS:
      printk("Attempting to access the device\n");
      configWrite(file, 0x0001);
      while(!(ioread32(addr) & 0x80000000)) {
        udelay(10);
      }
      break;
  }

  return SUCCESS;
}


/* Interrupt request handler for this driver */
static irqreturn_t devHandler(int irq, void *dev_id)
{
  intFlag = 1;
  wake_up_interruptible(&devQueue);
  return IRQ_HANDLED;
}


/* 
 * amba_bus_probe:Bus driver probe
 * Returns: N/A
 */
static int amba_bus_probe(struct amba_device *dev, const struct amba_id *id)
{
	printk(KERN_INFO "Entering amba_bus_probe\n");
	return 0;
}

/*
 * amba_bus_remove: Bus driver remove
 * Returns: N/A
 */

static int amba_bus_remove(struct amba_device *dev)
{
	return 1;
}


/* File operations structure */
static struct file_operations Fops = {
  .owner = THIS_MODULE,
  .unlocked_ioctl = (long)device_ioctl,   /* ioctl */
  .open = device_open,
  .release = device_release  /* a.k.a. close */
};

static int sam_dev_remove(struct platform_device *pdev)
{
  return 0;
}

/*static int sam_dev_probe(struct platform_device *pdev)
{ 
  printk("SUB: Entering my probe\n");
  struct resource *res;
  int irq;
  struct device *dev = &pdev->dev;

  struct of_device_id *match;
  sm = platform_get_drvdata(pdev);

  /*if (sm == NULL)
  {
    printk(KERN_ALERT "Platform get drvdata returned NULL\n");
    return -1;
  }

  
  match = of_match_device(of_match_ptr(my_of_ids), dev);
  irq=platform_get_irq(pdev, 0);
  printk("SUB: platform_get_irq returns %d\n", irq);

return 0;
}
*/



/*
 * This function returns 1 iff pdev isn't a device instatiated by dt, 0 iff it
 * could successfully get all information from dt or a negative errno.
 */


int sam_dev_probe_dt(struct platform_device *pdev, struct amba_port *sport)
{
	
	struct device_node *np = pdev->dev.of_node;
	int ret;

	sport->devdata = of_device_get_match_data(&pdev->dev);
	if (!sport->devdata)
    /* no device tree device */
		return 1;
    

	if (of_get_property(np, "gem5,io_acc", NULL))
		{
			printk(KERN_INFO "Unable to get compatible string\n");
		}
    
    
    return 0;
}

static int sam_dev_probe(struct amba_device *pdev)
{
	struct amba_port *sport;
int   ret_val = alloc_chrdev_region(&first, 0, 1, DEVICE_NAME);
       if (ret_val < 0) 
           {
           platform_driver_unregister(&sam_dev_drv);
           printk("SUB: Device Registration failed\n");
           return -1;
           }
       if ((cl = class_create(THIS_MODULE, "chardev")) == NULL) 
           {
           platform_driver_unregister(&sam_dev_drv);
           printk("Sam_Dev Class creation failed\n");
           unregister_chrdev_region(first, 1);
           return -1;
           }
     

  if (device_create(cl, NULL, first, NULL, "sam_dev_drv") == NULL) 
      {
       platform_driver_unregister(&sam_dev_drv);
       printk("Sam_Dev Device creation failed\n" );
       class_destroy(cl);
       unregister_chrdev_region(first, 1);
       return -1;
      }

  cdev_init(&c_dev, &Fops);

  if (cdev_add(&c_dev, first, 1)==-1) 
      {
      platform_driver_unregister(&sam_dev_drv);
      printk("Sam_Dev Device addition failed\n");
      device_destroy(cl, first);
      class_destroy(cl);
      unregister_chrdev_region(first, 1);
      return -1;
      }
  printk ("%s The major device number is %d.\n",
          "Registeration is a success",
          MAJOR_NUM);
  printk ("If you want to talk to the device driver,\n");
  printk ("you'll have to create a device file. \n");
  printk ("We suggest you use:\n");
  printk ("mknod %s c %d 0\n", DEVICE_FILE_NAME, 
          MAJOR_NUM);
  printk ("The device file name is important, because\n");
  printk ("the ioctl program assumes that's the\n");
  printk ("file you'll use.\n");
  

	printk( KERN_INFO "Entering sam_dev_probe\n");
	struct amba_port *sport;
	void __iomem *base;
	

	int ret = 0;
	struct resource *res;

	struct of_device_id *match;

	int irq;

	sport = devm_kzalloc(&pdev->dev, sizeof(*sport), GFP_KERNEL);
	if (!sport)
		return -ENOMEM;
	

	ret = sam_dev_probe_dt(pdev);
	if (ret == 0)
		return ret;

    match = of_match_device(of_match_ptr(my_of_ids), dev);

    irq=platform_get_irq(pdev, 0);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	base = devm_ioremap_resource(&pdev->dev, res);


	rxirq = platform_get_irq(pdev, 0);
	
		ret = devm_request_irq(&pdev->dev, irq, devHandler, 0,
				       DEVICE_NAME, sport);
		if (ret)
		    {
			dev_err(&pdev->dev, "failed to request rx irq: %d\n",
				ret);
			return ret;
		    }
	
	return 0;
	
}




/* Platform driver structure */

static struct platform_driver sam_dev_drv = {
  .probe = sam_dev_probe,
  .remove = sam_dev_remove,
  .driver = {
            .name = DEVICE_NAME,
            .owner = THIS_MODULE,
            .of_match_table = of_match_ptr(my_of_ids)
            }
};


static struct amba_driver amba_bus_driver = {

	.owner          = THIS_MODULE,
	.driver_name    = DRIVER_NAME,
	.dev_name       = DEV_NAME,
	.major          = SERIAL_IMX_MAJOR,
	.minor          = MINOR_START,
	
};


/* Initialize the module - Register the character device */
static int sam_dev_init(void)
{
  	/*Register with the bus - amba*/
    int ret = amba_driver_register(&amba_bus_driver);
    if(ret)
    {
    	printk(KERN_INFO "The amba Registration fails with ret=%d\n", ret);
    }

    /* Register with as a platform */
	int ret = platform_driver_register(&sam_dev_drv);
	if (ret != 0)
	{
		printk(KERN_INFO "Platform driver Registration fails\n");
	}

return 0;
}



/* Cleanup - unregister the appropriate file from /proc */

static void sam_dev_exit(void)
{
  /*
  unregister_chrdev(MAJOR_NUM, DEVICE_NAME);

  if ((mmr_res->start == MMR_ADDR) &&
      (mmr_res->end == (MMR_ADDR + MMR_SIZE))) {
    iounmap(addr);
    release_mem_region(MMR_ADDR, MMR_SIZE);
  }
  free_irq(DEV_IRQ, NULL);
  
  */

   printk(KERN_INFO "SUB: Entering our driver's exit routine \n");

   platform_driver_unregister(&sam_dev_drv);
   
   return amba_driver_unregister(&amba_bus_driver);
}


module_amba_driver(amba_bus_driver);

module_init(sam_dev_init);
module_exit(sam_dev_exit);


MODULE_AUTHOR("TeCSAR");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("IOAcc device driver");
