/*  sam_dev.c - Create an input/output character device */


/* Our own ioctl numbers */
#include "sam_dev.h"

/* Deal with CONFIG_MODVERSIONS */
#if CONFIG_MODVERSIONS==1
#define MODVERSIONS
#include <linux/modversions.h>

#endif

#include <asm/io.h> /* for ioremap(), ioread(), and iowrite() */
#include <asm/uaccess.h>  /* for get_user and put_user */
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/fs.h> /* The character device definitions are here */
#include <linux/interrupt.h> /* add interrupt support */
#include <linux/ioport.h>
#include <linux/kernel.h>
#include <linux/amba/bus.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/wait.h> /* wait_event() for blocking I/O */

#define SUCCESS 0

/* Device Declarations ******************************** */

/* The name for our device, as it will appear in 
 * /proc/devices */
#define DEVICE_NAME "sam_dev"
#define DEV_IRQ 320

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

struct sam_dev {
  int irq_num;
};




static const struct of_device_id my_of_ids[] = {
        { .compatible = "gem5,io_acc" },
        { }
    };


/* This function is called whenever a process attempts
 * to open the device file */
static int device_open(struct inode *inode, 
                       struct file *file)
{
  printk ("device_open(%p)\n", file);

  /* We don't want to talk to two processes at the 
   * same time */
  if (Device_Open)
    return -EBUSY;

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

static irqreturn_t devHandler(int irq, void *dev_id)//, struct pt_regs *regs)
{
  intFlag = 1;
  wake_up_interruptible(&devQueue);
  return IRQ_HANDLED;
}

static int amba_bus_probe(struct amba_device *dev, const struct amba_id *id)
{
	return 1;//pl011_register_port(uap);
}


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


/* Platform driver structure */
static struct platform_driver sam_dev_drv = {
  .probe = sam_dev_probe,
  .remove = sam_dev_remove,
  .driver = {
    .name = DEVICE_NAME,
    .owner = THIS_MODULE,
    .of_match_table = my_of_ids
  }
};


/* Probe the platform driver */
static int sam_dev_probe(struct platform_device *pdev)
{
  sm = platform_get_drvdata(pdev);

  if (sm == NULL)
  {
    printk(KERN_ALERT "Platform get drvdata returned NULL\n");
    return -1;
  }

int ret_val;
  /*ret_val = platform_driver_probe(&sam_dev_drv, &sam_dev_probe);
  if (ret_val < 0) {
    printk("Sam_Dev Platform Driver probe failed with %d\n", ret_val);
    return -1;
  } else {*/
    ret_val = alloc_chrdev_region(&first, 0, 1, DEVICE_NAME);
    if (ret_val < 0) {
      platform_driver_unregister(&sam_dev_drv);
      printk("ADC Device Registration failed\n");
      return -1;
    }
    if ((cl = class_create(THIS_MODULE, "chardev")) == NULL) {
      platform_driver_unregister(&sam_dev_drv);
      printk("Sam_Dev Class creation failed\n");
      unregister_chrdev_region(first, 1);
      return -1;
    }
  }

  if (device_create(cl, NULL, first, NULL, "sam_dev_drv") == NULL) {
    platform_driver_unregister(&sam_dev_drv);
    printk("Sam_Dev Device creation failed\n" );
    class_destroy(cl);
    unregister_chrdev_region(first, 1);
    return -1;
  }

  cdev_init(&c_dev, &Fops);

  if (cdev_add(&c_dev, first, 1)==-1) {
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
  
  printk ("Requesting memory region for device MMR\n");
  mmr_res = request_mem_region(MMR_ADDR, MMR_SIZE, DEVICE_FILE_NAME);
  
  if (mmr_res && ((mmr_res->start != MMR_ADDR) &&
      (mmr_res->end != (MMR_ADDR + MMR_SIZE)))) {
    printk ("Could not map memory mapped register for device!\n");
    return -1;
  } else {
    printk ("Registering device address: 0x%08x of size: %d\n", MMR_ADDR, MMR_SIZE);
    addr = ioremap_nocache(MMR_ADDR, MMR_SIZE);
  }
  printk("Attempting to reserve IRQ number: %d\n", DEV_IRQ);
  if (request_irq(DEV_IRQ, devHandler, 0, DEVICE_NAME, NULL)) {
    printk("Device could not be assigned IRQ %i\n", DEV_IRQ);
    return -1;
  }
  intFlag = 0;
  
  return 0;
}

/*static int sam_dev_probe(struct platform_device *pdev)
{
  sm = platform_get_drvdata(pdev);

  if (sm == NULL)
  {
    printk(KERN_ALERT "Platform get drvdata returned NULL\n");
    return -1;
  }

  return 0;
}*/

static int sam_dev_remove(struct platform_device *pdev)
{
  return 0;
}





static struct amba_driver amba_bus_driver = {
	
	.probe		= amba_bus_probe,
	.remove		= amba_bus_remove,
        .drv =     {
                   .name = DEVICE_NAME,
                   .owner = THIS_MODULE,
                   }

};

/* This structure will hold the functions to be called
 * when a process does something to the device we
 * created. Since a pointer to this structure is kept in
 * the devices table, it can't be local to
 * init_module. NULL is for unimplemented functions. */

/* Initialize the module - Register the character device */
static int sam_dev_init(void)
{
  	printk(KERN_INFO "SUB: Entering our driver's init routine \n");

	if (platform_driver_register(&sam_dev_drv))
            {
		pr_warn("SUB: could not register our platform driver\n");
            }
	return amba_driver_register(&amba_bus_driver);
}






/* Cleanup - unregister the appropriate file from /proc */
static void sam_dev_exit(void)
{
  unregister_chrdev(MAJOR_NUM, DEVICE_NAME);

  if ((mmr_res->start == MMR_ADDR) &&
      (mmr_res->end == (MMR_ADDR + MMR_SIZE))) {
    iounmap(addr);
    release_mem_region(MMR_ADDR, MMR_SIZE);
  }
  free_irq(DEV_IRQ, NULL);
}

module_init(sam_dev_init);
module_exit(sam_dev_exit);

MODULE_AUTHOR("Samuel Rogers");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("IOAcc device driver");
