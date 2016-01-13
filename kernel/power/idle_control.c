/*
	* kernel/power/idle_control.c - control idle mole  relation functionality.
	*
	* Copyright (C) 2014-2015  rock-chips <wxt@ rock-chips.com>
 
	* This program is free software; you can redistribute it and/or modify
	* it under the terms of the GNU General Public License as published by
	* the Free Software Foundation; either version 2 of the License, or
	* (at your option) any later version.

	* This program is distributed in the hope that it will be useful,
	* but WITHOUT ANY WARRANTY; without even the implied warranty of
	* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	* GNU General Public License for more details.

	* You should have received a copy of the GNU General Public License
	* along with this program;
 */

#include <linux/cdev.h>  
#include <linux/semaphore.h>  
#include <linux/init.h>  
#include <linux/module.h>  
#include <linux/types.h>  
#include <linux/fs.h>  
#include <linux/proc_fs.h>  
#include <linux/device.h>  
#include <asm/uaccess.h>  

#define IDLE_DEVICE_PROC	1
#define IDLE_DEVICE_DEV		0
#define IDLE_DEVICE_CLASS	0

#define IDLE_DEVICE_NODE_NAME  "idle"  
#define IDLE_DEVICE_FILE_NAME  "idle"  
#define IDLE_DEVICE_PROC_NAME  "idle"  
#define IDLE_DEVICE_CLASS_NAME "idle"  
  
struct idle_android_dev {  
	int val; 
    struct semaphore sem;  
    struct cdev dev;  
};  


static int idle_major = 0;  
static int idle_minor = 0;  
  
  
static struct class* idle_class = NULL;  
static struct idle_android_dev* idle_dev = NULL;  
  
  
static int idle_open(struct inode* inode, struct file* filp);  
static int idle_release(struct inode* inode, struct file* filp);  
static ssize_t idle_read(struct file* filp, char __user *buf, size_t count, loff_t* f_pos);  
static ssize_t idle_write(struct file* filp, const char __user *buf, size_t count, loff_t* f_pos);  
  
  
static struct file_operations idle_fops = {  
    .owner = THIS_MODULE,  
    .open = idle_open,  
    .release = idle_release,  
    .read = idle_read,  
    .write = idle_write,   
};  
  
  
static ssize_t idle_val_show(struct device* dev, struct device_attribute* attr,  char* buf);  
static ssize_t idle_val_store(struct device* dev, struct device_attribute* attr, const char* buf, size_t count);  
  
  
static DEVICE_ATTR(val, S_IRUGO | S_IWUSR, idle_val_show, idle_val_store);

static int idle_open(struct inode* inode, struct file* filp) {  
    struct idle_android_dev* dev;          
      
    dev = container_of(inode->i_cdev, struct idle_android_dev, dev);  
    filp->private_data = dev;  
      
    return 0;  
}  
  
static int idle_release(struct inode* inode, struct file* filp) {  
    return 0;  
}  
  
static ssize_t idle_read(struct file* filp, char __user *buf, size_t count, loff_t* f_pos) {  
    ssize_t err = 0;  
    struct idle_android_dev* dev = filp->private_data;          

    if(down_interruptible(&(dev->sem))) {  
        return -ERESTARTSYS;  
    }  
  
    if(count < sizeof(dev->val)) {  
        goto out;  
    }          
  
    if(copy_to_user(buf, &(dev->val), sizeof(dev->val))) {  
        err = -EFAULT;  
        goto out;  
    }  
  
    err = sizeof(dev->val);  
  
out:  
    up(&(dev->sem));  
    return err;  
}  
  
static ssize_t idle_write(struct file* filp, const char __user *buf, size_t count, loff_t* f_pos) {  
    struct idle_android_dev* dev = filp->private_data;  
    ssize_t err = 0;          

    if(down_interruptible(&(dev->sem))) {  
        return -ERESTARTSYS;          
    }          
  
    if(count != sizeof(dev->val)) {  
        goto out;          
    }          
 
    if(copy_from_user(&(dev->val), buf, count)) {  
        err = -EFAULT;  
        goto out;  
    }  
  
    err = sizeof(dev->val);  
  
out:  
    up(&(dev->sem));  
    return err;  
}  

static ssize_t __idle_get_val(struct idle_android_dev* dev, char* buf) {  

	int val = 0;          

 //   if(down_interruptible(&(dev->sem))) {                  
  //      return -ERESTARTSYS;          
 //   }          
#if defined (CONFIG_IDLE)
	dev->val =1;
#else
	dev->val =0;
#endif

    val = dev->val;          
  //  up(&(dev->sem));          
  
    return snprintf(buf, PAGE_SIZE, "%d\n", val);  
}  
  
static ssize_t __idle_set_val(struct idle_android_dev* dev, const char* buf, size_t count) {  

	int val = 0;                  
           
    val = simple_strtol(buf, NULL, 10);          
        
    //if(down_interruptible(&(dev->sem))) {                  
   //     return -ERESTARTSYS;          
   // }          

    dev->val = val;          
   // up(&(dev->sem));  
  
    return count;  
}  
  
static ssize_t idle_val_show(struct device* dev, struct device_attribute* attr, char* buf) {  
    struct idle_android_dev* hdev = (struct idle_android_dev*)dev_get_drvdata(dev);          
  
    return __idle_get_val(hdev, buf);  
}  
  
static ssize_t idle_val_store(struct device* dev, struct device_attribute* attr, const char* buf, size_t count) {   
    struct idle_android_dev* hdev = (struct idle_android_dev*)dev_get_drvdata(dev);    
      
    return __idle_set_val(hdev, buf, count);  
}  

static ssize_t idle_proc_read(char* page, char** start, off_t off, int count, int* eof, void* data) {  
    if(off > 0) {  
        *eof = 1;  
        return 0;  
    }  
  
    return __idle_get_val(idle_dev, page);  
}  
  
  
static ssize_t idle_proc_write(struct file* filp, const char __user *buff, unsigned long len, void* data) {  
    int err = 0;  
    char* page = NULL;  
  
    if(len > PAGE_SIZE) {  
        printk(KERN_ALERT"The buff is too large: %lu.\n", len);  
        return -EFAULT;  
    }  
  
    page = (char*)__get_free_page(GFP_KERNEL);  
    if(!page) {                  
        printk(KERN_ALERT"Failed to alloc page.\n");  
        return -ENOMEM;  
    }          
  
      
    if(copy_from_user(page, buff, len)) {  
        printk(KERN_ALERT"Failed to copy buff from user.\n");                  
        err = -EFAULT;  
        goto out;  
    }  
  
    err = __idle_set_val(idle_dev, page, len);  
  
out:  
    free_page((unsigned long)page);  
    return err;  
}  
  
  
static void idle_create_proc(void) {  
    struct proc_dir_entry* entry;  
      
    entry = create_proc_entry(IDLE_DEVICE_PROC_NAME, 0, NULL);  
    if(entry) {  
    //    entry->owner = THIS_MODULE;  
        entry->read_proc = idle_proc_read;  
        entry->write_proc = idle_proc_write;  
    }  
}  
  
  
static void idle_remove_proc(void) {  
    remove_proc_entry(IDLE_DEVICE_PROC_NAME, NULL);  
}  

static int  __idle_setup_dev(struct idle_android_dev* dev) {  
    int err;  
    dev_t devno = MKDEV(idle_major, idle_minor);  
  
    memset(dev, 0, sizeof(struct idle_android_dev));  
  
    cdev_init(&(dev->dev), &idle_fops);  
    dev->dev.owner = THIS_MODULE;  
    dev->dev.ops = &idle_fops;          
  
      
    err = cdev_add(&(dev->dev),devno, 1);  
    if(err) {  
        return err;  
    }          
  
      
 	sema_init(&(dev->sem),1);  
    dev->val = 0;  
  
    return 0;  
}  
  
  
static int __init idle_init(void){   
    int err = -1;  
    dev_t dev = 0;  
    struct device* temp = NULL;  

    err = alloc_chrdev_region(&dev, 0, 1, IDLE_DEVICE_NODE_NAME);  
    if(err < 0) {  
        printk(KERN_ALERT"Failed to alloc char dev region.\n");  
        goto fail;  
    }  
  
    idle_major = MAJOR(dev);  
    idle_minor = MINOR(dev);          
  
      
    idle_dev = kmalloc(sizeof(struct idle_android_dev), GFP_KERNEL);  
    if(!idle_dev) {  
        err = -ENOMEM;  
        printk(KERN_ALERT"Failed to alloc idle_dev.\n");  
        goto unregister;  
    }          
#if (IDLE_DEVICE_DEV)   	
    err = __idle_setup_dev(idle_dev);  
    if(err) {  
        printk(KERN_ALERT"Failed to setup dev: %d.\n", err);  
        goto cleanup;  
    }          
#endif    
	
#if (IDLE_DEVICE_CLASS)  

    idle_class = class_create(THIS_MODULE, IDLE_DEVICE_CLASS_NAME);  
    if(IS_ERR(idle_class)) {  
        err = PTR_ERR(idle_class);  
        printk(KERN_ALERT"Failed to create idle class.\n");  
        goto destroy_cdev;  
    }           

    temp = device_create(idle_class, NULL, dev, "%s", IDLE_DEVICE_FILE_NAME);  
    if(IS_ERR(temp)) {  
        err = PTR_ERR(temp);  
        printk(KERN_ALERT"Failed to create idle device.");  
        goto destroy_class;  
    }          
  
      
    err = device_create_file(temp, &dev_attr_val);  
    if(err < 0) {  
        printk(KERN_ALERT"Failed to create attribute val.");                  
        goto destroy_device;  
    }  
  
    dev_set_drvdata(temp, idle_dev);          
#endif
#if  (IDLE_DEVICE_PROC)   
    idle_create_proc();  
#endif 
    //printk("Succedded to initialize idle device.\n");  
    return 0;  
#if (IDLE_DEVICE_CLASS)   
destroy_device:  
    device_destroy(idle_class, dev);  
  
destroy_class:  
    class_destroy(idle_class);  

destroy_cdev:  
    cdev_del(&(idle_dev->dev)); 
#endif 
#if (IDLE_DEVICE_DEV)  
cleanup:  
    kfree(idle_dev);  
#endif 

unregister:  
    unregister_chrdev_region(MKDEV(idle_major, idle_minor), 1);  
fail:  
    return err;  


}  
  
  
static void __exit idle_exit(void) {  
    dev_t devno = MKDEV(idle_major, idle_minor);  
  
#if defined (IDLE_DEVICE_PROC)       
    idle_remove_proc();          
#endif  
#if defined (IDLE_DEVICE_CLASS)      
    if(idle_class) {  
        device_destroy(idle_class, MKDEV(idle_major, idle_minor));  
        class_destroy(idle_class);  
    }          
#endif 
#if defined (IDLE_DEVICE_DEV)     
    if(idle_dev) {  
        cdev_del(&(idle_dev->dev));  
        kfree(idle_dev);  
    }           
    unregister_chrdev_region(devno, 1);  
#endif
}  
  
MODULE_LICENSE("GPL");  
MODULE_DESCRIPTION("wxt@rock-chips.com for geting idle node");  
  
module_init(idle_init);  
module_exit(idle_exit);  

