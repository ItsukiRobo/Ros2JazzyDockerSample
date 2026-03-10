// 文献： http://homepage3.nifty.com/rio_i/lab/driver24/00201chardev.html
#include <linux/uaccess.h>
//#include <asm/uaccess.h>    /* copy_from_user, copy_to_user */
#include <asm/types.h>
#include <linux/errno.h>
#include <linux/fs.h>       /* inode, file, file_operations */
#include <linux/kernel.h>   /* printk */
#include <linux/module.h>
#include <linux/spinlock.h>
#include <linux/ioport.h>
#include <linux/pci.h>
#include <linux/delay.h>

MODULE_DESCRIPTION( "CONTEC AI-1616L-LPE Driver" );
MODULE_LICENSE( "GPL" );

static int   devmajor = 85;
static char* devname  = "AI1616L";
static char* msg      = "module [AI1616L]";


#define MAXBUF 512
static unsigned char devbuf[ MAXBUF ];
static unsigned char write_buf[ MAXBUF ];
static int buf_pos;

static int access_num;
static spinlock_t chardev_spin_lock;


unsigned short baseAddress;


void outpd(unsigned short int x, unsigned int y) {
    outl(y, x);
}

void outpw(unsigned short int x, unsigned short int y) {
    outw(y, x);
}

/*
 * open()
 */
static int 
chardev_open( struct inode* inode, struct file* filp )
{
	//printk( KERN_INFO "%s : open()  called\n", msg );

	spin_lock( &chardev_spin_lock );

	if ( access_num ) {
		spin_unlock( &chardev_spin_lock );
		return -EBUSY;
	}

	access_num ++;
	spin_unlock( &chardev_spin_lock );

	return 0;
}


/*
 * release()
 */
static int 
chardev_release( struct inode* inode, struct file* filp )
{
	//printk( KERN_INFO "%s : close() called\n", msg );

	spin_lock( &chardev_spin_lock );
	access_num --;
	spin_unlock( &chardev_spin_lock );

	return 0;
}


/*
 * read()
 */
static ssize_t 
chardev_read( struct file* filp, char* buf, size_t count, loff_t* pos )
{
	int copy_len;
	int i;
	
	int breakcount;
	
	unsigned short int ad[32];
	unsigned int AI_flg;
	
	
    // Internal gate open
        
    outpd( baseAddress + 0x30, 0x10000001 );
        
    breakcount = 0;
    do{
        AI_flg = inl(baseAddress+0x04) & 0x00000010;
        breakcount++;
        if(breakcount>1000000) {
            //printk(KERN_WARNING "[WARN] Internal Gate Busy\n");
            break;
        }
    }while(false);
    
    udelay(16);  // Based on experiment
    
    
    outpd( baseAddress + 0x38, 0x00000005);
    
	// wait for conversion
    breakcount = 0;
	do{
	    outpd( baseAddress + 0x38, 0x10000001);
	    AI_flg = inl( baseAddress + 0x3C ) & 0x80000000;
        breakcount++;
        if(breakcount>1000000) {
            //printk(KERN_WARNING "[WARN] AD Busy\n");
            break;
        }
	//}while(AI_flg != 0x80000000);
	}while(false);
	
	//for(i=0; i<5000; i++) udelay(100);
	udelay(180);    // Based on experiment
	
	// read data
    
	for(i=0; i<32; i++) {
	    // Read 16ch AD data
	    
	    if(i<32) ad[i] = inw(baseAddress);
	    else inw(baseAddress);
	    
	    	    
    }
    


	//printk( KERN_INFO "%s : read()  called\n", msg );

    // Text data copy
    /*
    copy_len = sprintf(devbuf, "%5d %5d %5d %5d  %5d %5d %5d %5d  %5d %5d %5d %5d  %5d %5d %5d %5d\n%5d %5d %5d %5d  %5d %5d %5d %5d  %5d %5d %5d %5d  %5d %5d %5d %5d\n\n",
     ad[0],ad[1],ad[2],ad[3],ad[4],ad[5],ad[6],ad[7],ad[8],ad[9],ad[10],ad[11],ad[12],ad[13],ad[14],ad[15],
     ad[16],ad[17],ad[18],ad[19],ad[20],ad[21],ad[22],ad[23],ad[24],ad[25],ad[26],ad[27],ad[28],ad[29],ad[30],ad[31]);
     */
    //copy_len = sprintf(devbuf, "%5d %5d %5d %5d  %5d %5d %5d %5d  %5d %5d %5d %5d  %5d %5d %5d %5d\n",
    // ad[0],ad[1],ad[2],ad[3],ad[4],ad[5],ad[6],ad[7],ad[8],ad[9],ad[10],ad[11],ad[12],ad[13],ad[14],ad[15]);
     
    // Binary data copy (unsigned short int (2bytes))
    
    for(i=0; i<16; i++) {
        memcpy(devbuf + i*sizeof(unsigned short int),
            &ad[i], sizeof(unsigned short int));
    }
    
    copy_len = sizeof(unsigned short int)*16;
    
	if ( copy_to_user( buf, devbuf, copy_len ) ) {
		//printk( KERN_INFO "%s : copy_to_user failed\n", msg );
		return -EFAULT;
	}

	//printk( KERN_INFO "%s : buf_pos = %d\n", msg, buf_pos );
	
	//printk( KERN_INFO "%s : BA = %d\n", msg, baseAddress );
	//printk( KERN_INFO "%s : ADDATA = %d\n", msg, addata );

	return copy_len;
}


/*
 * write()
 */
static ssize_t 
chardev_write( struct file* filp, const char* buf, size_t count, loff_t* pos )
{
	int copy_len;
	unsigned int outdata;

	//printk( KERN_INFO "%s : write() called\n", msg );

	if ( count > MAXBUF )
	    return -1;
	else
		copy_len = count;

	if ( copy_from_user( write_buf, buf, copy_len ) ) {
		//printk( KERN_INFO "%s : copy_from_user failed\n", msg );
		return -EFAULT;
	}
	
	// Write Digital Out Data
	outdata = buf[0] & 0x0000000F;
	outpd(baseAddress + 0x18, outdata);
	
	//printk(KERN_DEBUG "Outdata: %d\n", outdata);

	return copy_len;
}


static struct file_operations chardev_fops = 
{
	owner   : THIS_MODULE,
	read    : chardev_read,
	write   : chardev_write,
	open    : chardev_open,
	release : chardev_release,
};


/*
 * ¥â¥ž¥å¡Œ¥ë€ÎœéŽüœèÍý
 * insmod »þ€ËžÆ€Ð€ì€ë
 */
 
#define BASE_ADDRESS_NUM 6
 
int 
init_module( void )
{

    int i=0;
    int j=0;
    
    unsigned short port = 0;
    
    unsigned int AI_sts;
    unsigned int break_count;
    unsigned short int eepromdata;
    
    unsigned short int offset;
    unsigned short int gain;
    
	if ( register_chrdev( devmajor, devname, &chardev_fops ) ) {
		printk( KERN_INFO "%s : register_chrdev failed\n", msg );
		return -EBUSY;
	}

	//spin_lock_init( &chardev_spin_lock );
	//printk( KERN_INFO "%s : loaded  into kernel\n", msg );

    
    //printk(KERN_WARNING "AI1616L Driver init");
    struct pci_dev* device = NULL;
    
    
    baseAddress = 0;
    
    // 複数台は未対応
    device = pci_get_device(0x1221, 0x8623, device);
        
    if(device != NULL) {
        baseAddress = pci_resource_start( device, 0 );
        //printk(KERN_INFO "Device found [%016x] \n", baseAddress);
        port = baseAddress;
        
        //for ( j = 0; j < BASE_ADDRESS_NUM; j ++ ) {
        //    unsigned long resource_start = pci_resource_start( device, j );
        //    printk(KERN_WARNING "Base address: %d", resource_start);
        //}
    }
    
    
    /* Initialize */
    outpd( port + 0x38, 0x00000000 );    /* ECU Reset */
    outpd( port + 0x30, 0x10000000 );    /* AI Reset */
    outpd( port + 0x30, 0x20000000 );    /* AO Reset */
    outpd( port + 0x30, 0x30000000 );    /* DI Reset */
    outpd( port + 0x30, 0x40000000 );    /* DO Reset */
    outpd( port + 0x30, 0x50000000 );    /* CNT Reset */
    outpd( port + 0x30, 0x60000000 );    /* MEM Reset */
    
    /* ECU Setting Destination Sorce Select */
    outpd( port + 0x38, 0x00000003 );
    outpw( port + 0x3C, 0x0000 );    /* AI Start Condition */
    outpw( port + 0x3E, 0x0180 );    /* Genearal Commnd */
    outpd( port + 0x38, 0x00000003 );
    outpw( port + 0x3C, 0x0002 );    /* AI Stop Condition */
    outpw( port + 0x3E, 0x0011 );    /* BeforeTriggerSamplingEnd */
    outpd( port + 0x38, 0x00000003 );
    outpw( port + 0x3C, 0x0004 );    /* AI Clock Condition */
    outpw( port + 0x3E, 0x0004 );    /* InternalCLK */

    /* AI Setting */
    outpd( port + 0x30, 0x10000003 ); /* InternalCLK */
    outpd( port + 0x34, 0x0000018F );

    outpd( port + 0x30, 0x1000000C ); /* Input Method */
    outpd( port + 0x34, 0x00000001 );    /* Multi */

    outpd( port + 0x30, 0x10000005 ); /* Number of channels */
    outpd( port + 0x34, 0x0000000F );    /* 16 */

    outpd( port + 0x30, 0x10000009 ); /* Before Trigger Sampling Number */
    outpd( port + 0x34, 0x00000000 );   // One shot
    
    /* Read default adjustment data from EEPROM */
    outpd( port + 0x30, 0x10000021);    // EEPROM data read
    outpw( port + 0x34, 0x0200);        // Range: +-10V;
    
    break_count = 0;
    do {
        AI_sts = inl(port + 0x04) & 0x00000200;
        //if(break_count == 100) printk(KERN_WARNING "[Debug] status = %016x\n", inl(port+0x04));
        break_count++;
        if(break_count>1000000) {
            //printk(KERN_WARNING "[WARN] EEPROM Busy\n");
            break;
        }
    }while(AI_sts != 0x00000000);
    
    eepromdata = inw(port + 0x36);  // Read EEPROM data (8but gain & 8bit offset)
    
    /* Convert data to gain and offset */
    offset = eepromdata & 0x00FF;
    gain = (eepromdata >> 8) & 0x00FF;
    //printk(KERN_WARNING "Offset = %d, Gain = %d\n", offset, gain);
    
    /* Write the gain and offset to digital potention meter */
    outpd(port + 0x30, 0x10000020);
    outpw(port + 0x34, 0x0000);
    outpw(port + 0x36, offset);
    break_count = 0;
    do {
        AI_sts = inl(port + 0x04) & 0x00000100;
        break_count++;
        if(break_count>1000000) {
            //printk(KERN_WARNING "[WARN] Digital Potentio Busy (1)\n");
            break;
        }
    }while(AI_sts != 0x00000000);
    
    outpd(port + 0x30, 0x10000020);
    outpw(port + 0x34, 0x0001);
    outpw(port + 0x36, gain);
    break_count = 0;
    do {
        AI_sts = inl(port + 0x04) & 0x00000100;
        break_count++;
        if(break_count>1000000) {
            //printk(KERN_WARNING "[WARN] Digital Potentio Busy (2)\n");
            break;
        }
    }while(AI_sts != 0x00000000);
    
    //printk(KERN_WARNING "AI1616L started\n");
    
    /* FOR DEBUG PURPOSE */
    /* Test AD convert */
    
	unsigned short int addata;
	unsigned int AI_flg;
	
	addata = 0;
	
    // ad converter start
    outpd( port + 0x30, 0x10000001 );
        
    break_count = 0;
    do{
        AI_flg = inl(port+0x04) & 0x00000010;
        break_count++;
        if(break_count>1000000) {
            //printk(KERN_WARNING "[WARN] Internal Gate Busy (2)\n");
            break;
        }
    }while(AI_flg == 0x00000010);
    
    outpd( port + 0x38, 0x00000005);
    
	// wait for conversion
    break_count = 0;
	do{
	    outpd( port + 0x38, 0x10000001);
	    AI_flg = inl( port + 0x3C ) & 0x80000000;
        break_count++;
        if(break_count>1000000) {
            //printk(KERN_WARNING "[WARN] AD Busy\n");
            break;
        }
	}while(AI_flg != 0x80000000);
	// read data
	for(i=0; i<100; i++) {
    	addata = inw(port);
    	//printk( KERN_INFO "%s : ADDATA = %d\n", msg, addata );
    }

    
    
	return 0;
}


void 
cleanup_module( void )
{

	unregister_chrdev( devmajor, devname );

	//printk( KERN_INFO "%s : removed from kernel\n", msg );

}


/* End of chardev.c */
