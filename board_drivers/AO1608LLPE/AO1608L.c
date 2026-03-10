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

MODULE_DESCRIPTION( "CONTEC AO-1608L-LPE Driver" );
MODULE_LICENSE( "GPL" );

static int   devmajor = 200;
static char* devname  = "AO1608L";
static char* msg      = "module [AO1608L]";


#define MAXBUF 512
static unsigned char devbuf[ MAXBUF ];
static unsigned char write_buf[ MAXBUF ];
static int buf_pos;

static int access_num;
static spinlock_t chardev_spin_lock;


uint16_t baseAddress;


void outpd(uint16_t x, uint32_t y) {
    outl(y, x);
}

void outpw(uint16_t x, uint16_t y) {
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
	
	uint16_t ad[32];
	uint32_t AI_flg;
	
	buf[0] = 0;
    
    copy_len = 1;
    
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
	int i;
	uint16_t dadata[9]; // 8th is used for Digital Output

	//printk( KERN_INFO "%s : write() called\n", msg );
	    
	if ( count > MAXBUF )
	    return -1;
	else
		copy_len = count;

	if ( copy_from_user( write_buf, buf, copy_len ) ) {
		//printk( KERN_INFO "%s : copy_from_user failed\n", msg );
		return -EFAULT;
	}
	
	
    for(i=0; i<9; i++) {
        dadata[i] = 0;
        if(i*sizeof(uint16_t) >= count) break;
        memcpy(&dadata[i], write_buf + i*sizeof(uint16_t),
            sizeof(uint16_t));
            
        //printk("AD #%2d: %5d\n", i, dadata[i]);
    }
	
	//printk(KERN_DEBUG "Outdata: %d\n", outdata);
	
	// AD output data
	for(i=0; i<8; i++) {
	    outpd(baseAddress + 0x08, dadata[i] & 0x0000FFFF);
	}
	
	// Internal Gate Open
	outpd(baseAddress + 0x30, 0x20000001);
	
	outpd(baseAddress + 0x38, 0x00000005);  // AO Start
	
	udelay(200);    // Based on experiment
	
	// Write Digital Out Data
    outpd(baseAddress + 0x18, dadata[8] & 0x0000000F);

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
    
    uint16_t port = 0;
    
    uint32_t AO_sts;
    uint32_t break_count;
    uint16_t eepromdata;
    
    uint16_t offset;
    uint16_t gain;
    
	if ( register_chrdev( devmajor, devname, &chardev_fops ) ) {
		//printk( KERN_INFO "%s : register_chrdev failed\n", msg );
		return -EBUSY;
	}

	//spin_lock_init( &chardev_spin_lock );
	//printk( KERN_INFO "%s : loaded  into kernel\n", msg );

    
    //printk(KERN_WARNING "AI1616L Driver init");
    struct pci_dev* device = NULL;
    
    
    baseAddress = 0;
    
    // 複数台は未対応
    device = pci_get_device(0x1221, 0x86C3, device);
        
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
    outpd( port + 0x30, 0x20000000 );    /* AO Reset */
    outpd( port + 0x30, 0x30000000 );    /* DI Reset */
    outpd( port + 0x30, 0x40000000 );    /* DO Reset */
    outpd( port + 0x30, 0x50000000 );    /* CNT Reset */
    outpd( port + 0x30, 0x60000000 );    /* MEM Reset */
    
    /* ECU Setting Destination Sorce Select */
    outpd( port + 0x38, 0x00000003 );
    outpw( port + 0x3C, 0x0020 );    /* AO Start Condition */
    outpw( port + 0x3E, 0x0180 );    /* Genearal Commnd */
    outpd( port + 0x38, 0x00000003 );
    outpw( port + 0x3C, 0x0022 );    /* AI Stop Condition */
    outpw( port + 0x3E, 0x0050 );    /* BeforeTriggerSamplingEnd */
    outpd( port + 0x38, 0x00000003 );
    outpw( port + 0x3C, 0x0024 );    /* AI Clock Condition */
    outpw( port + 0x3E, 0x0042 );    /* InternalCLK */

    /* AO Setting */
    outpd( port + 0x30, 0x20000003 ); /* InternalCLK */
    outpd( port + 0x34, 0x0000018F );

    outpd( port + 0x30, 0x2000000C ); /* Input Method */
    outpd( port + 0x34, 0x00000001 );    /* Multi */

    outpd( port + 0x30, 0x20000005 ); /* Number of channels */
    outpd( port + 0x34, 0x00000007 );    /* 8 */

    outpd( port + 0x30, 0x20000009 ); /* Before Trigger Sampling Number */
    outpd( port + 0x34, 0x00000000 );   // One shot
    
    /* Read default adjustment data from EEPROM */
    outpd( port + 0x30, 0x20000021);    // EEPROM data read
    outpw( port + 0x34, 0x0200);        // Range: +-10V;
    
    break_count = 0;
    do {
        AO_sts = inl(port + 0x0C) & 0x00000200;
        //if(break_count == 100) printk(KERN_WARNING "[Debug] status = %016x\n", inl(port+0x04));
        break_count++;
        if(break_count>1000000) {
            //printk(KERN_WARNING "[WARN] EEPROM Busy\n");
            break;
        }
    }while(AO_sts != 0x00000000);
    
    eepromdata = inw(port + 0x36);  // Read EEPROM data (8but gain & 8bit offset)
    
    /* Convert data to gain and offset */
    offset = eepromdata & 0x00FF;
    gain = (eepromdata >> 8) & 0x00FF;
    //printk(KERN_WARNING "Offset = %d, Gain = %d\n", offset, gain);
    
    /* Write the gain and offset to digital potention meter */
    outpd(port + 0x30, 0x20000020);
    outpw(port + 0x34, 0x0000);
    outpw(port + 0x36, offset);
    break_count = 0;
    do {
        AO_sts = inl(port + 0x0C) & 0x00000100;
        break_count++;
        if(break_count>1000000) {
            //printk(KERN_WARNING "[WARN] Digital Potentio Busy (1)\n");
            break;
        }
    }while(AO_sts != 0x00000000);
    
    outpd(port + 0x30, 0x20000020);
    outpw(port + 0x34, 0x0001);
    outpw(port + 0x36, gain);
    break_count = 0;
    do {
        AO_sts = inl(port + 0x0C) & 0x00000100;
        break_count++;
        if(break_count>1000000) {
            //printk(KERN_WARNING "[WARN] Digital Potentio Busy (2)\n");
            break;
        }
    }while(AO_sts != 0x00000000);
    
    //printk(KERN_WARNING "AO1608L started\n");
    
    /* FOR DEBUG PURPOSE */
    /* Test AD convert */
    
	uint16_t addata;
	uint32_t AI_flg;
	
	addata = 0;
	/*
    // ad converter start
    outpd( port + 0x30, 0x10000001 );
        
    break_count = 0;
    do{
        AI_flg = inl(port+0x04) & 0x00000010;
        break_count++;
        if(break_count>1000000) {
            printk(KERN_WARNING "[WARN] Internal Gate Busy (2)\n");
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
            printk(KERN_WARNING "[WARN] AD Busy\n");
            break;
        }
	}while(AI_flg != 0x80000000);
	// read data
	for(i=0; i<100; i++) {
    	addata = inw(port);
    	printk( KERN_INFO "%s : ADDATA = %d\n", msg, addata );
    }
    */
    
    
	return 0;
}


void 
cleanup_module( void )
{

	unregister_chrdev( devmajor, devname );

	//printk( KERN_INFO "%s : removed from kernel\n", msg );

}


/* End of chardev.c */
