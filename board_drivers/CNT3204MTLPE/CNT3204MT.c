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

MODULE_DESCRIPTION( "CONTEC CNT-3204MT-LPE Driver" );
MODULE_LICENSE( "GPL" );

static int   devmajor = 87;
static char* devname  = "CNT3204MT";
static char* msg      = "module [CNT3204MT]";


#define MAXBUF 128
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
	
	uint32_t cntdata[4];
	uint32_t stsdata;
	
	// read data
    outpw(baseAddress+0x02, 0x00ff); /* 全チャネルラッチ */
    outpw(baseAddress+0x00, 0x0000); /* チャネル0のカウント値読み出しを指定 */
    cntdata[0] = inl(baseAddress+0x00); /* カウント値を読み出し(DWordアクセス以外不可)*/
    outpw(baseAddress+0x00, 0x0001); /* CH1 */
    cntdata[1] = inl(baseAddress+0x00);
    outpw(baseAddress+0x00, 0x0002); /* CH2 */
    cntdata[2] = inl(baseAddress+0x00);
    outpw(baseAddress+0x00, 0x0003); /* CH3 */
    cntdata[3] = inl(baseAddress+0x00);
    outpw(baseAddress+0x08, 0x003f); /* ステータスリードコマンド指定 */
    stsdata = inl(baseAddress+0x0c); /* ステータスを読み出し*/
    
    //printk(KERN_DEBUG "%08d %08d %08d %08d\n", cntdata[0],cntdata[1],cntdata[2],cntdata[3] );

    
	udelay(10);    // Based on experiment
	    
    for(i=0; i<4; i++) {
        memcpy(devbuf + i*sizeof(uint32_t),
            &cntdata[i], sizeof(uint32_t));
    }
    
    copy_len = sizeof(uint32_t)*4;
    
	if ( copy_to_user( buf, devbuf, copy_len ) ) {
		//printk( KERN_INFO "%s : copy_to_user failed\n", msg );
		return -EFAULT;
	}


	return copy_len;
}


/*
 * write()
 */
static ssize_t 
chardev_write( struct file* filp, const char* buf, size_t count, loff_t* pos )
{
    int i;
	int copy_len;
	unsigned char chs;  // Channels to be reseted
	
	unsigned char data_load_flg;

	if ( count > MAXBUF )
	    return -1;
	else
		copy_len = count;

	if ( copy_from_user( write_buf, buf, copy_len ) ) {
		//printk( KERN_INFO "%s : copy_from_user failed\n", msg );
		return -EFAULT;
	}
	
	chs = buf[0];
	// Reset counter values of specified channels;
	data_load_flg = chs & 0x0F;
	
    outpw(baseAddress+0x08, 0x003c);    /* プリセットデータ（0に設定済み）ロード */
    outpd(baseAddress+0x0c, data_load_flg); /* 指定チャネルロード */
	
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

    unsigned short port = 0;
        
	if ( register_chrdev( devmajor, devname, &chardev_fops ) ) {
		//printk( KERN_INFO "%s : register_chrdev failed\n", msg );
		return -EBUSY;
	}

	//spin_lock_init( &chardev_spin_lock );
	//printk( KERN_INFO "%s : loaded  into kernel\n", msg );

    
    //printk(KERN_WARNING "CNT3204MT Driver init");
    struct pci_dev* device = NULL;
    
    
    baseAddress = 0;
    
    // 複数台は未対応
    device = pci_get_device(0x1221, 0x8605, device);
        
    if(device != NULL) {
        baseAddress = pci_resource_start( device, 0 );
        //printk(KERN_INFO "Device found [%016x] \n", baseAddress);
        port = baseAddress;
        
    }
    
    outpw(baseAddress+0x08, 0x0008);    /* 動作モード設定 CH0 */
    outpd(baseAddress+0x0c, 0x00000022);    /* 差動入力（？）、CW=UP、2相入力、同期、4逓倍 */
    outpw(baseAddress+0x08, 0x0009);    /* 動作モード設定 CH1 */
    outpd(baseAddress+0x0c, 0x00000022);
    outpw(baseAddress+0x08, 0x000A);    /* 動作モード設定 CH2 */
    outpd(baseAddress+0x0c, 0x00000022);
    outpw(baseAddress+0x08, 0x000B);    /* 動作モード設定 CH3 */
    outpd(baseAddress+0x0c, 0x00000022);
    
    outpw(baseAddress+0x08, 0x0010);    /* プリセットレジスタ設定 CH0 */
    outpd(baseAddress+0x0c, 0x00000000);    /* データ"00000000"設定*/
    outpw(baseAddress+0x08, 0x0011);    /* プリセットレジスタ設定 CH1 */
    outpd(baseAddress+0x0c, 0x00000000);    /* データ"00000000"設定*/
    outpw(baseAddress+0x08, 0x0012);    /* プリセットレジスタ設定 CH2 */
    outpd(baseAddress+0x0c, 0x00000000);    /* データ"00000000"設定*/
    outpw(baseAddress+0x08, 0x0013);    /* プリセットレジスタ設定 CH3 */
    outpd(baseAddress+0x0c, 0x00000000);    /* データ"00000000"設定*/
    
    outpw(baseAddress+0x08, 0x0030);    /* Z相入力設定 CH0 */
    outpd(baseAddress+0x0c, 0x00000001); /* 無効 */
    outpw(baseAddress+0x08, 0x0031);    /* Z相入力設定 CH1 */
    outpd(baseAddress+0x0c, 0x00000001); /* 無効 */
    outpw(baseAddress+0x08, 0x0032);    /* Z相入力設定 CH2 */
    outpd(baseAddress+0x0c, 0x00000001); /* 無効 */
    outpw(baseAddress+0x08, 0x0033);    /* Z相入力設定 CH3 */
    outpd(baseAddress+0x0c, 0x00000001); /* 無効 */
    
    outpw(baseAddress+0x08, 0x003c);    /* プリセットデータロード */
    outpd(baseAddress+0x0c, 0x0000000f); /* 全チャネルロード */
    
    // Counter start
    outpw(baseAddress+0x04, 0x000F);    /* Start all channels */
    
	return 0;
}


void 
cleanup_module( void )
{

	unregister_chrdev( devmajor, devname );

	//printk( KERN_INFO "%s : removed from kernel\n", msg );

}


/* End of chardev.c */
