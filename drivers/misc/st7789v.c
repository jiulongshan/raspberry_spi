#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/compat.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/delay.h>
// #include <linux/dmaengine.h>

#include <linux/string.h>
#include <linux/mm.h>
#include <linux/syscalls.h>
#include <asm/unistd.h>
#include <asm/uaccess.h>

#include <linux/leds.h>

#include "st7789v_initlogo.h"

#define GPIO_OUT_ZERO 0
#define GPIO_OUT_ONE  1

//#define VDD_PIN      (GPIO7)   //VDD, no vdd control 
#define LCD_PWM      12//(GPIO12)led pwm,rpi_pin32

#define TE_PIN       7 //(GPIO7) TE,rpi_pin26
#define CS1_PIN      8 //(GPIO8) lcd1cs,rpi_pin24
#define CS2_PIN      8 //(GPIO8) lcd2cs
#define RS_PIN       27//(GPIO27)rs,rpi_pin13
#define RESET_PIN    17//(GPIO17)reset,rpi_pin11

//spi gpio
#define CLK_PIN      11//(GPIO11)clk,rpi_pin23
#define MI_PIN       9 //(GPIO9) mi,cpu spi data out,rpi_pin21
#define MO_PIN       10//(GPIO10)mo,cpu spi data in ,rpi_pin19

#define LCD_H 320
#define LCD_W 240
#define PAKET_SIZE   LCD_H*LCD_W*2 

#define LCD_L        0
#define LCD_R        1 
#define LCDALL       2
#define LCD_DISABLE  3
#define LCDINIT      3

#define OPEN_LOG 1 

#define SPI_LCD_INIT 1

#define YJX_LOG(mystring,arg...)    do{ \
    if(OPEN_LOG) {\
        printk(mystring, ##arg); \
    } \
} while(0)

static int major;
static struct class * class ;
static unsigned char * ker_buf;
static DEFINE_MUTEX(my_lock);
struct spi_device *st7789v_device = NULL;
u8  resumebuf[2] = {0};

typedef enum { 
    SLPOUT      = 0x11, 
    MADCTL      = 0x36, 
    COLMOD      = 0x3A, 
    RAMCTRL     = 0xB0, 
    PORCTRL     = 0xB2, 
    GCTRL       = 0xB7, 
    VCOMS       = 0xBB, 
    LCMCTRL     = 0xC0, 
    VDVVRHEN    = 0xC2, 
    VRHS        = 0xC3, 
    VDVSET      = 0xC4, 
    FRCTR2      = 0xC6, 
    PWCTRL1     = 0xD0, 
    PVGAMCTRL   = 0xE0, 
    NVGAMCTRL   = 0xE1,
    INVOFF      = 0x20,
    INVON       = 0x21, 
    DISPOFF     = 0x28, 
    DISPON      = 0x29, 
    CASET       = 0x2A, 
    RASET       = 0x2B, 
    RAMWR       = 0x2C, 
    GATECTRL    = 0xE4, 
    TEON        = 0x35, 
    STE         = 0x44,
} ST7789V_INSTRUCTION;
/**
 * 定义 ST7789V LCD 初始化结构体
 */
static struct spi_lcd_cmd {
    uint8_t reg_addr;   // 命令寄存器地址
    uint8_t len;        // 需要从 st7789v_init_datas 数组里发出数据字节数
    int delay_ms;       // 此命令发送数据完成后，需延时多久
} cmds[] = {
    {SLPOUT, 0x00, 0x78}, 
#ifdef ENABLE_TE_SYNC
    {GATECTRL, 0x03, 0x00},  
    {TEON, 0x01, 0x00},  
    {STE, 0x02, 0x00},  
#endif /* ENABLE_TE_SYNC */
    {MADCTL, 0x01, 0x00}, 
    {COLMOD, 0x01, 0x00},  
    {RAMCTRL, 0x02, 0x00},  
    {PORCTRL, 0x05, 0x00}, 
    {GCTRL, 0x01, 0x00}, 
    {VCOMS, 0x01, 0x00},  
    {LCMCTRL, 0x01, 0x00},  
    {VDVVRHEN, 0x01, 0x00},
    {VRHS, 0x01, 0x00}, 
    {VDVSET, 0x01, 0x00}, 
    {FRCTR2, 0x01, 0x00},  
    {PWCTRL1, 0x02, 0x00},  
    {PVGAMCTRL, 0x0E, 0x00},  
    {NVGAMCTRL, 0x0E, 0x00},  
    {INVON, 0x00, 0x00}, 
    //{INVOFF, 0x00, 0x00}, 
    {CASET, 0x04, 0x00},  
    {RASET, 0x04, 0x00},
    {DISPON,    0x00, 0x32},
    {RAMWR,     0x00, 0x00},
};

/**
 * 定义 ST7789V LCD  初始化数据
*/
uint8_t st7789v_init_datas[] = {
	    // command: SLPOUT
#ifdef ENABLE_TE_SYNC
        0x1D,0x00,0x10,     // command GATECTRL
        0x00,               // command: TEON
        0x00, 0x20,         // command: STE
#endif /*ENABLE_TE_SYNC*/
	    0x00,		// command: MADCTL
	    0x05,		// command: COLMOD
	    0x00, 0xF8,		// command: RAMCTRL
	    0x0C, 0x0C, 0x00, 0x33, 0x33,	// command: PORCTRL
	    0x35,		// command: GCTRL
	    0x19,		// command: VCOMS
	    0x2C,		// command: LCMCTRL
	    0x01,		// command: VDVVRHEN
	    0x12,		// command: VRHS
	    0x20,		// command: VDVSET
	    0x0F,		// command: FRCTR2
	    0xA4, 0xA1,		// command: PWCTRL1
	    //0xd0, 0x08, 0x11, 0x08, 0x0c, 0x15, 0x39, 0x33, 0x50, 0x36, 0x13, 0x14, 0x29, 0x2d,	// command: PVGAMCTRL
	    //0xd0, 0x08, 0x10, 0x08, 0x06, 0x06, 0x39, 0x44, 0x51, 0x0b, 0x16, 0x14, 0x2f, 0x31,	// command: NVGAMCTRL
        0x00,0x19,0x1e,0x0a,0x09,0x15,0x3d,0x44,0x51,0x12,0x03,0x00,0x3f,0x3f,
        0x00,0x18,0x1e,0x0a,0x09,0x25,0x3f,0x43,0x52,0x33,0x03,0x00,0x3f,0x3f,
        // command: INVON
	    0x00, 0x00, 0x00, 0xEF,	// command: CASET       // 240
	    //0x00, 0x00, 0x00, 0xEF,	// command: RASET       // 240
	    0x00, 0x00, 0x01, 0x3F,         // command: RASET    // 320
	    //command: DISPON
	    //command: RAMWR
};

static int st7789v_reset(void){
    gpio_set_value(RESET_PIN,GPIO_OUT_ONE);
    mdelay(20);
    gpio_set_value(RESET_PIN,GPIO_OUT_ZERO);
    mdelay(20);
    gpio_set_value(RESET_PIN,GPIO_OUT_ONE);
    mdelay(120);
    return 0; 
}
void lcd_chip_select(unsigned char cs){
#if 0
    switch(cs){
    case LCD_L:
        gpio_set_value(CS1_PIN,GPIO_OUT_ZERO);
        gpio_set_value(CS2_PIN,GPIO_OUT_ONE);
        break;
    case LCD_R:
        gpio_set_value(CS1_PIN,GPIO_OUT_ONE);
        gpio_set_value(CS2_PIN,GPIO_OUT_ZERO);
        break;
    case LCDALL: 
        gpio_set_value(CS1_PIN,GPIO_OUT_ZERO);
        gpio_set_value(CS2_PIN,GPIO_OUT_ZERO);
        break;
    case LCD_DISABLE: 
        gpio_set_value(CS1_PIN,GPIO_OUT_ONE);
        gpio_set_value(CS2_PIN,GPIO_OUT_ONE);
    }
#endif
}

#if 0
static int st7789v_spi_transfer(struct spi_device *spi, uint8_t * buff, unsigned int len)
{
	int status;
	struct spi_transfer t = { 
        .tx_buf = buff, 
        .len = len, 
        .speed_hz = 42000000, 
	};
	status = spi_sync_transfer(spi, &t, 1);
    if(status < 0){
        printk("st7789v_spi_transfer failed, status=%d\n",status);
    }else{
        printk("st7789v_spi_transfer success\n");
    }
    return status;
}
#else
static int st7789v_spi_transfer1(struct spi_device *spi, uint8_t * buff, unsigned int len)
{
	int status;
	struct spi_transfer t = { 
        .tx_buf = buff,
        .rx_buf =  buff,
        .len = len, 
        .speed_hz = 38000000, 
	};
	status = spi_sync_transfer(spi, &t, 1);
    if(status < 0){
        printk("st7789v_spi_transfer failed, status=%d\n",status);
    }else{
        ;//printk("st7789v_spi_transfer success\n");
    }
    return status;
}
#define DMA_DATA_LEN 32768
static int st7789v_spi_transfer(struct spi_device *spi, uint8_t * buff, unsigned int len)
{
	int status;
	unsigned int count = len/DMA_DATA_LEN;
    unsigned int sended = 0;
    uint8_t *pTemp = buff;
    while(count--){
        status = st7789v_spi_transfer1(spi, pTemp, DMA_DATA_LEN);
        sended+=DMA_DATA_LEN;
        pTemp = (uint8_t *)(buff+sended);
    }
    if(sended < len){
        status = st7789v_spi_transfer1(spi, pTemp, len-sended);    
    }
    return status;
}
#endif
static ssize_t st7789v_write_cmd(uint8_t cmd){
    gpio_set_value(RS_PIN,GPIO_OUT_ZERO);
    st7789v_spi_transfer(st7789v_device, &cmd, 1);
    gpio_set_value(RS_PIN,GPIO_OUT_ONE);
    return 0;
}

static ssize_t st7789v_write_data(uint8_t data){ 
    gpio_set_value(RS_PIN,GPIO_OUT_ONE);
    st7789v_spi_transfer(st7789v_device, &data, 1);
    return 0;
}

static void st7789v_init_lcd(struct spi_device *spi)
{
    int i, j, n = 0;
    lcd_chip_select(LCDALL);
    st7789v_reset();
    // 寄存器初始化
    for (i = 0; i < ARRAY_SIZE(cmds); i++) {
        // 发命令
        st7789v_write_cmd(cmds[i].reg_addr);
        // 如有数据则发数据
        for (j = 0; j < cmds[i].len; j++) {
            st7789v_write_data(st7789v_init_datas[n++]);
        }
        // 如有延时则延时
        if (cmds[i].delay_ms) {
            mdelay(cmds[i].delay_ms);
        }
    }
}

static int block_write(unsigned int x0, unsigned int y0, unsigned int x1, unsigned int y1){
    st7789v_write_cmd(0x2a);
    st7789v_write_data(x0>>8);
    st7789v_write_data(x0&0xff);
    st7789v_write_data(x1>>8);
    st7789v_write_data(x1&0xff);
    st7789v_write_cmd(0x2b);
    st7789v_write_data(y0>>8);
    st7789v_write_data(y0&0xff);
    st7789v_write_data(y1>>8);
    st7789v_write_data(y1&0xff);
    st7789v_write_cmd(0x2c);    
    return 0;
}

static void st7789v_gpio_Init(void){
    int ret = gpio_request(LCD_PWM, "disable");
    if(ret != 0){
        printk("gpio %d request failed \n",LCD_PWM);
    }else{
        gpio_direction_output(LCD_PWM,GPIOF_OUT_INIT_LOW);
        gpio_set_value(LCD_PWM,GPIO_OUT_ZERO);
        //gpio_free(LCD_PWM);
    }

    ret = gpio_request(CS1_PIN, "lcd1cs");
    if(ret != 0){
        printk("gpio %d request failed \n",CS1_PIN);
    }else{
        gpio_direction_output(CS1_PIN,GPIOF_OUT_INIT_LOW);
        gpio_set_value(CS1_PIN,GPIO_OUT_ZERO);
        //gpio_free(CS1_PIN);
    }

    ret = gpio_request(RS_PIN, "rs");
    if(ret != 0){
        printk("gpio %d request failed \n",RS_PIN);
    }else{
        gpio_direction_output(RS_PIN,GPIOF_OUT_INIT_LOW);
        gpio_set_value(RS_PIN,1);
        //gpio_free(RS_PIN);
    }
    ret = gpio_request(CS2_PIN, "lcd2cs");
    if(ret != 0){
        printk("gpio %d request failed \n",CS2_PIN);
    }else{
        gpio_direction_output(CS2_PIN,GPIOF_OUT_INIT_LOW);
        gpio_set_value(CS2_PIN,GPIO_OUT_ZERO);
        //gpio_free(CS2_PIN);
    }
    ret = gpio_request(RESET_PIN, "rst");
    if(ret != 0){
        printk("gpio %d request failed \n",RESET_PIN);
    }else{
        gpio_direction_output(RESET_PIN,GPIOF_OUT_INIT_LOW);
        gpio_set_value(RESET_PIN,GPIO_OUT_ZERO);
        //gpio_free(RESET_PIN);
    }
    mdelay(1); 
}


#define MAX_BACKLIGHT_BRIGHTLESS_LEVEL 16
static int backlight_level = MAX_BACKLIGHT_BRIGHTLESS_LEVEL;

static void st7789v_backlight_update(unsigned char level){
	int i  = 0;
	int signal_num = 0;
	if (level > MAX_BACKLIGHT_BRIGHTLESS_LEVEL){
		signal_num = backlight_level - level + MAX_BACKLIGHT_BRIGHTLESS_LEVEL;
	}else{	
		signal_num = backlight_level - level;
	}
	backlight_level = level;
	for(i = 0; i < signal_num; i++){
		gpio_set_value(LCD_PWM,GPIO_OUT_ZERO);
		udelay(2);
		gpio_set_value(LCD_PWM,GPIO_OUT_ONE);
	}
}
static ssize_t st7789v_backlight_enable(int enable){	
	if(enable == 1){
		gpio_set_value(LCD_PWM,GPIO_OUT_ONE);
	    mdelay(1);  // en
	    st7789v_backlight_update(backlight_level);	
	}
	if(enable == 0){
		gpio_set_value(LCD_PWM,GPIO_OUT_ZERO);
	    mdelay(3);  // disable
	}
   	return 0;
}

static void init_logo(void){
	unsigned int bootmode = 0,i = 0;
	//unsigned char * logo = boot_logo;
	for(i = 0; i < LCD_W*LCD_H*2; i++){
        //ker_buf[i] = logo[i];
        ker_buf[i] = 0xff;
    }
    ker_buf[LCD_W*LCD_H*2] = LCDALL;
} 

void st7789v_resume(void){
    st7789v_init_lcd(st7789v_device); 
    switch(ker_buf[LCD_W*LCD_H*2]){
    case LCD_L :
        YJX_LOG("lcd cs lcd_l \n");
        lcd_chip_select(LCD_L);
        gpio_set_value(RS_PIN,GPIO_OUT_ONE);
        st7789v_spi_transfer(st7789v_device,ker_buf,PAKET_SIZE);
        break;
    case LCD_R :
        YJX_LOG("lcd cs lcd_r \n");
        lcd_chip_select(LCD_R);
        gpio_set_value(RS_PIN,GPIO_OUT_ONE);
        st7789v_spi_transfer(st7789v_device,ker_buf,PAKET_SIZE);
        break;
    case LCDALL :
        YJX_LOG("lcd cs lcd_a \n");
        lcd_chip_select(LCDALL);
        gpio_set_value(RS_PIN,GPIO_OUT_ONE);
        st7789v_spi_transfer(st7789v_device,ker_buf,PAKET_SIZE);
        break;
    default :
        YJX_LOG("yjx  cmd error \n");
    }
    lcd_chip_select(LCD_DISABLE);
}
void st7789v_sleep(void){
	lcd_chip_select(LCDALL);
	st7789v_write_cmd(0x28);
	mdelay(50);
	st7789v_write_cmd(0x10);
	mdelay(120);
	lcd_chip_select(LCD_DISABLE);
}

// static long int st7789v_restore(struct file *file, unsigned int chan_id, unsigned long pos)
// {
//     return pos;
// }

static ssize_t st7789v_write (struct file *file, const char __user *buf, size_t count, loff_t *ppos){
    int ret = 0;
    int mode = 0;
    mutex_lock(&my_lock);
    // YJX_LOG("write data size = %d\n", (int)count);
    
    if(count == LCD_W*LCD_H*2+1){
        ret = copy_from_user(ker_buf,buf,count);
        if(ret)
            YJX_LOG("copy_from_user error ret = %d \n", ret );
        else{
            mode = ker_buf[LCD_W*LCD_H*2];
            // YJX_LOG("mode = %d, buf data %x %x %x %x %x %x \n", mode, ker_buf[0],ker_buf[1],  ker_buf[LCD_W*LCD_H],ker_buf[LCD_W*LCD_H+1], ker_buf[LCD_W*LCD_H*2-2], ker_buf[LCD_W*LCD_H*2-1]            );
        }
    }else if(count == 2){
        ret = copy_from_user(resumebuf,buf,count);
        if(resumebuf[0] == '0'){
            st7789v_resume();
            printk("lcd resume \n");
        }else if(resumebuf[0] == '1'){
            st7789v_sleep();
            printk("lcd sleep \n");
        }
        mutex_unlock(&my_lock);
        return count;
    }else{
        printk("write data count error, count = %d \n", (int)count);
        mutex_unlock(&my_lock);
        return count;
    }     
    lcd_chip_select(LCDALL);
    block_write(0, 0, LCD_W - 1, LCD_H - 1);
    //block_write(0, 80, 239, 319);
    lcd_chip_select(LCD_DISABLE);
    switch(mode){
    case LCD_L :
        // YJX_LOG("lcd cs lcd_l \n");
        lcd_chip_select(LCD_L);
        gpio_set_value(RS_PIN,GPIO_OUT_ONE);
        ret = st7789v_spi_transfer(st7789v_device,ker_buf,PAKET_SIZE);
        break;
    case LCD_R :
        // YJX_LOG("lcd cs lcd_r \n");
        lcd_chip_select(LCD_R);
        gpio_set_value(RS_PIN,GPIO_OUT_ONE);
        ret = st7789v_spi_transfer(st7789v_device,ker_buf,PAKET_SIZE);
        break;
    case LCDALL :
        // YJX_LOG("lcd cs lcd_a \n");
        lcd_chip_select(LCDALL);
        gpio_set_value(RS_PIN,GPIO_OUT_ONE);
        ret = st7789v_spi_transfer(st7789v_device,ker_buf,PAKET_SIZE);
        break;
    case LCDINIT : 
        st7789v_init_lcd(st7789v_device);
        break; 
    default :
        YJX_LOG("cmd error \n");
    }
    lcd_chip_select(LCD_DISABLE);
    // YJX_LOG("write data end \n");
    mutex_unlock(&my_lock);
    if(ret < 0){
        //printk("write data error, ret = %d \n", (int)ret);    
    }
    return count ;
}

static struct file_operations st7789v_ops = {
    .owner          =  THIS_MODULE,
    .write          =  st7789v_write,
    // .compat_ioctl   =  st7789v_restore,
};

static int st7789v_probe(struct spi_device *spi){
	unsigned int i = 0;
	YJX_LOG("st7789v_probe enter \n");
    st7789v_device = spi;
	ker_buf = kmalloc( LCD_W*LCD_H*2 + 1, GFP_KERNEL);
	if(ker_buf == NULL){
		printk("st7789v_probe error,no mem!\n");
		return -ENOMEM;
	}

    init_logo();
	st7789v_gpio_Init();
    st7789v_init_lcd(st7789v_device);
    
	YJX_LOG("st7789v_probe write buffer in \n");
    gpio_set_value(RS_PIN,GPIO_OUT_ONE);
    //st7789v_spi_transfer(st7789v_device,ker_buf,PAKET_SIZE);
    st7789v_spi_transfer(st7789v_device,initlogo,PAKET_SIZE);
	YJX_LOG("st7789v_probe write buffer out \n");
	st7789v_backlight_enable(1);

    lcd_chip_select(LCDALL);
    block_write(0, 0, LCD_W - 1, LCD_H - 1);
    lcd_chip_select(LCD_DISABLE);
	
    major = register_chrdev(0, "st7789v", &st7789v_ops);
    
    class = class_create(THIS_MODULE, "st7789v");
    device_create(class, NULL, MKDEV(major, 0), NULL, "st7789v");
    YJX_LOG("st7789v_probe enter end \n");
    return 0;
}

static int st7789v_remove(struct spi_device *spi){
    device_destroy(class, MKDEV(major, 0));
    class_destroy(class);
    unregister_chrdev(major, "st7789v");
    
    if(ker_buf)
        kfree(ker_buf);
    YJX_LOG("yjx st7789v_remove enter end\n");

    return 0;
}

static const struct of_device_id st7789v_dt_ids[] = {
	{.compatible = "sitronix,st7789v"}, 
	{},
};

static struct spi_driver st7789v_spi_driver = {
    .driver = {
        .name  = "st7789v",
        .owner = THIS_MODULE,
        .of_match_table = of_match_ptr(st7789v_dt_ids),
    },
    .probe  = st7789v_probe,
    .remove = st7789v_remove,
};

static int __init st7789v_init(void){
    return spi_register_driver(&st7789v_spi_driver);
}
module_init(st7789v_init);

static void __exit st7789v_exit(void){
    spi_unregister_driver(&st7789v_spi_driver);
}
module_exit(st7789v_exit);


MODULE_AUTHOR("yujixuan, <yujixuan@wind-mobi.com>");
MODULE_DESCRIPTION("User mode SPI device interface");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:st7789v");
