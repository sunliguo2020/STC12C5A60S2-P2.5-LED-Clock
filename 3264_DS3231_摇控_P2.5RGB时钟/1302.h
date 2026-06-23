

//#define CHONG 0x00	// ab=2D+8K
//#define DS1302TEST 0x5c

//	DS3231初始数据     秒    分    时   日     月   星期   年
//uchar code settime[] ={0x20, 0x30, 0x09, 0x01, 0x07, 0x05, 0x15};

//EEROM 初始数据         模式 AM/PM 亮度 最小亮 绿补 NTC 遥控
//uchar code setdata[] ={	0,    0,   10,   1,     1,  10,  1  };

sbit SCLK= P1^0;                   //DS1302时钟口
sbit IO  = P1^1;                     //DS1302数据口
sbit RST = P1^2;                    //DS1302片选口

//*****从DS1302读1字节数据***********/
uchar DS1302_Readuchar()
{
    uchar i;
    uchar dat = 0;

    for (i=0; i<8; i++)             //8位计数器
    {
        SCLK = 0;                   //时钟线拉低
        NOP4;                //延时等待
        dat >>= 1;	                //数据右移一位
        if (IO) dat |= 0x80;        //读取数据
        SCLK = 1;                   //时钟线拉高
        NOP4;                //延时等待
    }

    return dat;
}

//*******向DS1302写1字节数据**********/
void DS1302_Writeuchar(uchar dat)
{
    uchar i;

    for (i=0; i<8; i++)             //8位计数器
    {
        SCLK = 0;                   //时钟线拉低
        NOP4;                //延时等待
        dat >>= 1;                  //移出数据
        IO = CY;                    //送出到端口
        SCLK = 1;                   //时钟线拉高
        NOP4;                //延时等待
    }
}

//******读DS1302某地址的的数据**********/
uchar DS1302_ReadData(uchar addr)
{
    uchar dat;

    RST = 0;
    NOP4;
    SCLK = 0;
    NOP4;
    RST = 1;
    NOP4;
    DS1302_Writeuchar(addr);         //写地址
    dat = DS1302_Readuchar();        //读数据
    SCLK = 1;
    RST = 0;

    return dat;
}

//*******往DS1302的某个地址写入数据*********/
void DS1302_WriteData(uchar addr, uchar dat)
{
    RST = 0;
    NOP4;
    SCLK = 0;
    NOP4;
    RST = 1;
    NOP4;
    DS1302_Writeuchar(addr);         //写地址
    DS1302_Writeuchar(dat);          //写数据
    SCLK = 1;
    RST = 0;
}

DS1302_WriteBtye(uchar add, uchar dat)
{
    DS1302_WriteData(0x8e,0x00);   //允许写操作
    DS1302_WriteData(add, dat);   //写入DS1302
    DS1302_WriteData(0x8e,0x80);   //写保护，禁止写操作
}


/*******写入初始数据********/
void DS1302_SetData(uchar *p)
{
    uchar addr = 0xc0;
    uchar n = 7;

//   DS1302_WriteData(0x8e, 0x00);   //允许写操作
    while (n--)
    {
        DS1302_WriteData(addr, *p++);
        addr += 2;
    }
//    DS1302_WriteData(0x8e, 0x80);   //写保护
}



/**************************************
初始化DS1302并设初始时间
*************************************
void DS1302_Initial()
{

	DS1302_WriteData(0x8e, 0x00);   //允许写操作
	if(DS1302_ReadData(0xf1) != DS1302TEST)   //如RAM 标志不是原来的,测初始时钟
    {
	    RST = 0;
	    SCLK = 0;
		DS1302_WriteData(0x8e, 0x00);   //允许写操作
	    DS1302_WriteData(0x80, 0x00);   //时钟启动
	    DS1302_WriteData(0xf0,DS1302TEST);//写入初始化标志RAM（第00个RAM位置）
//		DS1302_SetTime(settime);           //设置初始时间
		DS1302_SetData(setdata);
//	    DS1302_WriteData(0x8e, 0x80);   //写保护

    }

	DS1302_WriteData(0x90, CHONG);   //充电


	if(DS1302_ReadData(0x81) & 0x80)
		DS1302_WriteData(0x80, 0x00);   //

	DS1302_WriteData(0x8e, 0x80);   //写保护
}
		   */


