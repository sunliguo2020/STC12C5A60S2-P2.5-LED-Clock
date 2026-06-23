#define		ulong	unsigned long
#define		uint	unsigned int
#define		uchar	unsigned char

//	DS3231初始数据     秒    分    时   日     月   星期   年
uchar code settime[] = {0x20, 0x30, 0x09, 0x01, 0x07, 0x05, 0x15};

//EEROM 初始数据         模式 AM/PM 亮度 最小亮 绿补 NTC 遥控
uchar code setdata[] = {	0,    0,   10,   1,     1,  10,  1  };

// IO口模拟I2C通信

// ------------------------------------------------------------
//sbit SCL=P2^5; //串行时钟
//sbit SDA=P2^6; //串行数据
sbit SCL=P3^6; //串行时钟
sbit SDA=P3^7; //串行数据

//EEprom读写地址
//#define AT24C32_ADD_WR  0xa0	   //模块中A0,A1,A2短接时的地址。10100000，每个模块可能不同。
//#define AT24C32_ADD_RD  0xa1
#define AT24C32_ADD_WR  0xae		//模块中A0,A1,A2悬空时的地址。10101110
#define AT24C32_ADD_RD  0xaf

/********************************************************************************************************
** 	DS3231常数定义
********************************************************************************************************/
#define DS3231_WriteAddress 0xD0    //器件写地址 
#define DS3231_ReadAddress  0xD1    //器件读地址
//#define DS3231_WriteAddress 0x67    //器件写地址
//#define DS3231_ReadAddress  0x68    //器件读地址
#define DS3231_SECOND       0x00    //秒
#define DS3231_MINUTE       0x01    //分
#define DS3231_HOUR         0x02    //时
#define DS3231_WEEK         0x03    //星期
#define DS3231_DAY          0x04    //日
#define DS3231_MONTH        0x05    //月
#define DS3231_YEAR         0x06    //年
#define DS3231_A2D        0x0d    //闹铃2天

#define NACK    1
#define ACK     0

//char hour,minute,second,year,month,day,week;
static uint TH3231;
bit	ack;							//应答标志位
extern void 	Delay (uchar t);
void	Delay5US() 	  //@24.000MHz	   延时5us
{
    _nop_();
    _nop_();
    _nop_();
    _nop_();
    _nop_();
    _nop_();
    _nop_();
    _nop_();
}

/**********************************************
//IIC Start
**********************************************/
void IIC_Start()
{
    SCL = 1;
    SDA = 1;
    SDA = 0;
    SCL = 0;
}

/**********************************************
//IIC Stop
**********************************************/
void IIC_Stop()
{
    SCL = 0;
    SDA = 0;
    SCL = 1;
    SDA = 1;
}


/********************************************************************************************************
** 	3231
********************************************************************************************************/

/************BCD转十 十转BCD****************
uchar	BCD2HEX(uchar val)
{
return	((val>>4)*10)+(val&0x0f);
}

uchar	HEX2BCD(uchar val)
{
return	(((val%100)/10)<<4)|(val%10);
}

***************************************************/
void SendByte(uchar c)
{
    uchar BitCnt;

    for(BitCnt=0; BitCnt<8; BitCnt++)       //要传送的数据长度为8位
    {
        if((c<<BitCnt)&0x80)
            SDA=1;                          //判断发送位
        else
            SDA=0;
        SCL=1;                            //置时钟线为高，通知被控器开始接收数据位
        Delay5US();                       //保证时钟高电平周期大于4μs
        SCL=0;
    }
    SDA=1;                                  //8位发送完后释放数据线，准备接收应答位
    SCL=1;
    Delay5US();
    if(SDA==1)
        ack=0;
    else
        ack=1;                              //判断是否接收到应答信号
    SCL=0;
    Delay5US();
}

uchar RcvByte()
{
    uchar retc;
    uchar BitCnt;

    retc=0;
    SDA=1;                           //置数据线为输入方式
    for(BitCnt=0; BitCnt<8; BitCnt++)
    {
        SCL=0;                      //置时钟线为低，准备接收数据位
        Delay5US();                 //时钟低电平周期大于4.7μs
        SCL=1;                      //置时钟线为高使数据线上数据有效
        Delay5US();
        retc=retc<<1;
        if(SDA==1)
            retc=retc+1;            //读数据位,接收的数据位放入retc中
        Delay5US();
    }
    SCL=0;
    return(retc);
}

void Ack_I2C(bit a)
{
    SDA	=	a;
    SCL=1;
    Delay5US();             //时钟低电平周期大于4us
    SCL=0;                  //清时钟线，钳住I2C总线以便继续接收
    Delay5US();
}

uchar DS3231_write_byte(uchar addr, uchar write_data)
{
    IIC_Start();
    SendByte(DS3231_WriteAddress);
    if (ack == 0)
        return 0;

    SendByte(addr);
    if (ack == 0)
        return 0;

    SendByte(write_data);
    if (ack == 0)
        return 0;

    IIC_Stop();
    Delay5US();
    Delay5US();
    return 1;
}

uchar read_current()
{
    uchar read_data;
    IIC_Start();
    SendByte(DS3231_ReadAddress);
    if(ack==0)
        return(0);
    read_data = RcvByte();
    Ack_I2C(1);
    IIC_Stop();
    return read_data;
}

uchar read_random(uchar random_addr)
{
    uchar Tmp;
    IIC_Start();
    SendByte(DS3231_WriteAddress);
    if(ack==0)
        return(0);
    SendByte(random_addr);
    if(ack==0)
        return(0);
    Tmp=read_current();
    /*if(random_addr==DS3231_HOUR)
    	Tmp&=0x3f;*/

    return Tmp;//BCD码输出
    //return BCD2HEX(Tmp);//都转10进制输出
}

void ModifyTime(uchar address,uchar num)
{
    uchar temp=0;
    if(address>6 && address <0) return;
    //temp=HEX2BCD(num);
    temp= num ;
    DS3231_write_byte(address,temp);
}

/***********读DS3231温度值函数*****************误差太大不用，用NTC测温*************
uint    read_temp()
{
		int     itemp;
		float   ftemp;
		//温度数据是以2 进制格式存储的并不需要数制转换
		DS3231_write_byte(0x0e,0x20);//0x0e寄存器的CONV位置1开启温度转换

        itemp = ( (int) read_random(0x11) << 5 );	  //放大32倍
        itemp += ( read_random(0x12)>> 3);
        IIC_Stop();
        if(itemp & 0x1000)
			itemp += 0xe000;

        ftemp = 0.3125 * (float) itemp+0.5;    //放大10倍
		return  (uint) ftemp;
}															 */

/***********************************************
函数名称：EEprom_WritePara
功    能：EEprom写入参数函数
入口参数：dat:欲写入的数据
返 回 值：无
备    注：无
************************************************/
//void EEprom_WritePara(unsigned int addr, uchar write_data)
//{

//    IIC_Start();            			//起始信号
//    SendByte(AT24C32_ADD_WR);     	//发送设备地址+写信号
//    SendByte(addr/256);     		//发送存储单元地址
//    SendByte(addr%256);
//    SendByte(write_data);					//写入数据

//    IIC_Stop();             			//停止信号
//}
/***********************************************
函数名称：EEprom_ReadPara
功    能：EEprom读数据函数
入口参数：无
返 回 值：unsigned char:读出的数据
备    注：无
************************************************/
//unsigned char EEprom_ReadPara(unsigned int random_addr)
//{
//    unsigned char dat;

//    IIC_Start();            			//起始信号
//    SendByte(AT24C32_ADD_WR);     	//发送设备地址+写信号
//    SendByte(random_addr/256);     		//发送存储单元地址
//    SendByte(random_addr%256);
//    IIC_Start();            			//起始信号
//    SendByte(AT24C32_ADD_RD);     	//发送设备地址+读信号

//    dat = RcvByte();				//读取数据
//    //SendACK(NACK); 				//最后一个数据需要NAK
//    Ack_I2C(1);
//    IIC_Stop();             			//停止信号
//    return dat;
//}

/**************************************
初始化DS3231并设初始时间
**************************************/
void DS3231_Initial()
{   Delay (200);
    if(read_random(DS3231_A2D) != 1)   //如DS3231闹铃2日标志值不是原来的1,测初始时钟DS3231
    {
        DS3231_write_byte(DS3231_A2D,1);//写入初始化标志RAM（第00个RAM位置）
        ModifyTime(DS3231_SECOND,0x00);	   //秒
        ModifyTime(DS3231_MINUTE,0x59);	   //3分
        ModifyTime(DS3231_HOUR,0x16);	   //时
        ModifyTime(DS3231_DAY,0x25);	   //1日
        ModifyTime(DS3231_MONTH,0x12);	   //7月
        ModifyTime(DS3231_YEAR,0x21);	   //15年

    }

}


/*******读取全部时间*********/
void GetAllTime()
{
    uchar temp;

    temp=read_random(DS3231_SECOND);//DS1302_ReadData(0x81);	   //秒
    miao1=temp>>4;
    miao2=temp&0x0f;

    temp=read_random(DS3231_MINUTE);//DS1302_ReadData(0x83);	   //分
    fen1=temp>>4;
    fen2=temp&0x0f;

    temp=read_random(DS3231_HOUR);//DS1302_ReadData(0x85);	   //时
    if(Zhi_12)
    {
        temp=temp/16*10+temp%16;
        if(temp>12)
        {
            PM=1;
            temp-=12;
        }
        else
            PM=0;
        shi1=temp/10;
        shi2=temp%10;
//		if(shi1)
//			shi1=10;
    }
    else
    {
        PM=0;
        shi1=temp>>4;
        shi2=temp&0x0f;
    }

    temp=read_random(DS3231_DAY);//DS1302_ReadData(0x87);	   //日
    ri1=temp>>4;
    ri2=temp&0x0f;

    temp=read_random(DS3231_MONTH);//DS1302_ReadData(0x89);	   //月
    yue1=temp>>4;
    yue2=temp&0x0f;

    temp=read_random(DS3231_YEAR);//DS1302_ReadData(0x8d);	   //年
    nian1=temp>>4;
    nian2=temp&0x0f;

    GetNL_ri();			 //农历转换
}
//************************读取一次时间**********************
void GetTime()
{
    uchar temp;

    temp=read_random(DS3231_SECOND);//DS1302_ReadData(0x81);	  	//读取秒
    miao1=temp>>4;
    miao2=temp&0x0f;

    if(temp==0)						   //00秒读取分
    {
        temp=read_random(DS3231_MINUTE);//DS1302_ReadData(0x83);
        fen1=temp>>4;
        fen2=temp&0x0f;

        if(temp==0)						  //0分读取时
        {
            temp=read_random(DS3231_HOUR);//DS1302_ReadData(0x85);
            if(Zhi_12)
            {
                temp=temp/16*10+temp%16;
                if(temp>12)
                {
                    PM=1;
                    temp-=12;
                }
                else
                    PM=0;
                shi1=temp/10;
                shi2=temp%10;
//				if(shi1)
//					shi1=10;
            }
            else
            {
                shi1=temp>>4;
                shi2=temp&0x0f;
            }
            if(temp==0)					   //0时读取日期
            {
                temp=read_random(DS3231_DAY);//DS1302_ReadData(0x87);
                ri1=temp>>4;
                ri2=temp&0x0f;

                temp=read_random(DS3231_MONTH);//DS1302_ReadData(0x89);
                yue1=temp>>4;
                yue2=temp&0x0f;

                temp=read_random(DS3231_YEAR);//DS1302_ReadData(0x8d);
                nian1=temp>>4;
                nian2=temp&0x0f;

                GetNL_ri();			 //农历转换
                GPS_POW=0;			//打开GPS电源
            }
        }
    }
}







