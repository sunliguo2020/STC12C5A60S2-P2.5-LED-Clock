//*********** 18B20驱动 **************************

sbit DQ =P1^7;  // -- 定义18b20通信端口

uchar idata T_int,T_dec;		//计算温度用
bit fu;
//延时
uchar code t_tab[16]= {0,0,1,1,2,3,3,4,5,5,6,6,7,8,8,9};	//小数第一位查表

void delay18b20(unsigned int x)
{
    unsigned char i;
    while(x--)
    {
        i = 20;
        while (--i);
    }
}

//复位
unsigned char DS18B20_TEST(void)
{
    unsigned char presence;
    DQ = 0;        //拉低总线
    delay18b20(70);    // 保持 480us
    DQ = 1;     // 释放总线

    delay18b20(6);     // 等待回复 15-60us

    presence = DQ; // 读取信号
    delay18b20(18);    // 等待结束信号	  60-240us
    return(presence); // 返回   0：正常 1：不存在
}
void DS18B20_RST(void)
{
//  unsigned char presence;
    DQ = 0;        //拉低总线
    delay18b20(45);    // 保持 480us
    DQ = 1;     // 释放总线
//
    delay18b20(18);     // 等待回复 15-60us
//
//  presence = DQ; // 读取信号
//  delay18b20(18);    // 等待结束信号	  60-240us
//  return(presence); // 返回   0：正常 1：不存在
}

//从 1-wire 总线上读取一个字节
unsigned char  read_byte(void)
{
    unsigned char i;
    unsigned char  value = 0;
    for (i=8; i>0; i--)
    {

        DQ = 0;
        value>>=1;
        DQ = 1;
        NOP4;
//	delay18b20(1);
        if(DQ)
            value|=0x80;
        delay18b20(4); 		  //60-120us
    }
    return(value);
}


//向 1-WIRE 总线上写一个字节
void write_byte(char val)
{
    unsigned char  i;
    for (i=8; i>0; i--) // 一次写一位
    {
        DQ = 0; //
        DQ = val&0x01;
        delay18b20(4); 		//15-60us
        DQ = 1;
        val>>=1;
    }
//  delay18b20(5);
}



////请求温度转换
//void Request_Temperature(void)
//{
//  DS18B20_RST();
//  write_byte(0xCC); // Skip ROM
//  write_byte(0x44); // 转换温度
//}



void Read18B20(void)//读取温度
{
    unsigned char data_H,data_L;


    DS18B20_RST();
    write_byte(0xCC); //Skip ROM
    write_byte(0x44); // 启动温度转换
    DS18B20_RST();
    write_byte(0xCC); //Skip ROM
    write_byte(0xbe); // 读取寄存器



    data_L=read_byte();
    data_H=read_byte();
    /*
        if(data_H>15) data_H=(~data_H+1),data_L=(~data_L+1); //低于零度为补码，取反加1
        T_int=(data_H<<4)|(data_L>>4);  //合并后得到原始温度数据 整数部分
        T_dec=t_tab[(data_L&0x0F)];
    */

    if(data_H > 0x7F)   //负温度
    {   fu=1;
        data_H=(~data_H+1);
        data_L=(~data_L+1);
//	  T_int=data_L/16+data_H*16-16;
        T_int=((data_H<<4)|(data_L>>4))-16;

    }
    else
    {   fu=0;
//	   T_int=data_L/16+data_H*16;
        T_int=((data_H<<4)|(data_L>>4));

        T_dec=t_tab[(data_L&0x0F)];
    }
    t1=T_int/10;
    t2=T_int%10;
}
