//原程序为Benli所创，只是把DS1302计时芯片换成DS3231+24C32模块。
// 本时钟使用STC12C5A60S2单片机,24/24.576M晶振,DS3231高精度RTC芯片
// 多模式显示,有三大模式,每个模式分别有时间粗体/细体,有秒/无秒,12时/24时8种模式,共计24模式
// GPS时间校正,每天定时开启GPS校正本地时间
// DS18B20/NTC温度传感器自适应,优先使用DS18B20
// 自动亮度调节,根据环境光线自动调节屏幕亮度,白天清晰，晚上柔和而且省电
// 学习型红外遥控
// 停电后由电池供电并进入省电模式

// 按键说明:
// 在非设置模式下,按+键切换显示模式,-键切换12/24时制
// 按SET分别设置  正常显示/年/月/日/时/分/秒/亮度/最低亮度/绿补偿/NTC补偿/遥控开关/学习模式
// 按+/-调整数据,
// ESC退出设置
// 在学习模式下按遥控学习按键

//#include <STC15.H>
#include <STC12C5A60S2.H>
#include <intrins.h>

#define		ulong	unsigned long
#define		uint	unsigned int
#define		uchar	unsigned char
#include "zimo2.h"
#define NOP _nop_()
#define NOP4 _nop_();_nop_();_nop_();_nop_()


#define XSMOD_Max 11	//显示模式最大数
#define Set_Max 15	   //设置项目数15为有遥控
//#define Set_Max 10	   //设置项目数10为无遥控
#define Liangdu_Max 19
#define Min_Liangdu_Max 9
#define Liangdu_G_Max 1   //颜色转换

#define ES_ON AUXR |= 0x10; ES=1
#define ES_OFF AUXR &= 0xEF; ES=0

//#define EN_OFF() OE=0		//使能低电平有效,高电平有效取反
//#define EN_ON() OE=1
#define EN_OFF() OE=1		//使能低电平有效,高电平有效取反
#define EN_ON() OE=0
#define DATAHI			//数据高电平有效保留，低电平有效需去掉


uchar code banben[4] = "2.4";
uchar idata GPRMC[] = "$GPRMC";  //GPS选择接收字符$GPRMC,
//uchar idata GPRMC[6]={0x24,0x47,0x50,0x52,0x4D,0x43};    //GPS选择接收字符$GPRMC,


sbit G1 = P0 ^ 5;	   	//红绿数据
sbit G2 = P0 ^ 4;
sbit R1 = P0 ^ 6;
sbit R2 = P4 ^ 4;
sbit B1 = P0 ^ 7;
sbit B2 = P2 ^ 7;
sbit LS = P2 ^ 4;   		//锁存
sbit CK = P2 ^ 6;  		 //时钟
sbit OE = P2 ^ 5;		 //使能，低电平有效


sbit IA  = P0 ^ 0; //行控制线A
sbit IB  = P0 ^ 1; //行控制线B
sbit IC  = P0 ^ 2; //行控制线C
sbit ID  = P0 ^ 3; //行控制线D

#define  scan0    {IA=0;IB=0;IC=0;ID=0;}
#define  scan1    {IA=1;IB=0;IC=0;ID=0;}
#define  scan2    {IA=0;IB=1;IC=0;ID=0;}
#define  scan3    {IA=1;IB=1;IC=0;ID=0;}
#define  scan4    {IA=0;IB=0;IC=1;ID=0;}
#define  scan5    {IA=1;IB=0;IC=1;ID=0;}
#define  scan6    {IA=0;IB=1;IC=1;ID=0;}
#define  scan7    {IA=1;IB=1;IC=1;ID=0;}
#define  scan8    {IA=0;IB=0;IC=0;ID=1;}
#define  scan9    {IA=1;IB=0;IC=0;ID=1;}
#define scan10    {IA=0;IB=1;IC=0;ID=1;}
#define scan11    {IA=1;IB=1;IC=0;ID=1;}
#define scan12    {IA=0;IB=0;IC=1;ID=1;}
#define scan13    {IA=1;IB=0;IC=1;ID=1;}
#define scan14    {IA=0;IB=1;IC=1;ID=1;}
#define scan15    {IA=1;IB=1;IC=1;ID=1;}
//
void scan(unsigned char Value)
{
    switch (Value)
    {
        case  0:
            scan0;
            break;

        case  1:
            scan1;
            break;

        case  2:
            scan2;
            break;

        case  3:
            scan3;
            break;

        case  4:
            scan4;
            break;

        case  5:
            scan5;
            break;

        case  6:
            scan6;
            break;

        case  7:
            scan7;
            break;

        case  8:
            scan8;
            break;

        case  9:
            scan9;
            break;

        case 10:
            scan10;
            break;

        case 11:
            scan11;
            break;

        case 12:
            scan12;
            break;

        case 13:
            scan13;
            break;

        case 14:
            scan14;
            break;

        case 15:
            scan15;
            break;

        default:
            break;
    }
}

sbit GPS_POW = P4 ^ 1;		//gps电源控制
sbit POW_OK = P4 ^ 6;		//外部电源检测
sbit IR_POW = P3 ^ 4;		//红外接收电源


sbit KeySet = P2 ^ 0;	 //调整按键
sbit KeyAdd = P2 ^ 1;	 //加按键
sbit KeyDec = P2 ^ 2;	 //减按键
sbit KeyEsc = P2 ^ 3;		 //退出键

bit K_ADD, K_DEC, K_SET, K_ESC;		//按键标志
bit WriteEpprom_S = 0;   //写EEROM状态位
uchar data row;					//行扫描变量
uchar data RD1[8];				//数据缓存
uchar data GD1[8];
uchar data BD1[8];
uchar data RD2[8];
uchar data GD2[8];
uchar data BD2[8];

uchar miao1, miao2;	  //秒数据，1十位，2个位
uchar fen1, fen2	;	  //分数据
uchar shi1, shi2	;	  //小时数据
uchar ri1, ri2	;	  //日数据
uchar yue1, yue2;	  //月数据
uchar nian1, nian2;	  //年数据
uchar week;	         //星期
bit SHAN, SHAN2;		//闪烁标志
uchar idata Set_S = 0;  //调节状态位
uchar idata Set_COLOR = 0;   //颜色调节指量位。
uchar idata COLORMODE = 0; //各区域颜色值。
uchar idata NL_yue;		//农历月 1~12
uchar idata NL_ri;		//农历日 1~30
uchar y1, y2, r1, r2;		//农历显示数据

uchar  SJ[6], RQ[6];

bit RX_over;   //GPS数据接收结束标志位
bit DW_OK;	   //GPS定位成功标志位

uint ADC_L;		  //ADC亮度数据
uint ADC_T;		  //ADC温度数据
bit ADCL;		  //亮度ADC标志
bit ADCT;		  //温度ADC标志
uchar t1, t2;	 //温度显示数据
bit NTC;		//是否使用NTC测温标志
bit Zhi_12, PM;	//12/24时制标示,0/24	1/12
uchar idata XSMOD;	//显示模式
char idata Liangdu;	//亮度
char idata Min_Liangdu;		//最低亮度
char idata Liangdu_G;		//颜色转换
char idata Colortime;		//颜色转换临时变量

char idata Light_R, Light_L, Light_G;	//亮度控制用变量
uchar idata Li;
uchar idata NTC_buchang;		//NTC补偿

uchar idata IR_date[7] = {0, 0, 0, 0, 0, 0, 0}; //date数组为存放地址原码，反码，数据原码，反码,标志位，
//连发标志，临时数据（执行数据
uchar idata IR_Key[5];	//红外键值,用户码,SET,+,-.ESC
bit xuexi_OK ;		//红外学习成功标示

#include "nongli.h"
#include "DS3231+AT24C32.h"
#include "18b20.h"
#include "set.h"
#include "eep.h"
void Delayus(uint x);

void Delay (uchar t)
{
    uchar i, j;

    for (i = 0; i < t; i++)

        for(j = 0; j < 10; j++);
}

void Delayus(uint x)
{
    while(x--);
}


void ShowLogo()
{
    uchar i, j;
    uint x = 400;
    //uchar code *p;

    while(x--)
    {
        WDT_CONTR = 0x34;

        for(row = 0; row < 16; row++)			 //扫描16行
        {
//            RD1[0]=ZMlogo[0][row];	//GPSclock
//            RD1[1]=GD1[1]=ZMlogo[1][row];
//            GD1[2]=ZMlogo[2][row];
//            RD1[3]=ZMlogo[3][row];
//            RD1[4]=GD1[4]=ZMlogo[4][row];
//            GD1[5]=ZMlogo[5][row];
//            RD1[6]=ZMlogo[6][row];
//            RD1[7]=GD1[7]=ZMlogo[7][row];
//
//            RD2[0]=ZMlogo[8][row]; 	//V
//            p=&banben;
//            RD2[1]=Num12[*p][row];		//2
//            RD2[2]=ZMlogo[9][row];	//.
//            p+=2;
//            RD2[3]=Num12[*p][row];
//            if(NTC)
//            {
//                GD2[5]=ZMlogo[11][row];	//N
//                GD2[6]=ZMlogo[12][row];	//T
//                GD2[7]=ZMlogo[13][row];	//C
//            }
//            else
//            {
//                GD2[5]=ZMlogo[10][row];	//D
//                GD2[6]=ZMlogo[2][row];	//S
//            }
//
            GD1[0] = HZ[50][row * 2];
            GD1[1] = HZ[50][row * 2 + 1];		 //万
            RD1[2] = HZ[51][row * 2];
            RD1[3] = HZ[51][row * 2 + 1];		 //年
            BD1[4] = HZ[52][row * 2];
            BD1[5] = HZ[52][row * 2 + 1];		 //历
            RD1[6] = GD1[6] = BD1[6] = HZ[53][row * 2];		 //钟
            RD1[7] = GD1[7] = BD1[7] = HZ[53][row * 2 + 1];

            GD2[0] = BD2[0] = HZ[54][row * 2];
            GD2[1] = BD2[1] = HZ[54][row * 2 + 1];
            GD2[2] = HZ[55][row * 2];
            GD2[3] = HZ[55][row * 2 + 1];
            RD2[4] = GD2[4] = HZ[56][row * 2];
            RD2[5] = GD2[5] = HZ[56][row * 2 + 1];
            RD2[6] = BD2[6] = HZ[57][row * 2];
            RD2[7] = BD2[7] = HZ[57][row * 2 + 1];



            for(i = 0; i < 8; i++)					 //扫描1行
            {
                for(j = 0; j < 8; j++)			 //发送1个数据
                {
                    #ifdef	DATAHI
                    RD1[i] <<= 1;				//数据左移1	  如是逆向取模则改为右移>>
                    R1 = CY;		//发送最高位，
                    RD2[i] <<= 1;
                    R2 = CY;
                    GD1[i] <<= 1;
                    G1 = CY;
                    GD2[i] <<= 1;
                    G2 = CY;
                    BD1[i] <<= 1;
                    B1 = CY;
                    BD2[i] <<= 1;
                    B2 = CY;
                    CK = 0;					//时钟下降
                    NOP4;
                    CK = 1;				 //时钟上升
                    #else
                    RD1[i] <<= 1;				//数据左移1	  如是逆向取模则改为右移>>
                    R1 = !CY;		//发送最高位，
                    RD2[i] <<= 1;
                    R2 = !CY;		//由于数据是低电平有效，所以进行取反
                    GD1[i] <<= 1;
                    G1 = !CY;
                    GD2[i] <<= 1;
                    G2 = !CY;
                    BD1[i] <<= 1;
                    B1 = !CY;
                    BD2[i] <<= 1;
                    B2 = !CY;
                    CK = 0;					//时钟下降
                    NOP4;
                    CK = 1;				 //时钟上升
                    #endif
                }
            }

            //ABCD=row;			//行选
            scan(row);
            LS = 0;
            NOP4;
            LS = 1;				//锁存上升，显示输出
            EN_ON();				//显示开
            Delay(10);
            TH0 = 255 - ADC_L;
            TR0 = 1;
            EN_OFF();			    //显示关
        }
    }
}


void Mode0123_RQ_G()		//  模式1234日期  绿数据
{
    uchar row1, row2;

    row1 = row + 8;		//	上1/2屏行扫描
    row2 = row - 8;		//	下1/2屏行扫描

    if(row < 8)		//上1/2屏
    {
        GD1[0] = (Num5[2][row] << 3) | (Num5[0][row] >> 3);
        GD1[1] = (Num5[0][row] << 5) | (Num5[nian1][row] >> 1);
        GD1[2] = (Num5[nian1][row] << 7) | (Num5[nian2][row] << 1);
        GD1[3] = Num5[yue1][row];
        GD1[4] = Num5[yue2][row] << 2;
        GD1[5] = Num5[ri1][row] << 1;
        GD1[6] = Num5[ri2][row] << 3;
        GD1[7] = Num8[week][row];

        if(row == 3)		//日期点
        {
            RD1[3] |= 0xC0;
            RD1[4] |= 0x01;
            RD1[5] |= 0x80;
        }
    }
    else	   //下1/2/屏
    {

        GD2[0] = Num8[y1][row2];	//农历日期
        GD2[1] = Num8[y2][row2];
        GD2[2] = Num8[11][row2];
        GD2[3] = NL8  [r1][row2];
        GD2[4] = Num8[r2][row2] >> 1;

        GD2[5] = Num8[r2][row2] << 7 | Num5[t1][row2 - 1] >> 2;	 //温度
        GD2[6] = Num5[t2][row2 - 1] | Num5[t1][row2 - 1] << 6;
        GD2[7] = Num5[17][row2];

//		GD2[0]=Num5[Zhi_12][row2];
//		GD2[1]=Num5[PM][row2];
//		GD2[0]=Num5[Li/10][row2];
//		GD2[1]=Num5[Li%10][row2];
//		GD2[2]=0;
//		GD2[3]=Num5[XSMOD/10][row2];
//		GD2[4]=Num5[XSMOD%10][row2]<<2;

        // GD2[5]=Num5[TH0/100][row2];
        // GD2[6]=Num5[TH0%100/10][row2];
        // GD2[7]=Num5[TH0%10][row2];
    }
}


void Mode0123_RQ_R()	  //模式1234日期 红数据
{
    uchar row1, row2;

    row1 = row + 8;		//	上1/2屏行扫描
    row2 = row - 8;		//	下1/2屏行扫描

    if(row < 8)
        RD1[7] = Num8[week][row];
    else
    {
        RD2[5] = Num5[t1][row2 - 1] >> 2;	 //温度
        RD2[6] = Num5[t2][row2 - 1] | Num5[t1][row2 - 1] << 6;
//		RD2[7]=Num5[17][row2];
    }
}

void Mode0_SJ_G()
{
    uchar data row1, row2;

    row1 = row + 8;		//	上1/2屏行扫描
    row2 = row - 8;		//	下1/2屏行扫描

    if(row < 8)
    {
        if(SHAN)
            NOP;
        else
        {
            GD2[2] |= maohao[0][row1];
            GD2[6] |= maohao[1][row1];
        }
    }
    else
    {
        if(SHAN)
            NOP;
        else
        {
            GD1[2] |= maohao[0][row2];
            GD1[6] |= maohao[1][row2];
        }
    }
}

//void Mode0_SJ_R()	   //三行有秒,粗体时间
//{
//    uchar row1, row2, row3;

//    if(row < 8)
//    {
//        row1 = (row + 8) * 2;		//	上1/2屏行扫描
//        row2 = (row + 8) * 2 + 1;		//	下1/2屏行扫描
//        row3 = row + 8;

//        if(shi1 == 0)
//        {
//            RD2[0] = 0;
//            RD2[1] = Num14B[shi2][row1] >> 3;
//        }
//        else
//        {
//            RD2[0] = Num14B[shi1][row1];	 //时
//            RD2[1] = Num14B[shi1][row2] | Num14B[shi2][row1] >> 3;
//        }

//        RD2[2] = Num14B[shi2][row1] << 5 | Num14B[shi2][row2] >> 3;
//        RD2[3] = Num14B[fen1][row1] >> 2;	 // 分
//        RD2[4] = Num14B[fen1][row1] << 6 | Num14B[fen1][row2] >> 2 | Num14B[fen2][row1] >> 5;
//        RD2[5] = Num14B[fen2][row1] << 3 | Num14B[fen2][row2] >> 5;
//        RD2[6] = Num12[miao1][row3] >> 2;		 //秒
//        RD2[7] = Num12[miao2][row3] >> 1 | Num12[miao1][row3] << 6;


//        if(GPS_POW)
//        {
//            RD2[2] |= maohao[0][row3];
//            RD2[6] |= maohao[1][row3];
//        }
//    }
//    else				  //时间上半部
//    {
//        row1 = (row - 8) * 2;		//	上1/2屏行扫描
//        row2 = (row - 8) * 2 + 1;		//	下1/2屏行扫描
//        row3 = row - 8;

//        if(shi1 == 0)
//        {
//            RD1[0] = 0;
//            RD1[1] = Num14B[shi2][row1] >> 3;
//        }
//        else
//        {
//            RD1[0] = Num14B[shi1][row1];	 //时
//            RD1[1] = Num14B[shi1][row2] | Num14B[shi2][row1] >> 3;
//        }

//        RD1[2] = Num14B[shi2][row1] << 5 | Num14B[shi2][row2] >> 3;
//        RD1[3] = Num14B[fen1][row1] >> 2;	 // 分
//        RD1[4] = Num14B[fen1][row1] << 6 | Num14B[fen1][row2] >> 2 | Num14B[fen2][row1] >> 5;
//        RD1[5] = Num14B[fen2][row1] << 3 | Num14B[fen2][row2] >> 5;
//        RD1[6] = Num12[miao1][row3] >> 2;		 //秒
//        RD1[7] = Num12[miao2][row3] >> 1 | Num12[miao1][row3] << 6;

//        if(PM)
//            RD1[0] |= Num12[15][row3];

//        if(GPS_POW)
//        {
//            RD1[2] |= maohao[0][row3];
//            RD1[6] |= maohao[1][row3];
//        }

//    }
//}

void Mode0_SJ_RGB()	   //三行有秒,粗体时间
{
    uchar row1, row2, row3;

    switch(COLORMODE)
    {
        case 0:
        {
            if(row < 8)
            {
                row1 = (row + 8) * 2;		//	上1/2屏行扫描
                row2 = (row + 8) * 2 + 1;		//	下1/2屏行扫描
                row3 = row + 8;


                if(shi1 == 0)
                {
                    RD2[0] = 0;
                    RD2[1] = Num14B[shi2][row1] >> 3;
                }
                else
                {
                    RD2[0] = Num14B[shi1][row1];	 //时
                    RD2[1] = Num14B[shi1][row2] | Num14B[shi2][row1] >> 3;
                }

                RD2[2] = Num14B[shi2][row1] << 5 | Num14B[shi2][row2] >> 3;
                RD2[3] = Num14B[fen1][row1] >> 2;	 // 分
                RD2[4] = Num14B[fen1][row1] << 6 | Num14B[fen1][row2] >> 2 | Num14B[fen2][row1] >> 5;
                RD2[5] = Num14B[fen2][row1] << 3 | Num14B[fen2][row2] >> 5;
                RD2[6] = Num12[miao1][row3] >> 2;		 //秒
                RD2[7] = Num12[miao2][row3] >> 1 | Num12[miao1][row3] << 6;


            }
            else				  //时间上半部
            {
                row1 = (row - 8) * 2;		//	上1/2屏行扫描
                row2 = (row - 8) * 2 + 1;		//	下1/2屏行扫描
                row3 = row - 8;

                if(shi1 == 0)
                {
                    RD1[0] = 0;
                    RD1[1] = Num14B[shi2][row1] >> 3;
                }
                else
                {
                    RD1[0] = Num14B[shi1][row1];	 //时
                    RD1[1] = Num14B[shi1][row2] | Num14B[shi2][row1] >> 3;
                }

                RD1[2] = Num14B[shi2][row1] << 5 | Num14B[shi2][row2] >> 3;
                RD1[3] = Num14B[fen1][row1] >> 2;	 // 分
                RD1[4] = Num14B[fen1][row1] << 6 | Num14B[fen1][row2] >> 2 | Num14B[fen2][row1] >> 5;
                RD1[5] = Num14B[fen2][row1] << 3 | Num14B[fen2][row2] >> 5;
                RD1[6] = Num12[miao1][row3] >> 2;		 //秒
                RD1[7] = Num12[miao2][row3] >> 1 | Num12[miao1][row3] << 6;

                if(PM)
                    RD1[0] |= Num12[15][row3];


            }

            break;
        }

        case 1:
        {
            if(row < 8)
            {
                row1 = (row + 8) * 2;		//	上1/2屏行扫描
                row2 = (row + 8) * 2 + 1;		//	下1/2屏行扫描
                row3 = row + 8;


                if(shi1 == 0)
                {
                    GD2[0] = 0;
                    GD2[1] = Num14B[shi2][row1] >> 3;
                }
                else
                {
                    GD2[0] = Num14B[shi1][row1];	 //时
                    GD2[1] = Num14B[shi1][row2] | Num14B[shi2][row1] >> 3;
                }

                GD2[2] = Num14B[shi2][row1] << 5 | Num14B[shi2][row2] >> 3;
                GD2[3] = Num14B[fen1][row1] >> 2;	 // 分
                GD2[4] = Num14B[fen1][row1] << 6 | Num14B[fen1][row2] >> 2 | Num14B[fen2][row1] >> 5;
                GD2[5] = Num14B[fen2][row1] << 3 | Num14B[fen2][row2] >> 5;
                GD2[6] = Num12[miao1][row3] >> 2;		 //秒
                GD2[7] = Num12[miao2][row3] >> 1 | Num12[miao1][row3] << 6;

            }
            else				  //时间上半部
            {
                row1 = (row - 8) * 2;		//	上1/2屏行扫描
                row2 = (row - 8) * 2 + 1;		//	下1/2屏行扫描
                row3 = row - 8;

                if(shi1 == 0)
                {
                    GD1[0] = 0;
                    GD1[1] = Num14B[shi2][row1] >> 3;
                }
                else
                {
                    GD1[0] = Num14B[shi1][row1];	 //时
                    GD1[1] = Num14B[shi1][row2] | Num14B[shi2][row1] >> 3;
                }

                GD1[2] = Num14B[shi2][row1] << 5 | Num14B[shi2][row2] >> 3;
                GD1[3] = Num14B[fen1][row1] >> 2;	 // 分
                GD1[4] = Num14B[fen1][row1] << 6 | Num14B[fen1][row2] >> 2 | Num14B[fen2][row1] >> 5;
                GD1[5] = Num14B[fen2][row1] << 3 | Num14B[fen2][row2] >> 5;
                GD1[6] = Num12[miao1][row3] >> 2;		 //秒
                GD1[7] = Num12[miao2][row3] >> 1 | Num12[miao1][row3] << 6;

                if(PM)
                    GD1[0] |= Num12[15][row3];

            }

            break;

        }

        case 2:
        {
            if(row < 8)
            {
                row1 = (row + 8) * 2;		//	上1/2屏行扫描
                row2 = (row + 8) * 2 + 1;		//	下1/2屏行扫描
                row3 = row + 8;


                if(shi1 == 0)
                {
                    BD2[0] = 0;
                    BD2[1] = Num14B[shi2][row1] >> 3;
                }
                else
                {
                    BD2[0] = Num14B[shi1][row1];	 //时
                    BD2[1] = Num14B[shi1][row2] | Num14B[shi2][row1] >> 3;
                }

                BD2[2] = Num14B[shi2][row1] << 5 | Num14B[shi2][row2] >> 3;
                BD2[3] = Num14B[fen1][row1] >> 2;	 // 分
                BD2[4] = Num14B[fen1][row1] << 6 | Num14B[fen1][row2] >> 2 | Num14B[fen2][row1] >> 5;
                BD2[5] = Num14B[fen2][row1] << 3 | Num14B[fen2][row2] >> 5;
                BD2[6] = Num12[miao1][row3] >> 2;		 //秒
                BD2[7] = Num12[miao2][row3] >> 1 | Num12[miao1][row3] << 6;


            }
            else				  //时间上半部
            {
                row1 = (row - 8) * 2;		//	上1/2屏行扫描
                row2 = (row - 8) * 2 + 1;		//	下1/2屏行扫描
                row3 = row - 8;

                if(shi1 == 0)
                {
                    BD1[0] = 0;
                    BD1[1] = Num14B[shi2][row1] >> 3;
                }
                else
                {
                    BD1[0] = Num14B[shi1][row1];	 //时
                    BD1[1] = Num14B[shi1][row2] | Num14B[shi2][row1] >> 3;
                }

                BD1[2] = Num14B[shi2][row1] << 5 | Num14B[shi2][row2] >> 3;
                BD1[3] = Num14B[fen1][row1] >> 2;	 // 分
                BD1[4] = Num14B[fen1][row1] << 6 | Num14B[fen1][row2] >> 2 | Num14B[fen2][row1] >> 5;
                BD1[5] = Num14B[fen2][row1] << 3 | Num14B[fen2][row2] >> 5;
                BD1[6] = Num12[miao1][row3] >> 2;		 //秒
                BD1[7] = Num12[miao2][row3] >> 1 | Num12[miao1][row3] << 6;

                if(PM)
                    BD1[0] |= Num12[15][row3];


            }

            break;

        }

        case 3:
        {
            if(row < 8)
            {
                row1 = (row + 8) * 2;		//	上1/2屏行扫描
                row2 = (row + 8) * 2 + 1;		//	下1/2屏行扫描
                row3 = row + 8;


                if(shi1 == 0)
                {
                    RD2[0] = GD2[0] = 0;
                    RD2[1] = GD2[1] = Num14B[shi2][row1] >> 3;
                }
                else
                {
                    RD2[0] = GD2[0] = Num14B[shi1][row1];	 //时
                    RD2[1] = GD2[1] = Num14B[shi1][row2] | Num14B[shi2][row1] >> 3;
                }

                RD2[2] = GD2[2] = Num14B[shi2][row1] << 5 | Num14B[shi2][row2] >> 3;
                RD2[3] = GD2[3] = Num14B[fen1][row1] >> 2;	 // 分
                RD2[4] = GD2[4] = Num14B[fen1][row1] << 6 | Num14B[fen1][row2] >> 2 | Num14B[fen2][row1] >> 5;
                RD2[5] = GD2[5] = Num14B[fen2][row1] << 3 | Num14B[fen2][row2] >> 5;
                RD2[6] = GD2[6] = Num12[miao1][row3] >> 2;		 //秒
                RD2[7] = GD2[7] = Num12[miao2][row3] >> 1 | Num12[miao1][row3] << 6;


            }
            else				  //时间上半部
            {
                row1 = (row - 8) * 2;		//	上1/2屏行扫描
                row2 = (row - 8) * 2 + 1;		//	下1/2屏行扫描
                row3 = row - 8;

                if(shi1 == 0)
                {
                    RD1[0] = GD1[0] = 0;
                    RD1[1] = GD1[1] = Num14B[shi2][row1] >> 3;
                }
                else
                {
                    RD1[0] = GD1[0] = Num14B[shi1][row1];	 //时
                    RD1[1] = GD1[1] = Num14B[shi1][row2] | Num14B[shi2][row1] >> 3;
                }

                RD1[2] = GD1[2] = Num14B[shi2][row1] << 5 | Num14B[shi2][row2] >> 3;
                RD1[3] = GD1[3] = Num14B[fen1][row1] >> 2;	 // 分
                RD1[4] = GD1[4] = Num14B[fen1][row1] << 6 | Num14B[fen1][row2] >> 2 | Num14B[fen2][row1] >> 5;
                RD1[5] = GD1[5] = Num14B[fen2][row1] << 3 | Num14B[fen2][row2] >> 5;
                RD1[6] = GD1[6] = Num12[miao1][row3] >> 2;		 //秒
                RD1[7] = GD1[7] = Num12[miao2][row3] >> 1 | Num12[miao1][row3] << 6;

                if(PM)
                {
                    RD1[0] |= Num12[15][row3];
                    GD1[0] |= Num12[15][row3];
                }

            }

            break;

        }

        case 4:
        {
            if(row < 8)
            {
                row1 = (row + 8) * 2;		//	上1/2屏行扫描
                row2 = (row + 8) * 2 + 1;		//	下1/2屏行扫描
                row3 = row + 8;


                if(shi1 == 0)
                {
                    RD2[0] = BD2[0] = 0;
                    RD2[1] = BD2[1] = Num14B[shi2][row1] >> 3;
                }
                else
                {
                    RD2[0] = BD2[0] = Num14B[shi1][row1];	 //时
                    RD2[1] = BD2[1] = Num14B[shi1][row2] | Num14B[shi2][row1] >> 3;
                }

                RD2[2] = BD2[2] = Num14B[shi2][row1] << 5 | Num14B[shi2][row2] >> 3;
                RD2[3] = BD2[3] = Num14B[fen1][row1] >> 2;	 // 分
                RD2[4] = BD2[4] = Num14B[fen1][row1] << 6 | Num14B[fen1][row2] >> 2 | Num14B[fen2][row1] >> 5;
                RD2[5] = BD2[5] = Num14B[fen2][row1] << 3 | Num14B[fen2][row2] >> 5;
                RD2[6] = BD2[6] = Num12[miao1][row3] >> 2;		 //秒
                RD2[7] = BD2[7] = Num12[miao2][row3] >> 1 | Num12[miao1][row3] << 6;


            }
            else				  //时间上半部
            {
                row1 = (row - 8) * 2;		//	上1/2屏行扫描
                row2 = (row - 8) * 2 + 1;		//	下1/2屏行扫描
                row3 = row - 8;

                if(shi1 == 0)
                {
                    RD1[0] = BD1[0] = 0;
                    RD1[1] = BD1[1] = Num14B[shi2][row1] >> 3;
                }
                else
                {
                    RD1[0] = BD1[0] = Num14B[shi1][row1];	 //时
                    RD1[1] = BD1[1] = Num14B[shi1][row2] | Num14B[shi2][row1] >> 3;
                }

                RD1[2] = BD1[2] = Num14B[shi2][row1] << 5 | Num14B[shi2][row2] >> 3;
                RD1[3] = BD1[3] = Num14B[fen1][row1] >> 2;	 // 分
                RD1[4] = BD1[4] = Num14B[fen1][row1] << 6 | Num14B[fen1][row2] >> 2 | Num14B[fen2][row1] >> 5;
                RD1[5] = BD1[5] = Num14B[fen2][row1] << 3 | Num14B[fen2][row2] >> 5;
                RD1[6] = BD1[6] = Num12[miao1][row3] >> 2;		 //秒
                RD1[7] = BD1[7] = Num12[miao2][row3] >> 1 | Num12[miao1][row3] << 6;

                if(PM)
                {
                    RD1[0] |= Num12[15][row3];
                    BD1[0] |= Num12[15][row3];
                }

            }

            break;


        }

        case 5:
        {
            if(row < 8)
            {
                row1 = (row + 8) * 2;		//	上1/2屏行扫描
                row2 = (row + 8) * 2 + 1;		//	下1/2屏行扫描
                row3 = row + 8;


                if(shi1 == 0)
                {
                    BD2[0] = GD2[0] = 0;
                    BD2[1] = GD2[1] = Num14B[shi2][row1] >> 3;
                }
                else
                {
                    BD2[0] = GD2[0] = Num14B[shi1][row1];	 //时
                    BD2[1] = GD2[1] = Num14B[shi1][row2] | Num14B[shi2][row1] >> 3;
                }

                BD2[2] = GD2[2] = Num14B[shi2][row1] << 5 | Num14B[shi2][row2] >> 3;
                BD2[3] = GD2[3] = Num14B[fen1][row1] >> 2;	 // 分
                BD2[4] = GD2[4] = Num14B[fen1][row1] << 6 | Num14B[fen1][row2] >> 2 | Num14B[fen2][row1] >> 5;
                BD2[5] = GD2[5] = Num14B[fen2][row1] << 3 | Num14B[fen2][row2] >> 5;
                BD2[6] = GD2[6] = Num12[miao1][row3] >> 2;		 //秒
                BD2[7] = GD2[7] = Num12[miao2][row3] >> 1 | Num12[miao1][row3] << 6;


            }
            else				  //时间上半部
            {
                row1 = (row - 8) * 2;		//	上1/2屏行扫描
                row2 = (row - 8) * 2 + 1;		//	下1/2屏行扫描
                row3 = row - 8;

                if(shi1 == 0)
                {
                    BD1[0] = GD1[0] = 0;
                    BD1[1] = GD1[1] = Num14B[shi2][row1] >> 3;
                }
                else
                {
                    BD1[0] = GD1[0] = Num14B[shi1][row1];	 //时
                    BD1[1] = GD1[1] = Num14B[shi1][row2] | Num14B[shi2][row1] >> 3;
                }

                BD1[2] = GD1[2] = Num14B[shi2][row1] << 5 | Num14B[shi2][row2] >> 3;
                BD1[3] = GD1[3] = Num14B[fen1][row1] >> 2;	 // 分
                BD1[4] = GD1[4] = Num14B[fen1][row1] << 6 | Num14B[fen1][row2] >> 2 | Num14B[fen2][row1] >> 5;
                BD1[5] = GD1[5] = Num14B[fen2][row1] << 3 | Num14B[fen2][row2] >> 5;
                BD1[6] = GD1[6] = Num12[miao1][row3] >> 2;		 //秒
                BD1[7] = GD1[7] = Num12[miao2][row3] >> 1 | Num12[miao1][row3] << 6;

                if(PM)
                {
                    BD1[0] |= Num12[15][row3];
                    GD1[0] |= Num12[15][row3];
                }

            }

            break;

        }

        case 6:
        {
            if(row < 8)
            {
                row1 = (row + 8) * 2;		//	上1/2屏行扫描
                row2 = (row + 8) * 2 + 1;		//	下1/2屏行扫描
                row3 = row + 8;


                if(shi1 == 0)
                {
                    RD2[0] = GD2[0] = BD2[0] = 0;
                    RD2[1] = GD2[1] = BD2[1] = Num14B[shi2][row1] >> 3;
                }
                else
                {
                    RD2[0] = GD2[0] =  BD2[0] = Num14B[shi1][row1];	 //时
                    RD2[1] = GD2[1] =  BD2[1] = Num14B[shi1][row2] | Num14B[shi2][row1] >> 3;
                }

                RD2[2] = GD2[2] =  BD2[2] = Num14B[shi2][row1] << 5 | Num14B[shi2][row2] >> 3;
                RD2[3] = GD2[3] =  BD2[3] = Num14B[fen1][row1] >> 2;	 // 分
                RD2[4] = GD2[4] =  BD2[4] = Num14B[fen1][row1] << 6 | Num14B[fen1][row2] >> 2 | Num14B[fen2][row1] >> 5;
                RD2[5] = GD2[5] =  BD2[5] = Num14B[fen2][row1] << 3 | Num14B[fen2][row2] >> 5;
                RD2[6] = GD2[6] =  BD2[6] = Num12[miao1][row3] >> 2;		 //秒
                RD2[7] = GD2[7] =  BD2[7] = Num12[miao2][row3] >> 1 | Num12[miao1][row3] << 6;


            }
            else				  //时间上半部
            {
                row1 = (row - 8) * 2;		//	上1/2屏行扫描
                row2 = (row - 8) * 2 + 1;		//	下1/2屏行扫描
                row3 = row - 8;

                if(shi1 == 0)
                {
                    RD1[0] = GD1[0] =  BD1[0] = 0;
                    RD1[1] = GD1[1] =  BD1[1] = Num14B[shi2][row1] >> 3;
                }
                else
                {
                    RD1[0] = GD1[0] =  BD1[0] = Num14B[shi1][row1];	 //时
                    RD1[1] = GD1[1] =  BD1[1] = Num14B[shi1][row2] | Num14B[shi2][row1] >> 3;
                }

                RD1[2] = GD1[2] =  BD1[2] = Num14B[shi2][row1] << 5 | Num14B[shi2][row2] >> 3;
                RD1[3] = GD1[3] =  BD1[3] = Num14B[fen1][row1] >> 2;	 // 分
                RD1[4] = GD1[4] =  BD1[4] = Num14B[fen1][row1] << 6 | Num14B[fen1][row2] >> 2 | Num14B[fen2][row1] >> 5;
                RD1[5] = GD1[5] =  BD1[5] = Num14B[fen2][row1] << 3 | Num14B[fen2][row2] >> 5;
                RD1[6] = GD1[6] =  BD1[6] = Num12[miao1][row3] >> 2;		 //秒
                RD1[7] = GD1[7] =  BD1[7] = Num12[miao2][row3] >> 1 | Num12[miao1][row3] << 6;

                if(PM)
                {
                    RD1[0] |= Num12[15][row3];
                    GD1[0] |= Num12[15][row3];
                    BD1[0] |= Num12[15][row3];
                }

            }

            break;

        }
    }
}




void Mode1_SJ_R()
{

    if(shi1 == 0)
    {
        RD1[0] = 0;
        RD1[1] = 0;
        RD2[0] = 0;
        RD2[1] = 0;
    }
    else
    {
        RD1[0] = Num16x32[shi1][row * 2];	 //时
        RD1[1] = Num16x32[shi1][row * 2 + 1];	 //时
        RD2[0] = Num16x32[shi1][32 + row * 2];	 //时
        RD2[1] = Num16x32[shi1][32 + row * 2 + 1];	 //时
    }

    RD1[2] = Num16x32[shi2][row * 2];	 //时2
    RD1[3] = Num16x32[shi2][row * 2 + 1];	 //时2
    RD2[2] = Num16x32[shi2][32 + row * 2];	 //时2
    RD2[3] = Num16x32[shi2][32 + row * 2 + 1];	 //时2

    RD1[4] = Num16x32[fen1][row * 2] >> 3;	 //分1
    RD1[5] = Num16x32[fen1][row * 2 + 1] >> 3 | Num16x32[fen1][row * 2] << 5;	 //分1
    RD2[4] = Num16x32[fen1][32 + row * 2] >> 3;	 //分1
    RD2[5] = Num16x32[fen1][32 + row * 2 + 1] >> 3 | Num16x32[fen1][32 + row * 2] << 5;	 //分1

    RD1[6] = Num16x32[fen2][row * 2] >> 3;	 //分2
    RD1[7] = Num16x32[fen2][row * 2 + 1] >> 3 | Num16x32[fen2][row * 2] << 5;	 //分2
    RD2[6] = Num16x32[fen2][32 + row * 2] >> 3;	 //分2
    RD2[7] = Num16x32[fen2][32 + row * 2 + 1] >> 3 | Num16x32[fen2][32 + row * 2] << 5;	 //分2



    if(PM)
        RD1[0] |= Num12[15][row];

    if(row < 8)
    {
        if(row == 5 | row == 6)
        {
            if(SHAN)
            {
                GD2[3] |= 0x01;
                GD2[4] |= 0x80;
            }
        }
    }
    else
    {
        if(row == 11 | row == 12)
        {
            if(SHAN)
            {
                GD1[3] |= 0x01;
                GD1[4] |= 0x80;
            }
        }
    }

}



void Mode1_SJ_G()
{
    if(shi1 == 0)
    {
        GD1[0] = 0;
        GD1[1] = 0;
        GD2[0] = 0;
        GD2[1] = 0;
    }
    else
    {
        GD1[0] = Num16x32[shi1][row * 2];	 //时
        GD1[1] = Num16x32[shi1][row * 2 + 1];	 //时
        GD2[0] = Num16x32[shi1][32 + row * 2];	 //时
        GD2[1] = Num16x32[shi1][32 + row * 2 + 1];	 //时
    }

    GD1[2] = Num16x32[shi2][row * 2];	 //时2
    GD1[3] = Num16x32[shi2][row * 2 + 1];	 //时2
    GD2[2] = Num16x32[shi2][32 + row * 2];	 //时2
    GD2[3] = Num16x32[shi2][32 + row * 2 + 1];	 //时2

    GD1[4] = Num16x32[fen1][row * 2] >> 3;	 //分1
    GD1[5] = Num16x32[fen1][row * 2 + 1] >> 3 | Num16x32[fen1][row * 2] << 5;	 //分1
    GD2[4] = Num16x32[fen1][32 + row * 2] >> 3;	 //分1
    GD2[5] = Num16x32[fen1][32 + row * 2 + 1] >> 3 | Num16x32[fen1][32 + row * 2] << 5;	 //分1

    GD1[6] = Num16x32[fen2][row * 2] >> 3;	 //分2
    GD1[7] = Num16x32[fen2][row * 2 + 1] >> 3 | Num16x32[fen2][row * 2] << 5;	 //分2
    GD2[6] = Num16x32[fen2][32 + row * 2] >> 3;	 //分2
    GD2[7] = Num16x32[fen2][32 + row * 2 + 1] >> 3 | Num16x32[fen2][32 + row * 2] << 5;	 //分2



    if(PM)
        RD1[0] |= Num12[15][row];

    if(row < 8)
    {
        if(row == 5 | row == 6)
        {
            if(SHAN)
            {
                RD2[3] |= 0x01;
                RD2[4] |= 0x80;
            }
        }
    }
    else
    {
        if(row == 11 | row == 12)
        {
            if(SHAN)
            {
                RD1[3] |= 0x01;
                RD1[4] |= 0x80;
            }
        }
    }
}


void Mode1_SJ_B()
{

    if(shi1 == 0)
    {
        BD1[0] = 0;
        BD1[1] = 0;
        BD2[0] = 0;
        BD2[1] = 0;
    }
    else
    {
        BD1[0] = Num16x32[shi1][row * 2];	 //时
        BD1[1] = Num16x32[shi1][row * 2 + 1];	 //时
        BD2[0] = Num16x32[shi1][32 + row * 2];	 //时
        BD2[1] = Num16x32[shi1][32 + row * 2 + 1];	 //时
    }

    BD1[2] = Num16x32[shi2][row * 2];	 //时2
    BD1[3] = Num16x32[shi2][row * 2 + 1];	 //时2
    BD2[2] = Num16x32[shi2][32 + row * 2];	 //时2
    BD2[3] = Num16x32[shi2][32 + row * 2 + 1];	 //时2

    BD1[4] = Num16x32[fen1][row * 2] >> 3;	 //分1
    BD1[5] = Num16x32[fen1][row * 2 + 1] >> 3 | Num16x32[fen1][row * 2] << 5;	 //分1
    BD2[4] = Num16x32[fen1][32 + row * 2] >> 3;	 //分1
    BD2[5] = Num16x32[fen1][32 + row * 2 + 1] >> 3 | Num16x32[fen1][32 + row * 2] << 5;	 //分1

    BD1[6] = Num16x32[fen2][row * 2] >> 3;	 //分2
    BD1[7] = Num16x32[fen2][row * 2 + 1] >> 3 | Num16x32[fen2][row * 2] << 5;	 //分2
    BD2[6] = Num16x32[fen2][32 + row * 2] >> 3;	 //分2
    BD2[7] = Num16x32[fen2][32 + row * 2 + 1] >> 3 | Num16x32[fen2][32 + row * 2] << 5;	 //分2



    if(PM)
        BD1[0] |= Num12[15][row];

    if(row < 8)
    {
        if(row == 5 | row == 6)
        {
            if(SHAN)
            {
                GD2[3] |= 0x01;
                GD2[4] |= 0x80;
            }
        }
    }
    else
    {
        if(row == 11 | row == 12)
        {
            if(SHAN)
            {
                GD1[3] |= 0x01;
                GD1[4] |= 0x80;
            }
        }
    }

}

void Mode1_SJ_RGB()
{
    uchar code miaodiandate[] = {0x00, 0x80, 0xc0, 0xe0, 0xf0, 0xf8, 0xfc, 0xfe, 0xff};
    uchar miao;

    switch(COLORMODE)
    {
        case 0:
        {

            Mode1_SJ_R();
            break;
        }

        case 1:
        {

            Mode1_SJ_G();
            break;
        }

        case 2:
        {

            Mode1_SJ_B();
            break;
        }

        case 3:
        {

            Mode1_SJ_R();
            Mode1_SJ_G();
            break;
        }

        case 4:
        {

            Mode1_SJ_R();
            Mode1_SJ_B();
            break;
        }

        case 5:
        {
            Mode1_SJ_G();
            Mode1_SJ_B();
            break;
        }

        case 6:
        {
            Mode1_SJ_R();
            Mode1_SJ_G();
            Mode1_SJ_B();
            break;
        }
    }

    miao = miao1 * 10 + miao2;

    if(row == 15)          //秒点进度条显示
    {
        if (COLORMODE == 0||COLORMODE == 4)  //字体红色可粉红色时显示绿色秒进度条。
        {
            if(miao > 56)
            {
                miao = miao - 8 * 7;
                GD2[0] = GD2[1] = GD2[2] = GD2[3] = GD2[4] = GD2[5] = GD2[6] = 0xff;
                GD2[7] = miaodiandate[miao];
            }

            else if(miao > 48)
            {
                miao = miao - 8 * 6;
                GD2[0] = GD2[1] = GD2[2] = GD2[3] = GD2[4] = GD2[5] = 0xff;
                GD2[6] = miaodiandate[miao];
            }

            else if(miao > 40)
            {
                miao = miao - 8 * 5;
                GD2[0] = GD2[1] = GD2[2] = GD2[3] = GD2[4] = 0xff;
                GD2[5] = miaodiandate[miao];
            }

            else if(miao > 32)
            {
                miao = miao - 8 * 4;
                GD2[0] = GD2[1] = GD2[2] = GD2[3] = 0xff;
                GD2[4] = miaodiandate[miao];
            }

            else if(miao > 24)
            {
                miao = miao - 8 * 3;
                GD2[0] = GD2[1] = GD2[2] = 0xff;
                GD2[3] = miaodiandate[miao];
            }

            else if(miao > 16)
            {
                miao = miao - 8 * 2;
                GD2[0] = GD2[1] = 0xff;
                GD2[2] = miaodiandate[miao];
            }

            else if(miao > 8)
            {
                miao = miao - 8 * 1;
                GD2[0] = 0xff;
                GD2[1] = miaodiandate[miao];
            }
            else if(miao < 9)
            {
                GD2[0] = miaodiandate[miao];
            }
        }

        else                         
        {
            if(miao > 56)
            {
                miao = miao - 8 * 7;
                RD2[0] = RD2[1] = RD2[2] = RD2[3] = RD2[4] = RD2[5] = RD2[6] = 0xff;
                RD2[7] = miaodiandate[miao];
            }

            else if(miao > 48)
            {
                miao = miao - 8 * 6;
                RD2[0] = RD2[1] = RD2[2] = RD2[3] = RD2[4] = RD2[5] = 0xff;
                RD2[6] = miaodiandate[miao];
            }

            else if(miao > 40)
            {
                miao = miao - 8 * 5;
                RD2[0] = RD2[1] = RD2[2] = RD2[3] = RD2[4] = 0xff;
                RD2[5] = miaodiandate[miao];
            }

            else if(miao > 32)
            {
                miao = miao - 8 * 4;
                RD2[0] = RD2[1] = RD2[2] = RD2[3] = 0xff;
                RD2[4] = miaodiandate[miao];
            }

            else if(miao > 24)
            {
                miao = miao - 8 * 3;
                RD2[0] = RD2[1] = RD2[2] = 0xff;
                RD2[3] = miaodiandate[miao];
            }

            else if(miao > 16)
            {
                miao = miao - 8 * 2;
                RD2[0] = RD2[1] = 0xff;
                RD2[2] = miaodiandate[miao];
            }

            else if(miao > 8)
            {
                miao = miao - 8 * 1;
                RD2[0] = 0xff;
                RD2[1] = miaodiandate[miao];
            }
            else if(miao < 9)
            {
                RD2[0] = miaodiandate[miao];
            }
        }
    }
}


void Mode2_SJ_R()
{
    uchar row3, row4;

    if(row < 8)
    {
        ////时间
        row3 = (row + 8) * 2;
        row4 = (row + 8) * 2 + 1;

        if(shi1 == 0) 	 //
            RD2[0] = RD2[1] = 0;
        else
        {
            RD2[0] = Num16B[shi1][row3];	 //时
            RD2[1] = Num16B[shi1][row4];
        }

        RD2[2] = Num16B[shi2][row3] << 3 | Num16B[shi2][row4] >> 5;
        RD2[3] = Num16B[shi2][row4] << 3;	 // 分
        RD2[4] = Num16B[fen1][row3] >> 3;
        RD2[5] = Num16B[fen1][row4] >> 3 | Num16B[fen1][row3] << 5;
        RD2[6] = Num16B[fen2][row3];		 //秒
        RD2[7] = Num16B[fen2][row4];


        //秒点
        if(SHAN)
        {
            RD2[3] |= maohao[4][row + 8];
            RD2[4] |= maohao[3][row + 8];
        }
        else
        {
            NOP4;
            NOP4;
        }
    }
    else				  //时间上半部
    {
//		if(shi1==0) RD1[0]=0; else
        row3 = (row - 8) * 2;
        row4 = (row - 8) * 2 + 1;

        if(shi1 == 0)
            RD1[0] = RD1[1] = 0;
        else
        {
            RD1[0] = Num16B[shi1][row3];	 //时
            RD1[1] = Num16B[shi1][row4];
        }

        RD1[2] = Num16B[shi2][row3] << 3 | Num16B[shi2][row4] >> 5;
        RD1[3] = Num16B[shi2][row4] << 3;	 // 分
        RD1[4] = Num16B[fen1][row3] >> 3;
        RD1[5] = Num16B[fen1][row4] >> 3 | Num16B[fen1][row3] << 5;
        RD1[6] = Num16B[fen2][row3];		 //秒
        RD1[7] = Num16B[fen2][row4];

        if(PM)
            RD1[0] |= Num12[15][row - 8];

        if(SHAN)
        {
            RD1[3] |= maohao[4][row - 8];
            RD1[4] |= maohao[3][row - 8];
        }
        else
        {
            NOP4;
            NOP4;
        }
    }

}


void Mode2_SJ_G()
{
    if(row < 8)
    {
        //秒点
        if(SHAN)
        {

            GD2[3] |= maohao[4][row + 8];
            GD2[4] |= maohao[3][row + 8];
        }
        else
            Delayus(20);
    }
    else				  //时间上半部
    {
        if(SHAN)
        {

            GD1[3] |= maohao[4][row - 8];
            GD1[4] |= maohao[3][row - 8];
        }
        else
            Delayus(20);
    }

}


void Mode3_SJ_R()
{
    uchar row3, row4;

    if(row < 8)
    {
        ////时间
        row3 = (row + 8) * 2;
        row4 = (row + 8) * 2 + 1;

        if(shi1 == 0) 	 //
            RD2[0] = RD2[1] = 0;
        else
        {
            RD2[0] = Num16[shi1][row3];	 //时
            RD2[1] = Num16[shi1][row4];
        }

        RD2[2] = Num16[shi2][row3] << 3 | Num16[shi2][row4] >> 5;
        RD2[3] = Num16[shi2][row4] << 3;	 // 分
        RD2[4] = Num16[fen1][row3] >> 3;
        RD2[5] = Num16[fen1][row4] >> 3 | Num16[fen1][row3] << 5;
        RD2[6] = Num16[fen2][row3];		 //秒
        RD2[7] = Num16[fen2][row4];


        //秒点
        if(SHAN)
        {

            RD2[3] |= maohao[4][row + 8];
            RD2[4] |= maohao[3][row + 8];
        }
        else
        {
        }
    }
    else				  //时间上半部
    {
//		if(shi1==0) RD1[0]=0; else
        row3 = (row - 8) * 2;
        row4 = (row - 8) * 2 + 1;

        if(shi1 == 0)
            RD1[0] = RD1[1] = 0;
        else
        {
            RD1[0] = Num16[shi1][row3];	 //时
            RD1[1] = Num16[shi1][row4];
        }

        RD1[2] = Num16[shi2][row3] << 3 | Num16[shi2][row4] >> 5;
        RD1[3] = Num16[shi2][row4] << 3;	 // 分
        RD1[4] = Num16[fen1][row3] >> 3;
        RD1[5] = Num16[fen1][row4] >> 3 | Num16[fen1][row3] << 5;
        RD1[6] = Num16[fen2][row3];		 //秒
        RD1[7] = Num16[fen2][row4];

        if(PM)
            RD1[0] |= Num12[15][row - 8];

        if(SHAN)
        {
            RD1[3] |= maohao[4][row - 8];
            RD1[4] |= maohao[3][row - 8];
        }
        else
        {
            NOP4;
            NOP4;
        }
    }

}


void Mode3_SJ_G()
{
    if(row < 8)
    {
        //秒点
        if(SHAN)
        {

            GD2[3] |= maohao[4][row + 8];
            GD2[4] |= maohao[3][row + 8];
        }
        else
            Delayus(20);
    }
    else				  //时间上半部
    {
        if(SHAN)
        {

            GD1[3] |= maohao[4][row - 8];
            GD1[4] |= maohao[3][row - 8];
        }
        else
            Delayus(20);
    }

}



void Mode4567_RQ_G()
{
    uchar row1, row2;

    row1 = row + 8;		//	上1/2屏行扫描
    row2 = row - 8;		//	下1/2屏行扫描

    if(row < 8)		//上1/2屏
    {
//		GD1[0]=(Num5[2][row]<<3)|(Num5[0][row]>>3);
//		GD1[1]=(Num5[0][row]<<5)|(Num5[nian1][row]>>1);
//		GD1[2]=(Num5[nian1][row]<<7)|(Num5[nian2][row]<<1);
//		GD1[3]=Num5[yue1][row];
//		GD1[4]=Num5[yue2][row]<<2;
//		GD1[5]=Num5[ri1][row]<<1;
//		GD1[6]=Num5[ri2][row]<<3;
        GD1[7] = Num8[week][row];

        if(row == 3)		//日期点
        {
            GD1[3] |= 0xC0;
            GD1[4] |= 0x01;
            GD1[5] |= 0x80;
        }
    }
    else	   //下1/2/屏
    {

        GD1[0] = Num8[y1][row2];	//农历日期
        GD1[1] = Num8[y2][row2];
        GD1[2] = Num8[11][row2];
        GD1[3] = NL8  [r1][row2];
        GD1[4] = Num8[r2][row2] >> 1;

        GD1[5] = Num8[r2][row2] << 7 | Num5[t1][row2 - 1] >> 2;	 //温度
        GD1[6] = Num5[t2][row2 - 1] | Num5[t1][row2 - 1] << 6;
        GD1[7] = Num5[17][row2];

        // GD1[0]=(Num5[Light_R/100][row2]<<3)|(Num5[Light_R%100/10][row2]>>3);
        // GD1[1]=(Num5[Light_R%100/10][row2]<<5)|(Num5[Light_R%10][row2]>>1);
        // GD1[2]=(Num5[Light_R%10][row2]<<7);

        // GD1[3]=Num5[XSMOD/10][row2];
        // GD1[4]=Num5[XSMOD%10][row2]<<2;

//		GD1[5]=Num5[ADC_T/100][row2];
//		GD1[6]=Num5[ADC_T%100/10][row2];
//		GD1[7]=Num5[ADC_T%10][row2];

//		GD1[5]=Num5[TH0/100][row2];
//		GD1[6]=Num5[TH0%100/10][row2];
//		GD1[7]=Num5[TH0%10][row2];
    }
}


void Mode4567_RQ_R()	  //模式4567日期 红数据
{
    uchar row1, row2;

    row1 = row + 8;		//	上1/2屏行扫描
    row2 = row - 8;		//	下1/2屏行扫描

    if(row < 8)
    {
        RD1[0] = (Num5[2][row] << 3) | (Num5[0][row] >> 3);
        RD1[1] = (Num5[0][row] << 5) | (Num5[nian1][row] >> 1);
        RD1[2] = (Num5[nian1][row] << 7) | (Num5[nian2][row] << 1);
        RD1[3] = Num5[yue1][row];
        RD1[4] = Num5[yue2][row] << 2;
        RD1[5] = Num5[ri1][row] << 1;
        RD1[6] = Num5[ri2][row] << 3;
        RD1[7] = Num8[week][row];
    }
    else
    {
        RD1[5] = Num5[t1][row2 - 1] >> 2;	 //温度
        RD1[6] = Num5[t2][row2 - 1] | Num5[t1][row2 - 1] << 6;
//		RD1[7]=Num5[17][row2];
    }
}


void Mode4_SJ_G()
{
    if(SHAN)
    {
        GD2[2] |= maohao[0][row];
        GD2[6] |= maohao[1][row];
    }
}


void Mode4_SJ_R()
{
    uchar row1, row2;
//	shi1=shi2=0;

    row1 = (row - 1) * 2;		//	上1/2屏行扫描
    row2 = (row - 1) * 2 + 1;		//	下1/2屏行扫描



    switch(COLORMODE)
    {
        case 0:
        {

            if(shi1 == 0)
            {
                RD2[0] = 0;
                RD2[1] = Num14B[shi2][row1] >> 3;
            }
            else
            {
                RD2[0] = Num14B[shi1][row1];	 //时
                RD2[1] = Num14B[shi1][row2] | Num14B[shi2][row1] >> 3;
            }

            RD2[2] = Num14B[shi2][row1] << 5 | Num14B[shi2][row2] >> 3;
            RD2[3] = Num14B[fen1][row1] >> 2;	 // 分
            RD2[4] = Num14B[fen1][row1] << 6 | Num14B[fen1][row2] >> 2 | Num14B[fen2][row1] >> 5;
            RD2[5] = Num14B[fen2][row1] << 3 | Num14B[fen2][row2] >> 5;
            RD2[6] = Num12[miao1][row - 1] >> 2;		 //秒
            RD2[7] = Num12[miao2][row - 1] >> 1 | Num12[miao1][row - 1] << 6;
            break;
        }

        case 1:
        {

            if(shi1 == 0)
            {
                GD2[0] = 0;
                GD2[1] = Num14B[shi2][row1] >> 3;
            }
            else
            {
                GD2[0] = Num14B[shi1][row1];	 //时
                GD2[1] = Num14B[shi1][row2] | Num14B[shi2][row1] >> 3;
            }

            GD2[2] = Num14B[shi2][row1] << 5 | Num14B[shi2][row2] >> 3;
            GD2[3] = Num14B[fen1][row1] >> 2;	 // 分
            GD2[4] = Num14B[fen1][row1] << 6 | Num14B[fen1][row2] >> 2 | Num14B[fen2][row1] >> 5;
            GD2[5] = Num14B[fen2][row1] << 3 | Num14B[fen2][row2] >> 5;
            GD2[6] = Num12[miao1][row - 1] >> 2;		 //秒
            GD2[7] = Num12[miao2][row - 1] >> 1 | Num12[miao1][row - 1] << 6;
            break;
        }

        case 2:
        {

            if(shi1 == 0)
            {
                BD2[0] = 0;
                BD2[1] = Num14B[shi2][row1] >> 3;
            }
            else
            {
                BD2[0] = Num14B[shi1][row1];	 //时
                BD2[1] = Num14B[shi1][row2] | Num14B[shi2][row1] >> 3;
            }

            BD2[2] = Num14B[shi2][row1] << 5 | Num14B[shi2][row2] >> 3;
            BD2[3] = Num14B[fen1][row1] >> 2;	 // 分
            BD2[4] = Num14B[fen1][row1] << 6 | Num14B[fen1][row2] >> 2 | Num14B[fen2][row1] >> 5;
            BD2[5] = Num14B[fen2][row1] << 3 | Num14B[fen2][row2] >> 5;
            BD2[6] = Num12[miao1][row - 1] >> 2;		 //秒
            BD2[7] = Num12[miao2][row - 1] >> 1 | Num12[miao1][row - 1] << 6;
            break;
        }

        case 3:
        {

            if(shi1 == 0)
            {
                RD2[0] = GD2[0] = 0;
                RD2[1] = GD2[1] = Num14B[shi2][row1] >> 3;
            }
            else
            {
                RD2[0] = GD2[0] = Num14B[shi1][row1];	 //时
                RD2[1] = GD2[1] = Num14B[shi1][row2] | Num14B[shi2][row1] >> 3;
            }

            RD2[2] = GD2[2] = Num14B[shi2][row1] << 5 | Num14B[shi2][row2] >> 3;
            RD2[3] = GD2[3] = Num14B[fen1][row1] >> 2;	 // 分
            RD2[4] = GD2[4] = Num14B[fen1][row1] << 6 | Num14B[fen1][row2] >> 2 | Num14B[fen2][row1] >> 5;
            RD2[5] = GD2[5] = Num14B[fen2][row1] << 3 | Num14B[fen2][row2] >> 5;
            RD2[6] = GD2[6] = Num12[miao1][row - 1] >> 2;		 //秒
            RD2[7] = GD2[7] = Num12[miao2][row - 1] >> 1 | Num12[miao1][row - 1] << 6;
            break;
        }

        case 4:
        {

            if(shi1 == 0)
            {
                RD2[0] = BD2[0] = 0;
                RD2[1] = BD2[1] = Num14B[shi2][row1] >> 3;
            }
            else
            {
                RD2[0] = BD2[0] = Num14B[shi1][row1];	 //时
                RD2[1] = BD2[1] = Num14B[shi1][row2] | Num14B[shi2][row1] >> 3;
            }

            RD2[2] = BD2[2] = Num14B[shi2][row1] << 5 | Num14B[shi2][row2] >> 3;
            RD2[3] = BD2[3] = Num14B[fen1][row1] >> 2;	 // 分
            RD2[4] = BD2[4] = Num14B[fen1][row1] << 6 | Num14B[fen1][row2] >> 2 | Num14B[fen2][row1] >> 5;
            RD2[5] = BD2[5] = Num14B[fen2][row1] << 3 | Num14B[fen2][row2] >> 5;
            RD2[6] = BD2[6] = Num12[miao1][row - 1] >> 2;		 //秒
            RD2[7] = BD2[7] = Num12[miao2][row - 1] >> 1 | Num12[miao1][row - 1] << 6;
            break;
        }

        case 5:
        {

            if(shi1 == 0)
            {
                GD2[0] = BD2[0] = 0;
                GD2[1] = BD2[1] = Num14B[shi2][row1] >> 3;
            }
            else
            {
                GD2[0] = BD2[0] = Num14B[shi1][row1];	 //时
                GD2[1] = BD2[1] = Num14B[shi1][row2] | Num14B[shi2][row1] >> 3;
            }

            GD2[2] = BD2[2] = Num14B[shi2][row1] << 5 | Num14B[shi2][row2] >> 3;
            GD2[3] = BD2[3] = Num14B[fen1][row1] >> 2;	 // 分
            GD2[4] = BD2[4] = Num14B[fen1][row1] << 6 | Num14B[fen1][row2] >> 2 | Num14B[fen2][row1] >> 5;
            GD2[5] = BD2[5] = Num14B[fen2][row1] << 3 | Num14B[fen2][row2] >> 5;
            GD2[6] = BD2[6] = Num12[miao1][row - 1] >> 2;		 //秒
            GD2[7] = BD2[7] = Num12[miao2][row - 1] >> 1 | Num12[miao1][row - 1] << 6;
            break;
        }

        case 6:
        {


            if(shi1 == 0)
            {
                RD2[0] = GD2[0] = BD2[0] = 0;
                RD2[1] = GD2[1] = BD2[1] = Num14B[shi2][row1] >> 3;
            }
            else
            {
                RD2[0] = GD2[0] = BD2[0] = Num14B[shi1][row1];	 //时
                RD2[1] = GD2[1] = BD2[1] = Num14B[shi1][row2] | Num14B[shi2][row1] >> 3;
            }

            RD2[2] = GD2[2] = BD2[2] = Num14B[shi2][row1] << 5 | Num14B[shi2][row2] >> 3;
            RD2[3] = GD2[3] = BD2[3] = Num14B[fen1][row1] >> 2;	 // 分
            RD2[4] = GD2[4] = BD2[4] = Num14B[fen1][row1] << 6 | Num14B[fen1][row2] >> 2 | Num14B[fen2][row1] >> 5;
            RD2[5] = GD2[5] = BD2[5] = Num14B[fen2][row1] << 3 | Num14B[fen2][row2] >> 5;
            RD2[6] = GD2[6] = BD2[6] = Num12[miao1][row - 1] >> 2;		 //秒
            RD2[7] = GD2[7] = BD2[7] = Num12[miao2][row - 1] >> 1 | Num12[miao1][row - 1] << 6;
            break;
        }
    }

    if(PM)
    {
        RD2[0] |= Num12[15][row];
//        GD2[0] |= Num12[15][row];
//        BD2[0] |= Num12[15][row];
    }

    if(SHAN)
    {

        RD2[2] |= maohao[0][row];
        RD2[6] |= maohao[1][row];
    }

}

void Mode5_SJ_G()
{
    if(SHAN)
    {

        GD2[2] |= maohao[2][row];
        GD2[5] |= maohao[0][row];
    }
}


void Mode5_SJ_R()
{
    uchar row1;

    row1 = row - 1;		//	上1/2屏行扫描
//	row2=row-8;		//	下1/2屏行扫描

//    if(row)
//    {
    switch(COLORMODE)
    {
        case 0:
        {

            if(shi1 == 0) RD2[0] = 0;
            else
                RD2[0] = Num14[shi1][row1];	 //时

            RD2[1] = Num14[shi2][row1] >> 2;
            RD2[2] = Num14[shi2][row1] << 6;
            RD2[3] = Num14[fen1][row1] >> 2;	 // 分
            RD2[4] = Num14[fen2][row1] >> 4 | Num14[fen1][row1] << 6;
            RD2[5] = Num14[fen2][row1] << 4 ;
            RD2[6] = Num12[miao1][row1] >> 1;		 //秒
            RD2[7] = Num12[miao2][row1];
            break;
        }

        case 1:
        {
            if(shi1 == 0) GD2[0] = 0;
            else
                GD2[0] = Num14[shi1][row1];	 //时

            GD2[1] = Num14[shi2][row1] >> 2;
            GD2[2] = Num14[shi2][row1] << 6;
            GD2[3] = Num14[fen1][row1] >> 2;	 // 分
            GD2[4] = Num14[fen2][row1] >> 4 | Num14[fen1][row1] << 6;
            GD2[5] = Num14[fen2][row1] << 4 ;
            GD2[6] = Num12[miao1][row1] >> 1;		 //秒
            GD2[7] = Num12[miao2][row1];
            break;
        }

        case 2:
        {
            if(shi1 == 0) BD2[0] = 0;
            else
                BD2[0] = Num14[shi1][row1];	 //时

            BD2[1] = Num14[shi2][row1] >> 2;
            BD2[2] = Num14[shi2][row1] << 6;
            BD2[3] = Num14[fen1][row1] >> 2;	 // 分
            BD2[4] = Num14[fen2][row1] >> 4 | Num14[fen1][row1] << 6;
            BD2[5] = Num14[fen2][row1] << 4 ;
            BD2[6] = Num12[miao1][row1] >> 1;		 //秒
            BD2[7] = Num12[miao2][row1];
            break;
        }

        case 3:
        {
            if(shi1 == 0) RD2[0] = GD2[0] = 0;
            else
                RD2[0] = GD2[0] = Num14[shi1][row1];	 //时

            RD2[1] = GD2[1] = Num14[shi2][row1] >> 2;
            RD2[2] = GD2[2] = Num14[shi2][row1] << 6;
            RD2[3] = GD2[3] = Num14[fen1][row1] >> 2;	 // 分
            RD2[4] = GD2[4] = Num14[fen2][row1] >> 4 | Num14[fen1][row1] << 6;
            RD2[5] = GD2[5] = Num14[fen2][row1] << 4 ;
            RD2[6] = GD2[6] = Num12[miao1][row1] >> 1;		 //秒
            RD2[7] = GD2[7] = Num12[miao2][row1];
            break;
        }

        case 4:
        {
            if(shi1 == 0) RD2[0] = BD2[0] = 0;
            else
                RD2[0] = BD2[0] = Num14[shi1][row1];	 //时

            RD2[1] = BD2[1] = Num14[shi2][row1] >> 2;
            RD2[2] = BD2[2] = Num14[shi2][row1] << 6;
            RD2[3] = BD2[3] = Num14[fen1][row1] >> 2;	 // 分
            RD2[4] = BD2[4] = Num14[fen2][row1] >> 4 | Num14[fen1][row1] << 6;
            RD2[5] = BD2[5] = Num14[fen2][row1] << 4 ;
            RD2[6] = BD2[6] = Num12[miao1][row1] >> 1;		 //秒
            RD2[7] = BD2[7] = Num12[miao2][row1];
            break;
        }

        case 5:
        {
            if(shi1 == 0) BD2[0] = GD2[0] = 0;
            else
                BD2[0] = GD2[0] = Num14[shi1][row1];	 //时

            BD2[1] = GD2[1] = Num14[shi2][row1] >> 2;
            BD2[2] = GD2[2] = Num14[shi2][row1] << 6;
            BD2[3] = GD2[3] = Num14[fen1][row1] >> 2;	 // 分
            BD2[4] = GD2[4] = Num14[fen2][row1] >> 4 | Num14[fen1][row1] << 6;
            BD2[5] = GD2[5] = Num14[fen2][row1] << 4 ;
            BD2[6] = GD2[6] = Num12[miao1][row1] >> 1;		 //秒
            BD2[7] = GD2[7] = Num12[miao2][row1];
            break;
        }

        case 6:
        {
            if(shi1 == 0) RD2[0] = BD2[0] = GD2[0] = 0;
            else
                RD2[0] = BD2[0] = GD2[0] = Num14[shi1][row1];	 //时

            RD2[1] = BD2[1] = GD2[1] = Num14[shi2][row1] >> 2;
            RD2[2] = BD2[2] = GD2[2] = Num14[shi2][row1] << 6;
            RD2[3] = BD2[3] = GD2[3] = Num14[fen1][row1] >> 2;	 // 分
            RD2[4] = BD2[4] = GD2[4] = Num14[fen2][row1] >> 4 | Num14[fen1][row1] << 6;
            RD2[5] = BD2[5] = GD2[5] = Num14[fen2][row1] << 4 ;
            RD2[6] = BD2[6] = GD2[6] = Num12[miao1][row1] >> 1;		 //秒
            RD2[7] = BD2[7] = GD2[7] = Num12[miao2][row1];
            break;
        }
    }

    if(SHAN)
    {

        RD2[2] |= maohao[2][row];
        RD2[5] |= maohao[0][row];
    }

    if(PM)
        RD2[0] |= Num12[15][row];

//    }
}

void Mode6_SJ_G()
{

//秒点
    if(SHAN)
    {

        GD2[3] |= maohao[4][row];
        GD2[4] |= maohao[3][row];

    }
    else
        Delayus(30);

}

void Mode6_SJ_R()
{
    uchar  row3, row4;

    if(row)
    {
        ////时间
        row3 = (row - 1) * 2;
        row4 = (row - 1) * 2 + 1;

        switch(COLORMODE)
        {
            case 0:
            {

                if(shi1 == 0) 	 //
                    RD2[0] = RD2[1] = 0;
                else
                {
                    RD2[0] = Num16B[shi1][row3];	 //时
                    RD2[1] = Num16B[shi1][row4];
                }

                RD2[2] = Num16B[shi2][row3] << 3 | Num16B[shi2][row4] >> 5;
                RD2[3] = Num16B[shi2][row4] << 3;	 // 分
                RD2[4] = Num16B[fen1][row3] >> 3;
                RD2[5] = Num16B[fen1][row4] >> 3 | Num16B[fen1][row3] << 5;
                RD2[6] = Num16B[fen2][row3];		 //秒
                RD2[7] = Num16B[fen2][row4];
                break;
            }

            case 1:
            {

                if(shi1 == 0) 	 //
                    GD2[0] = GD2[1] = 0;
                else
                {
                    GD2[0] = Num16B[shi1][row3];	 //时
                    GD2[1] = Num16B[shi1][row4];
                }

                GD2[2] = Num16B[shi2][row3] << 3 | Num16B[shi2][row4] >> 5;
                GD2[3] = Num16B[shi2][row4] << 3;	 // 分
                GD2[4] = Num16B[fen1][row3] >> 3;
                GD2[5] = Num16B[fen1][row4] >> 3 | Num16B[fen1][row3] << 5;
                GD2[6] = Num16B[fen2][row3];		 //秒
                GD2[7] = Num16B[fen2][row4];
                break;
            }

            case 2:
            {

                if(shi1 == 0) 	 //
                    BD2[0] = BD2[1] = 0;
                else
                {
                    BD2[0] = Num16B[shi1][row3];	 //时
                    BD2[1] = Num16B[shi1][row4];
                }

                BD2[2] = Num16B[shi2][row3] << 3 | Num16B[shi2][row4] >> 5;
                BD2[3] = Num16B[shi2][row4] << 3;	 // 分
                BD2[4] = Num16B[fen1][row3] >> 3;
                BD2[5] = Num16B[fen1][row4] >> 3 | Num16B[fen1][row3] << 5;
                BD2[6] = Num16B[fen2][row3];		 //秒
                BD2[7] = Num16B[fen2][row4];
                break;
            }

            case 3:
            {

                if(shi1 == 0) 	 //
                    RD2[0] = RD2[1] = GD2[0] = GD2[1] = 0;
                else
                {
                    RD2[0] = GD2[0] = Num16B[shi1][row3];	 //时
                    RD2[1] = GD2[1] = Num16B[shi1][row4];
                }

                RD2[2] = GD2[2] = Num16B[shi2][row3] << 3 | Num16B[shi2][row4] >> 5;
                RD2[3] = GD2[3] = Num16B[shi2][row4] << 3;	 // 分
                RD2[4] = GD2[4] = Num16B[fen1][row3] >> 3;
                RD2[5] = GD2[5] = Num16B[fen1][row4] >> 3 | Num16B[fen1][row3] << 5;
                RD2[6] = GD2[6] = Num16B[fen2][row3];		 //秒
                RD2[7] = GD2[7] = Num16B[fen2][row4];
                break;
            }

            case 4:
            {

                if(shi1 == 0) 	 //
                    RD2[0] = RD2[1] = BD2[0] = BD2[1] = 0;
                else
                {
                    RD2[0] = BD2[0] = Num16B[shi1][row3];	 //时
                    RD2[1] = BD2[1] = Num16B[shi1][row4];
                }

                RD2[2] = BD2[2] = Num16B[shi2][row3] << 3 | Num16B[shi2][row4] >> 5;
                RD2[3] = BD2[3] = Num16B[shi2][row4] << 3;	 // 分
                RD2[4] = BD2[4] = Num16B[fen1][row3] >> 3;
                RD2[5] = BD2[5] = Num16B[fen1][row4] >> 3 | Num16B[fen1][row3] << 5;
                RD2[6] = BD2[6] = Num16B[fen2][row3];		 //秒
                RD2[7] = BD2[7] = Num16B[fen2][row4];
                break;
            }

            case 5:
            {


                if(shi1 == 0) 	 //
                    GD2[0] = GD2[1] = BD2[0] = BD2[1] = 0;
                else
                {
                    GD2[0] = BD2[0] = Num16B[shi1][row3];	 //时
                    GD2[1] = BD2[1] = Num16B[shi1][row4];
                }

                GD2[2] = BD2[2] = Num16B[shi2][row3] << 3 | Num16B[shi2][row4] >> 5;
                GD2[3] = BD2[3] = Num16B[shi2][row4] << 3;	 // 分
                GD2[4] = BD2[4] = Num16B[fen1][row3] >> 3;
                GD2[5] = BD2[5] = Num16B[fen1][row4] >> 3 | Num16B[fen1][row3] << 5;
                GD2[6] = BD2[6] = Num16B[fen2][row3];		 //秒
                GD2[7] = BD2[7] = Num16B[fen2][row4];
                break;
            }

            case 6:
            {

                if(shi1 == 0) 	 //
                    RD2[0] = RD2[1] = GD2[0] = GD2[1] = BD2[0] = BD2[1] = 0;
                else
                {
                    RD2[0] = GD2[0] = BD2[0] = Num16B[shi1][row3];	 //时
                    RD2[1] = BD2[1] = GD2[1] = BD2[1] = Num16B[shi1][row4];
                }

                RD2[2] = GD2[2] = BD2[2] = Num16B[shi2][row3] << 3 | Num16B[shi2][row4] >> 5;
                RD2[3] = GD2[3] = BD2[3] = Num16B[shi2][row4] << 3;	 // 分
                RD2[4] = GD2[4] = BD2[4] = Num16B[fen1][row3] >> 3;
                RD2[5] = GD2[5] = BD2[5] = Num16B[fen1][row4] >> 3 | Num16B[fen1][row3] << 5;
                RD2[6] = GD2[6] = BD2[6] = Num16B[fen2][row3];		 //秒
                RD2[7] = GD2[7] = BD2[7] = Num16B[fen2][row4];
                break;
            }
        }

        //秒点
        if(SHAN)
        {
            RD2[3] |= maohao[4][row];
            RD2[4] |= maohao[3][row];
        }
    }

    if(PM)
        RD2[0] |= Num12[15][row];
}


void Mode7_SJ_G()
{

//秒点
    if(SHAN)
    {

        GD2[3] |= maohao[4][row];
        GD2[4] |= maohao[3][row];

    }
    else
        Delayus(30);

}
void Mode7_SJ_R()
{
    uchar  row3, row4;

    if(row)
    {
        ////时间
        row3 = (row - 1) * 2;
        row4 = (row - 1) * 2 + 1;

        switch(COLORMODE)
        {
            case 0:
            {

                if(shi1 == 0) 	 //
                    RD2[0] = RD2[1] = 0;
                else
                {
                    RD2[0] = Num16[shi1][row3];	 //时
                    RD2[1] = Num16[shi1][row4];
                }

                RD2[2] = Num16[shi2][row3] << 3 | Num16[shi2][row4] >> 5;
                RD2[3] = Num16[shi2][row4] << 3;	 // 分
                RD2[4] = Num16[fen1][row3] >> 3;
                RD2[5] = Num16[fen1][row4] >> 3 | Num16[fen1][row3] << 5;
                RD2[6] = Num16[fen2][row3];		 //秒
                RD2[7] = Num16[fen2][row4];
                break;
            }

            case 1:
            {

                if(shi1 == 0) 	 //
                    GD2[0] = GD2[1] = 0;
                else
                {
                    GD2[0] = Num16[shi1][row3];	 //时
                    GD2[1] = Num16[shi1][row4];
                }

                GD2[2] = Num16[shi2][row3] << 3 | Num16[shi2][row4] >> 5;
                GD2[3] = Num16[shi2][row4] << 3;	 // 分
                GD2[4] = Num16[fen1][row3] >> 3;
                GD2[5] = Num16[fen1][row4] >> 3 | Num16[fen1][row3] << 5;
                GD2[6] = Num16[fen2][row3];		 //秒
                GD2[7] = Num16[fen2][row4];
                break;
            }

            case 2:
            {

                if(shi1 == 0) 	 //
                    BD2[0] = BD2[1] = 0;
                else
                {
                    BD2[0] = Num16[shi1][row3];	 //时
                    BD2[1] = Num16[shi1][row4];
                }

                BD2[2] = Num16[shi2][row3] << 3 | Num16[shi2][row4] >> 5;
                BD2[3] = Num16[shi2][row4] << 3;	 // 分
                BD2[4] = Num16[fen1][row3] >> 3;
                BD2[5] = Num16[fen1][row4] >> 3 | Num16[fen1][row3] << 5;
                BD2[6] = Num16[fen2][row3];		 //秒
                BD2[7] = Num16[fen2][row4];
                break;
            }

            case 3:
            {

                if(shi1 == 0) 	 //
                    RD2[0] = RD2[1] = GD2[0] = GD2[1] = 0;
                else
                {
                    RD2[0] = GD2[0] = Num16[shi1][row3];	 //时
                    RD2[1] = GD2[1] = Num16[shi1][row4];
                }

                RD2[2] = GD2[2] = Num16[shi2][row3] << 3 | Num16[shi2][row4] >> 5;
                RD2[3] = GD2[3] = Num16[shi2][row4] << 3;	 // 分
                RD2[4] = GD2[4] = Num16[fen1][row3] >> 3;
                RD2[5] = GD2[5] = Num16[fen1][row4] >> 3 | Num16[fen1][row3] << 5;
                RD2[6] = GD2[6] = Num16[fen2][row3];		 //秒
                RD2[7] = GD2[7] = Num16[fen2][row4];
                break;
            }

            case 4:
            {

                if(shi1 == 0) 	 //
                    RD2[0] = RD2[1] = BD2[0] = BD2[1] = 0;
                else
                {
                    RD2[0] = BD2[0] = Num16[shi1][row3];	 //时
                    RD2[1] = BD2[1] = Num16[shi1][row4];
                }

                RD2[2] = BD2[2] = Num16[shi2][row3] << 3 | Num16[shi2][row4] >> 5;
                RD2[3] = BD2[3] = Num16[shi2][row4] << 3;	 // 分
                RD2[4] = BD2[4] = Num16[fen1][row3] >> 3;
                RD2[5] = BD2[5] = Num16[fen1][row4] >> 3 | Num16[fen1][row3] << 5;
                RD2[6] = BD2[6] = Num16[fen2][row3];		 //秒
                RD2[7] = BD2[7] = Num16[fen2][row4];
                break;
            }

            case 5:
            {


                if(shi1 == 0) 	 //
                    GD2[0] = GD2[1] = BD2[0] = BD2[1] = 0;
                else
                {
                    GD2[0] = BD2[0] = Num16[shi1][row3];	 //时
                    GD2[1] = BD2[1] = Num16[shi1][row4];
                }

                GD2[2] = BD2[2] = Num16[shi2][row3] << 3 | Num16[shi2][row4] >> 5;
                GD2[3] = BD2[3] = Num16[shi2][row4] << 3;	 // 分
                GD2[4] = BD2[4] = Num16[fen1][row3] >> 3;
                GD2[5] = BD2[5] = Num16[fen1][row4] >> 3 | Num16[fen1][row3] << 5;
                GD2[6] = BD2[6] = Num16[fen2][row3];		 //秒
                GD2[7] = BD2[7] = Num16[fen2][row4];
                break;
            }

            case 6:
            {

                if(shi1 == 0) 	 //
                    RD2[0] = RD2[1] = GD2[0] = GD2[1] = BD2[0] = BD2[1] = 0;
                else
                {
                    RD2[0] = GD2[0] = BD2[0] = Num16[shi1][row3];	 //时
                    RD2[1] = BD2[1] = GD2[1] = BD2[1] = Num16[shi1][row4];
                }

                RD2[2] = GD2[2] = BD2[2] = Num16[shi2][row3] << 3 | Num16[shi2][row4] >> 5;
                RD2[3] = GD2[3] = BD2[3] = Num16[shi2][row4] << 3;	 // 分
                RD2[4] = GD2[4] = BD2[4] = Num16[fen1][row3] >> 3;
                RD2[5] = GD2[5] = BD2[5] = Num16[fen1][row4] >> 3 | Num16[fen1][row3] << 5;
                RD2[6] = GD2[6] = BD2[6] = Num16[fen2][row3];		 //秒
                RD2[7] = GD2[7] = BD2[7] = Num16[fen2][row4];
                break;
            }
        }

        //秒点
        if(SHAN)
        {
            RD2[3] |= maohao[4][row];
            RD2[4] |= maohao[3][row];
        }
    }

    if(PM)
        RD2[0] |= Num12[15][row];
}



void Mode8_SJ_G()
{
    if(SHAN)
    {

        GD1[2] |= maohao[0][row];
        GD1[6] |= maohao[1][row];
    }
}

void Mode8_SJ_R()
{
    uchar  row1, row2;
//	shi1=shi2=0;

    row1 = (row) * 2;		//	上1/2屏行扫描
    row2 = (row) * 2 + 1;		//	下1/2屏行扫描

    switch(COLORMODE)
    {
        case 0:
        {

            if(shi1 == 0)
            {
                RD1[0] = 0;
                RD1[1] = Num14B[shi2][row1] >> 3;
            }
            else
            {
                RD1[0] = Num14B[shi1][row1];	 //时
                RD1[1] = Num14B[shi1][row2] | Num14B[shi2][row1] >> 3;
            }

            RD1[2] = Num14B[shi2][row1] << 5 | Num14B[shi2][row2] >> 3;
            RD1[3] = Num14B[fen1][row1] >> 2;	 // 分
            RD1[4] = Num14B[fen1][row1] << 6 | Num14B[fen1][row2] >> 2 | Num14B[fen2][row1] >> 5;
            RD1[5] = Num14B[fen2][row1] << 3 | Num14B[fen2][row2] >> 5;
            RD1[6] = Num12[miao1][row - 1] >> 2;		 //秒
            RD1[7] = Num12[miao2][row - 1] >> 1 | Num12[miao1][row - 1] << 6;
            break;
        }

        case 1:
        {

            if(shi1 == 0)
            {
                GD1[0] = 0;
                GD1[1] = Num14B[shi2][row1] >> 3;
            }
            else
            {
                GD1[0] = Num14B[shi1][row1];	 //时
                GD1[1] = Num14B[shi1][row2] | Num14B[shi2][row1] >> 3;
            }

            GD1[2] = Num14B[shi2][row1] << 5 | Num14B[shi2][row2] >> 3;
            GD1[3] = Num14B[fen1][row1] >> 2;	 // 分
            GD1[4] = Num14B[fen1][row1] << 6 | Num14B[fen1][row2] >> 2 | Num14B[fen2][row1] >> 5;
            GD1[5] = Num14B[fen2][row1] << 3 | Num14B[fen2][row2] >> 5;
            GD1[6] = Num12[miao1][row - 1] >> 2;		 //秒
            GD1[7] = Num12[miao2][row - 1] >> 1 | Num12[miao1][row - 1] << 6;
            break;
        }

        case 2:
        {

            if(shi1 == 0)
            {
                BD1[0] = 0;
                BD1[1] = Num14B[shi2][row1] >> 3;
            }
            else
            {
                BD1[0] = Num14B[shi1][row1];	 //时
                BD1[1] = Num14B[shi1][row2] | Num14B[shi2][row1] >> 3;
            }

            BD1[2] = Num14B[shi2][row1] << 5 | Num14B[shi2][row2] >> 3;
            BD1[3] = Num14B[fen1][row1] >> 2;	 // 分
            BD1[4] = Num14B[fen1][row1] << 6 | Num14B[fen1][row2] >> 2 | Num14B[fen2][row1] >> 5;
            BD1[5] = Num14B[fen2][row1] << 3 | Num14B[fen2][row2] >> 5;
            BD1[6] = Num12[miao1][row - 1] >> 2;		 //秒
            BD1[7] = Num12[miao2][row - 1] >> 1 | Num12[miao1][row - 1] << 6;
            break;
        }

        case 3:
        {

            if(shi1 == 0)
            {
                RD1[0] = GD1[0] = 0;
                RD1[1] = GD1[1] = Num14B[shi2][row1] >> 3;
            }
            else
            {
                RD1[0] = GD1[0] = Num14B[shi1][row1];	 //时
                RD1[1] = GD1[1] = Num14B[shi1][row2] | Num14B[shi2][row1] >> 3;
            }

            RD1[2] = GD1[2] = Num14B[shi2][row1] << 5 | Num14B[shi2][row2] >> 3;
            RD1[3] = GD1[3] = Num14B[fen1][row1] >> 2;	 // 分
            RD1[4] = GD1[4] = Num14B[fen1][row1] << 6 | Num14B[fen1][row2] >> 2 | Num14B[fen2][row1] >> 5;
            RD1[5] = GD1[5] = Num14B[fen2][row1] << 3 | Num14B[fen2][row2] >> 5;
            RD1[6] = GD1[6] = Num12[miao1][row - 1] >> 2;		 //秒
            RD1[7] = GD1[7] = Num12[miao2][row - 1] >> 1 | Num12[miao1][row - 1] << 6;
            break;
        }

        case 4:
        {

            if(shi1 == 0)
            {
                RD1[0] = BD1[0] = 0;
                RD1[1] = BD1[1] = Num14B[shi2][row1] >> 3;
            }
            else
            {
                RD1[0] = BD1[0] = Num14B[shi1][row1];	 //时
                RD1[1] = BD1[1] = Num14B[shi1][row2] | Num14B[shi2][row1] >> 3;
            }

            RD1[2] = BD1[2] = Num14B[shi2][row1] << 5 | Num14B[shi2][row2] >> 3;
            RD1[3] = BD1[3] = Num14B[fen1][row1] >> 2;	 // 分
            RD1[4] = BD1[4] = Num14B[fen1][row1] << 6 | Num14B[fen1][row2] >> 2 | Num14B[fen2][row1] >> 5;
            RD1[5] = BD1[5] = Num14B[fen2][row1] << 3 | Num14B[fen2][row2] >> 5;
            RD1[6] = BD1[6] = Num12[miao1][row - 1] >> 2;		 //秒
            RD1[7] = BD1[7] = Num12[miao2][row - 1] >> 1 | Num12[miao1][row - 1] << 6;
            break;
        }

        case 5:
        {

            if(shi1 == 0)
            {
                GD1[0] = BD1[0] = 0;
                GD1[1] = BD1[1] = Num14B[shi2][row1] >> 3;
            }
            else
            {
                GD1[0] = BD1[0] = Num14B[shi1][row1];	 //时
                GD1[1] = BD1[1] = Num14B[shi1][row2] | Num14B[shi2][row1] >> 3;
            }

            GD1[2] = BD1[2] = Num14B[shi2][row1] << 5 | Num14B[shi2][row2] >> 3;
            GD1[3] = BD1[3] = Num14B[fen1][row1] >> 2;	 // 分
            GD1[4] = BD1[4] = Num14B[fen1][row1] << 6 | Num14B[fen1][row2] >> 2 | Num14B[fen2][row1] >> 5;
            GD1[5] = BD1[5] = Num14B[fen2][row1] << 3 | Num14B[fen2][row2] >> 5;
            GD1[6] = BD1[6] = Num12[miao1][row - 1] >> 2;		 //秒
            GD1[7] = BD1[7] = Num12[miao2][row - 1] >> 1 | Num12[miao1][row - 1] << 6;
            break;
        }

        case 6:
        {


            if(shi1 == 0)
            {
                RD1[0] = GD1[0] = BD1[0] = 0;
                RD1[1] = GD1[1] = BD1[1] = Num14B[shi2][row1] >> 3;
            }
            else
            {
                RD1[0] = GD1[0] = BD1[0] = Num14B[shi1][row1];	 //时
                RD1[1] = GD1[1] = BD1[1] = Num14B[shi1][row2] | Num14B[shi2][row1] >> 3;
            }

            RD1[2] = GD1[2] = BD1[2] = Num14B[shi2][row1] << 5 | Num14B[shi2][row2] >> 3;
            RD1[3] = GD1[3] = BD1[3] = Num14B[fen1][row1] >> 2;	 // 分
            RD1[4] = GD1[4] = BD1[4] = Num14B[fen1][row1] << 6 | Num14B[fen1][row2] >> 2 | Num14B[fen2][row1] >> 5;
            RD1[5] = GD1[5] = BD1[5] = Num14B[fen2][row1] << 3 | Num14B[fen2][row2] >> 5;
            RD1[6] = GD1[6] = BD1[6] = Num12[miao1][row - 1] >> 2;		 //秒
            RD1[7] = GD1[7] = BD1[7] = Num12[miao2][row - 1] >> 1 | Num12[miao1][row - 1] << 6;
            break;
        }
    }

    if(SHAN)
    {
        RD1[2] |= maohao[0][row];
        RD1[6] |= maohao[1][row];
    }

    if(PM)
        RD1[0] |= Num12[15][row];
}

void Mode9_SJ_R()	   //细体时间 有秒
{
    if(!shi1) RD1[0] = 0;
    else
        RD1[0] = Num14[shi1][row];	 //时

    RD1[1] = Num14[shi2][row] >> 2;
    RD1[2] = Num14[shi2][row] << 6;
    RD1[3] = Num14[fen1][row] >> 2;	 // 分
    RD1[4] = Num14[fen2][row] >> 4 | Num14[fen1][row] << 6;
    RD1[5] = Num14[fen2][row] << 4 ;
    RD1[6] = Num12[miao1][row] >> 1;		 //秒
    RD1[7] = Num12[miao2][row] >> 1;

//	RD1[6]=Num12[z/100][row]>>1;		   //秒
//	RD1[7]=Num12[z%100/10][row]>>1;
    if(PM)
        RD1[0] |= Num12[15][row];

    if(SHAN)
    {
        RD1[2] |= maohao[2][row];
        RD1[5] |= maohao[0][row];
    }
}


void Mode9_SJ_G()
{
    if(SHAN)
    {
        GD1[2] |= maohao[2][row];
        GD1[5] |= maohao[0][row];

    }
}
void Mode10_SJ_G()
{

//秒点
    if(SHAN)
    {

        GD1[3] |= maohao[4][row];
        GD1[4] |= maohao[3][row];

    }
    else
        Delayus(30);

}

void Mode10_SJ_R()
{
    uchar row3, row4;

    ////时间
    row3 = row * 2;
    row4 = row * 2 + 1;

    if(shi1 == 0) 	 //
        RD1[0] = RD1[1] = 0;
    else
    {
        RD1[0] = Num16B[shi1][row3];	 //时
        RD1[1] = Num16B[shi1][row4];
    }

    RD1[2] = Num16B[shi2][row3] << 3 | Num16B[shi2][row4] >> 5;
    RD1[3] = Num16B[shi2][row4] << 3;	 // 分
    RD1[4] = Num16B[fen1][row3] >> 3;
    RD1[5] = Num16B[fen1][row4] >> 3 | Num16B[fen1][row3] << 5;
    RD1[6] = Num16B[fen2][row3];		 //秒
    RD1[7] = Num16B[fen2][row4];

    //秒点
    if(SHAN)
    {
        RD1[3] |= maohao[4][row];
        RD1[4] |= maohao[3][row];
    }

    if(PM)
        RD1[0] |= Num12[15][row];
}

void Mode11_SJ_G()
{

//秒点
    if(SHAN)
    {

        GD1[3] |= maohao[4][row];
        GD1[4] |= maohao[3][row];

    }
    else
        Delayus(30);

}

void Mode11_SJ_R()
{
    uchar row3, row4;

    ////时间
    row3 = row * 2;
    row4 = row * 2 + 1;

    if(shi1 == 0) 	 //
        RD1[0] = RD1[1] = 0;
    else
    {
        RD1[0] = Num16[shi1][row3];	 //时
        RD1[1] = Num16[shi1][row4];
    }

    RD1[2] = Num16[shi2][row3] << 3 | Num16[shi2][row4] >> 5;
    RD1[3] = Num16[shi2][row4] << 3;	 // 分
    RD1[4] = Num16[fen1][row3] >> 3;
    RD1[5] = Num16[fen1][row4] >> 3 | Num16[fen1][row3] << 5;
    RD1[6] = Num16[fen2][row3];		 //秒
    RD1[7] = Num16[fen2][row4];

    //秒点
    if(SHAN)
    {
        RD1[3] |= maohao[4][row];
        RD1[4] |= maohao[3][row];
    }
    else
    {


    }

    if(PM)
        RD1[0] |= Num12[15][row];
}



void Mode8_RQ_G()
{
    uchar row1 = row * 2;
    uchar row2 = row * 2 + 1;

    if(!yue1) GD1[0] = 0;
    else
        GD2[0] = Num13[yue1][row];	 //月十位

    GD2[1] = Num13[yue2][row];	 //月各位
    GD2[2] = HZ[19][row1];	//汉字月
    GD2[3] = HZ[19][row2] ;
    GD2[4] = Num13[ri1][row];	 //日十位
    GD2[5] = Num13[ri2][row];	 //日个位
    GD2[6] = HZ[20][row1];	 //汉字日
    GD2[7] = HZ[20][row2];
}

void Mode8_NL_G()		//农历显示
{
    uchar row1 = row * 2;
    uchar row2 = row * 2 + 1;

    GD2[0] = HZ[NL_yue][row1]; //月十位
    GD2[1] = HZ[NL_yue][row2];
    GD2[2] = HZ[14][row1];	//月
    GD2[3] = HZ[14][row2];	//日个位
    GD2[4] = NLday[r1][row1];
    GD2[5] = NLday[r1][row2];	//日十位
    GD2[6] = HZ[r2][row1];
    GD2[7] = HZ[r2][row2];

}


void Mode8_XQ_G()
{
    uchar row1 = row * 2;
    uchar row2 = row * 2 + 1;

    GD2[0] = HZ[13][row1];
    GD2[1] = HZ[13][row2];
    GD2[2] = HZ[week][row1];
    GD2[3] = HZ[week][row2];
    GD2[4] = Num13[t1][row] >> 5;
    GD2[5] = Num13[t1][row] << 3 | Num13[t2][row] >> 6;
    GD2[6] = Num13[t2][row] << 2 ;
    GD2[7] = Num12[10][row];
}


void Mode8_XQ_R()
{

    RD2[4] = Num13[t1][row] >> 5;
    RD2[5] = Num13[t1][row] << 3 | Num13[t2][row] >> 6;
    RD2[6] = Num13[t2][row] << 2 ;

}


void ModeSleep_R()
{
    if(PM)
        RD1[1] |= Num12[15][row];

    if(shi1)
        RD1[2] = Num9[shi1][row];

    RD1[3] = Num9[shi2][row];

    if(SHAN)
        RD1[4] = Num9[10][row];

    RD1[5] = Num9[fen1][row];
    RD1[6] = Num9[fen2][row];

    RD2[3] = Num9[t1][row];
    RD2[4] = Num9[t2][row];
    RD2[5] = Num12[10][row] >> 4;
    RD2[6] = Num12[10][row] << 4;

}

void UartInit(void)		//9600bps@24.576MHz
{
    PCON &= 0x7F;		//波特率不倍速
    SCON = 0x50;		//8位数据,可变波特率
    AUXR |= 0x04;		//独立波特率发生器时钟为Fosc,即1T
//	BRT = BRT_INT;		//设定独立波特率发生器重装值
    AUXR |= 0x01;		//串口1选择独立波特率发生器为波特率发生器
    AUXR |= 0x10;		//启动独立波特率发生器

}


void LoadData_G()
{

    switch (Set_S)
    {
        case 0:
            if (POW_OK)
            {
                switch (XSMOD)
                {
                    case 0:
                        Mode0123_RQ_G();
                        Mode0_SJ_G();
                        break;

                    case 1:
                        Mode1_SJ_RGB();
                        break;

                    case 2:
                        Mode0123_RQ_G();
                        Mode2_SJ_G();
                        break;

                    case 3:
                        Mode0123_RQ_G();
                        Mode3_SJ_G();
                        break;

                    case 4:
                        Mode4567_RQ_G();
                        Mode4_SJ_G();
                        break;

                    case 5:
                        Mode4567_RQ_G();
                        Mode5_SJ_G();
                        break;

                    case 6:
                        Mode4567_RQ_G();
                        Mode6_SJ_G();
                        break;

                    case 7:
                        Mode4567_RQ_G();
                        Mode7_SJ_G();
                        break;

                    case 8:
                        Mode8_SJ_G();

                        if (miao2 < 3)
                            Mode8_RQ_G();
                        else if (miao2 < 6)
                            Mode8_NL_G();
                        else
                            Mode8_XQ_G();

                        break;

                    case 9:
                        Mode9_SJ_G();

                        if (miao2 < 3)
                            Mode8_RQ_G();
                        else if (miao2 < 6)
                            Mode8_NL_G();
                        else
                            Mode8_XQ_G();

                        break;

                    case 10:
                        Mode10_SJ_G();

                        if (miao2 < 3)
                            Mode8_RQ_G();
                        else if (miao2 < 6)
                            Mode8_NL_G();
                        else
                            Mode8_XQ_G();

                        break;

                    case 11:
                        Mode11_SJ_G();

                        if (miao2 < 3)
                            Mode8_RQ_G();
                        else if (miao2 < 6)
                            Mode8_NL_G();
                        else
                            Mode8_XQ_G();

                        break;
                }
            }
            else
                Delay(60);

            break;

        case 1:
            SetNian_G();
            break;

        case 2:
            SetYue_G();
            break;

        case 3:
            SetRi_G();
            break;

        case 4:
            SetShi_G();
            break;

        case 5:
            SetFen_G();
            break;

        case 6:
            SetMiao_G();
            break;

        case 7:
            SetLiangdu_G();
            break;;

        case 8:
            SetMinLiangdu_G();
            break;

        case 9:
            SetLiangduG_G();
            break;

        case 10:
            NTCbuchang_G();
            break;

        case 11:
            Yaokong_G();
            break;

        case 12:
        case 13:
        case 14:
        case 15:
            Xuexi_G();
            break;
    }
}


void LoadData_R()
{

    switch (Set_S)
    {
        case 0:
            if (POW_OK)
            {
                switch (XSMOD)
                {
                    case 0:
                        Mode0_SJ_RGB();
                        break;

                    case 1:
                        Mode1_SJ_RGB();
                        break;

                    case 2:
                        Mode0123_RQ_R();
                        Mode2_SJ_R();
                        break;

                    case 3:
                        Mode0123_RQ_R();
                        Mode3_SJ_R();
                        break;

                    case 4:
                        Mode4567_RQ_R();
                        Mode4_SJ_R();
                        break;

                    case 5:
                        Mode4567_RQ_R();
                        Mode5_SJ_R();
                        break;

                    case 6:
                        Mode4567_RQ_R();
                        Mode6_SJ_R();
                        break;

                    case 7:
                        Mode4567_RQ_R();
                        Mode7_SJ_R();
                        break;

                    case 8:
                        Mode8_SJ_R();

                        if (miao2 > 5)
                            Mode8_XQ_R();

                        break;

                    case 9:
                        Mode9_SJ_R();

                        if (miao2 > 5)
                            Mode8_XQ_R();

                        break;

                    case 10:
                        Mode10_SJ_R();

                        if (miao2 > 5)
                            Mode8_XQ_R();

                        break;

                    case 11:
                        Mode11_SJ_R();

                        if (miao2 > 5)
                            Mode8_XQ_R();

                        break;

                }
            }
            else
                ModeSleep_R();

            break;

        case 1:
            SetNian_R();
            break;

        case 2:
            SetYue_R();
            break;

        case 3:
            SetRi_R();
            break;

        case 4:
            SetShi_R();
            break;

        case 5:
            SetFen_R();
            break;

        case 6:
            SetMiao_R();
            break;

        case 7:
            SetLiangdu_R();
            break;

        case 8:
            SetMinLiangdu_R();
            break;

        case 9:
            SetLiangduG_R();
            break;

        case 10:
            NTCbuchang_R();
            break;

        case 11:
            Yaokong_R();
            break;

        case 12:
        case 13:
        case 14:
        case 15:
            Xuexi_R();
            break;
    }
}

void SendData_G()
{
    uchar i, j;

    for(i = 0; i < 8; i++)					 //扫描1行
    {
        #ifdef DATAHI
        R1 = 0;
        R2 = 0;

        for(j = 0; j < 8; j++)			 //发送1个数据
        {

            GD1[i] <<= 1;	//数据左移1
            G1 = CY;		//发送最高位，
            GD2[i] <<= 1;
            G2 = CY;		//由于数据是低电平有效，所以进行取反
            CK = 0;					//时钟下降
            NOP4;
            CK = 1;	 //时钟上升
        }

        #else
        R1 = 1;
        R2 = 1;

        for(j = 0; j < 8; j++)
        {
            GD1[i] <<= 1;	//数据左移1
            G1 = !CY;		//发送最高位，
            GD2[i] <<= 1;
            G2 = !CY;		//由于数据是低电平有效，所以进行取反
            CK = 0;		//时钟下降
            NOP4;
            CK = 1;	//时钟上升
        }

        #endif

    }

    //ABCD=row;			//行选
    scan(row);
    LS = 0;
    NOP4;
    LS = 1;				//锁存上升，显示输出
    EN_ON();				//显示开
    TH0 = Light_G;
    TL0 = Light_L;
    TR0 = 1;
}


void SendData_R()
{
    uchar i, j;

    for(i = 0; i < 8; i++)					 //扫描1行
    {
        #ifdef DATAHI
        G1 = 0;
        G2 = 0;

        for(j = 0; j < 8; j++)			 //发送1个红数据
        {

            RD1[i] <<= 1;	//数据左移1
            R1 = CY;		//发送最高位，
            RD2[i] <<= 1;
            R2 = CY;		//由于数据是低电平有效，所以进行取反
            CK = 0;		//时钟下降
            NOP4;
            CK = 1;	 //时钟上升
        }

//        for(j=0; j<8; j++)			 //发送1个蓝色数据
//        {

//            BD1[i]<<=1;	//数据左移1
//            B1=CY;		//发送最高位，
//            BD2[i]<<=1;
//            B2=CY;		//由于数据是低电平有效，所以进行取反
//            CK=0;		//时钟下降
//            NOP4;
//            CK=1;	  //时钟上升
//        }
        #else
        G1 = 1;
        G2 = 1;

        for(j = 0; j < 8; j++)
        {
            RD1[i] <<= 1;	//数据左移1
            R1 = !CY;		//发送最高位，
            RD2[i] <<= 1;
            R2 = !CY;		//由于数据是低电平有效，所以进行取反
            CK = 0;		//时钟下降
            NOP4;
            CK = 1;	//时钟上升
        }

//        for(j=0; j<8; j++)
//        {
//            BD1[i]<<=1;	//数据左移1
//            B1=!CY;		//发送最高位，
//            BD2[i]<<=1;
//            B2=!CY;		//由于数据是低电平有效，所以进行取反
//            CK=0;		//时钟下降
//            NOP4;
//            CK=1;	//时钟上升
//        }
        #endif
    }

    //ABCD=row;			//行选
    scan(row);
    LS = 0;
    NOP4;
    LS = 1;				//锁存上升，显示输出
    EN_ON();				//显示开
//		Delay(80);
    TH0 = Light_R;
    TL0 = Light_L;
    TR0 = 1;
//		EN_OFF();			    //显示关
//		Delay(1);
}
void  SendDate_RGB()
{
    uchar i, j;

    for(i = 0; i < 8; i++)					 //扫描1行
    {
        for(j = 0; j < 8; j++)			 //发送1个数据
        {
            #ifdef	DATAHI
            RD1[i] <<= 1;				//数据左移1	  如是逆向取模则改为右移>>
            R1 = CY;		//发送最高位，
            RD2[i] <<= 1;
            R2 = CY;
            GD1[i] <<= 1;
            G1 = CY;
            GD2[i] <<= 1;
            G2 = CY;
            BD1[i] <<= 1;
            B1 = CY;
            BD2[i] <<= 1;
            B2 = CY;
            CK = 0;					//时钟下降
            NOP4;
            CK = 1;				 //时钟上升
            #else
            RD1[i] <<= 1;				//数据左移1	  如是逆向取模则改为右移>>
            R1 = !CY;		//发送最高位，
            RD2[i] <<= 1;
            R2 = !CY;		//由于数据是低电平有效，所以进行取反
            GD1[i] <<= 1;
            G1 = !CY;
            GD2[i] <<= 1;
            G2 = !CY;
            BD1[i] <<= 1;
            B1 = !CY;
            BD2[i] <<= 1;
            B2 = !CY;
            CK = 0;					//时钟下降
            NOP4;
            CK = 1;				 //时钟上升
            #endif
        }
    }

    scan(row);
    LS = 0;
    NOP4;
    LS = 1;				//锁存上升，显示输出
    EN_ON();				//显示开
//		Delay(80);
    TH0 = Light_R;
    TL0 = Light_L;
    TR0 = 1;
//		EN_OFF();			    //显示关
//		Delay(1);
}

void Err3231()
{

    for(row = 0; row < 16; row++)
    {
        RD1[0] = ZMlogo[10][row];
        RD1[1] = ZMlogo[2][row];
        RD1[2] = Num12[3][row];
        RD1[3] = Num12[2][row];
        RD1[4] = Num12[3][row];
        RD1[5] = Num12[1][row];
//		RD1[6]=Num12[1][row];

        RD2[0] = HZ[44][row * 2];
        RD2[1] = HZ[44][row * 2 + 1];
        RD2[2] = HZ[45][row * 2];
        RD2[3] = HZ[45][row * 2 + 1];
        RD2[4] = HZ[46][row * 2];
        RD2[5] = HZ[46][row * 2 + 1];
        RD2[6] = HZ[47][row * 2];
        RD2[7] = HZ[47][row * 2 + 1];

        SendData_G();
        SendData_R();
    }

}

void xianshi()
{
//	uchar i,j;

    for(row = 0; row < 16; row++)			 //扫描16行
    {
        LoadData_R();
//        SendData_R();
        LoadData_G();
//        SendData_G();

        SendDate_RGB();
    }
}


void TIM0() interrupt 1
{
//	z++;
    EN_OFF();
    TR0 = 0;
    TL0 = 60;
}

void SetTime(uchar address, char min, char max)
{
    char item;

    item = read_random(address);
    item = item / 16 * 10 + item % 16;

    if (K_ADD)
        item++;
    else
        item--;

    if (item > max)
        item = min;

    if (item < min)
        item = max;

    ModifyTime(address, item / 10 * 16 + item % 10); // 写入DS3231
    GetAllTime();
}


void IR_ICON()
{
    if(IR_date[6])
    {
        if(Set_S < 12)
        {
            if(IR_date[0] == IR_Key[0])
            {
                if(IR_date[6] == IR_Key[1])
                {
                    K_SET = 1;
                    IR_date[4] = 0;
                }

                if(IR_date[6] == IR_Key[2])
                {
                    K_ADD = 1;
                    IR_date[4] = 3;
                }

                if(IR_date[6] == IR_Key[3])
                {
                    K_DEC = 1;
                    IR_date[4] = 3;
                }

                if(IR_date[6] == IR_Key[4])
                {
                    K_ESC = 1;
                    IR_date[4] = 0;
                }
            }


//			switch(IR_date[6])
//			{
//				case IR_Key[1]:	K_SET=1;  IR_date[4]=0;	break;
//				case IR_Key[2]:	K_ADD=1;  IR_date[4]=3;	break;
//				case IR_Key[3]:	K_DEC=1;  IR_date[4]=3;	break;
//				case IR_Key[4]:	K_ESC=1;  IR_date[4]=0;	break;
//
//				default : break;
//			}
        }
        else
        {
            if(!xuexi_OK)
            {
                switch(Set_S)
                {
                    case 12:
                        IR_Key[0] = IR_date[0];
                        IR_Key[1] = IR_date[6];
                        //	IR_date[4]=0;
                        Write_EEP();
                        xuexi_OK = 1;
                        break;

                    case 13:
                        IR_Key[2] = IR_date[6];
                        //	IR_date[4]=0;
                        Write_EEP();
                        xuexi_OK = 1;
                        break;

                    case 14:
                        IR_Key[3] = IR_date[6];
                        //	IR_date[4]=0;
                        Write_EEP();
                        xuexi_OK = 1;
                        break;

                    case 15:
                        IR_Key[4] = IR_date[6];
                        //	IR_date[4]=0;
                        Write_EEP();
                        xuexi_OK = 1;
                        break;

                    default:
                        break;

                }
            }
        }

        IR_date[6] = 0;
    }
}

void KeyScan()	   //*按键扫描 */
{
    static bit KeySet_S, KeyAdd_S, KeyDec_S, KeyEsc_S;        //相应按键按下状态指示
    static uint idata Keyout, SaveTime;




    if(KeySet)            //设置按键,
        KeySet_S = 1; //

    if(!KeySet && KeySet_S)
    {
        xianshi();

        if(!KeySet)
        {
            K_SET = 1;
            KeySet_S = 0;
            Keyout = 0;
        }
    }

    if(KeyAdd) 		   //+按键,
        KeyAdd_S = 1;

    if(!KeyAdd && KeyAdd_S)
    {
        xianshi();

        if(!KeyAdd)
        {
            K_ADD = 1;		 //+键按下后，+=1
            KeyAdd_S = 0;
            Keyout = 0;
        }
    }

    if(KeyEsc) 		   //Esc按键,
        KeyEsc_S = 1;

    if(!KeyEsc && KeyEsc_S)
    {
        xianshi();

        if(!KeyEsc)
        {
            K_ESC = 1;		 //Esc键按下后，+=1
            KeyEsc_S = 0;
            Keyout = 0;
        }
    }

    if(KeyDec)
        KeyDec_S = 1; //-按键,

    if(!KeyDec && KeyDec_S)
    {
        xianshi();

        if(!KeyDec)
        {
            K_DEC = 1;		 //减键按下后，-键=1
            KeyDec_S = 0;
            Keyout = 0;

        }
    }

    IR_ICON();

    if(K_SET)
    {
        SaveTime = 1000;
        WriteEpprom_S = 1;            //键按下后，1分钟存一次，并只存一次。

        Set_S++;			//S键按下后，

        if(Set_S > Set_Max)
            Set_S = 0;


        K_SET = 0;
        xuexi_OK = 0;
    }

    if(Set_S)
    {
        Keyout++;

        if(Keyout > 10000)  		//1分后退出调整状态
            Keyout = 0, Set_S = 0;
    }

    if(K_ADD && K_DEC)
    {
        NOP;
    }

    if(K_ADD | K_DEC)
    {
        SaveTime = 1000;
        WriteEpprom_S = 1;            //键按下后，1分钟存一次，并只存一次。

        switch(Set_S)
        {
            case 0:
                if(K_ADD)
                {
                    XSMOD++;

                    if(XSMOD > XSMOD_Max)
                        XSMOD = 0;

                }
                else
                {
                    Zhi_12 = !Zhi_12;
                    GetAllTime();
                }

                break;

            case 1:
                SetTime(DS3231_YEAR, 15, 79);
                break;	//年

            case 2:
                SetTime(DS3231_MONTH, 1, 12);
                break;	//月

            case 3:
                SetTime(DS3231_DAY, 1, 31);
                break;  	//日

            case 4:
                SetTime(DS3231_HOUR, 0, 23);
                break;	//时

            case 5:
                SetTime(DS3231_MINUTE, 0, 59);
                break;	//分

            case 6:
                SetTime(DS3231_SECOND, 0, 59);
                break;	//秒

            case 7:
                if(K_ADD) Liangdu++;
                else Liangdu--;

                if(Liangdu > Liangdu_Max) Liangdu = 0;

                if(Liangdu < 0) Liangdu = Liangdu_Max;

                break;

            case 8:
                if(K_ADD) Min_Liangdu++;
                else Min_Liangdu--;

                if(Min_Liangdu > Min_Liangdu_Max) Min_Liangdu = 0;

                if(Min_Liangdu < 0) Min_Liangdu = Min_Liangdu_Max;

                break;

            case 9:
                if(K_ADD) Liangdu_G++;
                else Liangdu_G--;

                if(Liangdu_G > Liangdu_G_Max) Liangdu_G = 0;

                if(Liangdu_G < 0) Liangdu_G = Liangdu_G_Max;

                break;

            case 10:
                if(K_ADD) NTC_buchang++;
                else NTC_buchang--;

                if(NTC_buchang > 19) NTC_buchang = 19;

                if(NTC_buchang < 1) NTC_buchang = 1;

                break;

            case 11:
                IR_POW = !IR_POW;


            default :
                break;
        }

        K_ADD = 0;
        K_DEC = 0;
    }

    if(K_ESC)
    {

        if(Set_S == 0)
        {

            COLORMODE++;

            if (COLORMODE > 6)  COLORMODE = 0;

            SaveTime = 1000;
            WriteEpprom_S = 1;            //正常状态下按ESC键改颜色后，半分钟存一次，并只存一次。

        }
        else
        {
            Set_S = 0;

        }

        K_ESC = 0;
    }

    if( SaveTime > 0)  SaveTime--;      //检测任何按键按下后，1分钟后存参数一次，并只存一次。

    if (WriteEpprom_S == 1 && SaveTime == 0)
    {
        EN_OFF();   //先关显示，否则屏闪。
        Write_canshi(); //退出并存参数
        EN_ON();
        WriteEpprom_S = 0;   //清零写EEROM状态位
    }
}

void main()
{
    uchar idata count;
    uchar idata a, b;
    P4SW = 0XFF;
    P0M1 = 0x00;	 //以下为接LED点阵屏设为强推，适应有下拉或上拉电阻输入的屏,改引脚连接要先设引脚输出模式
    P0M0 = 0xff;
    P2M1 = 0x00;
    P2M0 = 0xf0;
    P3M1 = 0x00;
    P3M0 = 0x10;
    P4M1 = 0x00;
    P4M0 = 0x10;


    P1M1 = 0x18;			 //P13,P14高阻模式	,0001 1000
    P1ASF = 0x18;			 //P13,P14做ADC输入

    B1 = 0;
    B2 = 0;
    EN_OFF();			    //显示关
    ADC_CONTR = 0x80;	//开启ADC电源
    UartInit();			 //串口初始化
    TMOD = 0x11;			 //16bit
    AUXR |= 0x80;		//	T0/ 1T
    EA = 1;
    ES = 1;
    PS = 1;
    ET0 = 1;
    EADC = 1;	//开启ADC中断
    IT1 = 1;	//int1下降沿触发
    EX1 = 1;
    ET1 = 1;
    TR1 = 1;

    if(DS18B20_TEST())
    {
        Delay(10);

        if(DS18B20_TEST())
            NTC = 1;
        else
            NTC = 0;
    }
    else
        NTC = 0;

    ADCL = 1;
    ADC_CONTR = 0xcb;		 //进行亮度AD转换

    if(NTC)
    {
        while(ADCL);

        ADCT = 1;
        ADC_CONTR = 0xcc;		 //进行温度
    }
    else
        Read18B20();

    GPS_POW = 0;
    WDT_CONTR = 0x34;		 //启动看门狗，16分频，约1s多
    DS3231_Initial();		//根据DS3231状态是否进行初始化


    ADCL = 1;
    ADC_CONTR = 0xcb;		 //进行亮度AD转换

    if(NTC)
    {
        while(ADCL);

        ADCT = 1;
        ADC_CONTR = 0xcc;		 //进行温度
    }
    else
        Read18B20();

    while(read_random(DS3231_A2D) != 1)  //如DS3231闹铃2的日期位标志不是原来的1,初始化。
    {
        Err3231();
        WDT_CONTR = 0x34;
    }

    POW_OK = 1;

    if(Byte_Read(600) != 0xfd)
    {
        XSMOD = 0; //从EEROM中读显示模式
        Zhi_12 = 0; //读AM ，PM标志
        Liangdu = 2; //读亮度数据
        Min_Liangdu = 0; //读最低亮度数据
        Liangdu_G = 0; //读绿补数据
        NTC_buchang = 0; //读NTC温度补偿数据。
        IR_POW = 1; //读红外开关数据
        COLORMODE = 0; //读颜色模式数据
        Write_canshi();   //写初始参数

    }
    else
    {

        XSMOD = Byte_Read(601); //从EEROM中读显示模式
        Zhi_12 = Byte_Read(602); //读AM ，PM标志
        Liangdu = Byte_Read(603); //读亮度数据
        Min_Liangdu = Byte_Read(604); //读最低亮度数据
        Liangdu_G = Byte_Read(605); //读绿补数据
        NTC_buchang = Byte_Read(606); //读NTC温度补偿数据。
        IR_POW = Byte_Read(607); //读红外开关数据
        COLORMODE = Byte_Read(608); //读颜色模式数据

        if(XSMOD > XSMOD_Max)		XSMOD = 0;
    }

    ShowLogo();

    Read_EEP();			 //从单片机EEROM中读红外键位数据
    GetAllTime();		//读取全部时间，并进行农历运算

    while(1)
    {
        count++;

        if(count > 32)				//这个常数值根据晶振频率调整，使秒点闪1HZ
        {
            count = 0;
            a++;
            ADCL = 1;
            ADC_CONTR = 0xcb;			 //进行亮度AD转换
            WDT_CONTR |= 0x10;		 //喂狗
            GetTime();				   //读取时间

            if(Liangdu_G == 0)        //颜色转换模式为0(自动)时，10分钟换一次颜色。
            {
                if(Colortime != fen1)
                {
                    Colortime = fen1;
                    COLORMODE++;

                    if(COLORMODE > 6) COLORMODE = 0;
                }

            }

            if(a > 1)
            {
                a = 0;
                b++;
                SHAN2 = !SHAN2;
                SHAN = !SHAN;

                if(b > 1)
                {
                    b = 0;

//                   SHAN=!SHAN;
                    if(NTC)
                    {
                        while(ADCL);

                        ADCT = 1;
                        ADC_CONTR = 0xcc;		 //进行温度
                    }
                    else
                        Read18B20();
                }
            }
        }


        KeyScan();
        xianshi();
    }
}


void int1() interrupt 2 using 3
{
    static uchar a, b;
    uchar temp;
    EX1 = 0;
    TR1 = 0;
    temp = TH1;                       //将TH1数读入temp
    TH1 = 0;
    TL1 = 0;
    TR1 = 1;                         //启动定时器0

    switch (IR_date[4])
    {
        case 0: 		//开外部中断，允许新的遥控按键
            IR_date[4] = 1;
            a = 0, b = 0;
            break;

        case 1:                          //启动码判定
            if (temp < (0x64) || temp > (0x6f)) //如果周期小于6400h 12.8MS或大于6F00 14.2MS,错误退出
                IR_date[4] = 0xFF;
            else
                IR_date[4] = 2;

            break;


        case 2:                        //0 1数字判定
            if (temp < 0x08 || temp > 0x14) //如果周期小于800H 1.024MS或大于1400H 2.56MS,错误退出
                IR_date[4] = 0xFF;
            else
            {
                IR_date[a] = IR_date[a] >> 1; //接收一位数据

                if (temp > 0x0E)          //如果时间长于1.792,则为1
                    IR_date[a] |= 0x80;
                else
                    IR_date[a] &= 0x7F;

                b++;

                if (b > 7)
                {
                    a++;
                    b = 0;  //一字节数据接收完毕判断
                }

                if (a > 3)
                {
                    if (IR_date[2] == ~IR_date[3]) //判断按键码
                    {
                        IR_date[6] = IR_date[2];
                        IR_date[5] = 1;
                        IR_date[4] = 0x03;
                    }
                    else
                        IR_date[4] = 0xFF;
                }
            }

            break;

        default:
            if (IR_date[5])
            {
                IR_date[5] = 0;      //连发次数清零
                IR_date[4] = IR_date[4] + 1;		//如果产生了时间中断，连发状态位加一
            }

            if (IR_date[4] > 10) //连发次数
            {
                IR_date[4] = 3;
                IR_date[6] = IR_date[2];
            }

            break;
    }

    if (IR_date[4] == 0xFF)         //如果有错误，关闭定时0，
    {
        IR_date[4] = 0;
        IR_date[5] = 0;
        TR1 = 0;
        TH1 = 0;
        TL1 = 0;
    }

    EX1 = 1;                      //重开外中断允许
}


// 红外遥控超时处理

void time1() interrupt 3
{
    TR1 = 1;
    IR_date[5] = IR_date[5] + 1;

    if(IR_date[4] < 3 || IR_date[5] > 2) //如果不是连发而发生了中断或中断次数大于4次，判定为错误
    {
        IR_date[4] = 0;
        IR_date[5] = 0;
        TR1 = 0;
        TH1 = 0;
        TL1 = 0;
    }
}

void ADC() interrupt 5	 using 2
{
//	uchar t;
    uint ADC_temp;
    static uchar m;
    static uint TO;
    static uint max, min;


    ADC_CONTR &= 0xEF;	 //清除ADC中断标志位

    ADC_temp = ADC_RES << 2 | ADC_RESL	;

    if(m == 0)
        min = max = ADC_temp;

    if(ADC_temp > max)
        max = ADC_temp;

    if(ADC_temp < min)
        min = ADC_temp;

    TO += ADC_temp;
    m++;

    if(m < 10)
        ADC_CONTR |= 0x08; 	  //启动下次AD转换
    else
    {
        ADC_temp = (TO - min - max + 4) >> 3;	//去掉最大值最小值求平均值
        TO = 0;
        m = 0;

        if(ADCL)
        {
            ADC_L = 0xffff - (ADC_temp * Liangdu + Min_Liangdu * 30);
            Light_R = (ADC_L >> 8);
            Light_L = (ADC_L & 0xff);
            ADCL = 0;
        }

        if(ADCT)	//温度
        {
            ADC_T = ADC_temp >> 2;
            ADC_T = ADC_T + NTC_buchang - 10;

            if(ADC_T > 39 & ADC_T < 190)
            {
//				t=temp[ADC_T-40];
                t1 = temp[ADC_T - 40] / 10;
                t2 = temp[ADC_T - 40] % 10;

                if(t1 == 0)	 //首位0消隐
                {
                    if(XSMOD < 8)
                        t1 = 19;
                    else
                        t1 = 12;
                }

                if(ADC_T < 60) //低于0显示-
                {
                    if(XSMOD < 8)
                        t1 = 11;
                    else
                        t1 = 18;
                }
            }
            else
            {
                if(XSMOD < 8)
                    t1 = t2 = 18;
                else
                    t1 = t2 = 11;
            }

            ADCT = 0;
        }
    }
}


void Uart() interrupt 4 using 1
{
    uchar data Rx_temp ;
    static uchar fg_count = 0;	//数据间隔,计数
    static uchar Rx_count = 0;
    static uchar n = 0;		   //握手信号计数
    static uchar da_count = 0;	 //数据计数
//	static bit RX_start;    //GPS数据接收开始标志位
//	uchar idata GPRMC[6]={0x24,0x47,0x50,0x52,0x4D,0x43};    //GPS选择接收字符$GPRMC,

    if(RI)
    {
        RI = 0;
        Rx_temp = SBUF;

        /******* 监控握手信号进行软件复位自动下载 ******/
        if(Rx_temp == 0x7f)
        {
            //STC下载指令是0x7f
            n++;

            if(n > 20)   		 //如果连续收到20次0x7f
            {
                IAP_CONTR = 0x60; //复位至ISP区
                n = 0;
            }
        }
        else
            n = 0;

        /********* GPS数据处理 **************/
        if(Rx_count < 6)
        {
            if(Rx_temp == GPRMC[Rx_count])  //比较开始$GPRMC字符，
                Rx_count++;
            else
                Rx_count = 0;
        }
        else  //找到GPRMC开始保存数据
        {
            if(Rx_temp == ',')				//如果收到逗号，
            {
                fg_count++;					//计数增加
                da_count = 0;
            }
            else
            {
                if(fg_count == 1)				//保存时间数据
                {
                    if(da_count < 6)		 //只保存前6位时间数据
                        SJ[da_count++] = Rx_temp;
                }
                else if(fg_count == 2)			//保存定位信息
                {
                    if(Rx_temp == 'A')	//如果收到A
                        DW_OK = 1;		//则定位成功
                    else
                        DW_OK = 0;
                }
                else if(fg_count == 9)				 //保存日期数据
                    RQ[da_count++] = Rx_temp;
                else if(fg_count > 10)			 //接收完毕
                {
                    RX_over = 1;
//								RX_start=0;
                    fg_count = 0;
                    da_count = 0;
                    Rx_count = 0;
                }
            }
        }
    }

}