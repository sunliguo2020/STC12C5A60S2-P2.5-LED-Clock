#define DS3231_WriteAddress 0xD0    //器件写地址 
#define DS3231_ReadAddress  0xD1    //器件读地址
#define DS3231_SECOND       0x00    //秒
#define DS3231_MINUTE       0x01    //分
#define DS3231_HOUR         0x02    //时
#define DS3231_WEEK         0x03    //星期
#define DS3231_DAY          0x04    //日
#define DS3231_MONTH        0x05    //月
#define DS3231_YEAR         0x06    //年
#define DS3231_A2D        0x0d    //闹铃2天

DS3231_write_byte(uchar addr, uchar write_data)
read_random(uchar random_addr)
ModifyTime(uchar address,uchar num)
EEprom_WritePara(unsigned int addr, uchar write_data)
EEprom_ReadPara(unsigned int random_addr)