#ifndef _1302_H_
#define _1302_H_
 // #include <intrins.h>
#define uchar unsigned char
#define uint unsigned int   

code uchar table_week[12]={0,3,3,6,1,4,6,2,5,0,3,5}; //月修正数据表

uchar xdata time_data[7]=0;
uchar xdata time_data_1[7]=0;
uchar xdata time_data_2[7]=0;
uchar xdata time_data_3[7]=0;
uchar xdata time_data_4[7]=0;



//=====================================================================================
//=====================================================================================
//=====================================================================================
/*******************************************************************************
* 函 数 名         : Ds1302Write
* 函数功能		   : 向DS1302命令（地址+数据）
* 输    入         : addr,dat
* 输    出         : 无
*******************************************************************************/
sbit DSIO=P2^1;
sbit RST=P2^2;
sbit SCLK=P2^0;



uchar Conver_week(uchar year,uchar month,uchar day)
{//c=0 为21世纪,c=1 为19世纪 输入输出数据均为BCD数据
    uchar p1,p2,week;
    year+=0x64;  //如果为21世纪,年份数加100
    p1=year/0x4;  //所过闰年数只算1900年之后的
    p2=year+p1;
    p2=p2%0x7;  //为节省资源,先进行一次取余,避免数大于0xff,避免使用整型数据
    p2=p2+day+table_week[month-1];
    if (year%0x4==0&&month<3)p2-=1;
    week=p2%0x7;
	return week;
}



void ds1302write(uchar addr, uchar dat)
{
	uchar n;
	RST = 0;
	//_nop_();

	SCLK = 0;//先将SCLK置低电平。
	//_nop_();
	RST = 1; //然后将RST(CE)置高电平。
	//_nop_();

	for (n=0; n<8; n++)//开始传送八位地址命令
	{
		DSIO = addr & 0x01;//数据从低位开始传送
		addr >>= 1;
		SCLK = 1;//数据在上升沿时，DS1302读取数据
		//_nop_();
		SCLK = 0;
		//_nop_();
	}
	for (n=0; n<8; n++)//写入8位数据
	{
		DSIO = dat & 0x01;
		dat >>= 1;
		SCLK = 1;//数据在上升沿时，DS1302读取数据
		//_nop_();
		SCLK = 0;
		//_nop_();	
	}	
		 
	RST = 0;//传送数据结束
	//_nop_();
}

/*******************************************************************************
* 函 数 名         : Ds1302Read
* 函数功能		   : 读取一个地址的数据
* 输    入         : addr
* 输    出         : dat
*******************************************************************************/

uchar ds1302read(uchar addr)
{
	uchar n,dat,dat1;
	RST = 0;
	//_nop_();

	SCLK = 0;//先将SCLK置低电平。
	//_nop_();
	RST = 1;//然后将RST(CE)置高电平。
	//_nop_();

	for(n=0; n<8; n++)//开始传送八位地址命令
	{
		DSIO = addr & 0x01;//数据从低位开始传送
		addr >>= 1;
		SCLK = 1;//数据在上升沿时，DS1302读取数据
		//_nop_();
		SCLK = 0;//DS1302下降沿时，放置数据
		//_nop_();
	}
	//_nop_();
	for(n=0; n<8; n++)//读取8位数据
	{
		dat1 = DSIO;//从最低位开始接收
		dat = (dat>>1) | (dat1<<7);
		SCLK = 1;
		//_nop_();
		SCLK = 0;//DS1302下降沿时，放置数据
		//_nop_();
	}

	RST = 0;
	//_nop_();	//以下为DS1302复位的稳定时间,必须的。
	SCLK = 1;
	//_nop_();
	DSIO = 0;
	//_nop_();
	DSIO = 1;
	//_nop_();
	return dat;	
}






#endif