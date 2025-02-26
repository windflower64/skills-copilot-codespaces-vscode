#ifndef _LCD12864_H_
#define _LCD12864_H_
 #define uchar unsigned char  //宏定义
#define uint unsigned int 
#define ulong unsigned long 

sbit LCD12864_CS=P0^2;							 //12864控制I/O口
sbit LCD12864_SID=P0^1;
sbit LCD12864_CLK=P0^0;


/**********************************************************
#：函数名：SendWrite(uchar dat)
#：函数功能：发送8位数据给LCD12864
#：函数参数：dat位发送的数据变量
***********************************************************/
void SendWrite(uchar dat)
{
	uchar i;
	for(i=0;i<8;i++)
	{
		LCD12864_CLK=0;
		dat=dat<<1;
		LCD12864_SID=CY;
		LCD12864_CLK=1;
		LCD12864_CLK=0;
	}
}

/**********************************************************
#：函数名：uchar LCD12864_Read(void)
#：函数功能：读取LCD12864中的数据
#：函数参数：函数返回读取的数据内容
***********************************************************/
uchar LCD12864_Read(void)
{
	uchar i,dat1,dat2;
	dat1=dat2=0;
	for(i=0;i<8;i++)
	{
		dat1=dat1<<1;
		LCD12864_CLK = 0;
		LCD12864_CLK = 1;                
		LCD12864_CLK = 0;
		if(LCD12864_SID) dat1++;
	}
	for(i=0;i<8;i++)
	{
		dat2=dat2<<1;
		LCD12864_CLK = 0;
		LCD12864_CLK = 1;
		LCD12864_CLK = 0;
		if(LCD12864_SID) dat2++;
	}
	return ((0xf0&dat1)+(0x0f&dat2));
}
/**********************************************************
#：函数名：LCD12864_Busy( void )
#：函数功能：判忙函数
#：函数参数：无
***********************************************************/
void LCD12864_Busy( void )
{
	do SendWrite(0xfc);     //11111,RW(1),RS(0),0
	while(0x80&LCD12864_Read());
}

/**********************************************************
#：函数名：void LCD12864_write(bit cmd,uchar dat)
#：函数功能：向屏发送命令/数据 带发送数据
#：函数参数：cmd标志发送数据、命令，0为命令，1位数据；   dat 位数据内容
***********************************************************/
void LCD12864_write(bit cmd,uchar dat)  
{
	LCD12864_CS = 1;
	LCD12864_Busy();
	if(cmd==0) SendWrite(0xf8);
	else SendWrite(0xfa);          //11111,RW(0),RS(1),0
	SendWrite(0xf0&dat);
	SendWrite(0xf0&dat<<4);
	LCD12864_CS = 0;
}
/**********************************************************
#：函数名：void LCD12864_writebyte(uchar *prointer)
#：函数功能：指针发送显示数据
#：函数参数：prointer位指针内容
***********************************************************/
void LCD12864_writebyte(uchar *prointer)			
{
    while(*prointer!='\0')
    {
        LCD12864_write(1,*prointer);
        prointer++;
    }
}

/******************************************************************
                         lcd初始化函数
*******************************************************************/
void LCD12864_init(void)
{
     LCD12864_write(0,0x30);
     LCD12864_write(0,0x03);
     LCD12864_write(0,0x0c);
     LCD12864_write(0,0x01);
     LCD12864_write(0,0x06);
}

/**********************************************************
#：函数名：void LCD12864_pos(uchar x,y)
#：函数功能：设置屏幕显示的位置
#：函数参数：X，Y，为显示的坐标   X位行数据，Y位列数据
***********************************************************/
void LCD12864_pos(uchar x,y)
{
	switch(x)
	{
		case 0:
			x=0x80;break;
		case 1:
			x=0x90;break;
		case 2:
			x=0x88;break;
		case 3:
			x=0x98;break;
		default:
			x=0x80;
	}
	y=y&0x07;
	LCD12864_write(0,0x30);
	LCD12864_write(0,y+x);
	LCD12864_write(0,y+x);

}
/**********************************************************
#：函数名：void LCD12864_Qing( void )
#：函数功能：清除屏幕显示的你内容
#：函数参数：X，Y，为显示的坐标   X位行数据，Y位列数据
***********************************************************/
void LCD12864_Qing( void )
{
	unsigned char i;
	LCD12864_write(0,0x30);
	LCD12864_write(0,0x80);
	for(i=0;i<64;i++)
	LCD12864_write(1,0x20);
	LCD12864_pos(0,0);	    
}

#endif