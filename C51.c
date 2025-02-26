#include <reg51.H>
#include "LCD12864.h"
#include "1302.h"
#include "60S2EEPROM.h"		
#include <absacc.h>		  //???
#include <string.h>		  //???
#define  AA  3000000
//站点   GPS

				  
sbit key1=P1^0;
sbit key2=P1^1;
sbit key3=P1^2;
sbit key4=P1^3;
sbit key5=P1^4;
sbit key6=P1^5;
sbit key7=P1^6;
sbit key8=P1^7;		//定义按键IO

sbit led0=P3^4;
sbit led1=P3^5;
sbit led2=P3^6;
sbit led3=P3^7;	 	//定义指示灯IO

sbit Music_Busy=P3^2;	  //定义

bit key1_flag=0;
bit key2_flag=0;
bit key3_flag=0;
bit key4_flag=0;
bit key5_flag=0;
bit key6_flag=0;
bit key7_flag=0;
bit key8_flag=0;    //定义按键位变量



typedef struct {
ulong	shangxing_JD_da[5];// 上行经度数据（5 站）
ulong shangxing_WD_da[5];	// 上行维度度数据（5 站）
ulong xiaxing_JD_da[5];	// 下行经度数据（5 站）
ulong xiaxing_WD_da[5];	//下行维度度数据（5 站）
	
}gps_data;

typedef union
 {
     gps_data Vehicle_Obj;  
	   // 以结构体方式存储 GPS 数据  
     uchar gps_Bytes[sizeof(gps_data)]; 
	   // 以字节数组方式存储 GPS 数据
 }gps_shuju;
gps_shuju xdata gps_cun;
 //使用 xdata 关键字存储在 外部 RAM，保存 GPS 站点数据。
uchar flag_cun=0;//是否存储 GPS 数据的标志位
uchar Station_Count=1;//站点计数，初始为 1
sbit Busy=P3^2;// 语音播放模块的 BUSY 状态检测
bit position=0;// 位置标志位
bit Display_Reversal=0;// 显示翻转
uchar state=0;//显示状态变量（控制 LCD 界面）
bit s0=0;		//文字闪烁变量（可能用于 LCD12864）
uchar ms=0;			//定时器用到的变量
// state 作用：
//控制 LCD12864 显示内容，用于 功能选择、时间调整、站点管理。

//发现两个Bug一个是自动下，到终点站没有自动切换上行、下行
//一个是  自动下 上行站名和语音不照应

uchar sec=0;// 秒计数变量
uchar sec1=0;// 备用秒计数变量
bit memory_flag=0;// 记忆标志位（存储数据时可能用到）

uchar Sound=19;			//音量大小变量
uchar Station=5;		//车站总数变量

bit Mode=0;			    //等于0代表固化    等于1代表自定义
bit A_M=0;          //自动模式和手动模式
bit Upstream_Down=1;// 上行/下行标志位
uchar count=0;//计数变量（可能用于站点切换）

uint JD_Difference=100;	 //34.800340,113.499447
uint WD_Difference=100;// 允许的经纬度误差

uchar *pp;

bit Sound_flag=1; //语音播报标志（1 = 播报，0 = 静音）

 

uchar code xiaxing1[] ="  动物园总站    ";
uchar code xiaxing2[] ="  沙河顶站    　";
uchar code xiaxing3[] ="  沙河大街站  　";
uchar code xiaxing4[] ="  军体院站    　";
uchar code xiaxing5[] ="  省军区站    　";


uchar code shangxing1[] ="  省军区站    　";
uchar code shangxing2[] ="  军体院站    　";
uchar code shangxing3[] ="  沙河大街站  　";
uchar code shangxing4[] ="  沙河顶站    　";
uchar code shangxing5[] ="  动物园总站    ";


				

uchar xdata A_dat[20];	 //暂存从GPS提取的经纬度数据
uchar xdata B_dat[20];	 //暂存从GPS提取的经纬度数据
uchar xdata GPS_dat[100]; // 用于暂存 GPS 模块返回的完整 NMEA 协议数据
uchar subscript;		 //串口数据计数变量



uchar GPS_time=0;		    //检测GPS是否接收到有效数据变量
float latitude,longitude; // 存储 **转换后的纬度/经度** (浮点数格式)
unsigned long WD_A,JD_B; // 最终显示并用于比较的纬度/经度数据

bit GPS_Write=0;		    //是否开启GPS校时更新时间数据标志位
uchar xdata NowTime[7]=0;	//从GPS返回的数据里面提取出时间数据
bit memory_GPS_flag=1;// 记录是否需要存储 GPS 解析数据

uchar Sec_set=0;// 计时变量（可能用于 GPS 时间修正）

uchar Time_Calibration=0;   //校准时间标志位



// 函数的作用是：
// 将当前系统参数存入 EEPROM（非易失性存储器），保证掉电不丢失
//记录系统的关键状态（音量、车站总数、模式、校时开关、上下行方向）
void memory()
 {
   if(memory_flag) //判断是否需要存储数据
    {
	  memory_flag=0;//清除 memory_flag，防止重复存储
	  IapEraseSector(0x08000);//擦除EEPROM指定扇区，0x08000是EEPROM存储的起始地址。
	  IapProgramByte(0x08000,Sound);//将 Sound（音量大小变量）存储到 EEPROM 地址 0x08000。

	  IapProgramByte(0x08001,Station);	       //记录车站总数Station = 10，表示 有 10 个车站。

	  if(GPS_Write)
			IapProgramByte(0x08002,1);   //记录自动校时标志  GPS_Write=1,就记录为1
      else IapProgramByte(0x08002,0); //记录自动校时标志 不启用 GPS 校时
 	
	  if(Mode)
			IapProgramByte(0x08003,1);//记录固化还是自定义Mode=1, 用户自定义模式，存入 0x08003 = 1。
      else IapProgramByte(0x08003,0); //记录固化还是自定义  Mode=0,就记录为0  

	  if(A_M)
			IapProgramByte(0x08004,1);         //记录手动自动模式  A_M=1,自动模式（系统自动播放站点信息）
      else IapProgramByte(0x08004,0);           //记录手动自动模式  A_M=0,手动模式（人工按键触发站点信息）  

	  if(Upstream_Down) 
			IapProgramByte(0x08005,1);  //记录上行下行  Station_Count=1,就记录为1
      else IapProgramByte(0x08005,0);  //记录上行下行  Station_Count=0,就记录为0   

	}
 }





 
 

 
 
 
 

 
 
 
 //从 EEPROM 读取系统参数（音量、车站数、GPS 校时、模式等）。
//如果 EEPROM 里数据错误，恢复默认值（防止数据损坏）。
//读取完成后，重新存储，确保数据正确性。
 void read_memory() //从 EEPROM 读取存储的系统参数
 {
   	Sound=IapReadByte(0x08000);	 //读取存储的音量大小变量
//	Station=IapReadByte(0x08001); //读取的记录车站总数

	if(Sound>30||Station>5) // 如果读取的音量或车站总数超出正常范围
	 {
	   Sound=25;
	   Station=7;//恢复默认值
	 }
    if(IapReadByte(0x08002)!=0&&IapReadByte(0x08002)!=1) //读取GPS校时标志位GPS_Write只能是0禁用或1启用。
	 {
	   	GPS_Write=1;		//如果读取不对，默认打开
	 } 
	 else  
		  GPS_Write=IapReadByte(0x08002);	//如果对，进行赋值
    if(IapReadByte(0x08003)!=0&&IapReadByte(0x08003)!=1) //读取存储的Mode值
	 {
	  	Mode=0;// 默认 `Mode = 0`（固化模式）
	 }
	 else  
		  Mode=IapReadByte(0x08003);// 数据正确，则直接赋值

    if(IapReadByte(0x08004)!=0&&IapReadByte(0x08004)!=1) //检查 A_M
	 {
	  	A_M=0;// 默认 `A_M = 0`（手动模式）
	 }else  
	    A_M=IapReadByte(0x08004);// 读取的值合法，则直接赋值

    if(IapReadByte(0x08005)!=0&&IapReadByte(0x08005)!=1) //检查 Upstream_Down
	 {
	  	Upstream_Down=0;// 默认 `Upstream_Down = 0`（上行模式）
	 }else  
	    Upstream_Down=IapReadByte(0x08005); // 读取的值合法，则直接赋值

     memory_flag=1;	// 读取完成后，将数据再次存储到 EEPROM，确保数据正确

 }


 

 
 
 
 
 

 
 
 
 
 //该函数的主要功能是 将 GPS 站点的经纬度数据存入 EEPROM，用于掉电后恢复。
//GPS 数据分为 4 组，每组 5 个值，共 20 个站点：
//上行站点经度 (shangxing_JD_da[5])
//上行站点纬度 (shangxing_WD_da[5])
//下行站点经度 (xiaxing_JD_da[5])
//下行站点纬度 (xiaxing_WD_da[5])
//每个经纬度数据 4 字节存储（无符号长整型 ulong，32 位）。
//EEPROM 地址 0x08200 开始，每个站点数据占用 4 字节，共 20 × 4 = 80 字节。
void memory_GPS() // 存储 GPS 数据 到 EEPROM（非易失性存储器）
{
	// void memory_GPS()// 存储 GPS 数据 到 EEPROM（非易失性存储器）
// {
//   	  unsigned char i=0;//i 用于循环计数（0~19，共 20 个站点）
//	  if(memory_GPS_flag)//检查 memory_GPS_flag（是否需要存储）
//	   {	
//	     memory_GPS_flag=0; //清除标志，避免重复存储
//			 IapEraseSector(0x08200);// 擦除 EEPROM 扇区（0x08200 ~ 0x083FF）
//			 for(i=0;i<20;i++)		 // 遍历 20 个 GPS 站点    	   
//			 {
////							i = 0 ~ 4 → 上行经度 (shangxing_JD_da[5])
////							i = 5 ~ 9 → 上行纬度 (shangxing_WD_da[5])
////							i = 10 ~ 14 → 下行经度 (xiaxing_JD_da[5])
////							i = 15 ~ 19 → 下行纬度 (xiaxing_WD_da[5])
//						if(i<5) 
//				 {
//					IapProgramByte(0x08200+i*4,gps_cun.Vehicle_Obj.shangxing_JD_da[i]/16777216%256);//第 1 字节（高 8 位）
//					IapProgramByte(0x08200+i*4+1,gps_cun.Vehicle_Obj.shangxing_JD_da[i]/65536%256);//第 2 字节（次高 8 位）
//					IapProgramByte(0x08200+i*4+2,gps_cun.Vehicle_Obj.shangxing_JD_da[i]/256%256);//第 3 字节（次低 8 位）
//					IapProgramByte(0x08200+i*4+3,gps_cun.Vehicle_Obj.shangxing_JD_da[i]%256);//第 4 字节（最低 8 位）
//				 }
//				 else if(i<10) 
//				  {
//					IapProgramByte(0x08200+i*4,gps_cun.Vehicle_Obj.shangxing_WD_da[i-5]/16777216%256);
//					IapProgramByte(0x08200+i*4+1,gps_cun.Vehicle_Obj.shangxing_WD_da[i-5]/65536%256);
//					IapProgramByte(0x08200+i*4+2,gps_cun.Vehicle_Obj.shangxing_WD_da[i-5]/256%256);
//					IapProgramByte(0x08200+i*4+3,gps_cun.Vehicle_Obj.shangxing_WD_da[i-5]%256);
//				  }
//				 else if(i<15) 
//				  {
//					IapProgramByte(0x08200+i*4,gps_cun.Vehicle_Obj.xiaxing_JD_da[i-10]/16777216%256);
//					IapProgramByte(0x08200+i*4+1,gps_cun.Vehicle_Obj.xiaxing_JD_da[i-10]/65536%256);
//					IapProgramByte(0x08200+i*4+2,gps_cun.Vehicle_Obj.xiaxing_JD_da[i-10]/256%256);
//					IapProgramByte(0x08200+i*4+3,gps_cun.Vehicle_Obj.xiaxing_JD_da[i-10]%256);
//				  }
//				 else if(i<20) 
//				  {
//					IapProgramByte(0x08200+i*4,gps_cun.Vehicle_Obj.xiaxing_WD_da[i-15]/16777216%256);
//					IapProgramByte(0x08200+i*4+1,gps_cun.Vehicle_Obj.xiaxing_WD_da[i-15]/65536%256);
//					IapProgramByte(0x08200+i*4+2,gps_cun.Vehicle_Obj.xiaxing_WD_da[i-15]/256%256);
//					IapProgramByte(0x08200+i*4+3,gps_cun.Vehicle_Obj.xiaxing_WD_da[i-15]%256);
//				  }
//			 }
//	   }
// }

    unsigned char i = 0; // i 用于循环计数（0~19，共 20 个站点）

    if (memory_GPS_flag) // 检查 memory_GPS_flag（是否需要存储）
    {
        memory_GPS_flag = 0; // 清除标志，避免重复存储
        IapEraseSector(0x08200); // 擦除 EEPROM 扇区（0x08200 ~ 0x083FF）

        for (i = 0; i < 20; i++) // 遍历 20 个 GPS 站点
        {
            unsigned char* data_ptr;
            if (i < 5)
            {
                // 上行经度 (shangxing_JD_da[5])
                data_ptr = (unsigned char*)&gps_cun.Vehicle_Obj.shangxing_JD_da[i];
            }
            else if (i < 10)
            {
                // 上行纬度 (shangxing_WD_da[5])
                data_ptr = (unsigned char*)&gps_cun.Vehicle_Obj.shangxing_WD_da[i - 5];
            }
            else if (i < 15)
            {
                // 下行经度 (xiaxing_JD_da[5])
                data_ptr = (unsigned char*)&gps_cun.Vehicle_Obj.xiaxing_JD_da[i - 10];
            }
            else
            {
                // 下行纬度 (xiaxing_WD_da[5])
                data_ptr = (unsigned char*)&gps_cun.Vehicle_Obj.xiaxing_WD_da[i - 15];
            }

            // 将数据按字节写入 EEPROM
            IapProgramByte(0x08200 + i * 4, data_ptr[3]); // 第 1 字节（最高 8 位）
            IapProgramByte(0x08200 + i * 4 + 1, data_ptr[2]); // 第 2 字节
            IapProgramByte(0x08200 + i * 4 + 2, data_ptr[1]); // 第 3 字节
            IapProgramByte(0x08200 + i * 4 + 3, data_ptr[0]); // 第 4 字节（最低 8 位）
        }
    }
}
 


 
 
 
 



//该函数用于 从 EEPROM 读取 GPS 数据 
//并存入 gps_cun 结构体变量，以便后续处理。
//EEPROM 地址 0x08200 处存储了 上行/下行站点的经纬度数据。
void read_GPS()
 {
	   
	   pp=gps_cun.gps_Bytes;
	 // 将 gps_cun.gps_Bytes 的地址赋给指针 pp，方便操作数据
	   IapReadSector(0x08200, sizeof(gps_data), pp);
	 // 从 EEPROM 地址 0x08200 读取一整块数据
 }



 
 
 
//该函数用于通过串口发送一个字节的数据。
//将传入的字节数据存入串口发送缓冲寄存器。
//等待发送完成标志位被置位，表示数据发送完成。
//清除发送完成标志位，为下一次发送做准备。
 void Uart1Data(uchar dat) 	// 串口发送一个字节数据
{
	SBUF=dat;// 将传入的字节数据存入串口发送缓冲寄存器SBUF
	while(!TI);// 等待发送完成标志位TI置位，表示数据发送完成
	TI=0;	// 清除发送完成标志位TI，为下一次发送做准备
}





//该函数用于通过串口发送一串数据。
//'while（*byte ！=：循环遍历字符串，直到遇到字符串结束符 。
//'Uart1数据：调用函数发送当前字节数据，并将指针移动到下一个字节。
void UartData_Byte(uchar *byte)	//串口发送一串数据
{
	while(*byte != '\0')
		// 循环遍历字符串，直到遇到字符串结束符 '\0'
	{
	  Uart1Data(*byte++);
		// 发送当前字节数据，并将指针移动到下一个字节
	}
}





//该函数用于产生一个延迟，延迟的时间由传入决定。
// 循环递减传入的dat值，直到其变为0
void delay(uint dat) 
 {
   while(dat--);
 }



 
 
 
 //该函数用于校验从服务器返回的数据是否与本地存储的GPS_dat数据一致。
 uchar verify_GPSdat(uchar *dat)//读取服务器返回的数据
{
  uchar i=0;
  while(*dat != 0)// 遍历传入的数据，直到遇到字符串结束符 '\0'
		{
    if(*dat != GPS_dat[i])
			// 比较传入的数据和 GPS_dat 中的对应位置的数据
		{
      return 0; // 如果有不匹配的情况，返回 0 表示校验失败
    }
    i++; // 如果匹配，继续比较下一个字节
    dat++;
  }
  GPS_dat[0]=0;
	// 如果所有数据都匹配，将 GPS_dat 的第一个字节置 0
  return 1;// 返回 1 表示校验成功
}






//该函数用于通过串口发送一串十六进制数据。
//参数 是一个指向无符号字符p
//参数num是无符号字符，表示要发送的字符数量。
void Send_Hex(unsigned char *p,unsigned char num)
{
   	while(num--)   //剩余发送的字符数
	{
        SBUF = *p; //将要发送的数据赋给串口缓冲寄存器
		while(!TI);//等待发送结束
		TI = 0;    //软件清零
		p++;       // 指针指向下一个字符
	}	
}





//代码实现了一个校验和计算函数，
//用于计算字符串的校验和，
//并将结果存储在字符串的末尾两个字节中。
void DoSum(unsigned char *Str,unsigned char len)//校验位计算
{
	unsigned int xorsum = 0; 
// 定义一个无符号整型变量 xorsum 并初始化为 0，
//用于累加计算校验和
	unsigned char i;

	for(i=1;i<len;i++)
	// 循环遍历字符串，从索引 1 开始到 len-1，累加每个字符的值
	{
		xorsum = xorsum + Str[i];
	}
	xorsum = 0 - xorsum;// 将累加和取反
	*(Str+i)     = (unsigned char)(xorsum >> 8);
	// 将校验和的高 8 位存入字符串的第 len 位
	*(Str+i+1)   = (unsigned char)(xorsum & 0x00ff);
	 // 将校验和的低 8 位存入字符串的第 len + 1 位
}






//代码实现了一个发送音乐指令的函数，
//将指令数据按照特定格式填充到数组中，
//并计算校验码，最后通过串口发送完整的指令数据。
void Send_Appoint_Music(unsigned char dat )
 {	 //7E FF 06 03 00 00 01 FE F7 EF
		unsigned char Table[10];
		// 设置指令头
	  Table[0] = 0x7E;
		Table[1] = 0xFF;
		Table[2] = 0x06;
	
		Table[3] = 0x03; 
	  // 指令码，表示播放指定音乐
		Table[4] = 0x00; 
		Table[5] = 0x00;
		// 填充保留字段
	  Table[6] = dat; 
	  // 设置要播放的音乐编号
		DoSum(Table,7);// 计算校验码并填充
		Table[9] = 0xEF;//结束码
		
		Send_Hex(Table,10);//发送指令数据
 }


 
 
 
// 代码实现了一个发送设置音量指令的函数，
// 将指令数据按照特定格式填充到数组中，
// 并计算校验码，最后通过串口发送完整的指令数据。
  void Send_Appoint_Sound(unsigned char dat)
 {
   if(Music_Busy==0&&s0)  
		 // 检查 Music_Busy 和 s0 状态，如果条件满足则发送指令
    {
		unsigned char Table[10];
		// 设置指令头
		Table[0] = 0x7E;
		Table[1] = 0xFF;
		Table[2] = 0x06;
	
		Table[3] = 0x06; 
		// 指令码，表示设置音量
		Table[4] = 0x00;
		Table[5] = 0x00;
		// 填充保留字段
		Table[6] = dat;//音量
		DoSum(Table,7);//计算校验码	
		Table[9] = 0xEF;//结束码
		
		Send_Hex(Table,10);//发送指令数据
	}
 }


 
 
 
 
// 代码实现了一个读取和设置DS1302时间数据的函数，
// 通过判断的状态来决定是读取还是设置时间数据，
// 并进行相应的处理。
 void read_time1() //实时读取DS1302中的时间数据
{
 uchar i;
	
		 if(state==0)
		  {
				// 读取DS1302中的时间数据，并存储在time_data数组
			  time_data[0]=ds1302read(0x81);
				time_data[1]=ds1302read(0x83);
				time_data[2]=ds1302read(0x85);
				time_data[3]=ds1302read(0x87);
				time_data[4]=ds1302read(0x89);
				time_data[5]=ds1302read(0x8D);  	
				// 将读取到的时间数据转换为十进制格式，
				//存储在time_data_1数组中
			 	time_data_1[0]=time_data[0]/16*10+time_data[0]%16;
				time_data_1[1]=time_data[1]/16*10+time_data[1]%16;
				time_data_1[2]=time_data[2]/16*10+time_data[2]%16;
				time_data_1[3]=time_data[3]/16*10+time_data[3]%16;
				time_data_1[4]=time_data[4]/16*10+time_data[4]%16;
				time_data_1[5]=time_data[5]/16*10+time_data[5]%16;
				 if(time_data_1[0]>59)
				// 如果秒钟数据大于59，说明读取的数据有误，进行初始化
				   {
				 	ds1302write(0x8e,0x00); 
					ds1302write(0x80,0x80);
					ds1302write(0x80,0);
					ds1302write(0x8e,0x80);     
				   }
			    ds1302write(0x8e,0x80);				  
		  }	
		  else 
		   {
				ds1302write(0x8e,0x00); 
				ds1302write(0x80,0x80);
				// 将time_data_1中的时间数据转换为BCD码，
				 //并存储在time_data_4数组中
				for(i=0;i<7;i++)
				{
					time_data_2[i]=time_data_1[i]/10;
					time_data_3[i]=time_data_1[i]%10;	  
				}
				for(i=0;i<7;i++)
				{
					time_data_4[i]=time_data_2[i]*16+time_data_3[i];
				  
				}
				// 将转换后的时间数据写入DS1302中
				ds1302write(0x80,time_data_4[0]);
				ds1302write(0x82,time_data_4[1]);
				ds1302write(0x84,time_data_4[2]);
				ds1302write(0x86,time_data_4[3]);
				ds1302write(0x88,time_data_4[4]);
				ds1302write(0x8C,time_data_4[5]);
		   }     

}





//显示函数
void Display() {
    uchar dat = 0; // 定义一个无符号字符变量 dat 并初始化为 0

    // 根据 state 的值决定显示内容
    if (state <= 6) {
        LCD12864_pos(0, 0); // 设置 LCD 位置为第 0 行第 0 列

        // 显示年份
        if (state == 1 && s0) {
            LCD12864_writebyte("    "); // 如果 state 为 1 并且 s0 为真，显示空格
        } else {
            LCD12864_writebyte("20"); // 显示固定年份前两位 "20"
            LCD12864_write(1, 0x30 + time_data_1[5] / 10); // 显示年份的十位
            LCD12864_write(1, 0x30 + time_data_1[5] % 10); // 显示年份的个位
        }

        LCD12864_writebyte("-"); // 显示分隔符 "-"
        
        // 显示月份
        if (state == 2 && s0) {
            LCD12864_writebyte("  "); // 如果 state 为 2 并且 s0 为真，显示空格
        } else {
            LCD12864_write(1, 0x30 + time_data_1[4] / 10); // 显示月份的十位
            LCD12864_write(1, 0x30 + time_data_1[4] % 10); // 显示月份的个位
        }

        LCD12864_writebyte("-"); // 显示分隔符 "-"
        
        // 显示日期
        if (state == 3 && s0) {
            LCD12864_writebyte("  "); // 如果 state 为 3 并且 s0 为真，显示空格
        } else {
            LCD12864_write(1, 0x30 + time_data_1[3] / 10); // 显示日期的十位
            LCD12864_write(1, 0x30 + time_data_1[3] % 10); // 显示日期的个位
        }

        LCD12864_writebyte("  "); // 显示空格
        
        // 显示星期几
        switch (Conver_week(time_data_1[5], time_data_1[4], time_data_1[3])) {
            case 0: LCD12864_writebyte("Sun "); break; // 星期天
            case 1: LCD12864_writebyte("Mon "); break; // 星期一
            case 2: LCD12864_writebyte("Tue "); break; // 星期二
            case 3: LCD12864_writebyte("Wed "); break; // 星期三
            case 4: LCD12864_writebyte("Thu "); break; // 星期四
            case 5: LCD12864_writebyte("Fri "); break; // 星期五
            case 6: LCD12864_writebyte("Sat "); break; // 星期六
        } 

        LCD12864_pos(1, 0); // 设置 LCD 位置为第 1 行第 0 列
        
        // 显示小时
        if (state == 4 && s0) {
            LCD12864_writebyte("  "); // 如果 state 为 4 并且 s0 为真，显示空格
        } else {
            LCD12864_write(1, 0x30 + time_data_1[2] / 10); // 显示小时的十位
            LCD12864_write(1, 0x30 + time_data_1[2] % 10); // 显示小时的个位
        }

        LCD12864_writebyte(":"); // 显示分隔符 ":"
        
        // 显示分钟
        if (state == 5 && s0) {
            LCD12864_writebyte("  "); // 如果 state 为 5 并且 s0 为真，显示空格
        } else {
            LCD12864_write(1, 0x30 + time_data_1[1] / 10); // 显示分钟的十位
            LCD12864_write(1, 0x30 + time_data_1[1] % 10); // 显示分钟的个位
        }

        LCD12864_writebyte(":"); // 显示分隔符 ":"
        
        // 显示秒钟
        if (state == 6 && s0) {
            LCD12864_writebyte("  "); // 如果 state 为 6 并且 s0 为真，显示空格
        } else {
            LCD12864_write(1, 0x30 + time_data_1[0] / 10); // 显示秒钟的十位
            LCD12864_write(1, 0x30 + time_data_1[0] % 10); // 显示秒钟的个位
        }
    }
}			 
			// GPS连接状态
if (GPS_time == 0) { // 如果GPS时间为0，表示未连接
    if (s0) {
        LCD12864_writebyte("        "); 
        // 如果s0为真，显示空格
    } else {
        LCD12864_writebyte("  Search"); 
        // 如果s0为假，显示“  Search”
    }
} else {
    LCD12864_writebyte(" Connect"); 
    // 如果GPS时间不为0，表示已连接，显示“ Connect”
}

// 显示模式和方向
LCD12864_pos(2, 0); // 设置LCD位置为第2行第0列
if (A_M == 0) {
    LCD12864_writebyte("自动模式"); 
    // 如果A_M为0，显示“自动模式”
} else {
    LCD12864_writebyte("手动模式"); 
    // 如果A_M不为0，显示“手动模式”
}

if (Upstream_Down) {
    LCD12864_writebyte("    下行"); 
    // 如果Upstream_Down为真，显示“    下行”
} else {
    LCD12864_writebyte("    上行"); 
    // 如果Upstream_Down为假，显示“    上行”
}

// 显示站点信息
LCD12864_pos(3, 0); // 设置LCD位置为第3行第0列


if (Station_Count == 0 && position == 0) { 
	// 如果站点计数为0且位置为0
    if (A_M == 0) { // 如果A_M为0
        if (Display_Reversal)
            LCD12864_writebyte("    位置错误    "); 
				// 如果Display_Reversal为真，显示“    位置错误    ”
        else
            LCD12864_writebyte("请到附近站点定位"); 
				// 如果Display_Reversal为假，显示“请到附近站点定位”
    } else {
        LCD12864_writebyte("欢迎乘坐本次公交"); 
			// 如果A_M不为0，显示“欢迎乘坐本次公交”
    }
} else if (Station_Count != 0 && position == 0) { 
	// 如果站点计数不为0且位置为0
    if (A_M == 0) { // 如果A_M为0，表示自动模式
        if (Upstream_Down) { 
					// 如果Upstream_Down为真，表示下行
            switch (Station_Count) {
                case 1:
                    if (Display_Reversal)
                        LCD12864_writebyte("    下一站      "); 
										// 如果Display_Reversal为真，显示“    下一站      ”
                    else
                        LCD12864_writebyte(xiaxing2); 
										// 如果Display_Reversal为假，显示xiaxing2
                    break;

                case 2:
                    if (Station == 2) {
                        if (Display_Reversal)
                            LCD12864_writebyte("    终点站      "); 
												// 如果Display_Reversal为真，显示“    终点站      ”
                        else
                            LCD12864_writebyte(xiaxing2); 
												// 如果Display_Reversal为假，显示xiaxing2
                    } else {
                        if (Display_Reversal)
                            LCD12864_writebyte("    下一站      "); 
												// 如果Display_Reversal为真，显示“    下一站      ”
                        else
                            LCD12864_writebyte(xiaxing3); 
												// 如果Display_Reversal为假，显示xiaxing3
                    }
                    break;

                case 3:
                    if (Station == 3) {
                        if (Display_Reversal)
                            LCD12864_writebyte("    终点站      "); 
												// 如果Display_Reversal为真，显示“    终点站      ”
                        else
                            LCD12864_writebyte(xiaxing3); 
												// 如果Display_Reversal为假，显示xiaxing3
                    } else {
                        if (Display_Reversal)
                            LCD12864_writebyte("    下一站      "); 
												// 如果Display_Reversal为真，显示“    下一站      ”
                        else
                            LCD12864_writebyte(xiaxing4); 
												// 如果Display_Reversal为假，显示xiaxing4
                    }
                    break;

                case 4:
                    if (Station == 4) {
                        if (Display_Reversal)
                            LCD12864_writebyte("    终点站      "); 
												// 如果Display_Reversal为真，显示“    终点站      ”
                        else
                            LCD12864_writebyte(xiaxing4); 
												// 如果Display_Reversal为假，显示xiaxing4
                    } else {
                        if (Display_Reversal)
                            LCD12864_writebyte("    下一站      "); 
												// 如果Display_Reversal为真，显示“    下一站      ”
                        else
                            LCD12864_writebyte(xiaxing5); 
												// 如果Display_Reversal为假，显示xiaxing5
                    }
                    break;
										
								case 5 :	
												            if(Display_Reversal)  LCD12864_writebyte("    终点站      ");
								                    // 如果Display_Reversal为真，显示“    终点站      ”
															 else                 LCD12864_writebyte(xiaxing15);	
								                     // 如果Display_Reversal为假，显示xiaxing6
												  break;

            }
        }											 
	 else 	// 如果Upstream_Down为假，表示上行
				{
			switch (Station_Count) {
        case 1:
            // 当站点总数为1时
            if (Display_Reversal)
                // 如果需要反向显示信息
                LCD12864_writebyte("    下一站      "); // 显示“下一站”
            else
                // 否则显示上行站点2的名称
                LCD12864_writebyte(shangxing2);
            break;

        case 2:
            // 当站点总数为2时
            if (Station == 2) {
                // 如果当前站点是2
                if (Display_Reversal)
                    // 如果需要反向显示信息
                    LCD12864_writebyte("    终点站      "); // 显示“终点站”
                else
                    // 否则显示上行站点2的名称
                    LCD12864_writebyte(shangxing2);
            } else {
                // 当前站点不是2
                if (Display_Reversal)
                    // 如果需要反向显示信息
                    LCD12864_writebyte("    下一站      "); // 显示“下一站”
                else
                    // 否则显示上行站点3的名称
                    LCD12864_writebyte(shangxing3);
            }
            break;

        case 3:
            // 当站点总数为3时
            if (Station == 3) {
                // 如果当前站点是3
                if (Display_Reversal)
                    // 如果需要反向显示信息
                    LCD12864_writebyte("    终点站      "); // 显示“终点站”
                else
                    // 否则显示上行站点3的名称
                    LCD12864_writebyte(shangxing3);
            } else {
                // 当前站点不是3
                if (Display_Reversal)
                    // 如果需要反向显示信息
                    LCD12864_writebyte("    下一站      "); // 显示“下一站”
                else
                    // 否则显示上行站点4的名称
                    LCD12864_writebyte(shangxing4);
            }
            break;

        case 4:
            // 当站点总数为4时
            if (Station == 4) {
                // 如果当前站点是4
                if (Display_Reversal)
                    // 如果需要反向显示信息
                    LCD12864_writebyte("    终点站      "); // 显示“终点站”
                else
                    // 否则显示上行站点4的名称
                    LCD12864_writebyte(shangxing4);
            } else {
                // 当前站点不是4
                if (Display_Reversal)
                    // 如果需要反向显示信息
                    LCD12864_writebyte("    下一站      "); // 显示“下一站”
                else
                    // 否则显示上行站点5的名称
                    LCD12864_writebyte(shangxing5);
            }
            break;

						case 5 :	
						// 当站点总数为5时						
						if(Display_Reversal)  
							// 如果需要反向显示信息
						LCD12864_writebyte("    终点站      ");// 显示“终点站”
						else
							// 否则显示上行站点5的名称                 
							LCD12864_writebyte(shangxing5);									 		 
								break;
				
    }
}
				   	
	 else 		//手动模式
		{
     if (Busy == 0) { // 如果系统不忙
        if (Upstream_Down) { 
					// 如果Upstream_Down为真，表示下行
            switch (Station_Count - 1) {
                case 1:
                    if (Station_Count <= Station) {
                        // 当前站点数小于等于目标站点数
                        if (Display_Reversal)
                            LCD12864_writebyte("      到站      "); 
												// 显示“到站”
                        else
                            LCD12864_writebyte(xiaxing1); 
												// 显示下行站点1的名称
                    } else {
                        // 当前站点数大于目标站点数
                        if (Display_Reversal)
                            LCD12864_writebyte("    终点站      "); 
												// 显示“终点站”
                        else
                            LCD12864_writebyte(xiaxing1); 
												// 显示下行站点1的名称
                    }
                    break;

                case 2:
                    if (Station_Count <= Station) {
                        // 当前站点数小于等于目标站点数
                        if (Display_Reversal)
                            LCD12864_writebyte("      到站      "); 
												// 显示“到站”
                        else
                            LCD12864_writebyte(xiaxing2); 
												// 显示下行站点2的名称
                    } else {
                        // 当前站点数大于目标站点数
                        if (Display_Reversal)
                            LCD12864_writebyte("    终点站      "); 
												// 显示“终点站”
                        else
                            LCD12864_writebyte(xiaxing2); 
												// 显示下行站点2的名称
                    }
                    break;

                case 3:
                    if (Station_Count <= Station) {
                        // 当前站点数小于等于目标站点数
                        if (Display_Reversal)
                            LCD12864_writebyte("      到站      "); 
												// 显示“到站”
                        else
                            LCD12864_writebyte(xiaxing3); 
												// 显示下行站点3的名称
                    } else {
                        // 当前站点数大于目标站点数
                        if (Display_Reversal)
                            LCD12864_writebyte("    终点站      "); 
												// 显示“终点站”
                        else
                            LCD12864_writebyte(xiaxing3); 
												// 显示下行站点3的名称
                    }
                    break;

                case 4:
                    if (Station_Count <= Station) {
                        // 当前站点数小于等于目标站点数
                        if (Display_Reversal)
                            LCD12864_writebyte("      到站      "); 
												// 显示“到站”
                        else
                            LCD12864_writebyte(xiaxing4); 
												// 显示下行站点4的名称
                    } else {
                        // 当前站点数大于目标站点数
                        if (Display_Reversal)
                            LCD12864_writebyte("    终点站      "); 
												// 显示“终点站”
                        else
                            LCD12864_writebyte(xiaxing4); 
												// 显示下行站点4的名称
                    }
                    break;

                case 5 :	 
									if(Station_Count<=Station){ 	
										// 当前站点数小于等于目标站点数
										if(Display_Reversal)  
											LCD12864_writebyte("      到站      "); // 显示“到站”
										else                 
											LCD12864_writebyte(xiaxing5);	// 显示下行站点5的名称
								}else {
											// 当前站点数大于目标站点数
									if(Display_Reversal)  
												LCD12864_writebyte("    终点站      "); // 显示“终点站”
											else                  
												LCD12864_writebyte(xiaxing15);// 显示下行站点5的名称
										}	           								  									 		 
break;	

								
            }
        }
    }
	}						else {
    // 手动模式
    switch (Station + 1 - Station_Count) {
        // 根据 Station+1-Station_Count 的值选择不同的 case 分支
        case 1:
            if (Station_Count != 1 && Station_Count <= Station) {
                // 如果 Station_Count 不是 1 且 Station_Count 小于等于 Station
                if (Display_Reversal)
                    LCD12864_writebyte("      到站      "); // 显示“到站”
                else
                    LCD12864_writebyte(shangxing1); // 显示上行站点1的名称
            } else {
                // 否则
                if (Display_Reversal)
                    LCD12864_writebyte("    终点站      "); // 显示“终点站”
                else
                    LCD12864_writebyte(shangxing1); // 显示上行站点1的名称
            }
            break;

        case 2:
            if (Station_Count != 1 && Station_Count <= Station) {
                // 如果 Station_Count 不是 1 且 Station_Count 小于等于 Station
                if (Display_Reversal)
                    LCD12864_writebyte("      到站      "); // 显示“到站”
                else
                    LCD12864_writebyte(shangxing2); // 显示上行站点2的名称
            } else {
                // 否则
                if (Display_Reversal)
                    LCD12864_writebyte("    终点站      "); // 显示“终点站”
                else
                    LCD12864_writebyte(shangxing2); // 显示上行站点2的名称
            }
            break;

        case 3:
            if (Station_Count != 1 && Station_Count <= Station) {
                // 如果 Station_Count 不是 1 且 Station_Count 小于等于 Station
                if (Display_Reversal)
                    LCD12864_writebyte("      到站      "); // 显示“到站”
                else
                    LCD12864_writebyte(shangxing3); // 显示上行站点3的名称
            } else {
                // 否则
                if (Display_Reversal)
                    LCD12864_writebyte("    终点站      "); // 显示“终点站”
                else
                    LCD12864_writebyte(shangxing3); // 显示上行站点3的名称
            }
            break;

        case 4:
            if (Station_Count != 1 && Station_Count <= Station) {
                // 如果 Station_Count 不是 1 且 Station_Count 小于等于 Station
                if (Display_Reversal)
                    LCD12864_writebyte("      到站      "); // 显示“到站”
                else
                    LCD12864_writebyte(shangxing4); // 显示上行站点4的名称
            } else {
                // 否则
                if (Display_Reversal)
                    LCD12864_writebyte("    终点站      "); // 显示“终点站”
                else
                    LCD12864_writebyte(shangxing4); // 显示上行站点4的名称
            }
            break;

				case 5:
            if (Station_Count != 1 && Station_Count <= Station) {
                // 如果 Station_Count 不是 1 且 Station_Count 小于等于 Station
                if (Display_Reversal)
                    LCD12864_writebyte("      到站      "); // 显示“到站”
                else
                    LCD12864_writebyte(shangxing5); // 显示上行站点5的名称
            } else {
                // 否则
                if (Display_Reversal)
                    LCD12864_writebyte("    终点站      "); // 显示“终点站”
                else
                    LCD12864_writebyte(shangxing5); // 显示上行站点5的名称
            }
            break;
    }
}
											}
									}


							}else LCD12864_writebyte("欢迎乘坐本次公交");
	
					}
	else if(Station_Count!=0&&position!=0)    {
							      if(Upstream_Down) 
								   {
										switch(Station_Count)
									    {
										  case 1 :	 
										             if(Display_Reversal)  LCD12864_writebyte("      到站      ");	
													 else                 LCD12864_writebyte(xiaxing1);										 									 		 
										  break;			
										  case 2 :	if(Display_Reversal)   LCD12864_writebyte("      到站      ");
													 else                  LCD12864_writebyte(xiaxing2);										  									 		 
										  break;		
										  case 3 :	if(Display_Reversal)   LCD12864_writebyte("      到站      "); 
													 else                  LCD12864_writebyte(xiaxing3);									  									 		 
										  break;		
										  case 4 :	if(Display_Reversal)   LCD12864_writebyte("      到站      ");
													 else                  LCD12864_writebyte(xiaxing4);										  								 		 
										  break;
						
										  case 5 :	if(Display_Reversal)  LCD12864_writebyte("      到站      ");
													 else                 LCD12864_writebyte(xiaxing5);								  									 		 
										  break;
										}
								   }
								   else 
								    {
											switch(Station_Count)
										    {
											  case 1 :	if(Display_Reversal)  LCD12864_writebyte("      到站      ");	
														 else                 LCD12864_writebyte(shangxing1);										 									 		 
											  break;			
											  case 2 :	if(Display_Reversal)   LCD12864_writebyte("      到站      ");
														 else                  LCD12864_writebyte(shangxing2);										  									 		 
											  break;		
											  case 3 :	if(Display_Reversal)   LCD12864_writebyte("      到站      "); 
														 else                  LCD12864_writebyte(shangxing3);									  									 		 
											  break;		
											  case 4 :	if(Display_Reversal)   LCD12864_writebyte("      到站      ");
														 else                  LCD12864_writebyte(shangxing4);										  								 		 
											  break;
							
											  case 5 :	if(Display_Reversal)  LCD12864_writebyte("      到站      ");
														 else                 LCD12864_writebyte(shangxing5);								  									 		 
											  break;
											}
									}
			 }
	 }
 	else if(state<10)
	 {
			LCD12864_pos(0,0);
		    LCD12864_writebyte("    系统设置    ");
		   	LCD12864_pos(1,0);
		    LCD12864_writebyte("车站    : ");
			if(state==7&&s0) LCD12864_writebyte("  ");
			else            
			 {
			   LCD12864_write(1,0x30+Station/10%10);
			   LCD12864_write(1,0x30+Station%10);
			 } 
		    LCD12864_writebyte("    ");

		   	LCD12864_pos(2,0);
		    LCD12864_writebyte("音量大小: ");
			if(state==8&&s0) LCD12864_writebyte("  ");
			else 
			 {
			    LCD12864_write(1,0x30+Sound/10%10);
			    LCD12864_write(1,0x30+Sound%10);
			 }         
		    LCD12864_writebyte("    ");

			LCD12864_pos(3,0);
		    LCD12864_writebyte("自动校时: ");
			if(state==9&&s0) LCD12864_writebyte("       ");
			else 
			 {
				if(GPS_Write) LCD12864_writebyte("开    ");
				else 		  LCD12864_writebyte("关    ");
			 }   

	 }
	 else if(state<11)
	  {
	    
			LCD12864_pos(0,0);
			LCD12864_writebyte("    系统设置    ");
			LCD12864_pos(1,0);
		    LCD12864_writebyte("经纬度显示:     ");

   	        LCD12864_pos(2,0);
		    LCD12864_writebyte("纬度:");
			LCD12864_write(1,0x30+WD_A/100000000%10);
			LCD12864_write(1,0x30+WD_A/10000000%10);
			LCD12864_write(1,0x30+WD_A/1000000%10);
			LCD12864_writebyte(".");
			LCD12864_write(1,0x30+WD_A/100000%10);
			LCD12864_write(1,0x30+WD_A/10000%10);
			LCD12864_write(1,0x30+WD_A/1000%10);
			LCD12864_write(1,0x30+WD_A/100%10);
			LCD12864_write(1,0x30+WD_A/10%10);
			LCD12864_write(1,0x30+WD_A%10);
		    LCD12864_writebyte(" ");
            LCD12864_pos(3,0);
		    LCD12864_writebyte("经度:");
			LCD12864_write(1,0x30+JD_B/100000000%10);
			LCD12864_write(1,0x30+JD_B/10000000%10);
			LCD12864_write(1,0x30+JD_B/1000000%10);
			LCD12864_writebyte(".");
			LCD12864_write(1,0x30+JD_B/100000%10);
			LCD12864_write(1,0x30+JD_B/10000%10);
			LCD12864_write(1,0x30+JD_B/1000%10);
			LCD12864_write(1,0x30+JD_B/100%10);
			LCD12864_write(1,0x30+JD_B/10%10);
			LCD12864_write(1,0x30+JD_B%10);		
			LCD12864_writebyte(" ");
	  								  
	  
	  }
  	else if(state>=11&&(state-11<Station))
	 {flag_cun=1;
            LCD12864_pos(0,0);
		    LCD12864_writebyte("  车站下行");
			if(state-10>9) LCD12864_write(1,0x30+(state-10)/10%10);
		    else 	 	   LCD12864_writebyte(" ");
			 LCD12864_write(1,0x30+(state-10)%10);
			LCD12864_writebyte("    ");
            LCD12864_pos(1,0);
			switch(state-10) 
			 {
			   case 1 :   LCD12864_writebyte(xiaxing1);     break;
			   case 2 :   LCD12864_writebyte(xiaxing2);     break;
			   case 3:    LCD12864_writebyte(xiaxing3);     break;
			   case 4 :   LCD12864_writebyte(xiaxing4);     break;
			   case 5 :   LCD12864_writebyte(xiaxing5);     break;
			 }
											
            LCD12864_pos(2,0);
		    LCD12864_writebyte("纬度:");
			LCD12864_write(1,0x30+gps_cun.Vehicle_Obj.xiaxing_WD_da[state-11]/100000000%10);
			LCD12864_write(1,0x30+gps_cun.Vehicle_Obj.xiaxing_WD_da[state-11]/10000000%10);
			LCD12864_write(1,0x30+gps_cun.Vehicle_Obj.xiaxing_WD_da[state-11]/1000000%10);
			LCD12864_writebyte(".");
			LCD12864_write(1,0x30+gps_cun.Vehicle_Obj.xiaxing_WD_da[state-11]/100000%10);
			LCD12864_write(1,0x30+gps_cun.Vehicle_Obj.xiaxing_WD_da[state-11]/10000%10);
			LCD12864_write(1,0x30+gps_cun.Vehicle_Obj.xiaxing_WD_da[state-11]/1000%10);
			LCD12864_write(1,0x30+gps_cun.Vehicle_Obj.xiaxing_WD_da[state-11]/100%10);
			LCD12864_write(1,0x30+gps_cun.Vehicle_Obj.xiaxing_WD_da[state-11]/10%10);
			LCD12864_write(1,0x30+gps_cun.Vehicle_Obj.xiaxing_WD_da[state-11]%10);
		    LCD12864_writebyte(" ");
            LCD12864_pos(3,0);
		    LCD12864_writebyte("经度:");
			LCD12864_write(1,0x30+gps_cun.Vehicle_Obj.xiaxing_JD_da[state-11]/100000000%10);
			LCD12864_write(1,0x30+gps_cun.Vehicle_Obj.xiaxing_JD_da[state-11]/10000000%10);
			LCD12864_write(1,0x30+gps_cun.Vehicle_Obj.xiaxing_JD_da[state-11]/1000000%10);
			LCD12864_writebyte(".");
			LCD12864_write(1,0x30+gps_cun.Vehicle_Obj.xiaxing_JD_da[state-11]/100000%10);
			LCD12864_write(1,0x30+gps_cun.Vehicle_Obj.xiaxing_JD_da[state-11]/10000%10);
			LCD12864_write(1,0x30+gps_cun.Vehicle_Obj.xiaxing_JD_da[state-11]/1000%10);
			LCD12864_write(1,0x30+gps_cun.Vehicle_Obj.xiaxing_JD_da[state-11]/100%10);
			LCD12864_write(1,0x30+gps_cun.Vehicle_Obj.xiaxing_JD_da[state-11]/10%10);
			LCD12864_write(1,0x30+gps_cun.Vehicle_Obj.xiaxing_JD_da[state-11]%10);		
			LCD12864_writebyte(" ");			
	 }
  	else if(state>=11&&(state-11<Station*2))
	 {flag_cun=1;
            LCD12864_pos(0,0);
		    LCD12864_writebyte("  车站上行");
			if(state-10-Station>9) LCD12864_write(1,0x30+(state-10-Station)/10%10);
		    else 	 	           LCD12864_writebyte(" ");
			 LCD12864_write(1,0x30+(state-10-Station)%10);
			LCD12864_writebyte("    ");
            LCD12864_pos(1,0);
			switch(state-10-Station) 
			 {
			   case 1 :   LCD12864_writebyte(shangxing1);     break;
			   case 2 :   LCD12864_writebyte(shangxing2);     break;
			   case 3:    LCD12864_writebyte(shangxing3);     break;
			   case 4 :   LCD12864_writebyte(shangxing4);     break;
			   case 5 :   LCD12864_writebyte(shangxing5);     break;
			 }
            LCD12864_pos(2,0);
		    LCD12864_writebyte("纬度:");
			LCD12864_write(1,0x30+gps_cun.Vehicle_Obj.shangxing_WD_da[state-11-Station]/100000000%10);
			LCD12864_write(1,0x30+gps_cun.Vehicle_Obj.shangxing_WD_da[state-11-Station]/10000000%10);
			LCD12864_write(1,0x30+gps_cun.Vehicle_Obj.shangxing_WD_da[state-11-Station]/1000000%10);
			LCD12864_writebyte(".");
			LCD12864_write(1,0x30+gps_cun.Vehicle_Obj.shangxing_WD_da[state-11-Station]/100000%10);
			LCD12864_write(1,0x30+gps_cun.Vehicle_Obj.shangxing_WD_da[state-11-Station]/10000%10);
			LCD12864_write(1,0x30+gps_cun.Vehicle_Obj.shangxing_WD_da[state-11-Station]/1000%10);
			LCD12864_write(1,0x30+gps_cun.Vehicle_Obj.shangxing_WD_da[state-11-Station]/100%10);
			LCD12864_write(1,0x30+gps_cun.Vehicle_Obj.shangxing_WD_da[state-11-Station]/10%10);
			LCD12864_write(1,0x30+gps_cun.Vehicle_Obj.shangxing_WD_da[state-11-Station]%10);
		    LCD12864_writebyte(" ");
            LCD12864_pos(3,0);
		    LCD12864_writebyte("经度:");
			LCD12864_write(1,0x30+gps_cun.Vehicle_Obj.shangxing_JD_da[state-11-Station]/100000000%10);
			LCD12864_write(1,0x30+gps_cun.Vehicle_Obj.shangxing_JD_da[state-11-Station]/10000000%10);
			LCD12864_write(1,0x30+gps_cun.Vehicle_Obj.shangxing_JD_da[state-11-Station]/1000000%10);
			LCD12864_writebyte(".");
			LCD12864_write(1,0x30+gps_cun.Vehicle_Obj.shangxing_JD_da[state-11-Station]/100000%10);
			LCD12864_write(1,0x30+gps_cun.Vehicle_Obj.shangxing_JD_da[state-11-Station]/10000%10);
			LCD12864_write(1,0x30+gps_cun.Vehicle_Obj.shangxing_JD_da[state-11-Station]/1000%10);
			LCD12864_write(1,0x30+gps_cun.Vehicle_Obj.shangxing_JD_da[state-11-Station]/100%10);
			LCD12864_write(1,0x30+gps_cun.Vehicle_Obj.shangxing_JD_da[state-11-Station]/10%10);
			LCD12864_write(1,0x30+gps_cun.Vehicle_Obj.shangxing_JD_da[state-11-Station]%10);		
			LCD12864_writebyte(" ");			
	 }

//  我想的流程，GPS公交车，在指定点，显示  “当前站点”	“XXXX”	    ，不在指定点，就显示   “下一站”  “XXXXX”
//  根据上行下行来判断下一站地点		
 }


void key_dispose() 
 {
   	if(!key1) 
	 {
	 	if(key1_flag) 
		 {
		   key1_flag=0;
		   state=(state+1)%(11+(Station*2)); 
		   if(state==7)state=8;

		 } 
	 }
	 else key1_flag=1;


   	if(!key2) 
	 {
	 	if(key2_flag) 
		 {
		   key2_flag=0;
			  if(state==1)
			    {
				  time_data_1[5]=(time_data_1[5]%99)+1; 
				}
			   else	if(state==2)
			    {
	              time_data_1[4]=(time_data_1[4]%12)+1;
				}
	
			   else	if(state==3)
			    {
					if(time_data_1[4]==1|time_data_1[4]==3|time_data_1[4]==5|time_data_1[4]==7|time_data_1[4]==8|time_data_1[4]==10|time_data_1[4]==12)
					{
						time_data_1[3]=(time_data_1[3]%31)+1;
					}else if(time_data_1[4]==3|time_data_1[4]==6|time_data_1[4]==9|time_data_1[4]==11)
					{
						time_data_1[3]=(time_data_1[3]%30)+1;					
					}else   if((time_data_1[5]%4==0&&time_data_1[5]%100!=0)||(time_data_1[5]%400==0))
					{
						time_data_1[3]=(time_data_1[3]%29)+1;					
					}
					else 
					{
						time_data_1[3]=(time_data_1[3]%28)+1;					
					}
				}
			   else	if(state==4)
			    {
	               time_data_1[2]=(time_data_1[2]+1)%24;
				}
			   else	if(state==5)
			    {
	               time_data_1[1]=(time_data_1[1]+1)%60;
				}				 		 	   	  
			   else	if(state==6)
			    {
	               time_data_1[0]=(time_data_1[0]+1)%60;
				}
			   else	if(state==7)
			    {
//	               	if(Station<15)	 Station++;
				}
			   else	if(state==8)
			    {
	               	if(Sound<30)	 Sound++;
				}
			   else	if(state==9)
			    {
	               	GPS_Write=1; 
				}
			   else	if(state==10)
			    {
	             // 	Mode=0; 
				}	  
		 } 
		 if(sec==0) 
		  {
		          if(state==1)
				    {
					  time_data_1[5]=(time_data_1[5]%99)+1; 						  
					}
				   else	if(state==2)
				    {
		              time_data_1[4]=(time_data_1[4]%12)+1;
					}
		
				   else	if(state==3)
				    {
						if(time_data_1[4]==1|time_data_1[4]==3|time_data_1[4]==5|time_data_1[4]==7|time_data_1[4]==8|time_data_1[4]==10|time_data_1[4]==12)
						{
							time_data_1[3]=(time_data_1[3]%31)+1;
						}else if(time_data_1[4]==3|time_data_1[4]==6|time_data_1[4]==9|time_data_1[4]==11)
						{
							time_data_1[3]=(time_data_1[3]%30)+1;					
						}else   if((time_data_1[5]%4==0&&time_data_1[5]%100!=0)||(time_data_1[5]%400==0))
						{
							time_data_1[3]=(time_data_1[3]%29)+1;					
						}
						else 
						{
							time_data_1[3]=(time_data_1[3]%28)+1;					
						}
					}
				   else	if(state==4)
				    {
		               time_data_1[2]=(time_data_1[2]+1)%24;
					}
				   else	if(state==5)
				    {
		               time_data_1[1]=(time_data_1[1]+1)%60;
					}				 		 	   	  
				   else	if(state==6)
				    {
		               time_data_1[0]=(time_data_1[0]+1)%60;
					}
				   else	if(state==7)
				    {
//		               	if(Station<15)	 Station++;
					}
				   else	if(state==8)
				    {
		               	if(Sound<30)	 Sound++;
					}
				   else	if(state==9)
				    {
		               	GPS_Write=1; 
					}
				   else	if(state==10)
				    {
		        //   	Mode=0; 
					}
		  }
	 }
	 else
	  {
	     if(key2_flag==0)
		  {
		    key2_flag=1;
			memory_flag=1;	  Sound_flag=1;
		  }
		  sec=2;
	  } 

   	if(!key3) 
	 {
	 	if(key3_flag) 
		 {
		     key3_flag=0;
			  if(state==1)
			    {
				   if(time_data_1[5]>0)time_data_1[5]--;
				}
			   else	if(state==2)
			    {
	               if(time_data_1[4]>1)time_data_1[4]--;
				}
	
			   else	if(state==3)
			    {
	               if(time_data_1[3]>1)time_data_1[3]--;
				}
			   else	if(state==4)
			    {
	               if(time_data_1[2]>0)time_data_1[2]--;
				}
			   else	if(state==5)
			    {
	              if(time_data_1[1]>0)time_data_1[1]--;
				}
			   else	if(state==6)
			    {
	              if(time_data_1[0]>0)time_data_1[0]--;
				}
			  else	if(state==7)
			    {
//		            if(Station>0)	 Station--;
				}
		      else	if(state==8)
			    {
		           if(Sound>0)	 Sound--;
			    }
			  else	if(state==9)
				{
		           GPS_Write=0; 
				}
		     else	if(state==10)
				{
		        //        	Mode=1; 
				}	
		 }
		 if(sec1==0) 
		  {
				  if(state==1)
				    {
					   if(time_data_1[5]>0)time_data_1[5]--;
					}
				   else	if(state==2)
				    {
		               if(time_data_1[4]>1)time_data_1[4]--;
					}
		
				   else	if(state==3)
				    {
		               if(time_data_1[3]>1)time_data_1[3]--;
					}
				   else	if(state==4)
				    {
		               if(time_data_1[2]>0)time_data_1[2]--;
					}
				   else	if(state==5)
				    {
		              if(time_data_1[1]>0)time_data_1[1]--;
					}
				   else	if(state==6)
				    {
		              if(time_data_1[0]>0)time_data_1[0]--;
					}
				  else	if(state==7)
				    {
//			            if(Station>0)	 Station--;
					}
		         else	if(state==8)
			       {
		               	if(Sound>0)	 Sound--;
			       }
			     else	if(state==9)
				  {
		             GPS_Write=0; 
				  }
		         else if(state==10)
				  {
		        //        	Mode=1; 
				  }	
		  }   
	 }
	 else 
	  {
	    if(key3_flag==0) 
		 {
		   key3_flag=1;
		   memory_flag=1;   Sound_flag=1;
		 }
		 sec1=2;
	  }

	if(!key4) 
	 {
	   if(key4_flag)
	    {
		   key4_flag=0;
		   if(state==0) 
		    {
			   A_M=~A_M;
			   Upstream_Down=1;
			   memory_flag=1;
			}
		}
	 }
	 else 
	 {
	   key4_flag=1;
	 }

	if(!key5) 
	 {
	   if(key5_flag)
	    {
		   key5_flag=0;
		   if(state==0) 
		    {
			   Upstream_Down=~Upstream_Down;
			   if(A_M) 
			    {
				   if(Upstream_Down) 		  //说明是下行
				    {
						Station_Count=0;
					}
					else 					 //说明是上行
					 {
					   Station_Count=7;
					 }
				}
			}
		}
	 }
	 else 
	 {
	   if(key5_flag==0)
	    {
		  key5_flag=1;
		  memory_flag=1;
		}
	   
	 }

	if(!key6) 
	 {
	   if(key6_flag&&state==0)
	    {
		   key6_flag=0;
				   
				  if(A_M) 
				   {
					   position=0;
					   if(Upstream_Down) 	  //如果是下行
					    {
						   if(Station_Count<Station+1) 
						    {	  
							  Send_Appoint_Music(Station_Count);   //下行正常播报
							  Station_Count++;
							}
							else 
							 {
							   Upstream_Down=0;	
							   Send_Appoint_Music(Station+1); //终点站	
							   Station_Count--;
							 }
						}
						else				  //如果是上行
						 {
						   if(Station_Count>1) 
						    {
							  Station_Count--;
							  Send_Appoint_Music(((Station+1)-Station_Count)+Station); //终点站	
							}
							else 
							 {
							   Upstream_Down=1;
							   Send_Appoint_Music(Station_Count);   //下行正常播报
							   Station_Count++;
							 }
						 }				     
				   } 
		}
	 }
	 else 
	 {
	   key6_flag=1;
	 }
  if((!key7)&&(flag_cun!=0))//确定存储经纬度
	{ 
//		 LCD12864_pos(2,0);
//		    LCD12864_writebyte("纬度:");
//			LCD12864_write(1,0x30+WD_A/100000000%10);

//            LCD12864_pos(3,0);
//		    LCD12864_writebyte("经度:");
//			LCD12864_write(1,0x30+JD_B/100000000%10);
			
		if(state>=11&&(state-11<Station))
	 {  gps_cun.Vehicle_Obj.xiaxing_WD_da[state-11]=WD_A;
		  gps_cun.Vehicle_Obj.xiaxing_JD_da[state-11]=JD_B;
		  LCD12864_pos(2,0);
		  LCD12864_writebyte("纬度:");
			LCD12864_write(1,0x30+gps_cun.Vehicle_Obj.xiaxing_WD_da[state-11]/100000000%10);
			LCD12864_write(1,0x30+gps_cun.Vehicle_Obj.xiaxing_WD_da[state-11]/10000000%10);
			LCD12864_write(1,0x30+gps_cun.Vehicle_Obj.xiaxing_WD_da[state-11]/1000000%10);
			LCD12864_writebyte(".");
			LCD12864_write(1,0x30+gps_cun.Vehicle_Obj.xiaxing_WD_da[state-11]/100000%10);
			LCD12864_write(1,0x30+gps_cun.Vehicle_Obj.xiaxing_WD_da[state-11]/10000%10);
			LCD12864_write(1,0x30+gps_cun.Vehicle_Obj.xiaxing_WD_da[state-11]/1000%10);
			LCD12864_write(1,0x30+gps_cun.Vehicle_Obj.xiaxing_WD_da[state-11]/100%10);
			LCD12864_write(1,0x30+gps_cun.Vehicle_Obj.xiaxing_WD_da[state-11]/10%10);
			LCD12864_write(1,0x30+gps_cun.Vehicle_Obj.xiaxing_WD_da[state-11]%10);
		  LCD12864_writebyte(" ");
      LCD12864_pos(3,0);
		  LCD12864_writebyte("经度:");
			LCD12864_write(1,0x30+gps_cun.Vehicle_Obj.xiaxing_JD_da[state-11]/100000000%10);
			LCD12864_write(1,0x30+gps_cun.Vehicle_Obj.xiaxing_JD_da[state-11]/10000000%10);
			LCD12864_write(1,0x30+gps_cun.Vehicle_Obj.xiaxing_JD_da[state-11]/1000000%10);
			LCD12864_writebyte(".");
			LCD12864_write(1,0x30+gps_cun.Vehicle_Obj.xiaxing_JD_da[state-11]/100000%10);
			LCD12864_write(1,0x30+gps_cun.Vehicle_Obj.xiaxing_JD_da[state-11]/10000%10);
			LCD12864_write(1,0x30+gps_cun.Vehicle_Obj.xiaxing_JD_da[state-11]/1000%10);
			LCD12864_write(1,0x30+gps_cun.Vehicle_Obj.xiaxing_JD_da[state-11]/100%10);
			LCD12864_write(1,0x30+gps_cun.Vehicle_Obj.xiaxing_JD_da[state-11]/10%10);
			LCD12864_write(1,0x30+gps_cun.Vehicle_Obj.xiaxing_JD_da[state-11]%10);		
			LCD12864_writebyte(" ");			
	 }
  	else if(state>=11&&(state-11<Station*2))
	 {  gps_cun.Vehicle_Obj.shangxing_JD_da[state-11-Station]=JD_B;
		  gps_cun.Vehicle_Obj.shangxing_WD_da[state-11-Station]=WD_A;
      LCD12864_pos(2,0);
		  LCD12864_writebyte("纬度:");
			LCD12864_write(1,0x30+gps_cun.Vehicle_Obj.shangxing_WD_da[state-11-Station]/100000000%10);
			LCD12864_write(1,0x30+gps_cun.Vehicle_Obj.shangxing_WD_da[state-11-Station]/10000000%10);
			LCD12864_write(1,0x30+gps_cun.Vehicle_Obj.shangxing_WD_da[state-11-Station]/1000000%10);
			LCD12864_writebyte(".");
			LCD12864_write(1,0x30+gps_cun.Vehicle_Obj.shangxing_WD_da[state-11-Station]/100000%10);
			LCD12864_write(1,0x30+gps_cun.Vehicle_Obj.shangxing_WD_da[state-11-Station]/10000%10);
			LCD12864_write(1,0x30+gps_cun.Vehicle_Obj.shangxing_WD_da[state-11-Station]/1000%10);
			LCD12864_write(1,0x30+gps_cun.Vehicle_Obj.shangxing_WD_da[state-11-Station]/100%10);
			LCD12864_write(1,0x30+gps_cun.Vehicle_Obj.shangxing_WD_da[state-11-Station]/10%10);
			LCD12864_write(1,0x30+gps_cun.Vehicle_Obj.shangxing_WD_da[state-11-Station]%10);
		    LCD12864_writebyte(" ");
            LCD12864_pos(3,0);
		    LCD12864_writebyte("经度:");
			LCD12864_write(1,0x30+gps_cun.Vehicle_Obj.shangxing_JD_da[state-11-Station]/100000000%10);
			LCD12864_write(1,0x30+gps_cun.Vehicle_Obj.shangxing_JD_da[state-11-Station]/10000000%10);
			LCD12864_write(1,0x30+gps_cun.Vehicle_Obj.shangxing_JD_da[state-11-Station]/1000000%10);
			LCD12864_writebyte(".");
			LCD12864_write(1,0x30+gps_cun.Vehicle_Obj.shangxing_JD_da[state-11-Station]/100000%10);
			LCD12864_write(1,0x30+gps_cun.Vehicle_Obj.shangxing_JD_da[state-11-Station]/10000%10);
			LCD12864_write(1,0x30+gps_cun.Vehicle_Obj.shangxing_JD_da[state-11-Station]/1000%10);
			LCD12864_write(1,0x30+gps_cun.Vehicle_Obj.shangxing_JD_da[state-11-Station]/100%10);
			LCD12864_write(1,0x30+gps_cun.Vehicle_Obj.shangxing_JD_da[state-11-Station]/10%10);
			LCD12864_write(1,0x30+gps_cun.Vehicle_Obj.shangxing_JD_da[state-11-Station]%10);		
			LCD12864_writebyte(" ");			
	 }

		sequential_write_flash_in_one_sector(0x08200, sizeof(gps_data), pp);;
	  while(!key7);
	}
  if(!key8) 
   {
   	  if(key8_flag) 
	   {
	     key8_flag=0;
		   state=0;
			 flag_cun=0; 
	   }
   }
   else 
    {
	  key8_flag=1;
	}


 }
//配置定时0用作常规定时，使用定时器1用作串口1波特率发生器，9600
void TimeInt(void)		 //系统定时器初始化配置
{
	TMOD=0x21;	   //定义两个定时器
	TH1=0xFD;
	TL1=0xFD;	   //定时器1用于产生波特率  晶振11.0592
	TL0 = 0x00;		//设置定时初值
	TH0 = 0x4C;		//设置定时初值	
	SCON=0x50;
	PCON=0;
	EA=1;          //开总中断
	ES=1;          //ES-串行中断允许控制位   ES = 1   允许串行中断。
	TR1=1;         //启动定时器开始工作
	ET0=1;
	TR0=1; 	
}

void led_dispose()  //LED显示函数
 {
   if(A_M) 
    {
	  led0=0;
	  led1=1;
	}
	else 
	{
	  led0=1;
	  led1=0;
	}

   if(Upstream_Down) 
    {
	  led2=0;
	  led3=1;
	}
	else 
	{
	  led2=1;
	  led3=0;
	}
 }

void GPS_Route_Dispose()
 {
  if(A_M==0) 	 
   {		
		 
	  if((gps_cun.Vehicle_Obj.xiaxing_JD_da[0]+JD_Difference>=JD_B)&&(JD_B>=gps_cun.Vehicle_Obj.xiaxing_JD_da[0]-JD_Difference)&&(gps_cun.Vehicle_Obj.xiaxing_WD_da[0]+WD_Difference>=WD_A)&&(WD_A>=gps_cun.Vehicle_Obj.xiaxing_WD_da[0]-WD_Difference))
	   {
		    Upstream_Down=1;
	   }
	   else if((gps_cun.Vehicle_Obj.shangxing_JD_da[0]+JD_Difference>=JD_B)&&(JD_B>=gps_cun.Vehicle_Obj.shangxing_JD_da[0]-JD_Difference)&&(gps_cun.Vehicle_Obj.shangxing_WD_da[0]+WD_Difference>=WD_A)&&(WD_A>=gps_cun.Vehicle_Obj.shangxing_WD_da[0]-WD_Difference))
		{
		    Upstream_Down=0;
		}
		 
		 
		 if(Upstream_Down==1)   //等于1，说明自动模式下处于下行状态，这时候对检测到的GPS，做出对应处理
		   {
					  if((gps_cun.Vehicle_Obj.xiaxing_JD_da[0]+JD_Difference>=JD_B)&&(JD_B>=gps_cun.Vehicle_Obj.xiaxing_JD_da[0]-JD_Difference)&&(gps_cun.Vehicle_Obj.xiaxing_WD_da[0]+WD_Difference>=WD_A)&&(WD_A>=gps_cun.Vehicle_Obj.xiaxing_WD_da[0]-WD_Difference))
					   {
					     Station_Count=1;  position=1;
						 if(count!=Station_Count)
						  {
						    count=Station_Count;
							Send_Appoint_Music(Station_Count);   //下行正常播报

						  }
					   }
					   else  if((gps_cun.Vehicle_Obj.xiaxing_JD_da[1]+JD_Difference>JD_B)&&(JD_B>gps_cun.Vehicle_Obj.xiaxing_JD_da[1]-JD_Difference)&&(gps_cun.Vehicle_Obj.xiaxing_WD_da[1]+WD_Difference>WD_A)&&(WD_A>gps_cun.Vehicle_Obj.xiaxing_WD_da[1]-WD_Difference))
					   {
					     Station_Count=2;  position=1;
						 if(count!=Station_Count)
						  {
							 if(count!=Station_Count)
							  {
							    count=Station_Count;
								Send_Appoint_Music(Station_Count);   //下行正常播报
							  }
						  }
					   }
					   else  if((gps_cun.Vehicle_Obj.xiaxing_JD_da[2]+JD_Difference>JD_B)&&(JD_B>gps_cun.Vehicle_Obj.xiaxing_JD_da[2]-JD_Difference)&&(gps_cun.Vehicle_Obj.xiaxing_WD_da[2]+WD_Difference>WD_A)&&(WD_A>gps_cun.Vehicle_Obj.xiaxing_WD_da[2]-WD_Difference))
					   {
					     Station_Count=3;  position=1;
						 if(count!=Station_Count)
						  {
							 if(count!=Station_Count)
							  {
							    count=Station_Count;
								Send_Appoint_Music(Station_Count);   //下行正常播报	
							  }
						  }
					   }
					   else  if((gps_cun.Vehicle_Obj.xiaxing_JD_da[3]+JD_Difference>JD_B)&&(JD_B>gps_cun.Vehicle_Obj.xiaxing_JD_da[3]-JD_Difference)&&(gps_cun.Vehicle_Obj.xiaxing_WD_da[3]+WD_Difference>WD_A)&&(WD_A>gps_cun.Vehicle_Obj.xiaxing_WD_da[3]-WD_Difference))
					   {
					     Station_Count=4;  position=1;
						 if(count!=Station_Count)
						  {
							 if(count!=Station_Count)
							  {
							    count=Station_Count;
								Send_Appoint_Music(Station_Count);   //下行正常播报
							
							  }
						  }
					   }
					   else  if((gps_cun.Vehicle_Obj.xiaxing_JD_da[4]+JD_Difference>JD_B)&&(JD_B>gps_cun.Vehicle_Obj.xiaxing_JD_da[4]-JD_Difference)&&(gps_cun.Vehicle_Obj.xiaxing_WD_da[4]+WD_Difference>WD_A)&&(WD_A>gps_cun.Vehicle_Obj.xiaxing_WD_da[4]-WD_Difference))
					   {
					     Station_Count=5;   position=1;
						 if(count!=Station_Count)
						  {
							 if(count!=Station_Count)
							  {
							    count=Station_Count;
								Send_Appoint_Music(Station_Count);   //下行正常播报
							  }
						  }
					   }
					   else  position=0;
		   
		   }
		   else 		  //下面是上行数据
		    {			
						  if((gps_cun.Vehicle_Obj.shangxing_JD_da[0]+JD_Difference>=JD_B)&&(JD_B>=gps_cun.Vehicle_Obj.shangxing_JD_da[0]-JD_Difference)&&(gps_cun.Vehicle_Obj.shangxing_WD_da[0]+WD_Difference>=WD_A)&&(WD_A>=gps_cun.Vehicle_Obj.shangxing_WD_da[0]-WD_Difference))
						   {
						     Station_Count=1;  position=1;
							 if(count!=Station_Count)
							  {
							    count=Station_Count;
								Send_Appoint_Music(Station+1);   //下行正常播报
	
							  }
						   }
						   else  if((gps_cun.Vehicle_Obj.shangxing_JD_da[1]+JD_Difference>=JD_B)&&(JD_B>=gps_cun.Vehicle_Obj.shangxing_JD_da[1]-JD_Difference)&&(gps_cun.Vehicle_Obj.shangxing_WD_da[1]+WD_Difference>=WD_A)&&(WD_A>=gps_cun.Vehicle_Obj.shangxing_WD_da[1]-WD_Difference))
						   {
						     Station_Count=2;  position=1;
							 if(count!=Station_Count)
							  {
								 if(count!=Station_Count)
								  {
								    count=Station_Count;
									Send_Appoint_Music(Station+2);   //下行正常播报
								  }
							  }
						   }
						   else  if((gps_cun.Vehicle_Obj.shangxing_JD_da[2]+JD_Difference>=JD_B)&&(JD_B>=gps_cun.Vehicle_Obj.shangxing_JD_da[2]-JD_Difference)&&(gps_cun.Vehicle_Obj.shangxing_WD_da[2]+WD_Difference>=WD_A)&&(WD_A>=gps_cun.Vehicle_Obj.shangxing_WD_da[2]-WD_Difference))
						   {
						     Station_Count=3;  position=1;
							 if(count!=Station_Count)
							  {
								 if(count!=Station_Count)
								  {
								    count=Station_Count;
									Send_Appoint_Music(Station+3);   //下行正常播报	
								  }
							  }
						   }
						   else  if((gps_cun.Vehicle_Obj.shangxing_JD_da[3]+JD_Difference>=JD_B)&&(JD_B>=gps_cun.Vehicle_Obj.shangxing_JD_da[3]-JD_Difference)&&(gps_cun.Vehicle_Obj.shangxing_WD_da[3]+WD_Difference>=WD_A)&&(WD_A>=gps_cun.Vehicle_Obj.shangxing_WD_da[3]-WD_Difference))
						   {
						     Station_Count=4;  position=1;
							 if(count!=Station_Count)
							  {
								 if(count!=Station_Count)
								  {
								    count=Station_Count;
									Send_Appoint_Music(Station+4);   //下行正常播报
								
								  }
							  }
						   }
						   else  if((gps_cun.Vehicle_Obj.shangxing_JD_da[4]+JD_Difference>=JD_B)&&(JD_B>=gps_cun.Vehicle_Obj.shangxing_JD_da[4]-JD_Difference)&&(gps_cun.Vehicle_Obj.shangxing_WD_da[4]+WD_Difference>=WD_A)&&(WD_A>=gps_cun.Vehicle_Obj.shangxing_WD_da[4]-WD_Difference))
						   {
						     Station_Count=5;   position=1;
							 if(count!=Station_Count)
							  {
								 if(count!=Station_Count)
								  {
								    count=Station_Count;
									Send_Appoint_Music(Station+5);   //下行正常播报
								  }
							  }
						   }
						   else  position=0;			
			}	 
		 }
   }
void main()
 {
   TimeInt();
   LCD12864_init();	     //调用显示函数
// Send_Appoint_Music(1);

   read_memory();
	Mode=0;
   if(Mode) 
    {
	  
	}
    read_GPS();

   while(1) 
    {  
	  read_time1();  //读取时间函数，GPS_Write等于1，说明将获取的GPS是将赋值给时间
	  GPS_Route_Dispose();
	  Display();		 //显示函数
	  key_dispose();	 //按键处理函数
	  memory();
	  memory_GPS();
	  led_dispose();

	  if(Busy==0) 
	   {
	     if(Sound_flag)
		  {	 
		    Sound_flag=0;
		    Send_Appoint_Sound(Sound); 
		  }
	   }
	}
 }


//定时器0中断服务函数
void time0() interrupt 1
 {
   TH0=0x3C;
   TL0=0xb0; 	    //定时中断50ms
   ms++;			//ms变量相当于50ms加一
   key_dispose();
	if(ms%10==0) 	//500毫秒执行一次
	 { 
	   Sound_flag=1;
	   s0=~s0;
	   if(GPS_time!=0) GPS_time--; //判断GPS信号是否丢失变量每隔500ms减一
	 } 
	if(ms%20==0)	 
	 {
       if(sec!=0) sec--;
	   if(sec1!=0) sec1--; 	 Display_Reversal=~Display_Reversal;
	 }  

   if(ms>=40)
    {
	   ms=0;	 //一秒定时，清零 
	}
 }


//  通讯中断接收程序   中断函数无返回值
void uart_rx(void)  interrupt 4	  
{
	float abc,de,fghi;
	uchar i,j,k;
	if (RI)
    {
        RI=0; 
		GPS_dat[subscript]=SBUF;
		subscript=(subscript+1)%100;
		if(GPS_dat[(subscript+100-1)%100] == '\n'&&GPS_dat[(subscript+100-2)%100] == '\r')
		{			
			if(verify_GPSdat("$GPGLL,")==1)
			{
				if(subscript>30)
				{
					for(i=7;GPS_dat[i]!='N';i++)
					{
						if(i>21) break;
					}
					if(i<22)
					{
						for(k=0;k<(i-8);k++)
						{
							A_dat[k]=GPS_dat[7+k];	
							A_dat[k+1]=0;
						}
						if(A_dat[3]=='.')
						{
							abc=(A_dat[0]-0x30);
							de=(A_dat[1]-0x30)*10+(A_dat[2]-0x30);
							fghi=(A_dat[4]-0x30)*1000+(A_dat[5]-0x30)*100+(A_dat[6]-0x30)*10+(A_dat[7]-0x30)+(A_dat[8]-0x30)*1.0;
							latitude = abc+(de/60.0)+(fghi/600000.0);
							WD_A=latitude*1000000;

						}else if(A_dat[4]=='.')
						{
							abc=(A_dat[0]-0x30)*10+(A_dat[1]-0x30);
							de=(A_dat[2]-0x30)*10+(A_dat[3]-0x30);
							fghi=(A_dat[5]-0x30)*1000+(A_dat[6]-0x30)*100+(A_dat[7]-0x30)*10+(A_dat[8]-0x30)+(A_dat[9]-0x30)*0.1;
							latitude = abc+(de/60.0)+(fghi/600000.0);
							WD_A=latitude*1000000;
						}
					}
					for(j=i;GPS_dat[j]!='E';j++)
					{
						if(j>40) break;
					}
					if(j<40)
					{
						for(k=0;k<(j-i-3);k++)
						{
							B_dat[k]=GPS_dat[i+2+k];
							B_dat[k+1]=0;	
						}
						if(B_dat[4]=='.')
						{
							abc=(B_dat[0]-0x30)*10+(B_dat[1]-0x30);
							de=(B_dat[2]-0x30)*10+(B_dat[3]-0x30);
							fghi=(B_dat[5]-0x30)*1000+(B_dat[6]-0x30)*100+(B_dat[7]-0x30)*10+(B_dat[8]-0x30)+(B_dat[9]-0x30)*0.1;
							longitude = abc+(de/60.0)+(fghi/600000.0);
							JD_B=longitude*1000000;
						}else if(B_dat[5]=='.')
						{
							abc=(B_dat[0]-0x30)*100+(B_dat[1]-0x30)*10+(B_dat[2]-0x30);
							de=(B_dat[3]-0x30)*10+(B_dat[4]-0x30);
							fghi=(B_dat[6]-0x30)*1000+(B_dat[7]-0x30)*100+(B_dat[8]-0x30)*10+(B_dat[9]-0x30)+(B_dat[10]-0x30)*0.1;
							longitude = abc+(de/60.0)+(fghi/600000.0);
							JD_B=longitude*1000000;
						}
					}
					GPS_time=10;
				}else
				{
				//	longitude=0;
				//	latitude=0;	
					for(i=0;i<20;i++)
					{
						A_dat[i]=0;
						B_dat[i]=0;	
					}
				}																						
			}
			if(verify_GPSdat("$GPRMC,")==1 && GPS_dat[17] == 'A' && GPS_dat[51] == ',' && GPS_dat[52] == ','&&GPS_Write==1)
			{
				Time_Calibration=10;
				NowTime[2]=(GPS_dat[7]-0x30)*10+(GPS_dat[8]-0x30);
				NowTime[1]=(GPS_dat[9]-0x30)*10+(GPS_dat[10]-0x30);
				NowTime[0]=(GPS_dat[11]-0x30)*10+(GPS_dat[12]-0x30);
				
				NowTime[3]=(GPS_dat[53]-0x30)*10+(GPS_dat[54]-0x30);
				NowTime[4]=(GPS_dat[55]-0x30)*10+(GPS_dat[56]-0x30);
				NowTime[6]=(GPS_dat[57]-0x30)*10+(GPS_dat[58]-0x30);
				NowTime[2]=NowTime[2]+8;
				
				if(NowTime[2]>=24)
				{
					NowTime[2]=NowTime[2]%24;
					NowTime[3]=NowTime[3]+1;
					if(NowTime[4]==1|NowTime[4]==3|NowTime[4]==5|NowTime[4]==7|NowTime[4]==8|NowTime[4]==10|NowTime[4]==12)
					{
						if(NowTime[3]>31)
						{
							NowTime[3]=NowTime[3]-31;
							NowTime[4]=NowTime[4]+1;
							if(NowTime[4]>12)
							{
								NowTime[4]=NowTime[4]-12;
								NowTime[6]++;
							}	
						}
					}else if(NowTime[4]==4|NowTime[4]==6|NowTime[4]==9|NowTime[4]==11)
					{
						if(NowTime[3]>30)
						{
							NowTime[3]=NowTime[3]-30;
							NowTime[4]=NowTime[4]+1;
							if(NowTime[4]>12)
							{
								NowTime[4]=NowTime[4]-12;
								NowTime[6]++;
							}	
						}			
					}else
					{
						if(NowTime[3]>29)
						{
							NowTime[3]=NowTime[3]-29;
							NowTime[4]=NowTime[4]+1;
							if(NowTime[4]>12)
							{
								NowTime[4]=NowTime[4]-12;
								NowTime[6]++;
							}	
						}			
					}		
				}	  // 0 秒  1分  2时 ，三日  四月   6 年
						ds1302write(0x8e,0x00); 
						ds1302write(0x80,0x80);
						for(i=0;i<7;i++)
						{
							time_data_2[i]=NowTime[i]/10;
							time_data_3[i]=NowTime[i]%10;	  
						}
						for(i=0;i<7;i++)
						{
							time_data_4[i]=time_data_2[i]*16+time_data_3[i];						  
						}
						ds1302write(0x80,time_data_4[0]);
						ds1302write(0x82,time_data_4[1]);
						ds1302write(0x84,time_data_4[2]);
						ds1302write(0x86,time_data_4[3]);
						ds1302write(0x88,time_data_4[4]);
						ds1302write(0x8C,time_data_4[6]);
						GPS_Write=0;
			}
			subscript=0;
		}
	}
}