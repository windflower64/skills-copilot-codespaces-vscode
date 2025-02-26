#include <reg51.H>
#include "LCD12864.h"
#include "1302.h"
#include "60S2EEPROM.h"		
#include <absacc.h>		  //???
#include <string.h>		  //???
#define  AA  3000000
//վ��   GPS

				  
sbit key1=P1^0;
sbit key2=P1^1;
sbit key3=P1^2;
sbit key4=P1^3;
sbit key5=P1^4;
sbit key6=P1^5;
sbit key7=P1^6;
sbit key8=P1^7;		//���尴��IO

sbit led0=P3^4;
sbit led1=P3^5;
sbit led2=P3^6;
sbit led3=P3^7;	 	//����ָʾ��IO

sbit Music_Busy=P3^2;	  //����

bit key1_flag=0;
bit key2_flag=0;
bit key3_flag=0;
bit key4_flag=0;
bit key5_flag=0;
bit key6_flag=0;
bit key7_flag=0;
bit key8_flag=0;    //���尴��λ����



typedef struct {
ulong	shangxing_JD_da[5];// ���о������ݣ�5 վ��
ulong shangxing_WD_da[5];	// ����ά�ȶ����ݣ�5 վ��
ulong xiaxing_JD_da[5];	// ���о������ݣ�5 վ��
ulong xiaxing_WD_da[5];	//����ά�ȶ����ݣ�5 վ��
	
}gps_data;

typedef union
 {
     gps_data Vehicle_Obj;  
	   // �Խṹ�巽ʽ�洢 GPS ����  
     uchar gps_Bytes[sizeof(gps_data)]; 
	   // ���ֽ����鷽ʽ�洢 GPS ����
 }gps_shuju;
gps_shuju xdata gps_cun;
 //ʹ�� xdata �ؼ��ִ洢�� �ⲿ RAM������ GPS վ�����ݡ�
uchar flag_cun=0;//�Ƿ�洢 GPS ���ݵı�־λ
uchar Station_Count=1;//վ���������ʼΪ 1
sbit Busy=P3^2;// ��������ģ��� BUSY ״̬���
bit position=0;// λ�ñ�־λ
bit Display_Reversal=0;// ��ʾ��ת
uchar state=0;//��ʾ״̬���������� LCD ���棩
bit s0=0;		//������˸�������������� LCD12864��
uchar ms=0;			//��ʱ���õ��ı���
// state ���ã�
//���� LCD12864 ��ʾ���ݣ����� ����ѡ��ʱ�������վ�����

//��������Bugһ�����Զ��£����յ�վû���Զ��л����С�����
//һ����  �Զ��� ����վ������������Ӧ

uchar sec=0;// ���������
uchar sec1=0;// �������������
bit memory_flag=0;// �����־λ���洢����ʱ�����õ���

uchar Sound=19;			//������С����
uchar Station=5;		//��վ��������

bit Mode=0;			    //����0����̻�    ����1�����Զ���
bit A_M=0;          //�Զ�ģʽ���ֶ�ģʽ
bit Upstream_Down=1;// ����/���б�־λ
uchar count=0;//������������������վ���л���

uint JD_Difference=100;	 //34.800340,113.499447
uint WD_Difference=100;// ����ľ�γ�����

uchar *pp;

bit Sound_flag=1; //����������־��1 = ������0 = ������

 

uchar code xiaxing1[] ="  ����԰��վ    ";
uchar code xiaxing2[] ="  ɳ�Ӷ�վ    ��";
uchar code xiaxing3[] ="  ɳ�Ӵ��վ  ��";
uchar code xiaxing4[] ="  ����Ժվ    ��";
uchar code xiaxing5[] ="  ʡ����վ    ��";


uchar code shangxing1[] ="  ʡ����վ    ��";
uchar code shangxing2[] ="  ����Ժվ    ��";
uchar code shangxing3[] ="  ɳ�Ӵ��վ  ��";
uchar code shangxing4[] ="  ɳ�Ӷ�վ    ��";
uchar code shangxing5[] ="  ����԰��վ    ";


				

uchar xdata A_dat[20];	 //�ݴ��GPS��ȡ�ľ�γ������
uchar xdata B_dat[20];	 //�ݴ��GPS��ȡ�ľ�γ������
uchar xdata GPS_dat[100]; // �����ݴ� GPS ģ�鷵�ص����� NMEA Э������
uchar subscript;		 //�������ݼ�������



uchar GPS_time=0;		    //���GPS�Ƿ���յ���Ч���ݱ���
float latitude,longitude; // �洢 **ת�����γ��/����** (��������ʽ)
unsigned long WD_A,JD_B; // ������ʾ�����ڱȽϵ�γ��/��������

bit GPS_Write=0;		    //�Ƿ���GPSУʱ����ʱ�����ݱ�־λ
uchar xdata NowTime[7]=0;	//��GPS���ص�����������ȡ��ʱ������
bit memory_GPS_flag=1;// ��¼�Ƿ���Ҫ�洢 GPS ��������

uchar Sec_set=0;// ��ʱ�������������� GPS ʱ��������

uchar Time_Calibration=0;   //У׼ʱ���־λ



// �����������ǣ�
// ����ǰϵͳ�������� EEPROM������ʧ�Դ洢��������֤���粻��ʧ
//��¼ϵͳ�Ĺؼ�״̬����������վ������ģʽ��Уʱ���ء������з���
void memory()
 {
   if(memory_flag) //�ж��Ƿ���Ҫ�洢����
    {
	  memory_flag=0;//��� memory_flag����ֹ�ظ��洢
	  IapEraseSector(0x08000);//����EEPROMָ��������0x08000��EEPROM�洢����ʼ��ַ��
	  IapProgramByte(0x08000,Sound);//�� Sound��������С�������洢�� EEPROM ��ַ 0x08000��

	  IapProgramByte(0x08001,Station);	       //��¼��վ����Station = 10����ʾ �� 10 ����վ��

	  if(GPS_Write)
			IapProgramByte(0x08002,1);   //��¼�Զ�Уʱ��־  GPS_Write=1,�ͼ�¼Ϊ1
      else IapProgramByte(0x08002,0); //��¼�Զ�Уʱ��־ ������ GPS Уʱ
 	
	  if(Mode)
			IapProgramByte(0x08003,1);//��¼�̻������Զ���Mode=1, �û��Զ���ģʽ������ 0x08003 = 1��
      else IapProgramByte(0x08003,0); //��¼�̻������Զ���  Mode=0,�ͼ�¼Ϊ0  

	  if(A_M)
			IapProgramByte(0x08004,1);         //��¼�ֶ��Զ�ģʽ  A_M=1,�Զ�ģʽ��ϵͳ�Զ�����վ����Ϣ��
      else IapProgramByte(0x08004,0);           //��¼�ֶ��Զ�ģʽ  A_M=0,�ֶ�ģʽ���˹���������վ����Ϣ��  

	  if(Upstream_Down) 
			IapProgramByte(0x08005,1);  //��¼��������  Station_Count=1,�ͼ�¼Ϊ1
      else IapProgramByte(0x08005,0);  //��¼��������  Station_Count=0,�ͼ�¼Ϊ0   

	}
 }





 
 

 
 
 
 

 
 
 
 //�� EEPROM ��ȡϵͳ��������������վ����GPS Уʱ��ģʽ�ȣ���
//��� EEPROM �����ݴ��󣬻ָ�Ĭ��ֵ����ֹ�����𻵣���
//��ȡ��ɺ����´洢��ȷ��������ȷ�ԡ�
 void read_memory() //�� EEPROM ��ȡ�洢��ϵͳ����
 {
   	Sound=IapReadByte(0x08000);	 //��ȡ�洢��������С����
//	Station=IapReadByte(0x08001); //��ȡ�ļ�¼��վ����

	if(Sound>30||Station>5) // �����ȡ��������վ��������������Χ
	 {
	   Sound=25;
	   Station=7;//�ָ�Ĭ��ֵ
	 }
    if(IapReadByte(0x08002)!=0&&IapReadByte(0x08002)!=1) //��ȡGPSУʱ��־λGPS_Writeֻ����0���û�1���á�
	 {
	   	GPS_Write=1;		//�����ȡ���ԣ�Ĭ�ϴ�
	 } 
	 else  
		  GPS_Write=IapReadByte(0x08002);	//����ԣ����и�ֵ
    if(IapReadByte(0x08003)!=0&&IapReadByte(0x08003)!=1) //��ȡ�洢��Modeֵ
	 {
	  	Mode=0;// Ĭ�� `Mode = 0`���̻�ģʽ��
	 }
	 else  
		  Mode=IapReadByte(0x08003);// ������ȷ����ֱ�Ӹ�ֵ

    if(IapReadByte(0x08004)!=0&&IapReadByte(0x08004)!=1) //��� A_M
	 {
	  	A_M=0;// Ĭ�� `A_M = 0`���ֶ�ģʽ��
	 }else  
	    A_M=IapReadByte(0x08004);// ��ȡ��ֵ�Ϸ�����ֱ�Ӹ�ֵ

    if(IapReadByte(0x08005)!=0&&IapReadByte(0x08005)!=1) //��� Upstream_Down
	 {
	  	Upstream_Down=0;// Ĭ�� `Upstream_Down = 0`������ģʽ��
	 }else  
	    Upstream_Down=IapReadByte(0x08005); // ��ȡ��ֵ�Ϸ�����ֱ�Ӹ�ֵ

     memory_flag=1;	// ��ȡ��ɺ󣬽������ٴδ洢�� EEPROM��ȷ��������ȷ

 }


 

 
 
 
 
 

 
 
 
 
 //�ú�������Ҫ������ �� GPS վ��ľ�γ�����ݴ��� EEPROM�����ڵ����ָ���
//GPS ���ݷ�Ϊ 4 �飬ÿ�� 5 ��ֵ���� 20 ��վ�㣺
//����վ�㾭�� (shangxing_JD_da[5])
//����վ��γ�� (shangxing_WD_da[5])
//����վ�㾭�� (xiaxing_JD_da[5])
//����վ��γ�� (xiaxing_WD_da[5])
//ÿ����γ������ 4 �ֽڴ洢���޷��ų����� ulong��32 λ����
//EEPROM ��ַ 0x08200 ��ʼ��ÿ��վ������ռ�� 4 �ֽڣ��� 20 �� 4 = 80 �ֽڡ�
void memory_GPS() // �洢 GPS ���� �� EEPROM������ʧ�Դ洢����
{
	// void memory_GPS()// �洢 GPS ���� �� EEPROM������ʧ�Դ洢����
// {
//   	  unsigned char i=0;//i ����ѭ��������0~19���� 20 ��վ�㣩
//	  if(memory_GPS_flag)//��� memory_GPS_flag���Ƿ���Ҫ�洢��
//	   {	
//	     memory_GPS_flag=0; //�����־�������ظ��洢
//			 IapEraseSector(0x08200);// ���� EEPROM ������0x08200 ~ 0x083FF��
//			 for(i=0;i<20;i++)		 // ���� 20 �� GPS վ��    	   
//			 {
////							i = 0 ~ 4 �� ���о��� (shangxing_JD_da[5])
////							i = 5 ~ 9 �� ����γ�� (shangxing_WD_da[5])
////							i = 10 ~ 14 �� ���о��� (xiaxing_JD_da[5])
////							i = 15 ~ 19 �� ����γ�� (xiaxing_WD_da[5])
//						if(i<5) 
//				 {
//					IapProgramByte(0x08200+i*4,gps_cun.Vehicle_Obj.shangxing_JD_da[i]/16777216%256);//�� 1 �ֽڣ��� 8 λ��
//					IapProgramByte(0x08200+i*4+1,gps_cun.Vehicle_Obj.shangxing_JD_da[i]/65536%256);//�� 2 �ֽڣ��θ� 8 λ��
//					IapProgramByte(0x08200+i*4+2,gps_cun.Vehicle_Obj.shangxing_JD_da[i]/256%256);//�� 3 �ֽڣ��ε� 8 λ��
//					IapProgramByte(0x08200+i*4+3,gps_cun.Vehicle_Obj.shangxing_JD_da[i]%256);//�� 4 �ֽڣ���� 8 λ��
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

    unsigned char i = 0; // i ����ѭ��������0~19���� 20 ��վ�㣩

    if (memory_GPS_flag) // ��� memory_GPS_flag���Ƿ���Ҫ�洢��
    {
        memory_GPS_flag = 0; // �����־�������ظ��洢
        IapEraseSector(0x08200); // ���� EEPROM ������0x08200 ~ 0x083FF��

        for (i = 0; i < 20; i++) // ���� 20 �� GPS վ��
        {
            unsigned char* data_ptr;
            if (i < 5)
            {
                // ���о��� (shangxing_JD_da[5])
                data_ptr = (unsigned char*)&gps_cun.Vehicle_Obj.shangxing_JD_da[i];
            }
            else if (i < 10)
            {
                // ����γ�� (shangxing_WD_da[5])
                data_ptr = (unsigned char*)&gps_cun.Vehicle_Obj.shangxing_WD_da[i - 5];
            }
            else if (i < 15)
            {
                // ���о��� (xiaxing_JD_da[5])
                data_ptr = (unsigned char*)&gps_cun.Vehicle_Obj.xiaxing_JD_da[i - 10];
            }
            else
            {
                // ����γ�� (xiaxing_WD_da[5])
                data_ptr = (unsigned char*)&gps_cun.Vehicle_Obj.xiaxing_WD_da[i - 15];
            }

            // �����ݰ��ֽ�д�� EEPROM
            IapProgramByte(0x08200 + i * 4, data_ptr[3]); // �� 1 �ֽڣ���� 8 λ��
            IapProgramByte(0x08200 + i * 4 + 1, data_ptr[2]); // �� 2 �ֽ�
            IapProgramByte(0x08200 + i * 4 + 2, data_ptr[1]); // �� 3 �ֽ�
            IapProgramByte(0x08200 + i * 4 + 3, data_ptr[0]); // �� 4 �ֽڣ���� 8 λ��
        }
    }
}
 


 
 
 
 



//�ú������� �� EEPROM ��ȡ GPS ���� 
//������ gps_cun �ṹ��������Ա��������
//EEPROM ��ַ 0x08200 ���洢�� ����/����վ��ľ�γ�����ݡ�
void read_GPS()
 {
	   
	   pp=gps_cun.gps_Bytes;
	 // �� gps_cun.gps_Bytes �ĵ�ַ����ָ�� pp�������������
	   IapReadSector(0x08200, sizeof(gps_data), pp);
	 // �� EEPROM ��ַ 0x08200 ��ȡһ��������
 }



 
 
 
//�ú�������ͨ�����ڷ���һ���ֽڵ����ݡ�
//��������ֽ����ݴ��봮�ڷ��ͻ���Ĵ�����
//�ȴ�������ɱ�־λ����λ����ʾ���ݷ�����ɡ�
//���������ɱ�־λ��Ϊ��һ�η�����׼����
 void Uart1Data(uchar dat) 	// ���ڷ���һ���ֽ�����
{
	SBUF=dat;// ��������ֽ����ݴ��봮�ڷ��ͻ���Ĵ���SBUF
	while(!TI);// �ȴ�������ɱ�־λTI��λ����ʾ���ݷ������
	TI=0;	// ���������ɱ�־λTI��Ϊ��һ�η�����׼��
}





//�ú�������ͨ�����ڷ���һ�����ݡ�
//'while��*byte ��=��ѭ�������ַ�����ֱ�������ַ��������� ��
//'Uart1���ݣ����ú������͵�ǰ�ֽ����ݣ�����ָ���ƶ�����һ���ֽڡ�
void UartData_Byte(uchar *byte)	//���ڷ���һ������
{
	while(*byte != '\0')
		// ѭ�������ַ�����ֱ�������ַ��������� '\0'
	{
	  Uart1Data(*byte++);
		// ���͵�ǰ�ֽ����ݣ�����ָ���ƶ�����һ���ֽ�
	}
}





//�ú������ڲ���һ���ӳ٣��ӳٵ�ʱ���ɴ��������
// ѭ���ݼ������datֵ��ֱ�����Ϊ0
void delay(uint dat) 
 {
   while(dat--);
 }



 
 
 
 //�ú�������У��ӷ��������ص������Ƿ��뱾�ش洢��GPS_dat����һ�¡�
 uchar verify_GPSdat(uchar *dat)//��ȡ���������ص�����
{
  uchar i=0;
  while(*dat != 0)// ������������ݣ�ֱ�������ַ��������� '\0'
		{
    if(*dat != GPS_dat[i])
			// �Ƚϴ�������ݺ� GPS_dat �еĶ�Ӧλ�õ�����
		{
      return 0; // ����в�ƥ������������ 0 ��ʾУ��ʧ��
    }
    i++; // ���ƥ�䣬�����Ƚ���һ���ֽ�
    dat++;
  }
  GPS_dat[0]=0;
	// ����������ݶ�ƥ�䣬�� GPS_dat �ĵ�һ���ֽ��� 0
  return 1;// ���� 1 ��ʾУ��ɹ�
}






//�ú�������ͨ�����ڷ���һ��ʮ���������ݡ�
//���� ��һ��ָ���޷����ַ�p
//����num���޷����ַ�����ʾҪ���͵��ַ�������
void Send_Hex(unsigned char *p,unsigned char num)
{
   	while(num--)   //ʣ�෢�͵��ַ���
	{
        SBUF = *p; //��Ҫ���͵����ݸ������ڻ���Ĵ���
		while(!TI);//�ȴ����ͽ���
		TI = 0;    //�������
		p++;       // ָ��ָ����һ���ַ�
	}	
}





//����ʵ����һ��У��ͼ��㺯����
//���ڼ����ַ�����У��ͣ�
//��������洢���ַ�����ĩβ�����ֽ��С�
void DoSum(unsigned char *Str,unsigned char len)//У��λ����
{
	unsigned int xorsum = 0; 
// ����һ���޷������ͱ��� xorsum ����ʼ��Ϊ 0��
//�����ۼӼ���У���
	unsigned char i;

	for(i=1;i<len;i++)
	// ѭ�������ַ����������� 1 ��ʼ�� len-1���ۼ�ÿ���ַ���ֵ
	{
		xorsum = xorsum + Str[i];
	}
	xorsum = 0 - xorsum;// ���ۼӺ�ȡ��
	*(Str+i)     = (unsigned char)(xorsum >> 8);
	// ��У��͵ĸ� 8 λ�����ַ����ĵ� len λ
	*(Str+i+1)   = (unsigned char)(xorsum & 0x00ff);
	 // ��У��͵ĵ� 8 λ�����ַ����ĵ� len + 1 λ
}






//����ʵ����һ����������ָ��ĺ�����
//��ָ�����ݰ����ض���ʽ��䵽�����У�
//������У���룬���ͨ�����ڷ���������ָ�����ݡ�
void Send_Appoint_Music(unsigned char dat )
 {	 //7E FF 06 03 00 00 01 FE F7 EF
		unsigned char Table[10];
		// ����ָ��ͷ
	  Table[0] = 0x7E;
		Table[1] = 0xFF;
		Table[2] = 0x06;
	
		Table[3] = 0x03; 
	  // ָ���룬��ʾ����ָ������
		Table[4] = 0x00; 
		Table[5] = 0x00;
		// ��䱣���ֶ�
	  Table[6] = dat; 
	  // ����Ҫ���ŵ����ֱ��
		DoSum(Table,7);// ����У���벢���
		Table[9] = 0xEF;//������
		
		Send_Hex(Table,10);//����ָ������
 }


 
 
 
// ����ʵ����һ��������������ָ��ĺ�����
// ��ָ�����ݰ����ض���ʽ��䵽�����У�
// ������У���룬���ͨ�����ڷ���������ָ�����ݡ�
  void Send_Appoint_Sound(unsigned char dat)
 {
   if(Music_Busy==0&&s0)  
		 // ��� Music_Busy �� s0 ״̬�����������������ָ��
    {
		unsigned char Table[10];
		// ����ָ��ͷ
		Table[0] = 0x7E;
		Table[1] = 0xFF;
		Table[2] = 0x06;
	
		Table[3] = 0x06; 
		// ָ���룬��ʾ��������
		Table[4] = 0x00;
		Table[5] = 0x00;
		// ��䱣���ֶ�
		Table[6] = dat;//����
		DoSum(Table,7);//����У����	
		Table[9] = 0xEF;//������
		
		Send_Hex(Table,10);//����ָ������
	}
 }


 
 
 
 
// ����ʵ����һ����ȡ������DS1302ʱ�����ݵĺ�����
// ͨ���жϵ�״̬�������Ƕ�ȡ��������ʱ�����ݣ�
// ��������Ӧ�Ĵ���
 void read_time1() //ʵʱ��ȡDS1302�е�ʱ������
{
 uchar i;
	
		 if(state==0)
		  {
				// ��ȡDS1302�е�ʱ�����ݣ����洢��time_data����
			  time_data[0]=ds1302read(0x81);
				time_data[1]=ds1302read(0x83);
				time_data[2]=ds1302read(0x85);
				time_data[3]=ds1302read(0x87);
				time_data[4]=ds1302read(0x89);
				time_data[5]=ds1302read(0x8D);  	
				// ����ȡ����ʱ������ת��Ϊʮ���Ƹ�ʽ��
				//�洢��time_data_1������
			 	time_data_1[0]=time_data[0]/16*10+time_data[0]%16;
				time_data_1[1]=time_data[1]/16*10+time_data[1]%16;
				time_data_1[2]=time_data[2]/16*10+time_data[2]%16;
				time_data_1[3]=time_data[3]/16*10+time_data[3]%16;
				time_data_1[4]=time_data[4]/16*10+time_data[4]%16;
				time_data_1[5]=time_data[5]/16*10+time_data[5]%16;
				 if(time_data_1[0]>59)
				// ����������ݴ���59��˵����ȡ���������󣬽��г�ʼ��
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
				// ��time_data_1�е�ʱ������ת��ΪBCD�룬
				 //���洢��time_data_4������
				for(i=0;i<7;i++)
				{
					time_data_2[i]=time_data_1[i]/10;
					time_data_3[i]=time_data_1[i]%10;	  
				}
				for(i=0;i<7;i++)
				{
					time_data_4[i]=time_data_2[i]*16+time_data_3[i];
				  
				}
				// ��ת�����ʱ������д��DS1302��
				ds1302write(0x80,time_data_4[0]);
				ds1302write(0x82,time_data_4[1]);
				ds1302write(0x84,time_data_4[2]);
				ds1302write(0x86,time_data_4[3]);
				ds1302write(0x88,time_data_4[4]);
				ds1302write(0x8C,time_data_4[5]);
		   }     

}





//��ʾ����
void Display() {
    uchar dat = 0; // ����һ���޷����ַ����� dat ����ʼ��Ϊ 0

    // ���� state ��ֵ������ʾ����
    if (state <= 6) {
        LCD12864_pos(0, 0); // ���� LCD λ��Ϊ�� 0 �е� 0 ��

        // ��ʾ���
        if (state == 1 && s0) {
            LCD12864_writebyte("    "); // ��� state Ϊ 1 ���� s0 Ϊ�棬��ʾ�ո�
        } else {
            LCD12864_writebyte("20"); // ��ʾ�̶����ǰ��λ "20"
            LCD12864_write(1, 0x30 + time_data_1[5] / 10); // ��ʾ��ݵ�ʮλ
            LCD12864_write(1, 0x30 + time_data_1[5] % 10); // ��ʾ��ݵĸ�λ
        }

        LCD12864_writebyte("-"); // ��ʾ�ָ��� "-"
        
        // ��ʾ�·�
        if (state == 2 && s0) {
            LCD12864_writebyte("  "); // ��� state Ϊ 2 ���� s0 Ϊ�棬��ʾ�ո�
        } else {
            LCD12864_write(1, 0x30 + time_data_1[4] / 10); // ��ʾ�·ݵ�ʮλ
            LCD12864_write(1, 0x30 + time_data_1[4] % 10); // ��ʾ�·ݵĸ�λ
        }

        LCD12864_writebyte("-"); // ��ʾ�ָ��� "-"
        
        // ��ʾ����
        if (state == 3 && s0) {
            LCD12864_writebyte("  "); // ��� state Ϊ 3 ���� s0 Ϊ�棬��ʾ�ո�
        } else {
            LCD12864_write(1, 0x30 + time_data_1[3] / 10); // ��ʾ���ڵ�ʮλ
            LCD12864_write(1, 0x30 + time_data_1[3] % 10); // ��ʾ���ڵĸ�λ
        }

        LCD12864_writebyte("  "); // ��ʾ�ո�
        
        // ��ʾ���ڼ�
        switch (Conver_week(time_data_1[5], time_data_1[4], time_data_1[3])) {
            case 0: LCD12864_writebyte("Sun "); break; // ������
            case 1: LCD12864_writebyte("Mon "); break; // ����һ
            case 2: LCD12864_writebyte("Tue "); break; // ���ڶ�
            case 3: LCD12864_writebyte("Wed "); break; // ������
            case 4: LCD12864_writebyte("Thu "); break; // ������
            case 5: LCD12864_writebyte("Fri "); break; // ������
            case 6: LCD12864_writebyte("Sat "); break; // ������
        } 

        LCD12864_pos(1, 0); // ���� LCD λ��Ϊ�� 1 �е� 0 ��
        
        // ��ʾСʱ
        if (state == 4 && s0) {
            LCD12864_writebyte("  "); // ��� state Ϊ 4 ���� s0 Ϊ�棬��ʾ�ո�
        } else {
            LCD12864_write(1, 0x30 + time_data_1[2] / 10); // ��ʾСʱ��ʮλ
            LCD12864_write(1, 0x30 + time_data_1[2] % 10); // ��ʾСʱ�ĸ�λ
        }

        LCD12864_writebyte(":"); // ��ʾ�ָ��� ":"
        
        // ��ʾ����
        if (state == 5 && s0) {
            LCD12864_writebyte("  "); // ��� state Ϊ 5 ���� s0 Ϊ�棬��ʾ�ո�
        } else {
            LCD12864_write(1, 0x30 + time_data_1[1] / 10); // ��ʾ���ӵ�ʮλ
            LCD12864_write(1, 0x30 + time_data_1[1] % 10); // ��ʾ���ӵĸ�λ
        }

        LCD12864_writebyte(":"); // ��ʾ�ָ��� ":"
        
        // ��ʾ����
        if (state == 6 && s0) {
            LCD12864_writebyte("  "); // ��� state Ϊ 6 ���� s0 Ϊ�棬��ʾ�ո�
        } else {
            LCD12864_write(1, 0x30 + time_data_1[0] / 10); // ��ʾ���ӵ�ʮλ
            LCD12864_write(1, 0x30 + time_data_1[0] % 10); // ��ʾ���ӵĸ�λ
        }
    }
}			 
			// GPS����״̬
if (GPS_time == 0) { // ���GPSʱ��Ϊ0����ʾδ����
    if (s0) {
        LCD12864_writebyte("        "); 
        // ���s0Ϊ�棬��ʾ�ո�
    } else {
        LCD12864_writebyte("  Search"); 
        // ���s0Ϊ�٣���ʾ��  Search��
    }
} else {
    LCD12864_writebyte(" Connect"); 
    // ���GPSʱ�䲻Ϊ0����ʾ�����ӣ���ʾ�� Connect��
}

// ��ʾģʽ�ͷ���
LCD12864_pos(2, 0); // ����LCDλ��Ϊ��2�е�0��
if (A_M == 0) {
    LCD12864_writebyte("�Զ�ģʽ"); 
    // ���A_MΪ0����ʾ���Զ�ģʽ��
} else {
    LCD12864_writebyte("�ֶ�ģʽ"); 
    // ���A_M��Ϊ0����ʾ���ֶ�ģʽ��
}

if (Upstream_Down) {
    LCD12864_writebyte("    ����"); 
    // ���Upstream_DownΪ�棬��ʾ��    ���С�
} else {
    LCD12864_writebyte("    ����"); 
    // ���Upstream_DownΪ�٣���ʾ��    ���С�
}

// ��ʾվ����Ϣ
LCD12864_pos(3, 0); // ����LCDλ��Ϊ��3�е�0��


if (Station_Count == 0 && position == 0) { 
	// ���վ�����Ϊ0��λ��Ϊ0
    if (A_M == 0) { // ���A_MΪ0
        if (Display_Reversal)
            LCD12864_writebyte("    λ�ô���    "); 
				// ���Display_ReversalΪ�棬��ʾ��    λ�ô���    ��
        else
            LCD12864_writebyte("�뵽����վ�㶨λ"); 
				// ���Display_ReversalΪ�٣���ʾ���뵽����վ�㶨λ��
    } else {
        LCD12864_writebyte("��ӭ�������ι���"); 
			// ���A_M��Ϊ0����ʾ����ӭ�������ι�����
    }
} else if (Station_Count != 0 && position == 0) { 
	// ���վ�������Ϊ0��λ��Ϊ0
    if (A_M == 0) { // ���A_MΪ0����ʾ�Զ�ģʽ
        if (Upstream_Down) { 
					// ���Upstream_DownΪ�棬��ʾ����
            switch (Station_Count) {
                case 1:
                    if (Display_Reversal)
                        LCD12864_writebyte("    ��һվ      "); 
										// ���Display_ReversalΪ�棬��ʾ��    ��һվ      ��
                    else
                        LCD12864_writebyte(xiaxing2); 
										// ���Display_ReversalΪ�٣���ʾxiaxing2
                    break;

                case 2:
                    if (Station == 2) {
                        if (Display_Reversal)
                            LCD12864_writebyte("    �յ�վ      "); 
												// ���Display_ReversalΪ�棬��ʾ��    �յ�վ      ��
                        else
                            LCD12864_writebyte(xiaxing2); 
												// ���Display_ReversalΪ�٣���ʾxiaxing2
                    } else {
                        if (Display_Reversal)
                            LCD12864_writebyte("    ��һվ      "); 
												// ���Display_ReversalΪ�棬��ʾ��    ��һվ      ��
                        else
                            LCD12864_writebyte(xiaxing3); 
												// ���Display_ReversalΪ�٣���ʾxiaxing3
                    }
                    break;

                case 3:
                    if (Station == 3) {
                        if (Display_Reversal)
                            LCD12864_writebyte("    �յ�վ      "); 
												// ���Display_ReversalΪ�棬��ʾ��    �յ�վ      ��
                        else
                            LCD12864_writebyte(xiaxing3); 
												// ���Display_ReversalΪ�٣���ʾxiaxing3
                    } else {
                        if (Display_Reversal)
                            LCD12864_writebyte("    ��һվ      "); 
												// ���Display_ReversalΪ�棬��ʾ��    ��һվ      ��
                        else
                            LCD12864_writebyte(xiaxing4); 
												// ���Display_ReversalΪ�٣���ʾxiaxing4
                    }
                    break;

                case 4:
                    if (Station == 4) {
                        if (Display_Reversal)
                            LCD12864_writebyte("    �յ�վ      "); 
												// ���Display_ReversalΪ�棬��ʾ��    �յ�վ      ��
                        else
                            LCD12864_writebyte(xiaxing4); 
												// ���Display_ReversalΪ�٣���ʾxiaxing4
                    } else {
                        if (Display_Reversal)
                            LCD12864_writebyte("    ��һվ      "); 
												// ���Display_ReversalΪ�棬��ʾ��    ��һվ      ��
                        else
                            LCD12864_writebyte(xiaxing5); 
												// ���Display_ReversalΪ�٣���ʾxiaxing5
                    }
                    break;
										
								case 5 :	
												            if(Display_Reversal)  LCD12864_writebyte("    �յ�վ      ");
								                    // ���Display_ReversalΪ�棬��ʾ��    �յ�վ      ��
															 else                 LCD12864_writebyte(xiaxing15);	
								                     // ���Display_ReversalΪ�٣���ʾxiaxing6
												  break;

            }
        }											 
	 else 	// ���Upstream_DownΪ�٣���ʾ����
				{
			switch (Station_Count) {
        case 1:
            // ��վ������Ϊ1ʱ
            if (Display_Reversal)
                // �����Ҫ������ʾ��Ϣ
                LCD12864_writebyte("    ��һվ      "); // ��ʾ����һվ��
            else
                // ������ʾ����վ��2������
                LCD12864_writebyte(shangxing2);
            break;

        case 2:
            // ��վ������Ϊ2ʱ
            if (Station == 2) {
                // �����ǰվ����2
                if (Display_Reversal)
                    // �����Ҫ������ʾ��Ϣ
                    LCD12864_writebyte("    �յ�վ      "); // ��ʾ���յ�վ��
                else
                    // ������ʾ����վ��2������
                    LCD12864_writebyte(shangxing2);
            } else {
                // ��ǰվ�㲻��2
                if (Display_Reversal)
                    // �����Ҫ������ʾ��Ϣ
                    LCD12864_writebyte("    ��һվ      "); // ��ʾ����һվ��
                else
                    // ������ʾ����վ��3������
                    LCD12864_writebyte(shangxing3);
            }
            break;

        case 3:
            // ��վ������Ϊ3ʱ
            if (Station == 3) {
                // �����ǰվ����3
                if (Display_Reversal)
                    // �����Ҫ������ʾ��Ϣ
                    LCD12864_writebyte("    �յ�վ      "); // ��ʾ���յ�վ��
                else
                    // ������ʾ����վ��3������
                    LCD12864_writebyte(shangxing3);
            } else {
                // ��ǰվ�㲻��3
                if (Display_Reversal)
                    // �����Ҫ������ʾ��Ϣ
                    LCD12864_writebyte("    ��һվ      "); // ��ʾ����һվ��
                else
                    // ������ʾ����վ��4������
                    LCD12864_writebyte(shangxing4);
            }
            break;

        case 4:
            // ��վ������Ϊ4ʱ
            if (Station == 4) {
                // �����ǰվ����4
                if (Display_Reversal)
                    // �����Ҫ������ʾ��Ϣ
                    LCD12864_writebyte("    �յ�վ      "); // ��ʾ���յ�վ��
                else
                    // ������ʾ����վ��4������
                    LCD12864_writebyte(shangxing4);
            } else {
                // ��ǰվ�㲻��4
                if (Display_Reversal)
                    // �����Ҫ������ʾ��Ϣ
                    LCD12864_writebyte("    ��һվ      "); // ��ʾ����һվ��
                else
                    // ������ʾ����վ��5������
                    LCD12864_writebyte(shangxing5);
            }
            break;

						case 5 :	
						// ��վ������Ϊ5ʱ						
						if(Display_Reversal)  
							// �����Ҫ������ʾ��Ϣ
						LCD12864_writebyte("    �յ�վ      ");// ��ʾ���յ�վ��
						else
							// ������ʾ����վ��5������                 
							LCD12864_writebyte(shangxing5);									 		 
								break;
				
    }
}
				   	
	 else 		//�ֶ�ģʽ
		{
     if (Busy == 0) { // ���ϵͳ��æ
        if (Upstream_Down) { 
					// ���Upstream_DownΪ�棬��ʾ����
            switch (Station_Count - 1) {
                case 1:
                    if (Station_Count <= Station) {
                        // ��ǰվ����С�ڵ���Ŀ��վ����
                        if (Display_Reversal)
                            LCD12864_writebyte("      ��վ      "); 
												// ��ʾ����վ��
                        else
                            LCD12864_writebyte(xiaxing1); 
												// ��ʾ����վ��1������
                    } else {
                        // ��ǰվ��������Ŀ��վ����
                        if (Display_Reversal)
                            LCD12864_writebyte("    �յ�վ      "); 
												// ��ʾ���յ�վ��
                        else
                            LCD12864_writebyte(xiaxing1); 
												// ��ʾ����վ��1������
                    }
                    break;

                case 2:
                    if (Station_Count <= Station) {
                        // ��ǰվ����С�ڵ���Ŀ��վ����
                        if (Display_Reversal)
                            LCD12864_writebyte("      ��վ      "); 
												// ��ʾ����վ��
                        else
                            LCD12864_writebyte(xiaxing2); 
												// ��ʾ����վ��2������
                    } else {
                        // ��ǰվ��������Ŀ��վ����
                        if (Display_Reversal)
                            LCD12864_writebyte("    �յ�վ      "); 
												// ��ʾ���յ�վ��
                        else
                            LCD12864_writebyte(xiaxing2); 
												// ��ʾ����վ��2������
                    }
                    break;

                case 3:
                    if (Station_Count <= Station) {
                        // ��ǰվ����С�ڵ���Ŀ��վ����
                        if (Display_Reversal)
                            LCD12864_writebyte("      ��վ      "); 
												// ��ʾ����վ��
                        else
                            LCD12864_writebyte(xiaxing3); 
												// ��ʾ����վ��3������
                    } else {
                        // ��ǰվ��������Ŀ��վ����
                        if (Display_Reversal)
                            LCD12864_writebyte("    �յ�վ      "); 
												// ��ʾ���յ�վ��
                        else
                            LCD12864_writebyte(xiaxing3); 
												// ��ʾ����վ��3������
                    }
                    break;

                case 4:
                    if (Station_Count <= Station) {
                        // ��ǰվ����С�ڵ���Ŀ��վ����
                        if (Display_Reversal)
                            LCD12864_writebyte("      ��վ      "); 
												// ��ʾ����վ��
                        else
                            LCD12864_writebyte(xiaxing4); 
												// ��ʾ����վ��4������
                    } else {
                        // ��ǰվ��������Ŀ��վ����
                        if (Display_Reversal)
                            LCD12864_writebyte("    �յ�վ      "); 
												// ��ʾ���յ�վ��
                        else
                            LCD12864_writebyte(xiaxing4); 
												// ��ʾ����վ��4������
                    }
                    break;

                case 5 :	 
									if(Station_Count<=Station){ 	
										// ��ǰվ����С�ڵ���Ŀ��վ����
										if(Display_Reversal)  
											LCD12864_writebyte("      ��վ      "); // ��ʾ����վ��
										else                 
											LCD12864_writebyte(xiaxing5);	// ��ʾ����վ��5������
								}else {
											// ��ǰվ��������Ŀ��վ����
									if(Display_Reversal)  
												LCD12864_writebyte("    �յ�վ      "); // ��ʾ���յ�վ��
											else                  
												LCD12864_writebyte(xiaxing15);// ��ʾ����վ��5������
										}	           								  									 		 
break;	

								
            }
        }
    }
	}						else {
    // �ֶ�ģʽ
    switch (Station + 1 - Station_Count) {
        // ���� Station+1-Station_Count ��ֵѡ��ͬ�� case ��֧
        case 1:
            if (Station_Count != 1 && Station_Count <= Station) {
                // ��� Station_Count ���� 1 �� Station_Count С�ڵ��� Station
                if (Display_Reversal)
                    LCD12864_writebyte("      ��վ      "); // ��ʾ����վ��
                else
                    LCD12864_writebyte(shangxing1); // ��ʾ����վ��1������
            } else {
                // ����
                if (Display_Reversal)
                    LCD12864_writebyte("    �յ�վ      "); // ��ʾ���յ�վ��
                else
                    LCD12864_writebyte(shangxing1); // ��ʾ����վ��1������
            }
            break;

        case 2:
            if (Station_Count != 1 && Station_Count <= Station) {
                // ��� Station_Count ���� 1 �� Station_Count С�ڵ��� Station
                if (Display_Reversal)
                    LCD12864_writebyte("      ��վ      "); // ��ʾ����վ��
                else
                    LCD12864_writebyte(shangxing2); // ��ʾ����վ��2������
            } else {
                // ����
                if (Display_Reversal)
                    LCD12864_writebyte("    �յ�վ      "); // ��ʾ���յ�վ��
                else
                    LCD12864_writebyte(shangxing2); // ��ʾ����վ��2������
            }
            break;

        case 3:
            if (Station_Count != 1 && Station_Count <= Station) {
                // ��� Station_Count ���� 1 �� Station_Count С�ڵ��� Station
                if (Display_Reversal)
                    LCD12864_writebyte("      ��վ      "); // ��ʾ����վ��
                else
                    LCD12864_writebyte(shangxing3); // ��ʾ����վ��3������
            } else {
                // ����
                if (Display_Reversal)
                    LCD12864_writebyte("    �յ�վ      "); // ��ʾ���յ�վ��
                else
                    LCD12864_writebyte(shangxing3); // ��ʾ����վ��3������
            }
            break;

        case 4:
            if (Station_Count != 1 && Station_Count <= Station) {
                // ��� Station_Count ���� 1 �� Station_Count С�ڵ��� Station
                if (Display_Reversal)
                    LCD12864_writebyte("      ��վ      "); // ��ʾ����վ��
                else
                    LCD12864_writebyte(shangxing4); // ��ʾ����վ��4������
            } else {
                // ����
                if (Display_Reversal)
                    LCD12864_writebyte("    �յ�վ      "); // ��ʾ���յ�վ��
                else
                    LCD12864_writebyte(shangxing4); // ��ʾ����վ��4������
            }
            break;

				case 5:
            if (Station_Count != 1 && Station_Count <= Station) {
                // ��� Station_Count ���� 1 �� Station_Count С�ڵ��� Station
                if (Display_Reversal)
                    LCD12864_writebyte("      ��վ      "); // ��ʾ����վ��
                else
                    LCD12864_writebyte(shangxing5); // ��ʾ����վ��5������
            } else {
                // ����
                if (Display_Reversal)
                    LCD12864_writebyte("    �յ�վ      "); // ��ʾ���յ�վ��
                else
                    LCD12864_writebyte(shangxing5); // ��ʾ����վ��5������
            }
            break;
    }
}
											}
									}


							}else LCD12864_writebyte("��ӭ�������ι���");
	
					}
	else if(Station_Count!=0&&position!=0)    {
							      if(Upstream_Down) 
								   {
										switch(Station_Count)
									    {
										  case 1 :	 
										             if(Display_Reversal)  LCD12864_writebyte("      ��վ      ");	
													 else                 LCD12864_writebyte(xiaxing1);										 									 		 
										  break;			
										  case 2 :	if(Display_Reversal)   LCD12864_writebyte("      ��վ      ");
													 else                  LCD12864_writebyte(xiaxing2);										  									 		 
										  break;		
										  case 3 :	if(Display_Reversal)   LCD12864_writebyte("      ��վ      "); 
													 else                  LCD12864_writebyte(xiaxing3);									  									 		 
										  break;		
										  case 4 :	if(Display_Reversal)   LCD12864_writebyte("      ��վ      ");
													 else                  LCD12864_writebyte(xiaxing4);										  								 		 
										  break;
						
										  case 5 :	if(Display_Reversal)  LCD12864_writebyte("      ��վ      ");
													 else                 LCD12864_writebyte(xiaxing5);								  									 		 
										  break;
										}
								   }
								   else 
								    {
											switch(Station_Count)
										    {
											  case 1 :	if(Display_Reversal)  LCD12864_writebyte("      ��վ      ");	
														 else                 LCD12864_writebyte(shangxing1);										 									 		 
											  break;			
											  case 2 :	if(Display_Reversal)   LCD12864_writebyte("      ��վ      ");
														 else                  LCD12864_writebyte(shangxing2);										  									 		 
											  break;		
											  case 3 :	if(Display_Reversal)   LCD12864_writebyte("      ��վ      "); 
														 else                  LCD12864_writebyte(shangxing3);									  									 		 
											  break;		
											  case 4 :	if(Display_Reversal)   LCD12864_writebyte("      ��վ      ");
														 else                  LCD12864_writebyte(shangxing4);										  								 		 
											  break;
							
											  case 5 :	if(Display_Reversal)  LCD12864_writebyte("      ��վ      ");
														 else                 LCD12864_writebyte(shangxing5);								  									 		 
											  break;
											}
									}
			 }
	 }
 	else if(state<10)
	 {
			LCD12864_pos(0,0);
		    LCD12864_writebyte("    ϵͳ����    ");
		   	LCD12864_pos(1,0);
		    LCD12864_writebyte("��վ    : ");
			if(state==7&&s0) LCD12864_writebyte("  ");
			else            
			 {
			   LCD12864_write(1,0x30+Station/10%10);
			   LCD12864_write(1,0x30+Station%10);
			 } 
		    LCD12864_writebyte("    ");

		   	LCD12864_pos(2,0);
		    LCD12864_writebyte("������С: ");
			if(state==8&&s0) LCD12864_writebyte("  ");
			else 
			 {
			    LCD12864_write(1,0x30+Sound/10%10);
			    LCD12864_write(1,0x30+Sound%10);
			 }         
		    LCD12864_writebyte("    ");

			LCD12864_pos(3,0);
		    LCD12864_writebyte("�Զ�Уʱ: ");
			if(state==9&&s0) LCD12864_writebyte("       ");
			else 
			 {
				if(GPS_Write) LCD12864_writebyte("��    ");
				else 		  LCD12864_writebyte("��    ");
			 }   

	 }
	 else if(state<11)
	  {
	    
			LCD12864_pos(0,0);
			LCD12864_writebyte("    ϵͳ����    ");
			LCD12864_pos(1,0);
		    LCD12864_writebyte("��γ����ʾ:     ");

   	        LCD12864_pos(2,0);
		    LCD12864_writebyte("γ��:");
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
		    LCD12864_writebyte("����:");
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
		    LCD12864_writebyte("  ��վ����");
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
		    LCD12864_writebyte("γ��:");
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
		    LCD12864_writebyte("����:");
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
		    LCD12864_writebyte("  ��վ����");
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
		    LCD12864_writebyte("γ��:");
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
		    LCD12864_writebyte("����:");
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

//  ��������̣�GPS����������ָ���㣬��ʾ  ����ǰվ�㡱	��XXXX��	    ������ָ���㣬����ʾ   ����һվ��  ��XXXXX��
//  ���������������ж���һվ�ص�		
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
				   if(Upstream_Down) 		  //˵��������
				    {
						Station_Count=0;
					}
					else 					 //˵��������
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
					   if(Upstream_Down) 	  //���������
					    {
						   if(Station_Count<Station+1) 
						    {	  
							  Send_Appoint_Music(Station_Count);   //������������
							  Station_Count++;
							}
							else 
							 {
							   Upstream_Down=0;	
							   Send_Appoint_Music(Station+1); //�յ�վ	
							   Station_Count--;
							 }
						}
						else				  //���������
						 {
						   if(Station_Count>1) 
						    {
							  Station_Count--;
							  Send_Appoint_Music(((Station+1)-Station_Count)+Station); //�յ�վ	
							}
							else 
							 {
							   Upstream_Down=1;
							   Send_Appoint_Music(Station_Count);   //������������
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
  if((!key7)&&(flag_cun!=0))//ȷ���洢��γ��
	{ 
//		 LCD12864_pos(2,0);
//		    LCD12864_writebyte("γ��:");
//			LCD12864_write(1,0x30+WD_A/100000000%10);

//            LCD12864_pos(3,0);
//		    LCD12864_writebyte("����:");
//			LCD12864_write(1,0x30+JD_B/100000000%10);
			
		if(state>=11&&(state-11<Station))
	 {  gps_cun.Vehicle_Obj.xiaxing_WD_da[state-11]=WD_A;
		  gps_cun.Vehicle_Obj.xiaxing_JD_da[state-11]=JD_B;
		  LCD12864_pos(2,0);
		  LCD12864_writebyte("γ��:");
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
		  LCD12864_writebyte("����:");
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
		  LCD12864_writebyte("γ��:");
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
		    LCD12864_writebyte("����:");
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
//���ö�ʱ0�������涨ʱ��ʹ�ö�ʱ��1��������1�����ʷ�������9600
void TimeInt(void)		 //ϵͳ��ʱ����ʼ������
{
	TMOD=0x21;	   //����������ʱ��
	TH1=0xFD;
	TL1=0xFD;	   //��ʱ��1���ڲ���������  ����11.0592
	TL0 = 0x00;		//���ö�ʱ��ֵ
	TH0 = 0x4C;		//���ö�ʱ��ֵ	
	SCON=0x50;
	PCON=0;
	EA=1;          //�����ж�
	ES=1;          //ES-�����ж��������λ   ES = 1   �������жϡ�
	TR1=1;         //������ʱ����ʼ����
	ET0=1;
	TR0=1; 	
}

void led_dispose()  //LED��ʾ����
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
		 
		 
		 if(Upstream_Down==1)   //����1��˵���Զ�ģʽ�´�������״̬����ʱ��Լ�⵽��GPS��������Ӧ����
		   {
					  if((gps_cun.Vehicle_Obj.xiaxing_JD_da[0]+JD_Difference>=JD_B)&&(JD_B>=gps_cun.Vehicle_Obj.xiaxing_JD_da[0]-JD_Difference)&&(gps_cun.Vehicle_Obj.xiaxing_WD_da[0]+WD_Difference>=WD_A)&&(WD_A>=gps_cun.Vehicle_Obj.xiaxing_WD_da[0]-WD_Difference))
					   {
					     Station_Count=1;  position=1;
						 if(count!=Station_Count)
						  {
						    count=Station_Count;
							Send_Appoint_Music(Station_Count);   //������������

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
								Send_Appoint_Music(Station_Count);   //������������
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
								Send_Appoint_Music(Station_Count);   //������������	
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
								Send_Appoint_Music(Station_Count);   //������������
							
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
								Send_Appoint_Music(Station_Count);   //������������
							  }
						  }
					   }
					   else  position=0;
		   
		   }
		   else 		  //��������������
		    {			
						  if((gps_cun.Vehicle_Obj.shangxing_JD_da[0]+JD_Difference>=JD_B)&&(JD_B>=gps_cun.Vehicle_Obj.shangxing_JD_da[0]-JD_Difference)&&(gps_cun.Vehicle_Obj.shangxing_WD_da[0]+WD_Difference>=WD_A)&&(WD_A>=gps_cun.Vehicle_Obj.shangxing_WD_da[0]-WD_Difference))
						   {
						     Station_Count=1;  position=1;
							 if(count!=Station_Count)
							  {
							    count=Station_Count;
								Send_Appoint_Music(Station+1);   //������������
	
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
									Send_Appoint_Music(Station+2);   //������������
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
									Send_Appoint_Music(Station+3);   //������������	
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
									Send_Appoint_Music(Station+4);   //������������
								
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
									Send_Appoint_Music(Station+5);   //������������
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
   LCD12864_init();	     //������ʾ����
// Send_Appoint_Music(1);

   read_memory();
	Mode=0;
   if(Mode) 
    {
	  
	}
    read_GPS();

   while(1) 
    {  
	  read_time1();  //��ȡʱ�亯����GPS_Write����1��˵������ȡ��GPS�ǽ���ֵ��ʱ��
	  GPS_Route_Dispose();
	  Display();		 //��ʾ����
	  key_dispose();	 //����������
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


//��ʱ��0�жϷ�����
void time0() interrupt 1
 {
   TH0=0x3C;
   TL0=0xb0; 	    //��ʱ�ж�50ms
   ms++;			//ms�����൱��50ms��һ
   key_dispose();
	if(ms%10==0) 	//500����ִ��һ��
	 { 
	   Sound_flag=1;
	   s0=~s0;
	   if(GPS_time!=0) GPS_time--; //�ж�GPS�ź��Ƿ�ʧ����ÿ��500ms��һ
	 } 
	if(ms%20==0)	 
	 {
       if(sec!=0) sec--;
	   if(sec1!=0) sec1--; 	 Display_Reversal=~Display_Reversal;
	 }  

   if(ms>=40)
    {
	   ms=0;	 //һ�붨ʱ������ 
	}
 }


//  ͨѶ�жϽ��ճ���   �жϺ����޷���ֵ
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
				}	  // 0 ��  1��  2ʱ ������  ����   6 ��
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