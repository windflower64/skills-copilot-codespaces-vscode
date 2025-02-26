#ifndef _EEPROM52_H_
#define _EEPROM52_H_
#include <intrins.h>
/********STC89C51扇区分布*******
第一扇区：1000H--11FF
第二扇区：1200H--13FF
第三扇区：1400H--15FF
第四扇区：1600H--17FF
第五扇区：1800H--19FF
第六扇区：1A00H--1BFF
第七扇区：1C00H--1DFF
第八扇区：1E00H--1FFF
*****************/

/********STC89C52扇区分布*******
第一扇区：2000H--21FF
第二扇区：2200H--23FF
第三扇区：2400H--25FF
第四扇区：2600H--27FF
第五扇区：2800H--29FF
第六扇区：2A00H--2BFF
第七扇区：2C00H--2DFF
第八扇区：2E00H--2FFF
*****************/



#include "intrins.h"
#include	"config.h"
typedef unsigned char BYTE;
typedef unsigned int WORD;

/*Declare SFR associated with the IAP */
sfr IAP_DATA    =   0xE2;           //Flash data register
sfr IAP_ADDRH   =   0xE3;           //Flash address HIGH
sfr IAP_ADDRL   =   0xE4;           //Flash address LOW
sfr IAP_CMD     =   0xE5;           //Flash command register
sfr IAP_TRIG    =   0xE6;           //Flash command trigger
sfr IAP_CONTR   =   0xE7;           //Flash control register

/*Define ISP/IAP/EEPROM command*/
#define CMD_IDLE    0               //Stand-By
#define CMD_READ    1               //Byte-Read
#define CMD_PROGRAM 2               //Byte-Program
#define CMD_ERASE   3               //Sector-Erase

/*Define ISP/IAP/EEPROM operation const for IAP_CONTR*/
//#define ENABLE_IAP 0x80           //if SYSCLK<40MHz
#define ENABLE_IAP   0x81           //if SYSCLK<20MHz
//#define ENABLE_IAP x82            //if SYSCLK<10MHz
//#define ENABLE_IAP 0x83           //if SYSCLK<5MHz

//Start address for STC89C58xx EEPROM
#define IAP_ADDRESS 0x08000
/*Define ISP/IAP/EEPROM operation const for IAP_CONTR*/
//#define ENABLE_IAP 0x80           //if SYSCLK<40MHz
#define ENABLE_IAP   0x81           //if SYSCLK<20MHz
//#define ENABLE_IAP x82            //if SYSCLK<10MHz
//#define ENABLE_IAP 0x83           //if SYSCLK<5MHz
#define ERROR   0
#define OK      1



void Delay_eeprom(BYTE n);
void IapIdle();
BYTE IapReadByte(WORD addr);
void IapProgramByte(WORD addr, BYTE dat);
void IapEraseSector(WORD addr);
u8 sequential_write_flash_in_one_sector(u16 begin_addr, u16 counter, u8 array[]);
void IapReadSector(u16 begin_addr, u16 counter, u8 array[]);
/*----------------------------
Software Delay_eeprom function
----------------------------*/
void Delay_eeprom(BYTE n)
{
    WORD x;

    while (n--)
    {
        x = 0;
        while (++x);
    }
}

/*----------------------------
Disable ISP/IAP/EEPROM function
Make MCU in a safe state
----------------------------*/
void IapIdle()
{
    IAP_CONTR = 0;                  //Close IAP function
    IAP_CMD = 0;                    //Clear command to standby
    IAP_TRIG = 0;                   //Clear trigger register
    IAP_ADDRH = 0x80;               //Data ptr point to non-EEPROM area
    IAP_ADDRL = 0;                  //Clear IAP address to prevent misuse
}

/*----------------------------
Read one byte from ISP/IAP/EEPROM area
Input: addr (ISP/IAP/EEPROM address)
Output:Flash data
----------------------------*/
BYTE IapReadByte(WORD addr)
{
    BYTE dat;                       //Data buffer

    IAP_CONTR = ENABLE_IAP;         //Open IAP function, and set wait time
    IAP_CMD = CMD_READ;             //Set ISP/IAP/EEPROM READ command
    IAP_ADDRL = addr;               //Set ISP/IAP/EEPROM address low
    IAP_ADDRH = addr >> 8;          //Set ISP/IAP/EEPROM address high
    IAP_TRIG = 0x46;                //Send trigger command1 (0x46)
    IAP_TRIG = 0xb9;                //Send trigger command2 (0xb9)
    _nop_();                        //MCU will hold here until ISP/IAP/EEPROM operation complete
    dat = IAP_DATA;                 //Read ISP/IAP/EEPROM data
    IapIdle();                      //Close ISP/IAP/EEPROM function

    return dat;                     //Return Flash data
}

/*----------------------------
Program one byte to ISP/IAP/EEPROM area
Input: addr (ISP/IAP/EEPROM address)
       dat (ISP/IAP/EEPROM data)
Output:-
----------------------------*/
void IapProgramByte(WORD addr, BYTE dat)
{
    IAP_CONTR = ENABLE_IAP;         //Open IAP function, and set wait time
    IAP_CMD = CMD_PROGRAM;          //Set ISP/IAP/EEPROM PROGRAM command
    IAP_ADDRL = addr;               //Set ISP/IAP/EEPROM address low
    IAP_ADDRH = addr >> 8;          //Set ISP/IAP/EEPROM address high
    IAP_DATA = dat;                 //Write ISP/IAP/EEPROM data
    IAP_TRIG = 0x46;                //Send trigger command1 (0x46)
    IAP_TRIG = 0xb9;                //Send trigger command2 (0xb9)
    _nop_();                        //MCU will hold here until ISP/IAP/EEPROM operation complete
    IapIdle();
}

/*----------------------------
Erase one sector area
Input: addr (ISP/IAP/EEPROM address)
Output:-
----------------------------*/
void IapEraseSector(WORD addr)
{
    IAP_CONTR = ENABLE_IAP;         //Open IAP function, and set wait time
    IAP_CMD = CMD_ERASE;            //Set ISP/IAP/EEPROM ERASE command
    IAP_ADDRL = addr;               //Set ISP/IAP/EEPROM address low
    IAP_ADDRH = addr >> 8;          //Set ISP/IAP/EEPROM address high
    IAP_TRIG = 0x46;                //Send trigger command1 (0x46)
    IAP_TRIG = 0xb9;                //Send trigger command2 (0xb9)
    _nop_();                        //MCU will hold here until ISP/IAP/EEPROM operation complete
    IapIdle();
}
/* ???? ??Flash???, ?????????,??????? */
/* begin_addr,????Flash????;counter,????????; array[],????   */
u8 sequential_write_flash_in_one_sector(u16 begin_addr, u16 counter, u8 array[])
{
    u16 i = 0;
    u16 in_sector_begin_addr = 0;
    u16 sector_addr = 0;
    /* ?? ???/?? ??? */
    IapEraseSector(begin_addr);

    for(i=0; i<counter; i++)
    {
        /* ????? */
        IapProgramByte(begin_addr, array[i]);
        /*  ???? */
        if (IapReadByte(begin_addr) != array[i])
        {
            IapIdle();
            return ERROR;
        }
        begin_addr++;
    }
    IapIdle();
    return  OK;
}
//========================================================================
// ??: void IapReadSector(u16 begin_addr, u16 counter, u8 array[])
// ??: IapReadSector(Sector_addr_0, 10, array);
// ??: ??0????10??????array[]????10????
// ??: begin_addr:?????,counter:???????,array[]:???????.
// ??: none
// ??: VER1.0
// ??: 2015-06-03
// ??: 
//========================================================================
void IapReadSector(u16 begin_addr, u16 counter, u8 array[])
{
	u16 i = 0;
	for(i=0; i<counter; i++)
	{
		array[i]=IapReadByte(begin_addr++);
	}
}
		
#endif