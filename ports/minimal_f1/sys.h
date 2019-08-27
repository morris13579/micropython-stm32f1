#ifndef __SYS_H
#define __SYS_H	  
#include <stm32f103xe.h>   
#include <definename.h>   

//0,不支持OS
//1,支持OS
#define SYSTEM_SUPPORT_OS		0		//定義系統文件夾是否支持OS
																	    
	 
//位帶操作,實現51類似的GPIO控制功能
//具體實現思想,參考<<CM3權威指南>>第五章(87頁~92頁).
//IO口操作宏定義
#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2)) 
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr)) 
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum)) 
//IO口地址映射
#define GPIOA_ODR_Addr    (GPIOA_BASE+12) //0x4001080C 
#define GPIOB_ODR_Addr    (GPIOB_BASE+12) //0x40010C0C 
#define GPIOC_ODR_Addr    (GPIOC_BASE+12) //0x4001100C 
#define GPIOD_ODR_Addr    (GPIOD_BASE+12) //0x4001140C 
#define GPIOE_ODR_Addr    (GPIOE_BASE+12) //0x4001180C 
#define GPIOF_ODR_Addr    (GPIOF_BASE+12) //0x40011A0C    
#define GPIOG_ODR_Addr    (GPIOG_BASE+12) //0x40011E0C    

#define GPIOA_IDR_Addr    (GPIOA_BASE+8) //0x40010808 
#define GPIOB_IDR_Addr    (GPIOB_BASE+8) //0x40010C08 
#define GPIOC_IDR_Addr    (GPIOC_BASE+8) //0x40011008 
#define GPIOD_IDR_Addr    (GPIOD_BASE+8) //0x40011408 
#define GPIOE_IDR_Addr    (GPIOE_BASE+8) //0x40011808 
#define GPIOF_IDR_Addr    (GPIOF_BASE+8) //0x40011A08 
#define GPIOG_IDR_Addr    (GPIOG_BASE+8) //0x40011E08 
 
//IO口操作,只對單一的IO口!
//確保n的值小於16!
#define PAout(n)   BIT_ADDR(GPIOA_ODR_Addr,n)  //輸出 
#define PAin(n)    BIT_ADDR(GPIOA_IDR_Addr,n)  //輸入 

#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)  //輸出 
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)  //輸入 

#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr,n)  //輸出 
#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr,n)  //輸入 

#define PDout(n)   BIT_ADDR(GPIOD_ODR_Addr,n)  //輸出 
#define PDin(n)    BIT_ADDR(GPIOD_IDR_Addr,n)  //輸入 

#define PEout(n)   BIT_ADDR(GPIOE_ODR_Addr,n)  //輸出 
#define PEin(n)    BIT_ADDR(GPIOE_IDR_Addr,n)  //輸入

#define PFout(n)   BIT_ADDR(GPIOF_ODR_Addr,n)  //輸出 
#define PFin(n)    BIT_ADDR(GPIOF_IDR_Addr,n)  //輸入

#define PGout(n)   BIT_ADDR(GPIOG_ODR_Addr,n)  //輸出 
#define PGin(n)    BIT_ADDR(GPIOG_IDR_Addr,n)  //輸入
/////////////////////////////////////////////////////////////////
//Ex_NVIC_Config專用定義
#define GPIO_A 0
#define GPIO_B 1
#define GPIO_C 2
#define GPIO_D 3
#define GPIO_E 4
#define GPIO_F 5
#define GPIO_G 6 
#define FTIR   1  //下降沿觸發
#define RTIR   2  //上升沿觸發
								   

//JTAG模式設置定義
#define JTAG_SWD_DISABLE   0X02
#define SWD_ENABLE         0X01
#define JTAG_SWD_ENABLE    0X00	

/////////////////////////////////////////////////////////////////  
void Stm32_Clock_Init(u8 PLL);  //時鐘初始化  
void Sys_Soft_Reset(void);      //系統軟復位
void Sys_Standby(void);         //待機模式 	
void MY_NVIC_SetVectorTable(u32 NVIC_VectTab, u32 Offset);//設置偏移地址
void MY_NVIC_PriorityGroupConfig(u8 NVIC_Group);//設置NVIC分組
void MY_NVIC_Init(u8 NVIC_PreemptionPriority,u8 NVIC_SubPriority,u8 NVIC_Channel,u8 NVIC_Group);//設置中斷
void Ex_NVIC_Config(u8 GPIOx,u8 BITx,u8 TRIM);//外部中斷配置函數(只對GPIOA~G)
void JTAG_Set(u8 mode);
//////////////////////////////////////////////////////////////////////////////
//以下為彙編函數
void WFI_SET(void);		//執行WFI指令
void INTX_DISABLE(void);//關閉所有中斷
void INTX_ENABLE(void);	//開啟所有中斷
void MSR_MSP(u32 addr);	//設置堆棧地址

#endif











