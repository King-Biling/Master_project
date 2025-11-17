#include "delay.h"
#include "sys.h"
#include "system.h"
#include "main.h"
////////////////////////////////////////////////////////////////////////////////// 	 
//如果使用OS,则包括下面的头文件即可
#if SYSTEM_SUPPORT_OS
#include "FreeRTOS.h"					//FreeRTOS使用		  
#include "task.h"
#include "stm32f4xx_hal.h"
#include "FreeRTOSConfig.h"  // ?? configTICK_RATE_HZ ??
#endif


static uint32_t fac_us=0;							//us延时倍乘数			   
static uint32_t fac_ms=0;							//ms延时倍乘数,在os下,代表每个节拍的ms数

static u32 sysTickCnt=0;
extern void xPortSysTickHandler(void);
 
//systick中断服务函数,使用OS时用到
//void SysTick_Handler(void)
//{	
//    if(xTaskGetSchedulerState()!=taskSCHEDULER_NOT_STARTED)//系统已经运行
//    {
//        xPortSysTickHandler();	
//    }
//}
	/********************************************************
*getSysTickCnt()
*调度开启之前 返回 sysTickCnt
*调度开启之前 返回 xTaskGetTickCount()
*********************************************************/
u32 getSysTickCnt(void)
{
	if(xTaskGetSchedulerState()!=taskSCHEDULER_NOT_STARTED)	/*系统已经运行*/
		return xTaskGetTickCount();
	else
		return sysTickCnt;
}		   

//初始化延迟函数
//SYSTICK的时钟固定为AHB时钟，基础例程里面SYSTICK时钟频率为AHB/8
//这里为了兼容FreeRTOS，所以将SYSTICK的时钟频率改为AHB的频率！
//SYSCLK:系统时钟频率
/**
 * @brief  Initialize SysTick for delay and FreeRTOS Tick.
 *         - Set SysTick clock source to AHB (system clock).
 *         - Calculate delay factor and SysTick reload value for FreeRTOS.
 * @note   Must be called after SystemClock_Config() (to ensure SystemCoreClock is valid).
 */
void delay_init(void)
{
    // 1. Set SysTick clock source to AHB (system clock)
    //    Bit 2 of SysTick->CTRL: 0=AHB/8, 1=AHB (system clock)
    SysTick->CTRL |= (1U << 2);  

    // 2. Calculate fac_us: SysTick ticks per microsecond
    //    SysTick clock = SystemCoreClock (since source is AHB)
    //    Example: SystemCoreClock=16MHz ? fac_us=16 (16 ticks per 祍)
    fac_us = SystemCoreClock / 1000000UL;  

    // 3. Calculate SysTick reload value for FreeRTOS Tick
    //    Tick period = 1 / configTICK_RATE_HZ (seconds)
    //    Reload = (SysTick clock * Tick period) - 1 
    uint32_t reload = (SystemCoreClock / configTICK_RATE_HZ) - 1;  

    // 4. Configure SysTick for FreeRTOS
    SysTick->LOAD = reload;                  // Set reload value
    SysTick->VAL  = 0;                       // Clear current count
    SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;  // Enable SysTick interrupt
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;   // Start SysTick timer
}
						    

/**
 * @brief  Microsecond delay using SysTick counter.
 * @param  nus: Delay time in microseconds.
 * @note   Requires fac_us to be initialized (ticks per 祍).
 */
void delay_us(uint32_t nus)
{
    uint32_t ticks = nus * fac_us;  // Total SysTick ticks needed
    uint32_t told, tnow, cnt = 0;
    uint32_t reload = SysTick->LOAD;  // SysTick reload value (from FreeRTOS config)
    
    told = SysTick->VAL;  // Record initial counter value
    
    while (1)
    {
        tnow = SysTick->VAL;
        if (tnow != told)  // Counter changed (either decrement or overflow)
        {
            if (tnow < told)  // No overflow: count = told - tnow
            {
                cnt += told - tnow;
            }
            else  // Overflow: count = (reload - tnow) + told
            {
                cnt += reload - tnow + told;
            }
            told = tnow;  // Update initial value for next check
            
            if (cnt >= ticks)  // Delay time reached
            {
                break;
            }
        }
    }
}

/**
 * @brief  Millisecond delay with FreeRTOS support.
 * @param  nms: Delay time in milliseconds.
 * @note   Uses vTaskDelay if FreeRTOS is running, else uses delay_us.
 */
void delay_ms(uint32_t nms)
{
    // Check if FreeRTOS scheduler is active
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
    {
        if (nms >= fac_ms)  // Delay > 1ms granularity
        {
            vTaskDelay(nms / fac_ms);  // FreeRTOS delay (assume fac_ms = 1ms ticks)
            nms %= fac_ms;  // Remaining time < 1ms
        }
    }
    
    // Delay remaining time with normal method
    delay_us((uint32_t)(nms * 1000));
}


/**
 * @brief  Busy-wait millisecond delay (no task scheduling).
 * @param  nms: Delay time in milliseconds.
 * @note   Uses delay_us(1000) in a loop.
 */
void delay_xms(uint32_t nms)
{
    for (uint32_t i = 0; i < nms; i++)
    {
        delay_us(1000);  // Delay 1ms per iteration
    }
}
			 



































