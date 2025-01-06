/*********************************************************************
*               SEGGER MICROCONTROLLER GmbH & Co. KG                 *
*       Solutions for real time microcontroller applications         *
**********************************************************************
*                                                                    *
*       (c) 2015 - 2016  SEGGER Microcontroller GmbH & Co. KG        *
*                                                                    *
*       www.segger.com     Support: support@segger.com               *
*                                                                    *
**********************************************************************
*                                                                    *
*       SEGGER SystemView * Real-time application analysis           *
*                                                                    *
**********************************************************************
*                                                                    *
* All rights reserved.                                               *
*                                                                    *
* * This software may in its unmodified form be freely redistributed *
*   in source form.                                                  *
* * The source code may be modified, provided the source code        *
*   retains the above copyright notice, this list of conditions and  *
*   the following disclaimer.                                        *
* * Modified versions of this software in source or linkable form    *
*   may not be distributed without prior consent of SEGGER.          *
*                                                                    *
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS "AS IS" AND     *
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,  *
* THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A        *
* PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL               *
* SEGGER Microcontroller BE LIABLE FOR ANY DIRECT, INDIRECT,         *
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES           *
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS    *
* OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS            *
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,       *
* WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING          *
* NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS *
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.       *
*                                                                    *
**********************************************************************
*                                                                    *
*       SystemView version: V2.38                                    *
*                                                                    *
**********************************************************************
-------------------------- END-OF-HEADER -----------------------------

File    : SEGGER_SYSVIEW_Config_RTThread.c
Purpose : Sample setup configuration of SystemView with RT-Thread.
Revision: $Rev: 3735 $
*/
#include "rtthread.h"
#include "SEGGER_SYSVIEW.h"
#include "SEGGER_SYSVIEW_RTThread.h"

//
// SystemcoreClock can be used in most CMSIS compatible projects.
// In non-CMSIS projects define SYSVIEW_CPU_FREQ.
//
extern unsigned int SystemCoreClock;

/*********************************************************************
*
*       Defines, configurable
*
**********************************************************************
*/

// The application name to be displayed in SystemViewer
#ifndef   SYSVIEW_APP_NAME
	#ifndef PKG_USING_SYSTEMVIEW
		#define SYSVIEW_APP_NAME        "RT-Thread Trace"
	#else
		#define SYSVIEW_APP_NAME        PKG_SYSVIEW_APP_NAME
	#endif
#endif

// The target device name
#ifndef   SYSVIEW_DEVICE_NAME
	#ifndef PKG_USING_SYSTEMVIEW
		#define SYSVIEW_DEVICE_NAME     "Cortex-M4"
  	#else
		#define SYSVIEW_DEVICE_NAME        PKG_SYSVIEW_DEVICE_NAME
	#endif
#endif

// Frequency of the timestamp. Must match SEGGER_SYSVIEW_Conf.h
#ifndef   SYSVIEW_TIMESTAMP_FREQ
	#ifndef PKG_USING_SYSTEMVIEW
		#define SYSVIEW_TIMESTAMP_FREQ  (SystemCoreClock)
  	#else
		#if PKG_SYSVIEW_TIMESTAMP_FREQ == 0
			#define SYSVIEW_TIMESTAMP_FREQ        (SystemCoreClock)
		#else
			#define SYSVIEW_TIMESTAMP_FREQ        (PKG_SYSVIEW_TIMESTAMP_FREQ)
		#endif
	#endif
#endif

// System Frequency. SystemcoreClock is used in most CMSIS compatible projects.
#ifndef   SYSVIEW_CPU_FREQ
	#ifndef PKG_USING_SYSTEMVIEW
		#define SYSVIEW_CPU_FREQ        (SystemCoreClock)
  	#else
		#if PKG_SYSVIEW_CPU_FREQ == 0
			#define SYSVIEW_CPU_FREQ        (SystemCoreClock)
		#else
			#define SYSVIEW_CPU_FREQ        (PKG_SYSVIEW_CPU_FREQ)
		#endif
	#endif
#endif

// The lowest RAM address used for IDs (pointers)
#ifndef   SYSVIEW_RAM_BASE
	#ifndef PKG_USING_SYSTEMVIEW
		#define SYSVIEW_RAM_BASE        (0x20000000)
	#else
		#define SYSVIEW_RAM_BASE      (PKG_SYSVIEW_RAM_BASE)
	#endif
#endif

// Define as 1 if the Cortex-M cycle counter is used as SystemView timestamp. Must match SEGGER_SYSVIEW_Conf.h
#ifndef   USE_CYCCNT_TIMESTAMP
	#ifndef PKG_USING_SYSTEMVIEW
		#define USE_CYCCNT_TIMESTAMP    1
  	#else
		#ifdef PKG_SYSVIEW_USE_CYCCNT_TIMESTAMP
			#define USE_CYCCNT_TIMESTAMP    1
		#endif
	#endif
#endif



#ifndef   SYSVIEW_SYSDESC0
	#ifndef PKG_USING_SYSTEMVIEW
		#define SYSVIEW_SYSDESC0        "I#15=SysTick"
	#else
		#define SYSVIEW_SYSDESC0        PKG_SYSVIEW_SYSDESC0
	#endif
#endif

#ifndef   SYSVIEW_SYSDESC1
	#ifdef PKG_USING_SYSTEMVIEW
		#define SYSVIEW_SYSDESC1        PKG_SYSVIEW_SYSDESC1
	#endif
#endif

#ifndef   SYSVIEW_SYSDESC2
	#ifdef PKG_USING_SYSTEMVIEW
		#define SYSVIEW_SYSDESC2       PKG_SYSVIEW_SYSDESC2
	#endif
#endif






/*********************************************************************
*
*       Defines, fixed
*
**********************************************************************
*/
#define DWT_CTRL                  (*(volatile rt_uint32_t*) (0xE0001000uL))  // DWT Control Register
#define NOCYCCNT_BIT              (1uL << 25)                           // Cycle counter support bit
#define CYCCNTENA_BIT             (1uL << 0)                            // Cycle counter enable bit

/*********************************************************************
*
*       _cbSendSystemDesc()
*
*  Function description
*    Sends SystemView description strings.
*/
static void _cbSendSystemDesc(void) {
  SEGGER_SYSVIEW_SendSysDesc("N="SYSVIEW_APP_NAME",O=RT-Thread,D="SYSVIEW_DEVICE_NAME);
#ifdef SYSVIEW_SYSDESC0
  SEGGER_SYSVIEW_SendSysDesc(SYSVIEW_SYSDESC0);
#endif
#ifdef SYSVIEW_SYSDESC1
  SEGGER_SYSVIEW_SendSysDesc(SYSVIEW_SYSDESC1);
#endif
#ifdef SYSVIEW_SYSDESC2
  SEGGER_SYSVIEW_SendSysDesc(SYSVIEW_SYSDESC2);
#endif

#if defined(SF32LB55X)
    SEGGER_SYSVIEW_SendSysDesc("I#51=GPIO2,I#74=LCPU2HCPU,I#78=Epic,I#79=LCDC1,I#86=GPTIM1,I#87=GPTIM2,I#88=BTIM1,I#93=ExtDma,I#100=GPIO1,I#104=EZIP");
#elif defined(SF32LB56X)||defined(SF32LB58X)
    SEGGER_SYSVIEW_SendSysDesc("I#51=GPIO2,I#74=LCPU2HCPU,I#78=Epic,I#79=LCDC1,I#86=GPTIM1,I#87=GPTIM2,I#88=BTIM1,I#93=ExtDma,I#100=GPIO1,I#105=EZIP");
#elif defined(SF32LB52X)
    SEGGER_SYSVIEW_SendSysDesc("I#36=GPIO2,I#74=LCPU2HCPU,I#78=Epic,I#79=LCDC1,I#86=GPTIM1,I#87=GPTIM2,I#88=BTIM1,I#93=ExtDma,I#100=GPIO1,I#105=EZIP");
#endif

}

extern const SEGGER_SYSVIEW_OS_API SYSVIEW_X_OS_TraceAPI;
/*********************************************************************
*
*       Global functions
*
**********************************************************************
*/
void SEGGER_SYSVIEW_Conf(void) {
#if USE_CYCCNT_TIMESTAMP
  //
  //  The cycle counter must be activated in order
  //  to use time related functions.
  //
  if ((DWT_CTRL & NOCYCCNT_BIT) == 0) {       // Cycle counter supported?
    if ((DWT_CTRL & CYCCNTENA_BIT) == 0) {    // Cycle counter not enabled?
      DWT_CTRL |= CYCCNTENA_BIT;              // Enable Cycle counter
    }
  }
#endif
  SEGGER_SYSVIEW_Init(SYSVIEW_TIMESTAMP_FREQ, SYSVIEW_CPU_FREQ,
                      &SYSVIEW_X_OS_TraceAPI, _cbSendSystemDesc);
  SEGGER_SYSVIEW_SetRAMBase(SYSVIEW_RAM_BASE);
}

/*************************** End of file ****************************/
