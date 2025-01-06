#include "mcube_sleepmeter_hooks.h"
/// Include header files
//#include "mcube_custom_config.h"

#include "app.h"
/**
 ****************************************************************************************
 * @brief mCube sleepmeter state change hook function. It will be called by sleepmeter
 *        if the state of sleepmeter is changed. The states of sleepmeter are listed as
 *        below.
 *
 *        MCUBE_SLEEPMETER_AWAKE      User is awake.        (0x00000000)
 *        MCUBE_SLEEPMETER_SLEEPING   User is sleeping.     (0x00000001)
 *        MCUBE_SLEEPMETER_RESTLESS   User is restless.     (0x00000002)
 *        MCUBE_SLEEPMETER_ROLLOVER   User does roll-over.  (0x00000004)
 *
 * @param[in] oldState      Latest state of sleepmeter
 * @param[in] newState      Current state of sleepmeter
 * @param[in] time          Current time in seconds
 **@param[in] period        Ellapsed time since latest state
 * @param[in] covariant     Data covariant
 *
 ****************************************************************************************
*/
void mCubeSleep_onStateChange(mCubeSleepState_t oldState,
                              mCubeSleepState_t newState,
                              unsigned long time,
                              unsigned long period,
                              unsigned long covariant)
{
    //mcube_printf("sleep: oldState=%d,newState=%d,time =%d,period =%d\r\n",oldState,newState,time,period);
    if (newState !=  oldState)
    {

        if (newState == MCUBE_SLEEPMETER_AWAKE)
        {

            //  SEGGER_RTT_printf(0,"未进入睡眠 \r\n \r\n");
        }
        else if (newState == MCUBE_SLEEPMETER_SLEEPING)
        {

            //                         SEGGER_RTT_printf(0,"深度睡眠 \r\n \r\n");
        }
        else if (newState == MCUBE_SLEEPMETER_RESTLESS)
        {

            //   SEGGER_RTT_printf(0,"浅度睡眠 \r\n \r\n");
        }
        else if (newState == MCUBE_SLEEPMETER_IDLE)
        {

        }

    }
    // Store data in the device or send data to the receiver here.
    // For example, notify data in little endian order via BLE.
    //#error "Please implement mCubeSleep_onStateChange() with mcube_sleep.lib."    // Remove this line after this function is implemented
}

