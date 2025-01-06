/**
 * @file mcube_sleepmeter_hooks.h
 * @author Steven Chen (steven.chen@mcubemems.com)
 * @date 2015/12/7
 * @brief mCube sleepmeter hook functions
 *
 */

#ifndef MCUBE_SLEEPMETER_HOOKS_H
#define MCUBE_SLEEPMETER_HOOKS_H

#ifdef __cplusplus
extern "C" {
#endif

//#include <stdint.h>
//#include <stdbool.h>

/* Type Definitions */
typedef unsigned long mCubeSleepState_t;

#define MCUBE_SLEEPMETER_AWAKE          ((mCubeSleepState_t)0x00000000)
#define MCUBE_SLEEPMETER_SLEEPING       ((mCubeSleepState_t)0x00000001)
#define MCUBE_SLEEPMETER_RESTLESS       ((mCubeSleepState_t)0x00000002)
#define MCUBE_SLEEPMETER_ROLLOVER       ((mCubeSleepState_t)0x00000004)
#define MCUBE_SLEEPMETER_IDLE           ((mCubeSleepState_t)0x00000008)

#define MCUBE_SLEEPMETER_MASK           (MCUBE_SLEEPMETER_SLEEPING | \
                                         MCUBE_SLEEPMETER_RESTLESS | \
                                         MCUBE_SLEEPMETER_ROLLOVER | \
                                         MCUBE_SLEEPMETER_IDLE)




/**
 * -----------------------------------------------------------------------------
 * External linkage variables
 * -----------------------------------------------------------------------------
 */

/**
 * -----------------------------------------------------------------------------
 * API declarations
 * -----------------------------------------------------------------------------
 */

/**
 * @brief mCube sleepmeter state change hook function. It will be called by
 *        sleepmeter if the state of sleepmeter is changed. The states of
 *        sleepmeter are listed as below.
 *
 *        (0): MCUBE_SLEEPMETER_AWAKE      User is awake.
 *        (1): MCUBE_SLEEPMETER_SLEEPING   User is sleeping.
 *        (2): MCUBE_SLEEPMETER_RESTLESS   User is restless.
 *        (4): MCUBE_SLEEPMETER_ROLLOVER   User does roll-over.
 *        (8); MCUBE_SLEEPMETER_IDLE       User is idle
 * @param oldState Latest state of sleepmeter
 * @param newState Current state of sleepmeter
 * @param time Current time in milliseconds
 **@param period Elapsed time since latest state
 * @param covariant Data covariant
 *
 */
void mCubeSleep_onStateChange(mCubeSleepState_t oldState,
                              mCubeSleepState_t newState,
                              unsigned long time,
                              unsigned long period,
                              unsigned long covariant);

#ifdef __cplusplus
}
#endif

#endif // MCUBE_SLEEPMETER_HOOKS_H
