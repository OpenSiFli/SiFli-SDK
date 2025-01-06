/**
 * @file mcube_pedometer_hooks.h
 * @author Steven Chen (steven.chen@mcubemems.com)
 * @date 2015/11/17
 * @brief mCube pedometer hook functions
 *
 */

#ifndef MCUBE_PEDOMETER_HOOKS_H
#define MCUBE_PEDOMETER_HOOKS_H

/**
 * -----------------------------------------------------------------------------
 * Included headers
 * -----------------------------------------------------------------------------
 */


/**
 * -----------------------------------------------------------------------------
 * Type declarations
 * -----------------------------------------------------------------------------
 */



/**
 * mCube pedometer status
 */
typedef enum
{
    MCUBE_LIB_STATIONARY,                       /**< Stationary status */
    MCUBE_LIB_WALKING,                          /**< Walking status */
    MCUBE_LIB_RUNNING                           /**< Running status */
} mCubeLibPedState_t;

#ifdef __cpusplus
extern "C" {
#endif

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
     * @brief Callback if on step detected.
     *
     * @param stepCount Step count
     */
    void mCubePed_onStepDetected(unsigned long stepCount);

    /**
     * @brief Callback if on state changed
     *
     * @param newState New state
     */
    void mCubePed_onStateChange(mCubeLibPedState_t newState);

#ifdef __cpusplus
}
#endif

#endif /**< !MCUBE_PEDOMETER_HOOKS_H */

