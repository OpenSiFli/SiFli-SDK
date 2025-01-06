#include "mcube_pedometer_hooks.h"
/// Include header files
//#include "mcube_custom_config.h"

mCubeLibPedState_t cur_step_state;
unsigned long cur_total_step;


/**
 ****************************************************************************************
 * @brief mCube pedometer detect step hook function. It will be called by pedometer
 *        if the step is detected.
 *
 * @param[in] stepCount     Current step count
 *
 ****************************************************************************************
*/
void mCubePed_onStepDetected(unsigned long stepCount)
{
    //mcube_printf("Pedo: stepCount=%d\r\n",stepCount);

    cur_total_step = stepCount;

    // Store data in the device or send data to the receiver here.
    // For example, notify data in little endian order via BLE.
//#error "Please implement mCubePed_onStepDetected() with mcube_pedo.lib."      // Remove this line after this function is implemented
}

/**
 ****************************************************************************************
 * @brief mCube pedometer state change hook function. It will be called by pedometer
 *        if the state of pedometer is change. The states of mCube pedometer are listed
 *        as below.
 *
 *        MCUBE_LIB_STATIONARY          User is stationary.
 *        MCUBE_LIB_WALKING             User is walking.
 *        MCUBE_LIB_RUNNING             User is running.
 *
 * @param[in] newState     Current state of the pedometer
 *
 ****************************************************************************************
*/
void mCubePed_onStateChange(mCubeLibPedState_t newState)
{
    //mcube_printf("Pedo: newState =%d\r\n",newState);
    cur_step_state = newState;
    // Should store data in the device or send data to the receiver here.
    // For example, notify data in little endian order via BLE.
//#error "Please implement mCubePed_onStateChange() with mcube_pedo.lib."       // Remove this line after this function is implemented
}

