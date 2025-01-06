#ifndef __MCUBE_PED_INTERFACE_H__
#define __MCUBE_PED_INTERFACE_H__

#ifdef __cplusplus
extern "C" {
#endif

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
 * @brief Pedometer Open lib with default param.
 *
 * @return unsigned char,  1 means success;
 *                         0 means failure;
 */
extern unsigned char Ped_Open(void);

/**
 * @brief Pedometer Open lib with param.
 * FirstBlockSteps > = 0, the step count threshold for the algorithm learning the step pattern. default is 10.
 * MinRunningSteps,  the step counts in 3 sec to judge Running, default is 6 ;
 * MinRunningForce,  the force threshold to judge Running, default is 2500. 1g=1024.
 * @return unsigned char,  1 means success;
 *                         0 means failure;
*/
extern unsigned char Ped_OpenWithParam(short firstBlockSteps, short MinRunningSteps, short MinRunningForce);
/**
 * @brief Pedometer Open lib with param.
 * @return unsigned char,  1 means open;
 *                         0 means closed;
 */
extern unsigned char Ped_IsOpen(void);
/**
 * @brief Pedometer close lib.
 * @return unsigned char,  1 means success;
 *                         0 means failure;
 */
extern unsigned char Ped_Close(void);
/**
* @brief Pedometer Process Gsensor x,y,z Data, 1g=1024.
 * @return unsigned char,  1 means success;
 *                         0 means failure;
 */
extern unsigned char Ped_ProcessData(short x, short y, short z);
/**
 * @brief Pedometer reset step count.
 */
extern void Ped_ResetStepCount(void);

/**
 * @brief Pedometer get step count.
 */
extern unsigned long Ped_GetStepCount(void);

/**
 * @brief Pedometer get step stete.
 */
extern short Ped_GetState(void);

/**
 * @brief Get the version.
 *
 * @return uint32_t Return a 32 bits value it includes
 *         AlGOTITHM_MAJOR(4bits).MINOR(4bits).BUILD(4bits).Reserve(4bits).INTERFACE_MAJOR(4bits).MINOR(4bits).BUILD(4bits).Reserve(4bits)
 */
extern unsigned long Ped_GetVersion(void);

#ifdef __cplusplus
}
#endif

#endif // __MCUBE_PED_INTERFACE_H__

