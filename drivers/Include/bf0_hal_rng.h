/*
 * SPDX-FileCopyrightText: 2016 STMicroelectronics
 * SPDX-FileCopyrightText: 2019-2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: BSD-3-Clause AND Apache-2.0
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BF0_HAL_RNG_H
#define __BF0_HAL_RNG_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "bf0_hal_def.h"

/** @addtogroup BF0_HAL_Driver
  * @{
  */

/** @addtogroup RNG
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/** @defgroup RNG_Exported_Types RNG Exported Types
  * @{
  */

#if defined(RNG_CR_CED)
/**
  * @brief  RNG Configuration Structure definition
  */
typedef struct
{
    uint32_t                    ClockErrorDetection; /*!< Clock error detection */
} RNG_InitTypeDef;
#endif /* defined(RNG_CR_CED) */

/**
  * @brief  RNG HAL State Structure definition
  */
typedef enum
{
    HAL_RNG_STATE_RESET     = 0x00,  /*!< RNG not yet initialized or disabled */
    HAL_RNG_STATE_READY     = 0x01,  /*!< RNG initialized and ready for use   */
    HAL_RNG_STATE_BUSY      = 0x02,  /*!< RNG internal process is ongoing     */
    HAL_RNG_STATE_TIMEOUT   = 0x03,  /*!< RNG timeout state                   */
    HAL_RNG_STATE_ERROR     = 0x04   /*!< RNG error state                     */

} HAL_RNG_StateTypeDef;

/**
  * @brief  RNG Handle Structure definition
  */
typedef struct __RNG_HandleTypeDef
{
    TRNG_TypeDef                 *Instance;     /*!< Register base address        */

#if defined(RNG_CR_CED)
    RNG_InitTypeDef             Init;          /*!< RNG configuration parameters */
#endif /* defined(RNG_CR_CED) */

    HAL_LockTypeDef             Lock;          /*!< RNG locking object           */

    __IO HAL_RNG_StateTypeDef   State;         /*!< RNG communication state      */

    __IO  uint32_t              ErrorCode;     /*!< RNG Error code               */

    uint32_t                    RandomNumber;  /*!< Last Generated RNG Data      */

#if (USE_HAL_RNG_REGISTER_CALLBACKS == 1)
    void (* RandReadyCallback)(struct __RNG_HandleTypeDef *hrng, uint32_t random32bit);  /*!< RNG Data Ready Callback    */
    void (* SeedReadyCallback)(struct __RNG_HandleTypeDef *hrng);                        /*!< RNG Seed Ready Callback         */

    void (* MspInitCallback)(struct __RNG_HandleTypeDef *hrng);                          /*!< RNG Msp Init callback      */
    void (* MspDeInitCallback)(struct __RNG_HandleTypeDef *hrng);                        /*!< RNG Msp DeInit callback    */
#endif  /* USE_HAL_RNG_REGISTER_CALLBACKS */

} RNG_HandleTypeDef;

#if (USE_HAL_RNG_REGISTER_CALLBACKS == 1)
/**
  * @brief  HAL RNG Callback ID enumeration definition
  */
typedef enum
{
    HAL_RNG_SEED_READY_CB_ID              = 0x00U,     /*!< RNG Seed ready Callback ID          */
    HAL_RNG_RAND_READY_CB_ID              = 0x01U,     /*!< RNG Rand ready Callback ID          */

    HAL_RNG_MSPINIT_CB_ID                 = 0x02U,     /*!< RNG MspInit callback ID        */
    HAL_RNG_MSPDEINIT_CB_ID               = 0x03U      /*!< RNG MspDeInit callback ID      */

} HAL_RNG_CallbackIDTypeDef;

/**
  * @brief  HAL RNG Callback pointer definition
  */
typedef  void (*pRNG_SeedReadyCallbackTypeDef)(RNG_HandleTypeDef *hrng, uint32_t seed32bit);             /*!< pointer to an RNG Seed Ready callback function */
typedef  void (*pRNG_RandReadyCallbackTypeDef)(RNG_HandleTypeDef *hrng, uint32_t random32bit);   /*!< pointer to an RNG Rand Ready callback function */

#endif /* USE_HAL_RNG_REGISTER_CALLBACKS */

/**
  * @}
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup RNG_Exported_Constants RNG Exported Constants
  * @{
  */

/** @defgroup RNG_Interrupt_definition  RNG Interrupts Definition
  * @{
  */
#define RNG_IT_SEED  TRNG_IRQ_SEED_GEN_DONE   /*!< Random seed generation done raw interrupt  */
#define RNG_IT_RAND  TRNG_IRQ_RAND_NUM_AVAIL  /*!< Random number available raw interrupt */
#define RNG_IT_LOCK  TRNG_IRQ_PRNG_LOCKUP     /*!< prng lockup raw interrupt */

/**
  * @}
  */

/** @defgroup RNG_Flag_definition  RNG Flags Definition
  * @{
  */
#define RNG_FLAG_RAND_VALID    TRNG_STAT_RAND_NUM_VALID     /*!< Random number valid flag */
#define RNG_FLAG_RAND_BUSY     TRNG_STAT_RAND_NUM_GEN_BUSY  /*!< Random number engine busy flag */
#define RNG_FLAG_SEED_VALID    TRNG_STAT_SEED_VALID         /*!< Random seed valid flag */
#define RNG_FLAG_SEED_BUSY     TRNG_STAT_SEED_GEN_BUSY      /*!< Random seed engine busy flag */

/**
  * @}
  */


/** @defgroup RNG_CMD_definition  RNG Command Definition
  * @{
  */
#define RNG_CMD_SEED    TRNG_CTRL_GEN_SEED_START     /*!< Generate random seed*/
#define RNG_CMD_RAND    TRNG_CTRL_GEN_RAND_NUM_START /*!< Generate random data*/

/**
  * @}
  */


/** @defgroup RNG_Error_Definition   RNG Error Definition
  * @{
  */
#define  HAL_RNG_ERROR_NONE             ((uint32_t)0x00000000U)    /*!< No error                */
#if (USE_HAL_RNG_REGISTER_CALLBACKS == 1)
#define  HAL_RNG_ERROR_INVALID_CALLBACK ((uint32_t)0x00000001U)    /*!< Invalid Callback error  */
#endif /* USE_HAL_RNG_REGISTER_CALLBACKS */
/**
  * @}
  */

/**
  * @}
  */

/* Exported macros -----------------------------------------------------------*/
/** @defgroup RNG_Exported_Macros RNG Exported Macros
  * @{
  */

/** @brief Reset RNG handle state.
  * @param  \__HANDLE__: RNG Handle
  * @retval None
  */
#if (USE_HAL_RNG_REGISTER_CALLBACKS == 1)
#define __HAL_RNG_RESET_HANDLE_STATE(__HANDLE__)  do{                                                   \
                                                       (__HANDLE__)->State = HAL_RNG_STATE_RESET;       \
                                                       (__HANDLE__)->MspInitCallback = NULL;            \
                                                       (__HANDLE__)->MspDeInitCallback = NULL;          \
                                                    } while(0U)
#else
#define __HAL_RNG_RESET_HANDLE_STATE(__HANDLE__) ((__HANDLE__)->State = HAL_RNG_STATE_RESET)
#endif /*USE_HAL_RNG_REGISTER_CALLBACKS */

/**
  * @brief  RNG module command.
  * @param  \__HANDLE__: RNG Handle
  * @param  CMD:        RNG command
  *          This parameter can be one of the following values:
  *             @arg RNG_CMD_RAND:    Generate Random number
  *             @arg RNG_CMD_SEED :   Generate Random seed
  * @retval None
  */
#define __HAL_RNG_COMMAND(__HANDLE__,CMD)  (__HANDLE__)->Instance->CTRL=CMD

/**
  * @brief  Enable the RNG peripheral.
  * @param  \__HANDLE__: RNG Handle
  * @retval None
  */
#define __HAL_RNG_ENABLE(__HANDLE__)    do { \
                                            (__HANDLE__)->Instance->CTRL &= ~(TRNG_CTRL_GEN_SEED_STOP | TRNG_CTRL_GEN_RAND_NUM_STOP); \
                                        } while (0)

/**
  * @brief  Disable the RNG peripheral.
  * @param  \__HANDLE__: RNG Handle
  * @retval None
  */
#define __HAL_RNG_DISABLE(__HANDLE__)   do { \
                                            (__HANDLE__)->Instance->CTRL |= (TRNG_CTRL_GEN_SEED_STOP | TRNG_CTRL_GEN_RAND_NUM_STOP); \
                                        } while (0)

/**
  * @brief  Check whether the specified RNG flag is set or not.
  * @param  \__HANDLE__: RNG Handle
  * @param  \__FLAG__: RNG flag
  *          This parameter can be one of the following values:
  *             @arg RNG_FLAG_RAND_VALID:    Random number valid flag
  *             @arg RNG_FLAG_RAND_BUSY :    Random number engine busy flag
  *             @arg RNG_FLAG_SEED_VALID:    Random seed valid flag
  *             @arg RNG_FLAG_SEED_BUSY :    Random seed engine busy flag
  * @retval The new state of __FLAG__ (SET or RESET).
  */
#define __HAL_RNG_GET_FLAG(__HANDLE__, __FLAG__) (((__HANDLE__)->Instance->STAT & (__FLAG__)) == (__FLAG__))

/**
  * @brief  Clear the selected RNG flag status.
  * @param  \__HANDLE__: RNG handle
  * @param  \__FLAG__: RNG flag to clear
  * @note   WARNING: This is a dummy macro for HAL code alignment,
  *         flags RNG_FLAG_XXXX are read-only.
  * @retval None
  */
#define __HAL_RNG_CLEAR_FLAG(__HANDLE__, __FLAG__)                      /* dummy  macro */

/**
  * @brief  Enable the RNG interrupt.
  * @param  \__HANDLE__: RNG Handle
  * @retval None
  */
#define __HAL_RNG_ENABLE_IT(__HANDLE__) ((__HANDLE__)->Instance->IRQ |=  \
        (TRNG_IRQ_PRNG_LOCKUP_MSK|TRNG_IRQ_RAND_NUM_AVAIL_MSK|TRNG_IRQ_SEED_GEN_DONE_MSK))

/**
  * @brief  Disable the RNG interrupt.
  * @param  \__HANDLE__: RNG Handle
  * @retval None
  */
#define __HAL_RNG_DISABLE_IT(__HANDLE__) ((__HANDLE__)->Instance->IRQ &= \
        ~(TRNG_IRQ_PRNG_LOCKUP_MSK|TRNG_IRQ_RAND_NUM_AVAIL_MSK|TRNG_IRQ_SEED_GEN_DONE_MSK))

/**
  * @brief  Check whether the specified RNG interrupt has occurred or not.
  * @param  \__HANDLE__: RNG Handle
  * @param  \__INTERRUPT__: specifies the RNG interrupt status flag to check.
  *         This parameter can be one of the following values:
  *            @arg RNG_IT_SEED : Random seed generation done raw interrupt
  *            @arg RNG_IT_RAND : Random number available raw interrupt
  *            @arg RNG_IT_LOCK : prng lockup raw interrupt
  * @retval The new state of \__INTERRUPT__ (SET or RESET).
  */
#define __HAL_RNG_GET_IT(__HANDLE__, __INTERRUPT__) (((__HANDLE__)->Instance->IRQ & (__INTERRUPT__)) == (__INTERRUPT__))

/**
  * @brief  Clear the RNG interrupt status flags.
  * @param  \__HANDLE__: RNG Handle
  * @param  \__INTERRUPT__: specifies the RNG interrupt status flag to clear.
  *          This parameter can be one of the following values:
  *            @arg RNG_IT_SEED : Random seed generation done raw interrupt
  *            @arg RNG_IT_RAND : Random number available raw interrupt
  *            @arg RNG_IT_LOCK : prng lockup raw interrupt
  * @retval None
  */
#define __HAL_RNG_CLEAR_IT(__HANDLE__, __INTERRUPT__) (((__HANDLE__)->Instance->IRQ) = (__INTERRUPT__))

/**
  * @}
  */


/* Exported functions --------------------------------------------------------*/
/** @defgroup RNG_Exported_Functions RNG Exported Functions
  * @{
  */

/* Initialization and de-initialization functions  ******************************/
/** @defgroup RNG_Exported_Functions_Group1 Initialization and de-initialization functions
  * @{
  */

/**
  * @brief  Initialize the RNG peripheral and initialize the associated handle.
  * @param  hrng: pointer to a RNG_HandleTypeDef structure.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_RNG_Init(RNG_HandleTypeDef *hrng);

/**
  * @brief  DeInitialize the RNG peripheral.
  * @param  hrng: pointer to a RNG_HandleTypeDef structure.
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_RNG_DeInit(RNG_HandleTypeDef *hrng);

/**
  * @brief  Initialize the RNG MSP.
  * @param  hrng: pointer to a RNG_HandleTypeDef structure.
  * @retval None
  */
void HAL_RNG_MspInit(RNG_HandleTypeDef *hrng);

/**
  * @brief  DeInitialize the RNG MSP.
  * @param  hrng: pointer to a RNG_HandleTypeDef structure.
  * @retval None
  */
void HAL_RNG_MspDeInit(RNG_HandleTypeDef *hrng);

/* Callbacks Register/UnRegister functions  ***********************************/
#if (USE_HAL_RNG_REGISTER_CALLBACKS == 1)

/**
  * @brief  Register a User RNG Callback
  *         To be used instead of the weak predefined callback
  * @param  hrng RNG handle
  * @param  CallbackID ID of the callback to be registered
  *         This parameter can be one of the following values:
  *          @arg @ref HAL_RNG_ERROR_CB_ID Error callback ID
  *          @arg @ref HAL_RNG_MSPINIT_CB_ID MspInit callback ID
  *          @arg @ref HAL_RNG_MSPDEINIT_CB_ID MspDeInit callback ID
  * @param  pCallback pointer to the Callback function
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_RNG_RegisterCallback(RNG_HandleTypeDef *hrng, HAL_RNG_CallbackIDTypeDef CallbackID, pRNG_CallbackTypeDef pCallback);


/**
  * @brief  Unregister an RNG Callback
  *         RNG callabck is redirected to the weak predefined callback
  * @param  hrng RNG handle
  * @param  CallbackID ID of the callback to be unregistered
  *         This parameter can be one of the following values:
  *          @arg @ref HAL_RNG_ERROR_CB_ID Error callback ID
  *          @arg @ref HAL_RNG_MSPINIT_CB_ID MspInit callback ID
  *          @arg @ref HAL_RNG_MSPDEINIT_CB_ID MspDeInit callback ID
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_RNG_UnRegisterCallback(RNG_HandleTypeDef *hrng, HAL_RNG_CallbackIDTypeDef CallbackID);

/**
  * @brief  Register Data Ready RNG Callback
  *         To be used instead of the weak HAL_RNG_ReadyDataCallback() predefined callback
  * @param  hrng RNG handle
  * @param  pCallback pointer to the Data Ready Callback function
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_RNG_RegisterReadyDataCallback(RNG_HandleTypeDef *hrng, pRNG_ReadyDataCallbackTypeDef pCallback);

/**
  * @brief  UnRegister the Data Ready RNG Callback
  *         Data Ready RNG Callback is redirected to the weak HAL_RNG_ReadyDataCallback() predefined callback
  * @param  hrng RNG handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_RNG_UnRegisterReadyDataCallback(RNG_HandleTypeDef *hrng);
#endif /* USE_HAL_RNG_REGISTER_CALLBACKS */

/**
  * @}
  */

/* Peripheral Control functions  ************************************************/
/** @defgroup RNG_Exported_Functions_Group2 Peripheral Control functions
  * @{
  */
/**
  * @brief  Read latest generated random number.
  * @param  hrng: pointer to a RNG_HandleTypeDef structure.
  * @retval random value
  */
uint32_t HAL_RNG_ReadLastRandomNumber(RNG_HandleTypeDef *hrng);

#define HAL_RNG_GenerateRandomNumber(hrng,random32bit) HAL_RNG_Generate(hrng,random32bit,0)
#define HAL_RNG_GenerateRandomNumber_IT(hrng,random32bit) HAL_RNG_Generate_IT(hrng,random32bit,0)
#define HAL_RNG_GenerateRandomSeed(hrng,seed32bit) HAL_RNG_Generate(hrng,seed32bit,1)
#define HAL_RNG_GenerateRandomSeed_IT(hrng,seed32bit) HAL_RNG_Generate_IT(hrng,seed32bit,1)

/**
  * @brief  Generate a 32-bit random number or seed.
  * @note   Each time the random number data is read the RNG_FLAG_DRDY flag
  *         is automatically cleared.
  * @param  hrng: pointer to a RNG_HandleTypeDef structure.
  * @param  random32bit: pointer to generated random number variable if successful.
  * @param  is_seed: 1:generate seed. 0:Generate random number
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_RNG_Generate(RNG_HandleTypeDef *hrng, uint32_t *random32bit, uint8_t is_seed);

/**
  * @brief  Generate random number or seed in interrupt mode.
  * @param  hrng: pointer to a RNG_HandleTypeDef structure.
  * @param  is_seed: 1:generate seed. 0:Generate random number
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_RNG_Generate_IT(RNG_HandleTypeDef *hrng, uint8_t is_seed);

/**
  * @brief  Handle RNG interrupt request.
  * @note   In the case of a clock error, the RNG is no more able to generate
  *         random numbers because the PLL48CLK clock is not correct. User has
  *         to check that the clock controller is correctly configured to provide
  *         the RNG clock and clear the CEIS bit using __HAL_RNG_CLEAR_IT().
  *         The clock error has no impact on the previously generated
  *         random numbers, and the RNG_DR register contents can be used.
  * @note   In the case of a seed error, the generation of random numbers is
  *         interrupted as long as the SECS bit is '1'. If a number is
  *         available in the RNG_DR register, it must not be used because it may
  *         not have enough entropy. In this case, it is recommended to clear the
  *         SEIS bit using __HAL_RNG_CLEAR_IT(), then disable and enable
  *         the RNG peripheral to reinitialize and restart the RNG.
  * @note   RNG ErrorCallback() API is called once whether SEIS or CEIS are set.
  * @param  hrng: pointer to a RNG_HandleTypeDef structure.
  * @retval None

  */
void HAL_RNG_IRQHandler(RNG_HandleTypeDef *hrng);

/**
  * @brief  Random Data Ready callback in non-blocking mode.
  * @param  hrng: pointer to a RNG_HandleTypeDef structure.
  * @param  random32bit: generated random value
  * @retval None
  */
void HAL_RNG_RandReadyCallback(RNG_HandleTypeDef *hrng, uint32_t random32bit);


/**
  * @brief  Random Seed Ready callback in non-blocking mode.
  * @param  hrng: pointer to a RNG_HandleTypeDef structure.
  * @param  seed32bit: generated random seed
  * @retval None
  */
void HAL_RNG_SeedReadyCallback(RNG_HandleTypeDef *hrng, uint32_t seed32bit);
/**
  * @}
  */

/* Peripheral State functions  **************************************************/
/** @defgroup RNG_Exported_Functions_Group3 Peripheral State and Error functions
  * @{
  */

/**
  * @brief  Return the RNG handle state.
  * @param  hrng: pointer to a RNG_HandleTypeDef structure.
  * @retval HAL state
  */
HAL_RNG_StateTypeDef HAL_RNG_GetState(RNG_HandleTypeDef *hrng);

/**
  * @brief  Return the RNG handle error code.
  * @param  hrng: pointer to a RNG_HandleTypeDef structure.
  * @retval RNG Error Code
*/
uint32_t             HAL_RNG_GetError(RNG_HandleTypeDef *hrng);
/**
  * @}
  */

/**
  * @}
  */

/* Private types -------------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/** @addtogroup  RNG_Private_Macros   RNG Private Macros
  * @{
  */

#define IS_RNG_ALL_INSTANCE(__INSTANCE__)      ((__INSTANCE__) == TRNG)

/**
  * @}
  */
/* Private functions prototypes ----------------------------------------------*/

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __BF0_HAL_RNG_H */