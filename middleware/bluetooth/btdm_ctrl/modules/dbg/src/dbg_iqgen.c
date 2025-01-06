/**
 ****************************************************************************************
 *
 * @file dbg_iqgen.c
 *
 * @brief I&Q Samples Generator API.
 *
 * Copyright (C) RivieraWaves 2019
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup DBGIQGEN
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"

#if (EMB_PRESENT && BLE_IQ_GEN)

#include "dbg_iqgen.h"
#include "reg_iqgen.h"       // I&Q sample generator register functions


/*
 * DEFINES & ENUMERATIONS
 ****************************************************************************************
 */

// configure AoD switch delay (expressed in clock cycles)
#define DBG_IQGEN_AOD_SWITCH_DELAY      0

// configure I&Q transient delay (expressed in clock cycles)
#define DBG_IQGEN_IQ_TRANSIENT_DELAY    0

/*
 * GLOBAL VARIABLES
 ****************************************************************************************
 */

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

void dbg_iqgen_config_enable(uint8_t nb_antenna, uint8_t pattern_mode, uint8_t timing_mode, uint8_t ant_switch_mode)
{
    // validate configuration parameters
    ASSERT_ERR((timing_mode == DBG_IQGEN_1US_INTV) || (timing_mode == DBG_IQGEN_2US_INTV));
    ASSERT_ERR((pattern_mode == DBG_IQGEN_ANT_UPSWEEP) || (pattern_mode == DBG_IQGEN_ANT_UPDNSWEEP));
    ASSERT_ERR(nb_antenna <= DBG_IQGEN_MAX_ANTENNA);

    if (nb_antenna > 0)
    {
        // configure number of  antenna
        iqgen_nb_antenna_setf(nb_antenna);
        // configure antenna sweep pattern (0: antenna up-sweep, 1: antenna up-down sweep)
        iqgen_pattern_mode_setf(pattern_mode);
        // configure antenna timing mode (0: 1us switching/sampling interval, 1: 2us switching/sampling interval)
        iqgen_timing_mode_setf(timing_mode);

        // select internal generation
        iqgen_df_source_setf(0);
        // enable internal/external antenna switch control
        iqgen_antenna_switch_en_setf(ant_switch_mode);

        // configure AoD switch delay (expressed in clock cycles)
        iqgen_aod_switch_dly_setf(DBG_IQGEN_AOD_SWITCH_DELAY);
        // configure I&Q transient delay (expressed in clock cycles)
        iqgen_iq_invalid_dly_setf(DBG_IQGEN_IQ_TRANSIENT_DELAY);

        // enable I&Q sampling generation
        iqgen_iq_sampling_en_setf(1);
    }
    else
    {
        // invalid configuration; disable iqgen & raise assert warn
        dbg_iqgen_disable();
        ASSERT_WARN(nb_antenna, 0, 0);
    }
}

void dbg_iqgen_disable()
{
    // set dfsource to external generation, reset all other configurations to default
    iqgen_dfgencntl_pack(1 /*dfsource*/,
                         IQGEN_AOD_SWITCH_DLY_RST,
                         IQGEN_IQ_INVALID_DLY_RST,
                         IQGEN_NB_ANTENNA_RST,
                         IQGEN_PATTERN_MODE_RST,
                         IQGEN_TIMING_MODE_RST,
                         IQGEN_IQ_SAMPLING_EN_RST,
                         IQGEN_ANTENNA_SWITCH_EN_RST);
}

void dbg_iqgen_antenna_config(uint8_t antenna_id, uint8_t i_cntl, uint8_t q_cntl, uint8_t i_val, uint8_t q_val)
{
    switch (antenna_id)
    {
    case 0:
    {
        // configure pattern generation mode for I&Q samples from this antenna
        iqgen_i_cntl0_setf(i_cntl);
        iqgen_q_cntl0_setf(q_cntl);

        // configure initial values for I&Q samples from this antenna
        iqgen_i_offset0_setf(i_val);
        iqgen_q_offset0_setf(q_val);
    }
    break;
    case 1:
    {
        // configure pattern generation mode for I&Q samples from this antenna
        iqgen_i_cntl1_setf(i_cntl);
        iqgen_q_cntl1_setf(q_cntl);

        // configure initial values for I&Q samples from this antenna
        iqgen_i_offset1_setf(i_val);
        iqgen_q_offset1_setf(q_val);
    }
    break;
    case 2:
    {
        // configure pattern generation mode for I&Q samples from this antenna
        iqgen_i_cntl2_setf(i_cntl);
        iqgen_q_cntl2_setf(q_cntl);

        // configure initial values for I&Q samples from this antenna
        iqgen_i_offset2_setf(i_val);
        iqgen_q_offset2_setf(q_val);
    }
    break;
    case 3:
    {
        // configure pattern generation mode for I&Q samples from this antenna
        iqgen_i_cntl3_setf(i_cntl);
        iqgen_q_cntl3_setf(q_cntl);

        // configure initial values for I&Q samples from this antenna
        iqgen_i_offset3_setf(i_val);
        iqgen_q_offset3_setf(q_val);
    }
    break;
    case 4:
    {
        // configure pattern generation mode for I&Q samples from this antenna
        iqgen_iq_cntl4_setf(i_cntl);
        iqgen_q_cntl4_setf(q_cntl);

        // configure initial values for I&Q samples from this antenna
        iqgen_i_offset4_setf(i_val);
        iqgen_q_offset4_setf(q_val);
    }
    break;
    case 5:
    {
        // configure pattern generation mode for I&Q samples from this antenna
        iqgen_i_cntl5_setf(i_cntl);
        iqgen_q_cntl5_setf(q_cntl);

        // configure initial values for I&Q samples from this antenna
        iqgen_i_offset5_setf(i_val);
        iqgen_q_offset5_setf(q_val);
    }
    break;
    case 6:
    {
        // configure pattern generation mode for I&Q samples from this antenna
        iqgen_i_cntl6_setf(i_cntl);
        iqgen_q_cntl6_setf(q_cntl);

        // configure initial values for I&Q samples from this antenna
        iqgen_i_offset6_setf(i_val);
        iqgen_q_offset6_setf(q_val);
    }
    break;
    case 7:
    {
        // configure pattern generation mode for I&Q samples from this antenna
        iqgen_i_cntl7_setf(i_cntl);
        iqgen_q_cntl7_setf(q_cntl);

        // configure initial values for I&Q samples from this antenna
        iqgen_i_offset7_setf(i_val);
        iqgen_q_offset7_setf(q_val);
    }
    break;
    default:
    {
        ASSERT_ERR(0);
    }
    break;
    }
}

uint8_t dbg_iqgen_config(struct hci_dbg_iqgen_cfg_cmd const *cfg)
{
    uint8_t status = CO_ERROR_NO_ERROR;

    if (cfg->nb_antenna > 0)
    {
        if (cfg->nb_antenna <= DBG_IQGEN_MAX_ANTENNA)
        {
            uint8_t pattern_mode;
            uint8_t timing_mode;
            uint8_t ant_switch_mode;
            uint8_t i_val;
            uint8_t q_val;

            // fetch pattern & timing mode
            pattern_mode = (cfg->mode & DBG_IQGEN_PATTERN_MODE_BIT) ? DBG_IQGEN_ANT_UPDNSWEEP : DBG_IQGEN_ANT_UPSWEEP;
            timing_mode = (cfg->mode & DBG_IQGEN_TIMING_MODE_BIT) ? DBG_IQGEN_2US_INTV : DBG_IQGEN_1US_INTV;
            ant_switch_mode = (cfg->mode & DBG_IQGEN_ANT_ID_MODE_BIT) ? DBG_IQGEN_BASEBAND_SWITCHING : DBG_IQGEN_INTERNAL_SWITCHING;

            // configure number of antenna & mode
            dbg_iqgen_config_enable(cfg->nb_antenna, pattern_mode, timing_mode, ant_switch_mode);

            // determine antenna patterns
            for (int i = 0; i < cfg->nb_antenna; i++)
            {
                uint8_t i_cntl;
                uint8_t q_cntl;

                i_cntl = cfg->iq_ctrl[i].i;

                // I-control mapping
                if (i_cntl <= DBG_IQGEN_PRBSPATTERN)
                {
                    i_val = 0x00;
                }
                else
                {
                    i_val = i_cntl;
                    i_cntl = DBG_IQGEN_FIXEDVAL;
                }

                q_cntl = cfg->iq_ctrl[i].q;

                // Q-control mapping
                if (q_cntl <= DBG_IQGEN_PRBSPATTERN)
                {
                    q_val = 0x00;
                }
                else
                {
                    q_val = q_cntl;
                    q_cntl = DBG_IQGEN_FIXEDVAL;
                }

                // configure antenna
                dbg_iqgen_antenna_config(i, i_cntl, q_cntl, i_val, q_val);
            }
        }
        else
        {
            status = CO_ERROR_UNSUPPORTED;
        }
    }
    else
    {
        dbg_iqgen_disable();
    }

    return status;
}

#endif // (EMB_PRESENT && BLE_IQ_GEN)

/// @} DBGIQGEN

