/* This sample code teaches developer how to operate the MEMSIC alogrithm library
 * to get Yaw, Pitch and Roll respectively. if developer want to use it as a part
 * of his codes, he need to declare and define 3 functions by himself:
 * ReadPara;
 * Savepara;
 * please NOTE it is ONLY a sample code and can be changed freely by developers.
 */

#include "MemsicAlgo.h"
#include "MemsicCompass.h"
#include "MMC36X0KJ.h"
#include "MMC36X_Customer.h"

#include <rtdbg.h>


#define A_LAYOUT    0
#define M_LAYOUT    0

/* magnetic sensor soft iron calibration matrix, default is unit matrix */
float mag_smm[9] = { 1.0f, 0.0f, 0.0f, \
                     0.0f, 1.0f, 0.0f, \
                     0.0f, 0.0f, 1.0f
                   };

/* default hard iron offset and magnetic field strength*/
float mag_hmm[4] = {0.0f, 0.0f, 0.0f, 0.5f};

/* These variables are used to save the magnetic sensor calibrated data. */
float calMagX, calMagY, calMagZ;

/* These variables are used to save the calibrated orientation output. */
float fAzimuth, fPitch, fRoll;

/* This variable is used to save the calibration accuracy. */
int iAccuracy;

int dlog_flag = 0;

/* Read saved calibration parameters. These para is saved in somewhere that can not disappear when power-off
 * ReadPara function should be create by customer.If customer can not implement this function, should comment it out
 * If there is no parameter saved, ReadPara() should return -1, else return 1;
 * If there is no para saved, pass the default value to the algorithm library
 */
int ReadPara(float *p);

/* Save calibration parameters in somewhere that can not lose after power-off.
 * When the system power on next time, need to read this parameters by the function ReadPara() for algorithm initial.
 * If save parameter successfully, return 1, else return -1;
 */
int SavePara(float *p);

/* Delay functon.
 * Parameter nms means delay n ms;
 */
extern void Delay_Ms(int nms);

/**
 * @brief Convert the sensor coordinate to right-front-up coordinate system;
 */
void acc_coordinate_raw_to_real(int layout, float *in, float *out);

/**
 * @brief Convert the sensor coordinate to right-front-up coordinate system;
 */
void mag_coordinate_raw_to_real(int layout, float *in, float *out);

/*******************************************************************************
* Function Name  : main
* Description    : the main function
*******************************************************************************/
void compass_entry(void *param)
{
    int i;
    int outflag = 0;

    /* This variable is used to store the accelerometer and magnetometer raw data.
     * please NOTE that the data is going to store here MUST has been transmitted
     * to match the Right-Handed coordinate sytem already.
     */
    float acc_raw_data[3] = {0.0f}; //accelerometer field vector, unit is g
    float acc_real_data[3] = {0.0f};

    float mag_raw_data[3] = {0.0f}; //magnetic field vector, unit is gauss
    float mag_real_data[3] = {0.0f};

    /* This variable is used to save the calibrated mag sensor XYZ output. */
    float caliMag[3] = {0.0f};

    /* This variable is used to save the calibrated orientation data. */
    float caliOri[3] = {0.0f};

    /* This variable is used to judge whether or not the developer
     * should save the parameters in one place where will be read
     * by the algorithm every time when the device restart.
     * save_flag is a flag to if we need to save algorithm parameter into the system file.
     */
    int save_flag = 0;

    /* This variable is used to save calibrated mag sensor parameters */
    float caliMagPara[4] = {0.0f};

    /* This variable is used to save calibrated mag para that need to be saved in system file */
    float saveData[4] = {0.0f};

    /* Read saved calibration parameters. These para is saved in somewhere that can not disappear when power-off
     * ReadPara function should be create by customer.If customer can not implement this function, should comment it out
     * If there is no parameter saved, ReadPara() should return -1, else return 1;
     * If there is no para saved, pass the default value to the algorithm library
     */
    if (ReadPara(saveData) == 1)
    {
        for (i = 0; i < 4; i++)
        {
            mag_hmm[i] = saveData[i];
        }
    }

    /* Initial the acc, mag, and calibrated parameters
     * if already saved the calibrated offset and radius last time, read it out and init the magOffset and magRadius
     */
    InitialAlgorithm(mag_smm, mag_hmm);

    LOG_I("entry loop\n");

    while (1)
    {
        /* Get the acc raw data, unit is g*/
        acc_raw_data[0] = 0.0;
        acc_raw_data[1] = 0.0;
        acc_raw_data[2] = 0.0;

        /* convert the coordinate system */
        acc_coordinate_raw_to_real(A_LAYOUT, acc_raw_data, acc_real_data);

        /* Get the mag raw data, unit is gauss */
        //mag_raw_data[0] = ;
        //mag_raw_data[1] = ;
        //mag_raw_data[2] = ;
        MMC36X0KJ_GetData(mag_raw_data);

        /* Convert the coordinate system */
        mag_coordinate_raw_to_real(M_LAYOUT, mag_raw_data, mag_real_data);

        /* Below functions are algorithm interface.
         * input acc, mag data into algorithm
         * make sure that acc and mag XYZ data meet the right-hand(right-front-up) coordinate system
         */
        MainAlgorithmProcess(acc_real_data, mag_real_data, 1);

        /* Get calibrated mag data */
        GetCalMag(caliMag);

        /* Calibrated magnetic sensor output, unit is gauss */
        calMagX = caliMag[0];
        calMagY = caliMag[1];
        calMagZ = caliMag[2];

        /* Get orientation vector */
        GetCalOri(acc_real_data, caliMag, caliOri);

        /* Get the fAzimuth Pitch Roll for the eCompass */
        fAzimuth  = caliOri[0];
        fPitch    = caliOri[1];
        fRoll     = caliOri[2];

        outflag++;

        /* Get the accuracy of the algorithm */
        iAccuracy = GetMagAccuracy();
        if (((outflag & 0x3f) == 0) && dlog_flag == 1)
            LOG_I("North angle %f, pitch %f, roll %f, %d\n", fAzimuth, fPitch, fRoll, iAccuracy);

        /* Get the SET Flag from algorithm */
        if (GetMagSaturation())
        {
            LOG_I("Set Mag?\n");
            //MEMSIC_Magnetic_Sensor_SET(); //Do SET action
        }

        /* Get corrected mag data */
        save_flag = GetCalPara(caliMagPara);

        if (save_flag == 1)
        {
            LOG_I("Param can saved\n");
            /* Get the offset and radius that need to be saved in system file */
            for (i = 0; i < 4; i++)
            {
                saveData[i] = caliMagPara[i];
            }
            /* Save the calpara buffer into the system file */
            /* This function should be create by customer.If customer can not implement this function, should comment it out */
            SavePara(saveData);
        }

        /* Sampling interval is 20ms. */
        Delay_Ms(20);
    }
}

static rt_thread_t comp_thread = NULL;

void mmc_sample_start(void)
{
    comp_thread = rt_thread_create("compass", compass_entry, NULL, 2048, RT_MAIN_THREAD_PRIORITY, 10);
    if (comp_thread != NULL)
    {
        rt_thread_startup(comp_thread);
        LOG_I("compass thread started\n");
    }
    else
        LOG_I("Create compas thread fail\n");

}

float mmc_get_north_angle(void)
{
    if (iAccuracy > 0)
        return fAzimuth;
    else
        return 1000.00;
}

enum
{
    MMC_NORTH = 0,
    MMC_NEAST = 1,
    MMC_EAST = 2,
    MMC_SEAST = 3,
    MMC_SOUTH = 4,
    MMC_SWEST = 5,
    MMC_WEST = 6,
    MMC_NWEST = 7,
};

int mmc_get_direction(void)
{
    if (iAccuracy == 0) // not stable
        return 8;

    if ((fAzimuth < 22.5) || (fAzimuth > 337.5))
        return MMC_NORTH;
    else if ((fAzimuth >= 22.5) && (fAzimuth < 67.5))
        return MMC_NEAST;
    else if ((fAzimuth >= 67.5) && (fAzimuth < 92.5))
        return MMC_EAST;
    else if ((fAzimuth >= 92.5) && (fAzimuth < 157.5))
        return MMC_SEAST;
    else if ((fAzimuth >= 157.5) && (fAzimuth < 202.5))
        return MMC_SOUTH;
    else if ((fAzimuth >= 202.5) && (fAzimuth < 247.5))
        return MMC_SWEST;
    else if ((fAzimuth >= 247.5) && (fAzimuth < 292.5))
        return MMC_WEST;
    else if ((fAzimuth >= 292.5) && (fAzimuth < 337.5))
        return MMC_NWEST;

    return MMC_NORTH;
}

void mmc_comp_log(int en)
{
    dlog_flag = en;
}

int mmc_comp_is_accu(void)
{
    return iAccuracy;
}


/* Read saved calibration parameters. These para is saved in somewhere that can not disappear when power-off
 * ReadPara function should be create by customer.If customer can not implement this function, should comment it out
 * If there is no parameter saved, ReadPara() should return -1, else return 1;
 * If there is no para saved, pass the default value to the algorithm library
 */
static float sp[4] = {0.0f, 0.0f, 0.0f, 0.5f};
int ReadPara(float *p)
{
    int i;

    /*
    .
    . Need to be implemented by user.
    .
    */

    for (i = 0; i < 4; i++)
    {
        p[i] = sp[i];
    }
    return 1;
}

/* Save calibration parameters in somewhere that can not lose after power-off.
 * When the system power on next time, need to read this parameters by the function ReadPara() for algorithm initial.
 * If save parameter successfully, return 1, else return -1;
 */
int SavePara(float *p)
{
    int i;

    /*
    .
    . Need to be implemented by user.
    .
    */
    for (i = 0; i < 4; i++)
    {
        sp[i] = p[i] ;
    }

    return 1;
}
#if 0
/* delay function
 */
void Delay_Ms(int nms)
{
    /*  cnt is the time to wait */

    /*
    .
    . Need to be implemented by user.
    .
    */

    return;
}
#endif
/* Convert the sensor coordinate to right-front-up coordinate system;
 */
void acc_coordinate_raw_to_real(int layout, float *in, float *out)
{
    switch (layout)
    {
    case 0:
        out[0] =  in[0];
        out[1] =  in[1];
        out[2] =  in[2];
        break;
    case 1:
        out[0] = -in[1];
        out[1] =  in[0];
        out[2] =  in[2];
        break;
    case 2:
        out[0] = -in[0];
        out[1] = -in[1];
        out[2] =  in[2];
        break;
    case 3:
        out[0] =  in[1];
        out[1] = -in[0];
        out[2] =  in[2];
        break;
    case 4:
        out[0] =  in[1];
        out[1] =  in[0];
        out[2] = -in[2];
        break;
    case 5:
        out[0] = -in[0];
        out[1] =  in[1];
        out[2] = -in[2];
        break;
    case 6:
        out[0] = -in[1];
        out[1] = -in[0];
        out[2] = -in[2];
        break;
    case 7:
        out[0] =  in[0];
        out[1] = -in[1];
        out[2] = -in[2];
        break;
    default:
        out[0] =  in[0];
        out[1] =  in[1];
        out[2] =  in[2];
        break;
    }
}
/* Convert the sensor(MMC3630KJ) coordinate to right-front-up coordinate system;
 */
void mag_coordinate_raw_to_real(int layout, float *in, float *out)
{
    switch (layout)
    {
    case 0:
        out[0] =  in[0];
        out[1] =  in[1];
        out[2] =  in[2];
        break;
    case 1:
        out[0] = -in[1];
        out[1] =  in[0];
        out[2] =  in[2];
        break;
    case 2:
        out[0] = -in[0];
        out[1] = -in[1];
        out[2] =  in[2];
        break;
    case 3:
        out[0] =  in[1];
        out[1] = -in[0];
        out[2] =  in[2];
        break;
    case 4:
        out[0] =  in[1];
        out[1] =  in[0];
        out[2] = -in[2];
        break;
    case 5:
        out[0] = -in[0];
        out[1] =  in[1];
        out[2] = -in[2];
        break;
    case 6:
        out[0] = -in[1];
        out[1] = -in[0];
        out[2] = -in[2];
        break;
    case 7:
        out[0] =  in[0];
        out[1] = -in[1];
        out[2] = -in[2];
        break;
    default:
        out[0] =  in[0];
        out[1] =  in[1];
        out[2] = -in[2];
        break;
    }
}

