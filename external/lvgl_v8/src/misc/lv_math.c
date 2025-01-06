/**
 * @file lv_math.c
 *
 */

/*********************
 *      INCLUDES
 *********************/
#include "lv_math.h"

/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 *  STATIC PROTOTYPES
 **********************/

/**********************
 *  STATIC VARIABLES
 **********************/
static const int16_t sin0_90_table[] = {
    0,     572,   1144,  1715,  2286,  2856,  3425,  3993,  4560,  5126,  5690,  6252,  6813,  7371,  7927,  8481,
    9032,  9580,  10126, 10668, 11207, 11743, 12275, 12803, 13328, 13848, 14364, 14876, 15383, 15886, 16383, 16876,
    17364, 17846, 18323, 18794, 19260, 19720, 20173, 20621, 21062, 21497, 21925, 22347, 22762, 23170, 23571, 23964,
    24351, 24730, 25101, 25465, 25821, 26169, 26509, 26841, 27165, 27481, 27788, 28087, 28377, 28659, 28932, 29196,
    29451, 29697, 29934, 30162, 30381, 30591, 30791, 30982, 31163, 31335, 31498, 31650, 31794, 31927, 32051, 32165,
    32269, 32364, 32448, 32523, 32587, 32642, 32687, 32722, 32747, 32762, 32767
};

static int16_t sin0_90_table_0p1[] =
{
    0, 57, 114, 172, 229, 286, 343, 400, 458, 515,
    572, 629, 686, 743, 801, 858, 915, 972, 1029, 1086,
    1144, 1201, 1258, 1315, 1372, 1429, 1486, 1544, 1601, 1658,
    1715, 1772, 1829, 1886, 1943, 2000, 2058, 2115, 2172, 2229,
    2286, 2343, 2400, 2457, 2514, 2571, 2628, 2685, 2742, 2799,
    2856, 2913, 2970, 3027, 3084, 3141, 3198, 3255, 3311, 3368,
    3425, 3482, 3539, 3596, 3653, 3709, 3766, 3823, 3880, 3937,
    3993, 4050, 4107, 4164, 4220, 4277, 4334, 4390, 4447, 4504,
    4560, 4617, 4674, 4730, 4787, 4843, 4900, 4957, 5013, 5070,
    5126, 5183, 5239, 5295, 5352, 5408, 5465, 5521, 5577, 5634,
    5690, 5746, 5803, 5859, 5915, 5971, 6028, 6084, 6140, 6196,
    6252, 6309, 6365, 6421, 6477, 6533, 6589, 6645, 6701, 6757,
    6813, 6869, 6925, 6981, 7036, 7092, 7148, 7204, 7260, 7315,
    7371, 7427, 7483, 7538, 7594, 7650, 7705, 7761, 7816, 7872,
    7927, 7983, 8038, 8094, 8149, 8204, 8260, 8315, 8370, 8426,
    8481, 8536, 8591, 8647, 8702, 8757, 8812, 8867, 8922, 8977,
    9032, 9087, 9142, 9197, 9252, 9307, 9361, 9416, 9471, 9526,
    9580, 9635, 9690, 9744, 9799, 9854, 9908, 9963, 10017, 10071,
    10126, 10180, 10235, 10289, 10343, 10397, 10452, 10506, 10560, 10614,
    10668, 10722, 10776, 10830, 10884, 10938, 10992, 11046, 11100, 11154,
    11207, 11261, 11315, 11368, 11422, 11476, 11529, 11583, 11636, 11690,
    11743, 11796, 11850, 11903, 11956, 12010, 12063, 12116, 12169, 12222,
    12275, 12328, 12381, 12434, 12487, 12540, 12593, 12645, 12698, 12751,
    12803, 12856, 12909, 12961, 13014, 13066, 13119, 13171, 13223, 13276,
    13328, 13380, 13432, 13485, 13537, 13589, 13641, 13693, 13745, 13797,
    13848, 13900, 13952, 14004, 14055, 14107, 14159, 14210, 14262, 14313,
    14365, 14416, 14467, 14519, 14570, 14621, 14672, 14723, 14774, 14825,
    14876, 14927, 14978, 15029, 15080, 15131, 15181, 15232, 15283, 15333,
    15384, 15434, 15485, 15535, 15585, 15636, 15686, 15736, 15786, 15836,
    15886, 15936, 15986, 16036, 16086, 16136, 16185, 16235, 16285, 16334,
    16384, 16434, 16483, 16532, 16582, 16631, 16680, 16729, 16779, 16828,
    16877, 16926, 16975, 17024, 17072, 17121, 17170, 17219, 17267, 17316,
    17364, 17413, 17461, 17510, 17558, 17606, 17654, 17703, 17751, 17799,
    17847, 17895, 17943, 17990, 18038, 18086, 18134, 18181, 18229, 18276,
    18324, 18371, 18418, 18466, 18513, 18560, 18607, 18654, 18701, 18748,
    18795, 18842, 18889, 18935, 18982, 19028, 19075, 19121, 19168, 19214,
    19261, 19307, 19353, 19399, 19445, 19491, 19537, 19583, 19629, 19675,
    19720, 19766, 19812, 19857, 19902, 19948, 19993, 20039, 20084, 20129,
    20174, 20219, 20264, 20309, 20354, 20399, 20443, 20488, 20533, 20577,
    20622, 20666, 20710, 20755, 20799, 20843, 20887, 20931, 20975, 21019,
    21063, 21107, 21150, 21194, 21238, 21281, 21325, 21368, 21411, 21455,
    21498, 21541, 21584, 21627, 21670, 21713, 21756, 21798, 21841, 21884,
    21926, 21969, 22011, 22053, 22096, 22138, 22180, 22222, 22264, 22306,
    22348, 22390, 22431, 22473, 22514, 22556, 22597, 22639, 22680, 22721,
    22763, 22804, 22845, 22886, 22927, 22967, 23008, 23049, 23089, 23130,
    23170, 23211, 23251, 23291, 23332, 23372, 23412, 23452, 23492, 23532,
    23571, 23611, 23651, 23690, 23730, 23769, 23808, 23848, 23887, 23926,
    23965, 24004, 24043, 24082, 24120, 24159, 24198, 24236, 24275, 24313,
    24351, 24390, 24428, 24466, 24504, 24542, 24580, 24617, 24655, 24693,
    24730, 24768, 24805, 24843, 24880, 24917, 24954, 24991, 25028, 25065,
    25102, 25138, 25175, 25212, 25248, 25285, 25321, 25357, 25393, 25429,
    25466, 25501, 25537, 25573, 25609, 25645, 25680, 25716, 25751, 25786,
    25822, 25857, 25892, 25927, 25962, 25997, 26031, 26066, 26101, 26135,
    26170, 26204, 26238, 26273, 26307, 26341, 26375, 26409, 26442, 26476,
    26510, 26543, 26577, 26610, 26644, 26677, 26710, 26743, 26776, 26809,
    26842, 26875, 26907, 26940, 26973, 27005, 27037, 27070, 27102, 27134,
    27166, 27198, 27230, 27261, 27293, 27325, 27356, 27388, 27419, 27450,
    27482, 27513, 27544, 27575, 27605, 27636, 27667, 27698, 27728, 27758,
    27789, 27819, 27849, 27879, 27909, 27939, 27969, 27999, 28029, 28058,
    28088, 28117, 28146, 28176, 28205, 28234, 28263, 28292, 28321, 28349,
    28378, 28406, 28435, 28463, 28492, 28520, 28548, 28576, 28604, 28632,
    28660, 28687, 28715, 28742, 28770, 28797, 28824, 28851, 28879, 28906,
    28932, 28959, 28986, 29013, 29039, 29066, 29092, 29118, 29144, 29170,
    29197, 29222, 29248, 29274, 29300, 29325, 29351, 29376, 29401, 29427,
    29452, 29477, 29502, 29526, 29551, 29576, 29600, 29625, 29649, 29674,
    29698, 29722, 29746, 29770, 29794, 29818, 29841, 29865, 29888, 29912,
    29935, 29958, 29981, 30004, 30027, 30050, 30073, 30096, 30118, 30141,
    30163, 30185, 30208, 30230, 30252, 30274, 30296, 30317, 30339, 30360,
    30382, 30403, 30425, 30446, 30467, 30488, 30509, 30530, 30550, 30571,
    30592, 30612, 30632, 30653, 30673, 30693, 30713, 30733, 30753, 30772,
    30792, 30811, 30831, 30850, 30869, 30888, 30908, 30926, 30945, 30964,
    30983, 31001, 31020, 31038, 31056, 31075, 31093, 31111, 31129, 31146,
    31164, 31182, 31199, 31217, 31234, 31251, 31269, 31286, 31303, 31319,
    31336, 31353, 31369, 31386, 31402, 31419, 31435, 31451, 31467, 31483,
    31499, 31514, 31530, 31545, 31561, 31576, 31591, 31607, 31622, 31637,
    31651, 31666, 31681, 31695, 31710, 31724, 31739, 31753, 31767, 31781,
    31795, 31808, 31822, 31836, 31849, 31863, 31876, 31889, 31902, 31915,
    31928, 31941, 31954, 31966, 31979, 31991, 32004, 32016, 32028, 32040,
    32052, 32064, 32076, 32087, 32099, 32110, 32122, 32133, 32144, 32155,
    32166, 32177, 32188, 32198, 32209, 32219, 32230, 32240, 32250, 32260,
    32270, 32280, 32290, 32300, 32309, 32319, 32328, 32337, 32346, 32356,
    32365, 32373, 32382, 32391, 32400, 32408, 32416, 32425, 32433, 32441,
    32449, 32457, 32465, 32473, 32480, 32488, 32495, 32502, 32510, 32517,
    32524, 32531, 32537, 32544, 32551, 32557, 32564, 32570, 32576, 32582,
    32588, 32594, 32600, 32606, 32612, 32617, 32623, 32628, 32633, 32638,
    32643, 32648, 32653, 32658, 32662, 32667, 32671, 32676, 32680, 32684,
    32688, 32692, 32696, 32700, 32703, 32707, 32710, 32714, 32717, 32720,
    32723, 32726, 32729, 32732, 32734, 32737, 32739, 32742, 32744, 32746,
    32748, 32750, 32752, 32754, 32755, 32757, 32758, 32760, 32761, 32762,
    32763, 32764, 32765, 32766, 32766, 32767, 32767, 32767, 32767, 32767,
    32767,
};

/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

/**
 * Return with sinus of an angle
 * @param angle
 * @return sinus of 'angle'. sin(-90) = -32767, sin(90) = 32767
 */
LV_ATTRIBUTE_FAST_MEM int16_t lv_trigo_sin(int16_t angle)
{
    int16_t ret = 0;
    angle       = angle % 360;

    if(angle < 0) angle = 360 + angle;

    if(angle < 90) {
        ret = sin0_90_table[angle];
    }
    else if(angle >= 90 && angle < 180) {
        angle = 180 - angle;
        ret   = sin0_90_table[angle];
    }
    else if(angle >= 180 && angle < 270) {
        angle = angle - 180;
        ret   = -sin0_90_table[angle];
    }
    else {   /*angle >=270*/
        angle = 360 - angle;
        ret   = -sin0_90_table[angle];
    }

    return ret;
}

int16_t _lv_trigo_sin_0p1(int16_t angle)
{
    int16_t ret = 0;
    angle       = angle % 3600;

    if (angle < 0) angle = 3600 + angle;

    if (angle < 900)
    {
        ret = sin0_90_table_0p1[angle];
    }
    else if (angle >= 900 && angle < 1800)
    {
        angle = 1800 - angle;
        ret   = sin0_90_table_0p1[angle];
    }
    else if (angle >= 1800 && angle < 2700)
    {
        angle = angle - 1800;
        ret   = -sin0_90_table_0p1[angle];
    }
    else     /*angle >=270*/
    {
        angle = 3600 - angle;
        ret   = -sin0_90_table_0p1[angle];
    }

    return ret;
}

/**
 * Calculate a value of a Cubic Bezier function.
 * @param t time in range of [0..LV_BEZIER_VAL_MAX]
 * @param u0 start values in range of [0..LV_BEZIER_VAL_MAX]
 * @param u1 control value 1 values in range of [0..LV_BEZIER_VAL_MAX]
 * @param u2 control value 2 in range of [0..LV_BEZIER_VAL_MAX]
 * @param u3 end values in range of [0..LV_BEZIER_VAL_MAX]
 * @return the value calculated from the given parameters in range of [0..LV_BEZIER_VAL_MAX]
 */
uint32_t lv_bezier3(uint32_t t, uint32_t u0, uint32_t u1, uint32_t u2, uint32_t u3)
{
    uint32_t t_rem  = 1024 - t;
    uint32_t t_rem2 = (t_rem * t_rem) >> 10;
    uint32_t t_rem3 = (t_rem2 * t_rem) >> 10;
    uint32_t t2     = (t * t) >> 10;
    uint32_t t3     = (t2 * t) >> 10;

    uint32_t v1 = (t_rem3 * u0) >> 10;
    uint32_t v2 = (3 * t_rem2 * t * u1) >> 20;
    uint32_t v3 = (3 * t_rem * t2 * u2) >> 20;
    uint32_t v4 = (t3 * u3) >> 10;

    return v1 + v2 + v3 + v4;
}

/**
 * Get the square root of a number
 * @param x integer which square root should be calculated
 * @param q store the result here. q->i: integer part, q->f: fractional part in 1/256 unit
 * @param mask optional to skip some iterations if the magnitude of the root is known.
 * Set to 0x8000 by default.
 * If root < 16: mask = 0x80
 * If root < 256: mask = 0x800
 * Else: mask = 0x8000
 */
LV_ATTRIBUTE_FAST_MEM void lv_sqrt(uint32_t x, lv_sqrt_res_t * q, uint32_t mask)
{
    x = x << 8; /*To get 4 bit precision. (sqrt(256) = 16 = 4 bit)*/

    uint32_t root = 0;
    uint32_t trial;
    // http://ww1.microchip.com/...en/AppNotes/91040a.pdf
    do {
        trial = root + mask;
        if(trial * trial <= x) root = trial;
        mask = mask >> 1;
    } while(mask);

    q->i = root >> 4;
    q->f = (root & 0xf) << 4;
}

/**
 * Calculate the atan2 of a vector.
 * @param x
 * @param y
 * @return the angle in degree calculated from the given parameters in range of [0..360]
 */
uint16_t lv_atan2(int x, int y)
{
    // Fast XY vector to integer degree algorithm - Jan 2011 www.RomanBlack.com
    // Converts any XY values including 0 to a degree value that should be
    // within +/- 1 degree of the accurate value without needing
    // large slow trig functions like ArcTan() or ArcCos().
    // NOTE! at least one of the X or Y values must be non-zero!
    // This is the full version, for all 4 quadrants and will generate
    // the angle in integer degrees from 0-360.
    // Any values of X and Y are usable including negative values provided
    // they are between -1456 and 1456 so the 16bit multiply does not overflow.

    unsigned char negflag;
    unsigned char tempdegree;
    unsigned char comp;
    unsigned int degree;     // this will hold the result
    unsigned int ux;
    unsigned int uy;

    // Save the sign flags then remove signs and get XY as unsigned ints
    negflag = 0;
    if(x < 0) {
        negflag += 0x01;    // x flag bit
        x = (0 - x);        // is now +
    }
    ux = x;                // copy to unsigned var before multiply
    if(y < 0) {
        negflag += 0x02;    // y flag bit
        y = (0 - y);        // is now +
    }
    uy = y;                // copy to unsigned var before multiply

    // 1. Calc the scaled "degrees"
    if(ux > uy) {
        degree = (uy * 45) / ux;   // degree result will be 0-45 range
        negflag += 0x10;    // octant flag bit
    }
    else {
        degree = (ux * 45) / uy;   // degree result will be 0-45 range
    }

    // 2. Compensate for the 4 degree error curve
    comp = 0;
    tempdegree = degree;    // use an unsigned char for speed!
    if(tempdegree > 22) {    // if top half of range
        if(tempdegree <= 44) comp++;
        if(tempdegree <= 41) comp++;
        if(tempdegree <= 37) comp++;
        if(tempdegree <= 32) comp++;  // max is 4 degrees compensated
    }
    else {   // else is lower half of range
        if(tempdegree >= 2) comp++;
        if(tempdegree >= 6) comp++;
        if(tempdegree >= 10) comp++;
        if(tempdegree >= 15) comp++;  // max is 4 degrees compensated
    }
    degree += comp;   // degree is now accurate to +/- 1 degree!

    // Invert degree if it was X>Y octant, makes 0-45 into 90-45
    if(negflag & 0x10) degree = (90 - degree);

    // 3. Degree is now 0-90 range for this quadrant,
    // need to invert it for whichever quadrant it was in
    if(negflag & 0x02) { // if -Y
        if(negflag & 0x01)   // if -Y -X
            degree = (180 + degree);
        else        // else is -Y +X
            degree = (180 - degree);
    }
    else {   // else is +Y
        if(negflag & 0x01)   // if +Y -X
            degree = (360 - degree);
    }
    return degree;
}

/**
 * Calculate the integer exponents.
 * @param base
 * @param power
 * @return base raised to the power exponent
 */
int64_t lv_pow(int64_t base, int8_t exp)
{
    int64_t result = 1;
    while(exp) {
        if(exp & 1)
            result *= base;
        exp >>= 1;
        base *= base;
    }

    return result;
}

/**
 * Get the mapped of a number given an input and output range
 * @param x integer which mapped value should be calculated
 * @param min_in min input range
 * @param max_in max input range
 * @param min_out max output range
 * @param max_out max output range
 * @return the mapped number
 */
int32_t lv_map(int32_t x, int32_t min_in, int32_t max_in, int32_t min_out, int32_t max_out)
{
    if(max_in >= min_in && x >= max_in) return max_out;
    if(max_in >= min_in && x <= min_in) return min_out;

    if(max_in <= min_in && x <= max_in) return max_out;
    if(max_in <= min_in && x >= min_in) return min_out;

    /**
     * The equation should be:
     *   ((x - min_in) * delta_out) / delta in) + min_out
     * To avoid rounding error reorder the operations:
     *   (x - min_in) * (delta_out / delta_min) + min_out
     */

    int32_t delta_in = max_in - min_in;
    int32_t delta_out = max_out - min_out;

    return ((x - min_in) * delta_out) / delta_in + min_out;
}

uint32_t lv_rand(uint32_t min, uint32_t max)
{
    static uint32_t a = 0x1234ABCD; /*Seed*/

    /*Algorithm "xor" from p. 4 of Marsaglia, "Xorshift RNGs"*/
    uint32_t x = a;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    a = x;

    return (a % (max - min + 1)) + min;
}

/**********************
 *   STATIC FUNCTIONS
 **********************/
