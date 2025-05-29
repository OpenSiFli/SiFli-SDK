#ifndef __WFMLIB_H__
#define __WFMLIB_H__

/*
 * @brief Get the total number of frames required for screen refresh.
 * @param mode  Refresh mode: 1 for partial refresh, 2 for full refresh.
 * @param temp  Current device temperature (unit: Celsius).
 * @return      Total number of frames required for screen refresh, returns -1 on failure.
 */
int get_frame(unsigned char mode, unsigned char temp);

/*
 * @brief Generate waveform data for the specified frame number.
 * @param gray_buffer Input grayscale data buffer, 8-bit data (high 4 bits for old pixel, low 4 bits for new pixel).
 * @param wfm_buffer  Output waveform data buffer, used for subsequent output to the module.
 * @param w           Image width (pixels).
 * @param h           Image height (pixels).
 * @param frame_num   Frame number to process.
 * @return            0: success, -1: failure.
 */
int get_waveform(unsigned char *gray_buffer, int *wfm_buffer, int w, int h, unsigned char frame_num);

#endif /*__WFMLIB_H__*/
