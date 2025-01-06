#ifndef __SOUNDPLUS_ADAPTER_H__
#define __SOUNDPLUS_ADAPTER_H__

#ifdef __cplusplus
extern "C" {
#endif

/***************************************************************
    FunctionName:   soundplus_auth
    Purpose:        algorithm authorize
    Parameter:
                    key_buf : license buffer
                    key_len : 16 bytes
    Return:
                    return 1  the function call normal
                    return 0  the function aready called
****************************************************************/
int soundplus_auth(char *key_buf, int key_len);

/***************************************************************
    FunctionName:   soundplus_auth_status
    Purpose:        get algorithm authorize status
    Parameter:
    Return:
                    return 1  algorithm authorize success
                    return 0  algorithm authorize fail
****************************************************************/
int soundplus_auth_status(void);

/***************************************************************
    FunctionName:   soundplus_deal_Tx
    Purpose:        algorithm Tx AEC/ENC process
    Parameter:
                    buf : mic pcm data
                    ref : ref pcm data
                    buf_len : the length of mic pcm data
                    buf_len : the length of ref pcm data
    Return:
                    return 0  algorithm process success
****************************************************************/
int soundplus_deal_Tx(short *buf, short *ref, int buf_len, int ref_len);

/***************************************************************
    FunctionName:   soundplus_deal_Rx
    Purpose:        algorithm Rx DRC process
    Parameter:
                    out : out pcm data
                    ref : ref pcm data
                    ref_len : the length of ref pcm data
    Return:
                    return 0  algorithm process success
****************************************************************/
int soundplus_deal_Rx(short *out, short *ref, int ref_len);

/***************************************************************
    FunctionName:   soundplus_init
    Purpose:        algorithm init
    Parameter:
                    type_result : the result of soundplus_deal_Tx
                    for ASR if type_result input 0, the result of
                    soundplus_deal_Tx for WB Call if type_result
                    input 1, the result of soundplus_deal_Tx for
                    NB Call if type_result input 2
    Return:
                    return 0  algorithm init success
                    return -1  algorithm init fail
****************************************************************/
int soundplus_init(int type_result);

/***************************************************************
    FunctionName:   soundplus_deinit
    Purpose:        algorithm deinit
    Parameter:
    Return:
****************************************************************/
void soundplus_deinit(void);

/***************************************************************
    FunctionName:   soundplus_rx_init
    Purpose:        algorithm rx init
    Parameter:
                    type_result : the result of soundplus_deal_Tx
                    for ASR if type_result input 0, the result of
                    soundplus_deal_Tx for WB Call if type_result
                    input 1, the result of soundplus_deal_Tx for
                    NB Call if type_result input 2
    Return:
                    return 0  algorithm rx init success
                    return -1  algorithm rx init fail
****************************************************************/
int soundplus_rx_init(int typeR);

/***************************************************************
    FunctionName:   soundplus_rx_deinit
    Purpose:        algorithm rx deinit
    Parameter:
    Return:
****************************************************************/
void soundplus_rx_deinit(void);

#ifdef __cplusplus
}
#endif


#endif
