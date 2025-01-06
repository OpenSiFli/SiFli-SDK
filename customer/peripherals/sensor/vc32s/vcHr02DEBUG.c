#if DUG_VCXX_BLOOD_OXYGEN_SUPPORT
    vcxxSendDugBuff[algoCallNum * 18 + 0] = sampleData->ppgValue[2 * algoCallNum] >> 8 & 0xff;
    vcxxSendDugBuff[algoCallNum * 18 + 1] = sampleData->ppgValue[2 * algoCallNum] & 0xff;
    vcxxSendDugBuff[algoCallNum * 18 + 2] = sampleData->ppgValue[2 * algoCallNum + 1] >> 8 & 0xff;
    vcxxSendDugBuff[algoCallNum * 18 + 3] = sampleData->ppgValue[2 * algoCallNum + 1] & 0xff;
    vcxxSendDugBuff[algoCallNum * 18 + 4] = algoInputData.axes.x >> 8 & 0xff;
    vcxxSendDugBuff[algoCallNum * 18 + 5] = algoInputData.axes.x & 0xff;
    vcxxSendDugBuff[algoCallNum * 18 + 6] = algoInputData.axes.y >> 8 & 0xff;
    vcxxSendDugBuff[algoCallNum * 18 + 7] = algoInputData.axes.y & 0xff;
    vcxxSendDugBuff[algoCallNum * 18 + 8] = algoInputData.axes.z >> 8 & 0xff;
    vcxxSendDugBuff[algoCallNum * 18 + 9] = algoInputData.axes.z & 0xff;
    vcxxSendDugBuff[algoCallNum * 18 + 10] = systemParam.bloodOxygenValue;
    vcxxSendDugBuff[algoCallNum * 18 + 11] = sampleData->pdResValue[1];
    vcxxSendDugBuff[algoCallNum * 18 + 12] = sampleData->pdResValue[2];
    vcxxSendDugBuff[algoCallNum * 18 + 13] = sampleData->currentValue[1];
    vcxxSendDugBuff[algoCallNum * 18 + 14] = sampleData->currentValue[2];
    vcxxSendDugBuff[algoCallNum * 18 + 15] = sampleData->psValue;
    vcxxSendDugBuff[algoCallNum * 18 + 16] = vcSportFlag;
    vcxxSendDugBuff[algoCallNum * 18 + 17] = vcxxDugIrgCnt;
    vcxxDugIrgCnt++;
#endif
#if DUG_VCXX_BLOOD_OXYGEN_SUPPORT
if (algoCallNum <= 10)
{
    ute_ble5_0_svc_send_to_phone(&vcxxSendDugBuff[0], algoCallNum * 18);
}
else
{
    ute_ble5_0_svc_send_to_phone(&vcxxSendDugBuff[0], 180);
    ute_ble5_0_svc_send_to_phone(&vcxxSendDugBuff[180], (algoCallNum - 10) * 18);
}
#endif


#if DUG_VCXX_HR_OXYGEN_SUPPORT
    vcxxSendDugBuff[i * 15] = algoInputData.ppgSample >> 8 & 0xff;
    vcxxSendDugBuff[i * 15 + 1] = algoInputData.ppgSample & 0xff;
    vcxxSendDugBuff[i * 15 + 2] = algoInputData.axes.x >> 8 & 0xff;
    vcxxSendDugBuff[i * 15 + 3] = algoInputData.axes.x & 0xff;
    vcxxSendDugBuff[i * 15 + 4] = algoInputData.axes.y >> 8 & 0xff;
    vcxxSendDugBuff[i * 15 + 5] = algoInputData.axes.y & 0xff;
    vcxxSendDugBuff[i * 15 + 6] = algoInputData.axes.z >> 8 & 0xff;
    vcxxSendDugBuff[i * 15 + 7] = algoInputData.axes.z & 0xff;
    vcxxSendDugBuff[i * 15 + 8] = drvHeartVcxxGetValue() & 0xff;
    vcxxSendDugBuff[i * 15 + 9] = sampleData->pdResValue[0];
    vcxxSendDugBuff[i * 15 + 10] = sampleData->currentValue[0];
    vcxxSendDugBuff[i * 15 + 11] = sampleData->envValue[2];
    vcxxSendDugBuff[i * 15 + 12] = sampleData->preValue[0];
    vcxxSendDugBuff[i * 15 + 13] = sampleData->psValue;
    vcxxSendDugBuff[i * 15 + 14] = vcxxDugIrgCnt;
    vcxxDugIrgCnt++;
#endif

#if DUG_VCXX_HR_OXYGEN_SUPPORT
if (i <= 10)
{
    ute_ble5_0_svc_send_to_phone(&vcxxSendDugBuff[0], i * 15);
}
else
{
    ute_ble5_0_svc_send_to_phone(&vcxxSendDugBuff[0], 150);
    ute_ble5_0_svc_send_to_phone(&vcxxSendDugBuff[150], (i - 10) * 15);
}
#endif
