#include "vl53l1_tests.h"

#ifdef VL53L1CB_RANGING_TEST

void vl53l1cb_ranging_test(VL53L1_Dev_t* Dev)
{
    static VL53L1_MultiRangingData_t MultiRangingData;
    VL53L1_MultiRangingData_t *pMultiRangingData = &MultiRangingData;
    uint8_t NewDataReady = 0;
    int no_of_object_found = 0, j;
    printf("Ranging loop starts\n");
    VL53L1_Error status = VL53L1_WaitDeviceBooted(Dev);
    status = VL53L1_DataInit(Dev);
    status = VL53L1_StaticInit(Dev);
/* VL53L1_SetPresetMode function is mandatory to be called even if default PresetMode is the VL53L1_PRESETMODE_RANGING */
    status = VL53L1_SetPresetMode(Dev, VL53L1_PRESETMODE_RANGING);
    // status = VL53L1_SetDistanceMode(Dev, VL53L1_DISTANCEMODE_SHORT);
    // status = VL53L1_SetMeasurementTimingBudgetMicroSeconds(Dev, 60000);
    status = VL53L1_StartMeasurement(Dev);
    if(status){
        printf("VL53L1_StartMeasurement failed: error = %d \n", status);
        while(1);
    }
    do{ // polling mode
        status = VL53L1_GetMeasurementDataReady(Dev, &NewDataReady);
        HAL_Delay(1);
        if((!status)&&(NewDataReady!=0)){
            status = VL53L1_GetMultiRangingData(Dev, pMultiRangingData);
            no_of_object_found=pMultiRangingData->NumberOfObjectsFound;
            printf("Count=%5d, ", pMultiRangingData->StreamCount);
            printf("#Objs=%1d ", no_of_object_found);
            for(j=0;j<no_of_object_found;j++){
                  if(j!=0)printf("\n                     ");
                  printf("status=%d, D=%5dmm, Signal=%2.2f Mcps, Ambient=%2.2f Mcps",
                         pMultiRangingData->RangeData[j].RangeStatus,
                         pMultiRangingData->RangeData[j].RangeMilliMeter,
                         pMultiRangingData->RangeData[j].SignalRateRtnMegaCps/65536.0,
                         pMultiRangingData->RangeData[j].AmbientRateRtnMegaCps/65536.0);
            }
            printf ("\n");
            if (status==0){
              status = VL53L1_ClearInterruptAndStartMeasurement(Dev);
            }
        }
    } while (1);
}

#endif // VL53L1CB_RANGING_TEST

#ifdef VL53L1CB_MULTIZONE_TEST

void vl53l1cb_multizone_test(VL53L1_Dev_t* Dev)
{
    static VL53L1_MultiRangingData_t MultiRangingData;
    VL53L1_MultiRangingData_t *pMultiRangingData = &MultiRangingData;
    uint8_t NewDataReady = 0;
    uint8_t NumROI = 2, RoiNumber=0, RoiStatus=0;
    int no_of_object_found = 0, j;
    VL53L1_RoiConfig_t pRoiConfig;
    printf("Multizones loop starts\n");
    VL53L1_Error status = VL53L1_WaitDeviceBooted(Dev);
    status = VL53L1_DataInit(Dev);
    status = VL53L1_StaticInit(Dev);
    /*VL53L1_SetPresetMode function is mandatory to be called even if default PresetMode is the VL53L1_PRESETMODE_RANGING */
    status = VL53L1_SetPresetMode(Dev, VL53L1_PRESETMODE_MULTIZONES_SCANNING);
    pRoiConfig.NumberOfRoi=NumROI;

    pRoiConfig.UserRois[0].TopLeftX=0;
    pRoiConfig.UserRois[0].TopLeftY=15;
    pRoiConfig.UserRois[0].BotRightX=7;
    pRoiConfig.UserRois[0].BotRightY=0;

    pRoiConfig.UserRois[1].TopLeftX=8;
    pRoiConfig.UserRois[1].TopLeftY=15;
    pRoiConfig.UserRois[1].BotRightX=15;
    pRoiConfig.UserRois[1].BotRightY=0;
  
/* VL53L1_SetROI must be called after VL53L1_SetPresetMode */
  status = VL53L1_SetROI(Dev, &pRoiConfig);
    if (!status){
        status = VL53L1_StartMeasurement(Dev);
    }
    else{
        printf("VL53L1_SetROI failed: error =%d\n", status);
        while(1){};
    };
    if(status){
        printf("VL53L1_StartMeasurement failed: error = %d \n", status);
        while(1);
    }
    do{ // polling mode
        status = VL53L1_GetMeasurementDataReady(Dev, &NewDataReady);
        HAL_Delay(1);
        if((!status)&&(NewDataReady!=0)){
            status = VL53L1_GetMultiRangingData(Dev, pMultiRangingData);
            RoiNumber=pMultiRangingData->RoiNumber;
            RoiStatus=pMultiRangingData->RoiStatus;
            no_of_object_found=pMultiRangingData->NumberOfObjectsFound;
            printf("Count=%5d, ", pMultiRangingData->StreamCount);
            printf("RoiNumber%1d, ", RoiNumber);
            printf("RoiStatus=%1d, ", RoiStatus);
            printf("#Objs=%1d, ", no_of_object_found);
            for(j=0;j<no_of_object_found;j++){
                if(j!=0)printf("\n                                               ");
                printf("status=%d, D=%5dmm, Signal=%2.2f Mcps, Ambient=%2.2f Mcps",
                pMultiRangingData->RangeData[j].RangeStatus,
                pMultiRangingData->RangeData[j].RangeMilliMeter,
                pMultiRangingData->RangeData[j].SignalRateRtnMegaCps/65536.0,
                pMultiRangingData->RangeData[j].AmbientRateRtnMegaCps/65536.0);
            }
            printf ("\n");
            if (status==0){
                status = VL53L1_ClearInterruptAndStartMeasurement(Dev);
            }
        }
    } while (1);
}

#endif // VL53L1CB_MULTIZONE_TEST
