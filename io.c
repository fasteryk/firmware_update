#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "iflytype.h"
#include "LocalSdk.h"


#define MCU_BOOT_MODE       7
#define MCU_RESET           6


static int hdev = -1;


int init_io(void)
{
    int ret;

    ret = DVRSDK_startup(&hdev);
	if (ret != NETDVR_SUCCESS)
        return -1;

    return 0;
}

void cleanup(void)
{
    DVRSDK_cleanup(hdev);
}

void enter_boot_mode(void)
{
    struct NETDVR_AlarmVal_t output_param;

    output_param.alarmid = MCU_BOOT_MODE;
    output_param.val = 0;
    NETDVR_setAlarmOutVal(hdev, &output_param);
    sleep(1);

    output_param.alarmid = MCU_RESET;
    output_param.val = 0;
    NETDVR_setAlarmOutVal(hdev, &output_param);
    sleep(1);

    output_param.alarmid = MCU_RESET;
    output_param.val = 1 << MCU_RESET;
    NETDVR_setAlarmOutVal(hdev, &output_param);
    sleep(1);
}

void reset_mcu(void)
{
    struct NETDVR_AlarmVal_t output_param;

    output_param.alarmid = MCU_BOOT_MODE;
    output_param.val = 1 << MCU_BOOT_MODE;
    NETDVR_setAlarmOutVal(hdev, &output_param);
    sleep(1);

    output_param.alarmid = MCU_RESET;
    output_param.val = 0;
    NETDVR_setAlarmOutVal(hdev, &output_param);
    sleep(1);

    output_param.alarmid = MCU_RESET;
    output_param.val = 1 << MCU_RESET;
    NETDVR_setAlarmOutVal(hdev, &output_param);
    sleep(1);
}



