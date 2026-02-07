/*
根据步频控制左右灯交替照明的计算函数。
*/

#include <stdio.h>
#include "cJSON.h"
#include "LeftRightDetector.h"

void compute_freq(cJSON *data) {
    // Check if json_obj is valid
    if (data == NULL) {
        printf("Error: NULL JSON object\n");
        return;
    }

    // Get stride_freq field from cJSON object
    cJSON *stride_freq_item = cJSON_GetObjectItem(data, "stride_freq");
    if (stride_freq_item == NULL) {// Check if field exists and is a number
        printf("Error: 'stride_freq' field not found\n");
        return;
    }
    if (!cJSON_IsNumber(stride_freq_item)) {
        printf("Error: 'stride_freq' is not a number\n");
        return;
    }
    int stride_freq = stride_freq_item->valueint; // Retrieve the value
    printf("Original stride frequency: %.2f Hz\n", stride_freq);

    // Get 3-DoF acceleration
    cJSON *accXYZ = cJSON_GetObjectItem(data, "accXYZ");
    

    
    // ========== Your calculation here ==========
    LeftRightDetector detector;
    StepEvent step = detector.detectStep(2.0f, 3.5f, 10.5f, 0.0f);
}

