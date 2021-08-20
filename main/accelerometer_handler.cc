/* Copyright 2019 The TensorFlow Authors. All Rights Reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
==============================================================================*/

#include "accelerometer_handler.h"
#include "mpu_task.h"

#include "slope_micro_features_data.h"
#include "ring_micro_features_data.h"
#include "string.h"
#include <iostream>

using namespace std;

static constexpr int INPUT_BUFFER_SIZE             = 8; 
static constexpr int OUTPUT_BUFFER_SIZE            = 128; 
static constexpr int CIRCULAR_INDEX                = OUTPUT_BUFFER_SIZE /  INPUT_BUFFER_SIZE; 

int begin_index = 0;
int circular_index_counter = 0;
float * buffer;
float * acc_input;

TfLiteStatus SetupAccelerometer(tflite::ErrorReporter* error_reporter) {
  buffer = (float *) malloc(OUTPUT_BUFFER_SIZE * sizeof(float) * 3);
  acc_input = (float *) malloc(INPUT_BUFFER_SIZE * sizeof(float) * 3);
  return kTfLiteOk;
}

bool ReadAccelerometer(tflite::ErrorReporter* error_reporter, float* input, int length) {
  int new_data =  get_acc_data(acc_input);  // Gets 100 3 axis points > 3 * 100 floats and keeps them in a circular buffer
  if (!new_data){
    return false;
  }

  for (int i = 0; i < (OUTPUT_BUFFER_SIZE - INPUT_BUFFER_SIZE) * 3; i ++){
    buffer[i] = buffer[i + INPUT_BUFFER_SIZE * 3];
  }

  for (int i = 0; i < INPUT_BUFFER_SIZE * 3; i++){
    buffer[i + (OUTPUT_BUFFER_SIZE - INPUT_BUFFER_SIZE) * 3] = acc_input[i];
  }

  for (int i = 0 ; i < OUTPUT_BUFFER_SIZE * 3; i++){
      input[i] = buffer[i];
  }
  return true;
}
