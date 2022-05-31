/*
* Copyright 2019-2020, Synopsys, Inc.
* All rights reserved.
*
* This source code is licensed under the BSD-3-Clause license found in
* the LICENSE file in the root directory of this source tree.
*
*/

#include <cstdint>
#include "model_settings.h"

struct TestSample {
  int label;
  float image[50*3];
};

extern const int kNumSamples;
extern const TestSample test_samples[50];