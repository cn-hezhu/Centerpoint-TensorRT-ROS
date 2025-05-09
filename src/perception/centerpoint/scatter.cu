/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

// headers in local files
#include "scatter.h"

__global__ void scatter_kernel(const int feature_num,
                               const int grid_x_size,
                               const int grid_y_size,
                               const int* dev_pillar_coors, 
                               const float* pfe_feature,
                               float* canvas_feature) {
  int ith_pillar = blockIdx.x;
  int ith_feature = threadIdx.x;
  int x_ind = dev_pillar_coors[ith_pillar * 4 + 3];
  int y_ind = dev_pillar_coors[ith_pillar * 4 + 2];
  float feature = pfe_feature[ith_pillar * feature_num + ith_feature];

  canvas_feature[ith_feature * grid_y_size * grid_x_size + y_ind * grid_x_size + x_ind] = feature;
}

ScatterCuda::ScatterCuda(const int feature_num, const int grid_x_size,
                         const int grid_y_size)
    : feature_num_(feature_num),
      grid_x_size_(grid_x_size),
      grid_y_size_(grid_y_size) {}

void ScatterCuda::DoScatterCuda(const int pillar_count, 
                                const int* dev_pillar_coors,
                                const float* pfe_feature, 
                                float* canvas_feature) {
  scatter_kernel<<<pillar_count, feature_num_>>>(feature_num_,
                                                 grid_x_size_, 
                                                 grid_y_size_,
                                                 dev_pillar_coors,
                                                 pfe_feature, 
                                                 canvas_feature);
}
