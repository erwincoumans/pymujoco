// Copyright 2020 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "pymujoco_includes.h"

namespace py = pybind11;

inline mjModel* mj_loadXML2(const char* filename)
{
    char error[500] = "";
    mjModel* mnew = 0;
    mjModel* m = mj_loadXML(filename, 0, error, 500);
    return m;
}

mjModel* mj_loadModel2(const char* filename)
{
    mjModel* m = mj_loadModel(filename, 0);
    return m;
}
PYBIND11_MODULE(pymujoco, m) {
#include "pymujoco.inl"
}