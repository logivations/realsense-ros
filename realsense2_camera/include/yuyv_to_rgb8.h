// Copyright 2026 Logivations GmbH. All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace realsense2_camera
{
    // YUYV (YUY2) colour frame -> RGB8 for publishing (parameter convert_yuyv_to_rgb8).
    // librealsense converts every frame of an RGB8 profile on its own thread, also the frames the
    // driver then drops (color_fps > publish_fps); streaming YUYV and converting here touches only
    // the published frames. BT.601 limited range, as librealsense's RGB8: within 1 level for luma
    // from the black level 16 up; below it OpenCV clamps Y - 16 at 0 (librealsense does not).
    inline void yuyvToRgb8(const cv::Mat& yuyv, cv::Mat& rgb)
    {
        cv::cvtColor(yuyv, rgb, cv::COLOR_YUV2RGB_YUYV);
    }
}  // namespace realsense2_camera
