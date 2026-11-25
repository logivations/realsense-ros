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

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <cstdlib>

#include "yuyv_to_rgb8.h"

namespace
{
uint8_t clampByte(int v) { return static_cast<uint8_t>(std::min(255, std::max(0, v))); }

// librealsense's YUY2 -> RGB8 (unpack_yuy2, BT.601 limited range): what the consumers of the
// colour image (segmentation, AprilTags, pallet detection) got so far.
void librealsenseRgb(int y, int u, int v, uint8_t rgb[3])
{
    const int c = y - 16, d = u - 128, e = v - 128;
    rgb[0] = clampByte((298 * c + 409 * e + 128) >> 8);
    rgb[1] = clampByte((298 * c - 100 * d - 208 * e + 128) >> 8);
    rgb[2] = clampByte((298 * c + 516 * d + 128) >> 8);
}
}  // namespace

// Publishing YUYV converted by the driver must give the consumers the colours of the RGB8
// profile; a full-range or BT.709 conversion would shift them by up to ~20 levels.
// Luma from the black level 16 up: below it OpenCV clamps Y - 16 at 0 and librealsense does not,
// which differs by up to 19 levels for strongly coloured pixels darker than black.
TEST(YuyvToRgb8, MatchesLibrealsenseRgb8)
{
    // one row of pixel pairs over a grid of Y, U, V (both pixels of a pair share U and V)
    const int step = 15;
    int pairs = 0;
    for (int y = 16; y < 256; y += step)
        for (int u = 0; u < 256; u += step)
            for (int v = 0; v < 256; v += step)
                ++pairs;
    cv::Mat yuyv(1, pairs * 2, CV_8UC2);
    int i = 0;
    for (int y = 16; y < 256; y += step)
        for (int u = 0; u < 256; u += step)
            for (int v = 0; v < 256; v += step, ++i)
            {
                yuyv.at<cv::Vec2b>(0, 2 * i) = cv::Vec2b(y, u);
                yuyv.at<cv::Vec2b>(0, 2 * i + 1) = cv::Vec2b(271 - y, v);
            }

    cv::Mat rgb;
    realsense2_camera::yuyvToRgb8(yuyv, rgb);
    ASSERT_EQ(rgb.type(), CV_8UC3);
    ASSERT_EQ(rgb.cols, pairs * 2);

    int max_diff = 0;
    for (int px = 0; px < rgb.cols; ++px)
    {
        const cv::Vec2b yu = yuyv.at<cv::Vec2b>(0, px);
        const int pair = px / 2;
        const int u = yuyv.at<cv::Vec2b>(0, 2 * pair)[1];
        const int v = yuyv.at<cv::Vec2b>(0, 2 * pair + 1)[1];
        uint8_t expected[3];
        librealsenseRgb(yu[0], u, v, expected);
        const cv::Vec3b got = rgb.at<cv::Vec3b>(0, px);
        for (int ch = 0; ch < 3; ++ch)
            max_diff = std::max(max_diff, std::abs(int(got[ch]) - int(expected[ch])));
    }
    EXPECT_LE(max_diff, 1);
}
