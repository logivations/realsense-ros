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
#include <cmath>
#include <deque>
#include <vector>

#include "sw_auto_exposure.h"

using realsense2_camera::SwAutoExposure;
using realsense2_camera::SwAutoExposureConfig;

namespace
{
// Colour sensor model: luma grows with (scene brightness * exposure) ^ 0.57 (D455 colour at gain
// 64: luma 124 at exposure 116), saturates at 255, and a new exposure shows up two frames after it
// was set.
struct Camera
{
    double scene = 1.0;  // relative scene brightness
    std::deque<int> pipeline;
    int exposure;

    explicit Camera(int start) : pipeline(2, start), exposure(start) {}
    double frame()
    {
        const int exposed = pipeline.front();
        pipeline.pop_front();
        pipeline.push_back(exposure);
        return std::min(255.0, 124.0 * std::pow(scene * exposed / 116.0, 0.57));
    }
};

std::vector<int> run(SwAutoExposure& ae, Camera& camera, int frames)
{
    std::vector<int> exposures;
    for (int i = 0; i < frames; ++i)
    {
        if (const auto next = ae.addFrame(camera.frame()))
        {
            camera.exposure = *next;
            exposures.push_back(*next);
        }
    }
    return exposures;
}

SwAutoExposureConfig config(int max_exposure)
{
    SwAutoExposureConfig c;
    c.target_luma = 120;
    c.max_exposure = max_exposure;
    return c;
}
}  // namespace

// The point of the software AE: in a dark aisle the firmware AE exposes up to the frame period
// (200 ms at 5 fps); ours must stop at max_exposure and leave the image darker.
TEST(SwAutoExposure, DarkSceneStaysAtMaxExposure)
{
    SwAutoExposure ae;
    ae.configure(config(100));
    Camera camera(ae.reset(1660));  // stored firmware value, far above the limit
    EXPECT_EQ(camera.exposure, 100);
    camera.scene = 0.2;
    const auto exposures = run(ae, camera, 100);
    for (int e : exposures)
        EXPECT_LE(e, 100);
    EXPECT_EQ(camera.exposure, 100);
}

// From the cap at a normal hall (needs ~110) and with the two-frame latency, the exposure settles
// inside the deadband from one side, without hunting back and forth.
TEST(SwAutoExposure, ConvergesWithoutOscillation)
{
    SwAutoExposure ae;
    ae.configure(config(333));
    Camera camera(ae.reset(333));
    const auto exposures = run(ae, camera, 60);
    ASSERT_FALSE(exposures.empty());
    EXPECT_TRUE(std::is_sorted(exposures.rbegin(), exposures.rend())) << "exposure went back up";
    const double luma = camera.frame();
    EXPECT_NEAR(luma, 120.0, 12.0);
    EXPECT_TRUE(run(ae, camera, 30).empty()) << "keeps adjusting a settled image";
}

// Sunlight through a dock door: the scene gets 30x brighter and saturates; the exposure has to
// come down to a usable image within four seconds at 5 fps.
TEST(SwAutoExposure, FollowsSunlight)
{
    SwAutoExposure ae;
    ae.configure(config(100));
    Camera camera(ae.reset(60));
    run(ae, camera, 40);
    camera.scene = 30.0;
    run(ae, camera, 20);
    EXPECT_NEAR(camera.frame(), 120.0, 20.0);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
