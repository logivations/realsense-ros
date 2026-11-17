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
#include <utility>
#include <vector>

#include "sw_auto_exposure.h"

using realsense2_camera::SwAutoExposure;
using realsense2_camera::SwAutoExposureConfig;
using realsense2_camera::SwAutoExposureSetting;

namespace
{
// Colour sensor model: luma grows with (scene brightness * exposure) ^ 0.57 (D455 colour at gain
// 64: luma 124 at exposure 116) and about linearly with the gain (D415 fork: gain 128 ~2x the luma
// of 64), saturates at 255, and a new setting shows up two frames after it was set.
struct Camera
{
    double scene = 1.0;  // relative scene brightness
    std::deque<std::pair<int, int>> pipeline;
    int exposure;
    int gain = 64;

    explicit Camera(int start, int start_gain = 64)
        : pipeline(2, {start, start_gain}), exposure(start), gain(start_gain) {}
    double frame()
    {
        const auto [exposed, exposed_gain] = pipeline.front();
        pipeline.pop_front();
        pipeline.push_back({exposure, gain});
        return std::min(255.0, 124.0 * std::pow(scene * exposed / 116.0, 0.57) * exposed_gain / 64.0);
    }
};

std::vector<SwAutoExposureSetting> run(SwAutoExposure& ae, Camera& camera, int frames)
{
    std::vector<SwAutoExposureSetting> settings;
    for (int i = 0; i < frames; ++i)
    {
        if (const auto next = ae.addFrame(camera.frame()))
        {
            camera.exposure = next->exposure;
            if (next->gain >= 0)
                camera.gain = next->gain;
            settings.push_back(*next);
        }
    }
    return settings;
}

SwAutoExposureConfig config(int max_exposure, int max_gain = 0)
{
    SwAutoExposureConfig c;
    c.target_luma = 120;
    c.max_exposure = max_exposure;
    c.max_gain = max_gain;
    return c;
}

// Scene brightness that gives `luma` at exposure 333 and gain 64.
double sceneFor(double luma) { return std::pow(luma / 124.0, 1.0 / 0.57) * 116.0 / 333.0; }
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
    const auto settings = run(ae, camera, 100);
    for (const auto& s : settings)
        EXPECT_LE(s.exposure, 100);
    EXPECT_EQ(camera.exposure, 100);
}

// From the cap at a normal hall (needs ~110) and with the two-frame latency, the exposure settles
// inside the deadband from one side, without hunting back and forth.
TEST(SwAutoExposure, ConvergesWithoutOscillation)
{
    SwAutoExposure ae;
    ae.configure(config(333));
    Camera camera(ae.reset(333));
    const auto settings = run(ae, camera, 60);
    ASSERT_FALSE(settings.empty());
    std::vector<int> exposures;
    for (const auto& s : settings)
        exposures.push_back(s.exposure);
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

// Without max_gain the user's gain is never changed, even in a dark scene at max_exposure.
TEST(SwAutoExposure, GainStaysWithoutMaxGain)
{
    SwAutoExposure ae;
    ae.configure(config(333));
    Camera camera(ae.reset(333, 64));
    EXPECT_FALSE(ae.gainRamp());
    camera.scene = sceneFor(85.0);
    for (const auto& s : run(ae, camera, 60))
        EXPECT_EQ(s.gain, 64);
    EXPECT_EQ(camera.gain, 64);
    EXPECT_EQ(camera.exposure, 333);
}

// Miele MG08 in the afternoon (17.11.2026): the fork image is at luma ~85 with 33.3 ms and gain 64.
// With max_gain the gain makes up the rest once the exposure is at its limit, never the exposure.
TEST(SwAutoExposure, DarkSceneRaisesGainAtMaxExposure)
{
    SwAutoExposure ae;
    ae.configure(config(333, 128));
    Camera camera(ae.reset(200, 64));
    EXPECT_TRUE(ae.gainRamp());
    camera.scene = sceneFor(85.0);
    for (const auto& s : run(ae, camera, 60))
    {
        EXPECT_LE(s.exposure, 333);
        if (s.gain > 64)
        {
            EXPECT_EQ(s.exposure, 333) << "gain raised before the exposure reached its limit";
        }
    }
    EXPECT_EQ(camera.exposure, 333);
    EXPECT_GT(camera.gain, 64);
    EXPECT_NEAR(camera.frame(), 120.0, 12.0);
    EXPECT_TRUE(run(ae, camera, 30).empty()) << "keeps adjusting a settled image";
}

// A scene too dark even at max_gain stops at max_gain (amr1 15:50: luma ~40 at 33.3 ms / 64).
TEST(SwAutoExposure, GainStopsAtMaxGain)
{
    SwAutoExposure ae;
    ae.configure(config(333, 128));
    Camera camera(ae.reset(333, 64));
    camera.scene = sceneFor(40.0);
    for (const auto& s : run(ae, camera, 80))
        EXPECT_LE(s.gain, 128);
    EXPECT_EQ(camera.exposure, 333);
    EXPECT_EQ(camera.gain, 128);
}

// When the light comes back, the gain goes back to the user's before the exposure gets shorter:
// a raised gain only ever goes with the exposure at its limit.
TEST(SwAutoExposure, BrighterSceneLowersGainFirst)
{
    SwAutoExposure ae;
    ae.configure(config(333, 128));
    Camera camera(ae.reset(333, 64));
    camera.scene = sceneFor(70.0);
    run(ae, camera, 60);
    ASSERT_GT(camera.gain, 64);
    camera.scene = sceneFor(70.0) * 20.0;
    for (const auto& s : run(ae, camera, 60))
    {
        if (s.exposure < 333)
        {
            EXPECT_EQ(s.gain, 64) << "exposure shortened with the gain still raised";
        }
    }
    EXPECT_EQ(camera.gain, 64);
    EXPECT_LT(camera.exposure, 333);
    EXPECT_NEAR(camera.frame(), 120.0, 15.0);
}

// A user gain at or above max_gain disables the ramp (Miele instance config gain 128).
TEST(SwAutoExposure, NoRampWhenUserGainAtMaxGain)
{
    SwAutoExposure ae;
    ae.configure(config(333, 128));
    Camera camera(ae.reset(333, 128), 128);
    EXPECT_FALSE(ae.gainRamp());
    camera.scene = sceneFor(30.0);
    for (const auto& s : run(ae, camera, 40))
        EXPECT_EQ(s.gain, 128);
}

// Lowering max_gain at runtime pulls a raised gain down at once.
TEST(SwAutoExposure, LowerMaxGainClampsGain)
{
    SwAutoExposure ae;
    ae.configure(config(333, 128));
    Camera camera(ae.reset(333, 64));
    camera.scene = sceneFor(60.0);
    run(ae, camera, 60);
    ASSERT_GT(ae.gain(), 96);
    ae.configure(config(333, 96));
    EXPECT_EQ(ae.gain(), 96);
    ae.configure(config(333, 0));
    EXPECT_EQ(ae.gain(), 64);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
