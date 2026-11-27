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
#include "yuyv_to_rgb8.h"

using realsense2_camera::ColourCastWatch;
using realsense2_camera::MeanColour;
using realsense2_camera::PixelLayout;
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

namespace
{
// Image means of the front camera on wfal-amr3: the broken colour ISP session of 2026-11-24 and
// warm light at a charging station on wfal-amr4 (2026-11-23 15:18), which is healthy.
const MeanColour kBrokenColour{113.0, 124.0, 93.0};
const MeanColour kWarmLight{82.7, 74.9, 56.6};
const MeanColour kHealthyColour{120.0, 121.0, 122.0};

using Events = std::vector<std::pair<double, ColourCastWatch::Event>>;

// Feeds `seconds` of frames at 5 fps and, like the driver at the end of the holdoff, ends a
// warm-up right after each Rewarm (the holdoff frames are not fed to the watch). Returns the events
// with the second they came at.
Events feed(ColourCastWatch& watch, ColourCastWatch::Clock::time_point& now, double seconds, const MeanColour& colour)
{
    Events events;
    const auto start = now;
    for (int i = 0; i < static_cast<int>(seconds * 5); ++i)
    {
        now += std::chrono::milliseconds(200);
        const auto event = watch.addFrame(now, colour);
        if (event == ColourCastWatch::Event::Rewarm)
            watch.warmupDone(now);
        if (event != ColourCastWatch::Event::None)
            events.push_back({std::chrono::duration<double>(now - start).count(), event});
    }
    return events;
}
}  // namespace

// The front camera streams YUYV and publishes RGB8 converted with yuyvToRgb8: the colour the watch
// sees from the YUYV frame has to be the one in the published images, where the thresholds come from.
TEST(ColourCastWatch, YuyvMeanColourMatchesPublishedRgb)
{
    // pairs of pixels alternating green-yellow (U 110, V 140) and grey (U = V = 128), Y per pixel
    cv::Mat yuyv(8, 16, CV_8UC2);
    for (int y = 0; y < yuyv.rows; ++y)
        for (int x = 0; x < yuyv.cols; ++x)
        {
            const bool cast = (x / 2) % 2 == 0;
            const uint8_t chroma = x % 2 == 0 ? (cast ? 110 : 128) : (cast ? 140 : 128);  // U on even, V on odd pixels
            yuyv.at<cv::Vec2b>(y, x) = {static_cast<uint8_t>(110 + x), chroma};
        }
    cv::Mat rgb;
    realsense2_camera::yuyvToRgb8(yuyv, rgb);
    const MeanColour from_yuyv = realsense2_camera::imageMeans(yuyv.data, yuyv.cols, yuyv.rows, static_cast<int>(yuyv.step),
                                                               realsense2_camera::kYuyvLayout, 1).colour;
    const MeanColour from_rgb = realsense2_camera::imageMeans(rgb.data, rgb.cols, rgb.rows, static_cast<int>(rgb.step),
                                                              PixelLayout{3, 0, 1, 2}, 1).colour;
    EXPECT_NEAR(from_yuyv.r, from_rgb.r, 1.0);
    EXPECT_NEAR(from_yuyv.g, from_rgb.g, 1.0);
    EXPECT_NEAR(from_yuyv.b, from_rgb.b, 1.0);
    EXPECT_LT(from_yuyv.b / from_yuyv.g, 0.9);
}

// wfal-amr3 2026-11-24: the front camera kept the broken colour for 25 h after a start whose
// warm-up looked fine; the lift gate markers then decoded one at a time and the transit failed.
// The watch asks for another warm-up once the colour stayed broken for kHold after the settle time.
TEST(ColourCastWatch, BrokenColourAsksForAnotherWarmup)
{
    ColourCastWatch watch;
    ColourCastWatch::Clock::time_point now{};
    EXPECT_TRUE(feed(watch, now, 60.0, kBrokenColour).empty());  // no warm-up ended yet
    watch.warmupDone(now);
    const auto events = feed(watch, now, 120.0, kBrokenColour);
    ASSERT_FALSE(events.empty());
    EXPECT_EQ(events.front().second, ColourCastWatch::Event::Rewarm);
    const double settle_and_hold = std::chrono::duration<double>(ColourCastWatch::kSettle + ColourCastWatch::kHold).count();
    EXPECT_GE(events.front().first, settle_and_hold);
    EXPECT_LE(events.front().first, settle_and_hold + 2 * std::chrono::duration<double>(ColourCastWatch::kBlock).count());
}

// wfal-amr2 2026-11-27: a milder cast, B/G 0.87-0.92 in front of the lift gate (one image per block
// here, same pose). With 0.90 as the limit the blocks at 0.90-0.92 counted as healthy, reset the
// hold and the camera kept the cast all day.
TEST(ColourCastWatch, MildCastAsksForAnotherWarmup)
{
    const std::vector<std::pair<double, double>> blue_red_green{
        {0.88, 0.98}, {0.88, 0.93}, {0.91, 0.95}, {0.91, 0.94}, {0.88, 0.86}, {0.90, 0.95},
        {0.90, 0.94}, {0.91, 0.94}, {0.88, 0.86}, {0.92, 0.97}, {0.90, 0.93}, {0.90, 0.93}};
    ColourCastWatch watch;
    ColourCastWatch::Clock::time_point now{};
    watch.warmupDone(now);
    Events events;
    for (int round = 0; round < 2; ++round)
        for (const auto& [blue_green, red_green] : blue_red_green)
        {
            const auto block = feed(watch, now, 5.0, MeanColour{125.0 * red_green, 125.0, 125.0 * blue_green});
            events.insert(events.end(), block.begin(), block.end());
        }
    ASSERT_FALSE(events.empty());
    EXPECT_EQ(events.front().second, ColourCastWatch::Event::Rewarm);
}

// Warm light lowers B/G like the broken state but raises R/G; a healthy camera there must not
// lose 8 s of exposure control to a warm-up.
TEST(ColourCastWatch, WarmLightIsNotBrokenColour)
{
    ColourCastWatch watch;
    ColourCastWatch::Clock::time_point now{};
    watch.warmupDone(now);
    EXPECT_TRUE(feed(watch, now, 600.0, kWarmLight).empty());
}

// A camera the warm-up does not repair must not run the firmware AE (exposure without limit,
// motion blur) over and over: at most kMaxRewarms warm-ups a minute apart, then one give-up.
TEST(ColourCastWatch, GivesUpAfterMaxRewarms)
{
    ColourCastWatch watch;
    ColourCastWatch::Clock::time_point now{};
    watch.warmupDone(now);
    const auto events = feed(watch, now, 600.0, kBrokenColour);
    ASSERT_EQ(events.size(), static_cast<std::size_t>(ColourCastWatch::kMaxRewarms + 1));
    for (int i = 0; i < ColourCastWatch::kMaxRewarms; ++i)
        EXPECT_EQ(events[i].second, ColourCastWatch::Event::Rewarm);
    EXPECT_EQ(events.back().second, ColourCastWatch::Event::GiveUp);
    for (std::size_t i = 1; i < events.size(); ++i)
        EXPECT_GE(events[i].first - events[i - 1].first, 59.0);
}

// The warm-up repaired the colour (amr47: one firmware AE run clears it): reported once, only on
// healthy images (warm light tells nothing), and no further warm-ups while the colour stays normal.
TEST(ColourCastWatch, StopsOnceColourIsBack)
{
    ColourCastWatch watch;
    ColourCastWatch::Clock::time_point now{};
    watch.warmupDone(now);
    const auto broken = feed(watch, now, 90.0, kBrokenColour);
    ASSERT_EQ(broken.size(), 1u);
    ASSERT_EQ(broken.front().second, ColourCastWatch::Event::Rewarm);
    EXPECT_TRUE(feed(watch, now, 60.0, kWarmLight).empty());
    const auto healthy = feed(watch, now, 600.0, kHealthyColour);
    ASSERT_EQ(healthy.size(), 1u);
    EXPECT_EQ(healthy.front().second, ColourCastWatch::Event::Cleared);
    EXPECT_EQ(watch.rewarms(), 1);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
