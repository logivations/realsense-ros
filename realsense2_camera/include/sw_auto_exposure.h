// Copyright 2026 Logivations GmbH. All Rights Reserved.
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

#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace realsense2_camera
{
    // Software auto exposure for the colour sensor.
    //
    // The firmware auto exposure of the D4xx colour sensor has no exposure limit (only the depth
    // module has auto_exposure_limit): it exposes up to the frame period, 33 ms at 30 fps and
    // 200 ms at 5 fps, which smears every image taken while driving. This controller holds the mean
    // luma of the image at a target with the exposure clamped to [min_exposure, max_exposure].
    // The gain stays at the value the user set (<module>.gain) unless max_gain is above it: then a
    // scene too dark at max_exposure raises the gain up to max_gain (the firmware AE did the same
    // once it reached the frame period), and a brighter scene brings the gain back to the user's
    // value before it shortens the exposure.
    struct SwAutoExposureConfig
    {
        int target_luma = 120;       // mean luma 0..255 to hold (D455 firmware AE settles near 120)
        int min_exposure = 1;        // sensor units, 100 us on the D4xx colour sensor
        int max_exposure = 100;
        int max_gain = 0;            // gain ramp above the user's gain once at max_exposure; 0 = off
        int frames_per_update = 2;   // luma is averaged over this many frames per correction
        double deadband = 0.1;       // relative luma error that is left alone

        bool valid() const
        {
            return target_luma > 0 && target_luma < 255 && min_exposure >= 1 &&
                   max_exposure >= min_exposure && max_gain >= 0 && frames_per_update >= 1 && deadband >= 0.0;
        }
    };

    // Exposure and gain the controller wants the sensor to have. gain is -1 while the controller
    // does not manage the gain (no gain ramp: the user's gain stays).
    struct SwAutoExposureSetting
    {
        int exposure = 0;
        int gain = -1;

        bool operator==(const SwAutoExposureSetting& other) const
        {
            return exposure == other.exposure && gain == other.gain;
        }
        bool operator!=(const SwAutoExposureSetting& other) const { return !(*this == other); }
    };

    class SwAutoExposure
    {
    public:
        // Frames still exposed with the previous value after a change (UVC control latency).
        static constexpr int kSettleFrames = 2;
        // Correction per step = (target / luma) ^ kStepExponent. Luma grows about with
        // exposure ^ 0.57 (gamma, measured on a D455 colour sensor), so 1.0 under-corrects a bit
        // and the loop converges from one side without overshoot. The gain acts on the luma about
        // linearly (D415 fork camera: gain 128 gives ~2x the luma of gain 64, amr47 2026-11-11), so
        // the same factor on the gain corrects about exactly.
        static constexpr double kStepExponent = 1.0;
        static constexpr double kMaxStepFactor = 4.0;
        // A saturated image (sunlight) hides how far off it is: step down by the maximum.
        static constexpr double kSaturatedLuma = 250.0;

        void configure(const SwAutoExposureConfig& config)
        {
            _config = config;
            _exposure = clampExposure(_exposure);
            if (_base_gain >= 0)
                _gain = clampGain(_gain);
            restartAveraging();
        }

        const SwAutoExposureConfig& config() const { return _config; }
        int exposure() const { return _exposure; }
        // The gain to apply, -1 while the gain is not managed (no base gain known).
        int gain() const { return _base_gain >= 0 ? _gain : -1; }
        SwAutoExposureSetting setting() const { return {exposure(), gain()}; }
        // The user's gain the ramp starts from and returns to, -1 when unknown.
        int baseGain() const { return _base_gain; }
        // Whether the gain may rise above the user's gain.
        bool gainRamp() const { return _base_gain >= 0 && _config.max_gain > _base_gain; }

        // Starts from the exposure the sensor has now and the user's gain (`base_gain`, -1 when
        // unknown: the gain is then never touched); returns the clamped exposure to apply.
        int reset(int current_exposure, int base_gain = -1)
        {
            _exposure = clampExposure(current_exposure);
            _base_gain = base_gain;
            _gain = base_gain;
            restartAveraging();
            _settle = kSettleFrames;
            return _exposure;
        }

        // Feeds the mean luma of one frame; returns exposure and gain to apply when they change.
        std::optional<SwAutoExposureSetting> addFrame(double mean_luma)
        {
            if (_settle > 0)
            {
                --_settle;
                return std::nullopt;
            }
            _luma_sum += mean_luma;
            if (++_luma_count < _config.frames_per_update)
                return std::nullopt;
            const double luma = std::max(_luma_sum / _luma_count, 1.0);
            restartAveraging();

            const double ratio = _config.target_luma / luma;
            if (std::abs(std::log(ratio)) <= std::log1p(_config.deadband))
                return std::nullopt;
            const double factor = luma >= kSaturatedLuma
                ? 1.0 / kMaxStepFactor
                : std::clamp(std::pow(ratio, kStepExponent), 1.0 / kMaxStepFactor, kMaxStepFactor);

            int next_exposure = _exposure;
            int next_gain = _gain;
            if (gainRamp() && factor > 1.0 && _exposure * factor > _config.max_exposure)
            {
                // too dark: the exposure up to its limit, the rest of the correction on the gain
                next_exposure = _config.max_exposure;
                const double rest = factor * _exposure / _config.max_exposure;
                next_gain = clampGain(static_cast<int>(std::lround(_gain * rest)));
            }
            else if (gainRamp() && factor < 1.0 && _gain > _base_gain)
            {
                // too bright: the gain back down to the user's first, then the exposure
                const double wanted_gain = _gain * factor;
                if (wanted_gain >= _base_gain)
                    next_gain = clampGain(static_cast<int>(std::lround(wanted_gain)));
                else
                {
                    next_gain = _base_gain;
                    next_exposure = clampExposure(
                        static_cast<int>(std::lround(_exposure * wanted_gain / _base_gain)));
                }
            }
            else
                next_exposure = clampExposure(static_cast<int>(std::lround(_exposure * factor)));

            if (next_exposure == _exposure && next_gain == _gain)
            {
                // small values: a factor near 1 rounds back to the same unit; one unit of the
                // control that is moving (the gain while above the user's or at max_exposure)
                const bool on_gain = gainRamp() &&
                    (_gain > _base_gain || (ratio > 1.0 && _exposure >= _config.max_exposure));
                if (on_gain)
                    next_gain = clampGain(_gain + (ratio > 1.0 ? 1 : -1));
                else
                    next_exposure = clampExposure(_exposure + (ratio > 1.0 ? 1 : -1));
            }
            if (next_exposure == _exposure && next_gain == _gain)  // at the limit
                return std::nullopt;
            _exposure = next_exposure;
            _gain = next_gain;
            _settle = kSettleFrames;
            return setting();
        }

    private:
        int clampExposure(int exposure) const
        {
            return std::clamp(exposure, _config.min_exposure, _config.max_exposure);
        }
        // [user's gain, max_gain]; just the user's gain without a gain ramp
        int clampGain(int gain) const
        {
            return std::clamp(gain, _base_gain, std::max(_base_gain, _config.max_gain));
        }
        void restartAveraging()
        {
            _luma_sum = 0.0;
            _luma_count = 0;
        }

        SwAutoExposureConfig _config;
        int _exposure = 1;
        int _base_gain = -1;
        int _gain = -1;
        int _settle = 0;
        double _luma_sum = 0.0;
        int _luma_count = 0;
    };

    // Byte layout of an 8-bit interleaved colour image: bytes per pixel and the offsets of R, G, B
    // (all three equal to the Y offset for YUV formats). For the 4:2:2 YUV formats u and v are the
    // offsets of U and V in the 4-byte pair of pixels; -1 for RGB formats.
    struct PixelLayout
    {
        int bytes_per_pixel;
        int r, g, b;
        int u = -1, v = -1;
    };
    constexpr PixelLayout kYuyvLayout{2, 0, 0, 0, 1, 3};
    constexpr PixelLayout kUyvyLayout{2, 1, 1, 1, 0, 2};

    struct MeanColour
    {
        double r = 0.0, g = 0.0, b = 0.0;
    };

    struct ImageMeans
    {
        double luma = 0.0;  // BT.601 luma; the Y mean for YUV formats
        MeanColour colour;  // YUV formats: converted like the published frames (BT.601 limited range, yuyvToRgb8)
    };

    // Mean luma and colour of an image, sampled every `step` pixels in both directions.
    inline ImageMeans imageMeans(const uint8_t* data, int width, int height, int stride_bytes,
                                 const PixelLayout& layout, int step = 8)
    {
        if (!data || width <= 0 || height <= 0 || step <= 0)
            return {};
        const bool yuv = layout.u >= 0 && layout.v >= 0;
        // c1, c2: G and B in the pixel, or U and V in its pair of pixels
        const int c1 = yuv ? layout.u : layout.g, c2 = yuv ? layout.v : layout.b, pair_mask = yuv ? ~1 : ~0;
        double s0 = 0.0, s1 = 0.0, s2 = 0.0;
        std::size_t count = 0;
        for (int y = step / 2; y < height; y += step)
        {
            const uint8_t* row = data + static_cast<std::size_t>(y) * stride_bytes;
            for (int x = step / 2; x < width; x += step)
            {
                s0 += row[static_cast<std::size_t>(x) * layout.bytes_per_pixel + layout.r];
                const uint8_t* group = row + static_cast<std::size_t>(x & pair_mask) * layout.bytes_per_pixel;
                s1 += group[c1];
                s2 += group[c2];
                ++count;
            }
        }
        if (!count)
            return {};
        s0 /= count;
        s1 /= count;
        s2 /= count;
        if (!yuv)
            return {0.299 * s0 + 0.587 * s1 + 0.114 * s2, {s0, s1, s2}};
        const double y = 1.164 * (s0 - 16.0), u = s1 - 128.0, v = s2 - 128.0;
        return {s0, {y + 1.596 * v, y - 0.392 * u - 0.813 * v, y + 2.017 * u}};
    }

    // Watches the images for the broken colour ISP state the D455 sometimes keeps for a whole
    // session despite the firmware AE warm-up at the start: blue at half to four fifths of green,
    // a green-yellow image. One more firmware AE warm-up clears it on a running stream.
    //
    // Each block of frames is healthy (B/G at least kMaxBlueGreen), broken (B/G below it and R/G
    // below kMaxRedGreen) or tells nothing (warm light lowers B/G too but raises R/G; too dark or
    // saturated). Broken blocks for kHold, after kSettle since the last warm-up, ask for another
    // warm-up, at most kMaxRewarms times per session. Thresholds from 3178 front images of four
    // Westfalia AMRs (2026-08..11): broken sessions B/G 0.57-0.89 (wfal-amr3) and 0.58-0.92
    // (wfal-amr2, milder) at R/G 0.62-0.98, healthy B/G >= 0.97 (1st percentile; 2 dim frames at a
    // charger below 0.93), warm light B/G 0.76 at R/G 1.10.
    class ColourCastWatch
    {
    public:
        using Clock = std::chrono::steady_clock;
        static constexpr double kMaxBlueGreen = 0.93;
        static constexpr double kMaxRedGreen = 1.05;
        static constexpr double kMinGreen = 40.0;   // darker: noise
        static constexpr double kMaxGreen = 230.0;  // brighter: saturated
        static constexpr std::chrono::seconds kBlock{5};
        static constexpr std::chrono::seconds kHold{30};
        // the ISP and the software AE settle after a warm-up; with kHold a minute between warm-ups
        static constexpr std::chrono::seconds kSettle{30};
        static constexpr int kMaxRewarms = 3;

        enum class Event
        {
            None,
            Rewarm,   // run the firmware AE warm-up again
            Cleared,  // healthy again after a warm-up asked for here
            GiveUp,   // still broken after kMaxRewarms warm-ups (reported once)
        };

        // A warm-up ended, or the watch (re)starts: the colour counts again after kSettle.
        void warmupDone(Clock::time_point now)
        {
            _settle_until = now + kSettle;
            _sum = {};
            _count = 0;
            _cast_since.reset();
        }

        Event addFrame(Clock::time_point now, const MeanColour& colour)
        {
            if (now < _settle_until)
                return Event::None;
            if (_count == 0)
                _block_start = now;
            _sum.r += colour.r;
            _sum.g += colour.g;
            _sum.b += colour.b;
            ++_count;
            if (now - _block_start < kBlock)
                return Event::None;

            const double g = _sum.g / _count;
            const MeanColour sum = _sum;
            _sum = {};
            _count = 0;
            if (g < kMinGreen || g > kMaxGreen)
                return Event::None;
            _blue_green = sum.b / sum.g;
            _red_green = sum.r / sum.g;
            if (_blue_green >= kMaxBlueGreen)  // healthy
            {
                _cast_since.reset();
                if (!_awaiting_clear)
                    return Event::None;
                _awaiting_clear = false;
                return Event::Cleared;
            }
            if (_red_green >= kMaxRedGreen)  // warm light: neither breaks nor extends the run
                return Event::None;
            if (!_cast_since)
                _cast_since = _block_start;
            if (now - *_cast_since < kHold)
                return Event::None;
            _cast_since.reset();
            if (_rewarms < kMaxRewarms)
            {
                ++_rewarms;
                _awaiting_clear = true;
                return Event::Rewarm;
            }
            if (_gave_up)
                return Event::None;
            _gave_up = true;
            return Event::GiveUp;
        }

        int rewarms() const { return _rewarms; }
        // B/G and R/G of the last block that was neither too dark nor saturated
        double blueGreen() const { return _blue_green; }
        double redGreen() const { return _red_green; }

    private:
        int _rewarms = 0;
        bool _gave_up = false;
        bool _awaiting_clear = false;
        Clock::time_point _settle_until = Clock::time_point::max();  // nothing counts before the first warm-up ended
        Clock::time_point _block_start;
        std::optional<Clock::time_point> _cast_since;
        MeanColour _sum;
        int _count = 0;
        double _blue_green = 0.0;
        double _red_green = 0.0;
    };
}  // namespace realsense2_camera
