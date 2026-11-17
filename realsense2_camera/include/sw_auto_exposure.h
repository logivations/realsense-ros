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
    // (all three equal to the Y offset for YUV formats).
    struct PixelLayout
    {
        int bytes_per_pixel;
        int r, g, b;
    };

    // Mean luma (BT.601) of an image, sampled every `step` pixels in both directions.
    inline double meanLuma(const uint8_t* data, int width, int height, int stride_bytes,
                           const PixelLayout& layout, int step = 8)
    {
        if (!data || width <= 0 || height <= 0 || step <= 0)
            return 0.0;
        double sum = 0.0;
        std::size_t count = 0;
        for (int y = step / 2; y < height; y += step)
        {
            const uint8_t* row = data + static_cast<std::size_t>(y) * stride_bytes;
            for (int x = step / 2; x < width; x += step)
            {
                const uint8_t* px = row + static_cast<std::size_t>(x) * layout.bytes_per_pixel;
                sum += 0.299 * px[layout.r] + 0.587 * px[layout.g] + 0.114 * px[layout.b];
                ++count;
            }
        }
        return count ? sum / count : 0.0;
    }
}  // namespace realsense2_camera
