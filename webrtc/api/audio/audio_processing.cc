/*
 *  Copyright (c) 2016 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "api/audio/audio_processing.h"
#include <string>

#include "rtc_base/checks.h"
#include "rtc_base/strings/string_builder.h"

namespace webrtc {
namespace {

using Agc1Config = AudioProcessing::Config::GainController1;
using Agc2Config = AudioProcessing::Config::GainController2;

std::string NoiseSuppressionLevelToString(
    const AudioProcessing::Config::NoiseSuppression::Level& level) {
  switch (level) {
    case AudioProcessing::Config::NoiseSuppression::Level::kLow:
      return "Low";
    case AudioProcessing::Config::NoiseSuppression::Level::kModerate:
      return "Moderate";
    case AudioProcessing::Config::NoiseSuppression::Level::kHigh:
      return "High";
    case AudioProcessing::Config::NoiseSuppression::Level::kVeryHigh:
      return "VeryHigh";
  }
  RTC_CHECK_NOTREACHED();
}

std::string GainController1ModeToString(const Agc1Config::Mode& mode) {
  switch (mode) {
    case Agc1Config::Mode::kAdaptiveAnalog:
      return "AdaptiveAnalog";
    case Agc1Config::Mode::kAdaptiveDigital:
      return "AdaptiveDigital";
    case Agc1Config::Mode::kFixedDigital:
      return "FixedDigital";
  }
  RTC_CHECK_NOTREACHED();
}

}  // namespace

constexpr int AudioProcessing::kNativeSampleRatesHz[];

void CustomProcessing::SetRuntimeSetting(
    AudioProcessing::RuntimeSetting setting) {}

bool Agc1Config::operator==(const Agc1Config& rhs) const {
  const auto& analog_lhs = analog_gain_controller;
  const auto& analog_rhs = rhs.analog_gain_controller;
  return enabled == rhs.enabled && mode == rhs.mode &&
         target_level_dbfs == rhs.target_level_dbfs &&
         compression_gain_db == rhs.compression_gain_db &&
         enable_limiter == rhs.enable_limiter &&
         analog_lhs.enabled == analog_rhs.enabled &&
         analog_lhs.startup_min_volume == analog_rhs.startup_min_volume &&
         analog_lhs.clipped_level_min == analog_rhs.clipped_level_min &&
         analog_lhs.enable_digital_adaptive ==
             analog_rhs.enable_digital_adaptive &&
         analog_lhs.clipped_level_step == analog_rhs.clipped_level_step &&
         analog_lhs.clipped_ratio_threshold ==
             analog_rhs.clipped_ratio_threshold &&
         analog_lhs.clipped_wait_frames == analog_rhs.clipped_wait_frames &&
         analog_lhs.clipping_predictor.mode ==
             analog_rhs.clipping_predictor.mode &&
         analog_lhs.clipping_predictor.window_length ==
             analog_rhs.clipping_predictor.window_length &&
         analog_lhs.clipping_predictor.reference_window_length ==
             analog_rhs.clipping_predictor.reference_window_length &&
         analog_lhs.clipping_predictor.reference_window_delay ==
             analog_rhs.clipping_predictor.reference_window_delay &&
         analog_lhs.clipping_predictor.clipping_threshold ==
             analog_rhs.clipping_predictor.clipping_threshold &&
         analog_lhs.clipping_predictor.crest_factor_margin ==
             analog_rhs.clipping_predictor.crest_factor_margin &&
         analog_lhs.clipping_predictor.use_predicted_step ==
             analog_rhs.clipping_predictor.use_predicted_step;
}

bool Agc2Config::AdaptiveDigital::operator==(
    const Agc2Config::AdaptiveDigital& rhs) const {
  return enabled == rhs.enabled && headroom_db == rhs.headroom_db &&
         max_gain_db == rhs.max_gain_db &&
         initial_gain_db == rhs.initial_gain_db &&
         max_gain_change_db_per_second == rhs.max_gain_change_db_per_second &&
         max_output_noise_level_dbfs == rhs.max_output_noise_level_dbfs;
}

bool Agc2Config::InputVolumeController::operator==(
    const Agc2Config::InputVolumeController& rhs) const {
  return enabled == rhs.enabled;
}

bool Agc2Config::operator==(const Agc2Config& rhs) const {
  return enabled == rhs.enabled &&
         fixed_digital.gain_db == rhs.fixed_digital.gain_db &&
         adaptive_digital == rhs.adaptive_digital &&
         input_volume_controller == rhs.input_volume_controller;
}

bool AudioProcessing::Config::CaptureLevelAdjustment::operator==(
    const AudioProcessing::Config::CaptureLevelAdjustment& rhs) const {
  return enabled == rhs.enabled && pre_gain_factor == rhs.pre_gain_factor &&
         post_gain_factor == rhs.post_gain_factor &&
         analog_mic_gain_emulation == rhs.analog_mic_gain_emulation;
}

bool AudioProcessing::Config::CaptureLevelAdjustment::AnalogMicGainEmulation::
operator==(const AudioProcessing::Config::CaptureLevelAdjustment::
               AnalogMicGainEmulation& rhs) const {
  return enabled == rhs.enabled && initial_level == rhs.initial_level;
}

std::string AudioProcessing::Config::ToString() const {
  char buf[2048];
  rtc::SimpleStringBuilder builder(buf);
  builder << "AudioProcessing::Config{ "
             "pipeline: { "
             "maximum_internal_processing_rate: "
          << pipeline.maximum_internal_processing_rate
          << ", multi_channel_render: " << pipeline.multi_channel_render
          << ", multi_channel_capture: " << pipeline.multi_channel_capture
          << " }, pre_amplifier: { enabled: " << pre_amplifier.enabled
          << ", fixed_gain_factor: " << pre_amplifier.fixed_gain_factor
          << " },capture_level_adjustment: { enabled: "
          << capture_level_adjustment.enabled
          << ", pre_gain_factor: " << capture_level_adjustment.pre_gain_factor
          << ", post_gain_factor: " << capture_level_adjustment.post_gain_factor
          << ", analog_mic_gain_emulation: { enabled: "
          << capture_level_adjustment.analog_mic_gain_emulation.enabled
          << ", initial_level: "
          << capture_level_adjustment.analog_mic_gain_emulation.initial_level
          << " }}, high_pass_filter: { enabled: " << high_pass_filter.enabled
          << " }, echo_canceller: { enabled: " << echo_canceller.enabled
          << ", mobile_mode: " << echo_canceller.mobile_mode
          << ", enforce_high_pass_filtering: "
          << echo_canceller.enforce_high_pass_filtering
          << " }, noise_suppression: { enabled: " << noise_suppression.enabled
          << ", level: "
          << NoiseSuppressionLevelToString(noise_suppression.level)
          << " }, transient_suppression: { enabled: "
          << transient_suppression.enabled
          << " }, gain_controller1: { enabled: " << gain_controller1.enabled
          << ", mode: " << GainController1ModeToString(gain_controller1.mode)
          << ", target_level_dbfs: " << gain_controller1.target_level_dbfs
          << ", compression_gain_db: " << gain_controller1.compression_gain_db
          << ", enable_limiter: " << gain_controller1.enable_limiter
          << ", analog_gain_controller { enabled: "
          << gain_controller1.analog_gain_controller.enabled
          << ", startup_min_volume: "
          << gain_controller1.analog_gain_controller.startup_min_volume
          << ", clipped_level_min: "
          << gain_controller1.analog_gain_controller.clipped_level_min
          << ", enable_digital_adaptive: "
          << gain_controller1.analog_gain_controller.enable_digital_adaptive
          << ", clipped_level_step: "
          << gain_controller1.analog_gain_controller.clipped_level_step
          << ", clipped_ratio_threshold: "
          << gain_controller1.analog_gain_controller.clipped_ratio_threshold
          << ", clipped_wait_frames: "
          << gain_controller1.analog_gain_controller.clipped_wait_frames
          << ", clipping_predictor:  { enabled: "
          << gain_controller1.analog_gain_controller.clipping_predictor.enabled
          << ", mode: "
          << gain_controller1.analog_gain_controller.clipping_predictor.mode
          << ", window_length: "
          << gain_controller1.analog_gain_controller.clipping_predictor
                 .window_length
          << ", reference_window_length: "
          << gain_controller1.analog_gain_controller.clipping_predictor
                 .reference_window_length
          << ", reference_window_delay: "
          << gain_controller1.analog_gain_controller.clipping_predictor
                 .reference_window_delay
          << ", clipping_threshold: "
          << gain_controller1.analog_gain_controller.clipping_predictor
                 .clipping_threshold
          << ", crest_factor_margin: "
          << gain_controller1.analog_gain_controller.clipping_predictor
                 .crest_factor_margin
          << ", use_predicted_step: "
          << gain_controller1.analog_gain_controller.clipping_predictor
                 .use_predicted_step
          << " }}}, gain_controller2: { enabled: " << gain_controller2.enabled
          << ", fixed_digital: { gain_db: "
          << gain_controller2.fixed_digital.gain_db
          << " }, adaptive_digital: { enabled: "
          << gain_controller2.adaptive_digital.enabled
          << ", headroom_db: " << gain_controller2.adaptive_digital.headroom_db
          << ", max_gain_db: " << gain_controller2.adaptive_digital.max_gain_db
          << ", initial_gain_db: "
          << gain_controller2.adaptive_digital.initial_gain_db
          << ", max_gain_change_db_per_second: "
          << gain_controller2.adaptive_digital.max_gain_change_db_per_second
          << ", max_output_noise_level_dbfs: "
          << gain_controller2.adaptive_digital.max_output_noise_level_dbfs
          << " }, input_volume_control : { enabled "
          << gain_controller2.input_volume_controller.enabled << "}}";
  return builder.str();
}

}  // namespace webrtc

// ========================================================================= //
//                      C-API IMPLEMENTATION                                 //
// ========================================================================= //

// Creation and destruction
webrtc_apm* webrtc_apm_create() {
    const auto apm = new webrtc_apm();
    apm->apm = webrtc::AudioProcessingBuilder().Create();
    return apm;
}

void webrtc_apm_destroy(const webrtc_apm* apm) {
    delete apm;
}

// Configuration management
webrtc_apm_config* webrtc_apm_config_create() {
    return new webrtc_apm_config();
}

void webrtc_apm_config_destroy(const webrtc_apm_config* config) {
        delete config;
}

// Config setters
void webrtc_apm_config_set_echo_canceller(webrtc_apm_config* config, int enabled, int mobile_mode) {
    if (config) {
        config->config.echo_canceller.enabled = enabled != 0;
        config->config.echo_canceller.mobile_mode = mobile_mode != 0;
    }
}

void webrtc_apm_config_set_noise_suppression(webrtc_apm_config* config, int enabled, webrtc_apm_ns_level level) {
    if (config) {
        config->config.noise_suppression.enabled = enabled != 0;
        switch (level) {
            case WEBRTC_APM_NS_LOW: config->config.noise_suppression.level = webrtc::AudioProcessing::Config::NoiseSuppression::kLow; break;
            case WEBRTC_APM_NS_MODERATE: config->config.noise_suppression.level = webrtc::AudioProcessing::Config::NoiseSuppression::kModerate; break;
            case WEBRTC_APM_NS_HIGH: config->config.noise_suppression.level = webrtc::AudioProcessing::Config::NoiseSuppression::kHigh; break;
            case WEBRTC_APM_NS_VERY_HIGH: config->config.noise_suppression.level = webrtc::AudioProcessing::Config::NoiseSuppression::kVeryHigh; break;
        }
    }
}

void webrtc_apm_config_set_gain_controller1(webrtc_apm_config* config, int enabled, webrtc_apm_gc_mode mode,
                                                   int target_level_dbfs, int compression_gain_db, int enable_limiter) {
    if (config) {
        config->config.gain_controller1.enabled = enabled != 0;
        switch (mode) {
            case WEBRTC_APM_GC_ADAPTIVE_ANALOG: config->config.gain_controller1.mode = webrtc::AudioProcessing::Config::GainController1::kAdaptiveAnalog; break;
            case WEBRTC_APM_GC_ADAPTIVE_DIGITAL: config->config.gain_controller1.mode = webrtc::AudioProcessing::Config::GainController1::kAdaptiveDigital; break;
            case WEBRTC_APM_GC_FIXED_DIGITAL: config->config.gain_controller1.mode = webrtc::AudioProcessing::Config::GainController1::kFixedDigital; break;
        }
        config->config.gain_controller1.target_level_dbfs = target_level_dbfs;
        config->config.gain_controller1.compression_gain_db = compression_gain_db;
        config->config.gain_controller1.enable_limiter = enable_limiter != 0;
    }
}

void webrtc_apm_config_set_gain_controller2(webrtc_apm_config* config, int enabled) {
    if (config) {
        config->config.gain_controller2.enabled = enabled != 0;
    }
}

void webrtc_apm_config_set_high_pass_filter(webrtc_apm_config* config, int enabled) {
    if (config) {
        config->config.high_pass_filter.enabled = enabled != 0;
    }
}

void webrtc_apm_config_set_pre_amplifier(webrtc_apm_config* config, int enabled, float fixed_gain_factor) {
    if (config) {
        config->config.pre_amplifier.enabled = enabled != 0;
        config->config.pre_amplifier.fixed_gain_factor = fixed_gain_factor;
    }
}

void webrtc_apm_config_set_pipeline(webrtc_apm_config* config, int max_internal_rate,
                                           int multi_channel_render, int multi_channel_capture,
                                           webrtc_apm_downmix_method downmix_method) {
    if (config) {
        config->config.pipeline.maximum_internal_processing_rate = max_internal_rate;
        config->config.pipeline.multi_channel_render = multi_channel_render != 0;
        config->config.pipeline.multi_channel_capture = multi_channel_capture != 0;
        switch (downmix_method) {
            case WEBRTC_APM_DOWNMIX_AVERAGE_CHANNELS: config->config.pipeline.capture_downmix_method = webrtc::AudioProcessing::Config::Pipeline::DownmixMethod::kAverageChannels; break;
            case WEBRTC_APM_DOWNMIX_USE_FIRST_CHANNEL: config->config.pipeline.capture_downmix_method = webrtc::AudioProcessing::Config::Pipeline::DownmixMethod::kUseFirstChannel; break;
        }
    }
}

// Apply configuration to APM
webrtc_apm_error webrtc_apm_apply_config(const webrtc_apm* apm, const webrtc_apm_config* config) {
    if (!apm || !config) {
        return WEBRTC_APM_BAD_PARAMETER;
    }
    apm->apm->ApplyConfig(config->config);
    return WEBRTC_APM_NO_ERROR;
}

// Stream configuration
webrtc_stream_config* webrtc_apm_stream_config_create(int sample_rate_hz, size_t num_channels) {
    auto* config = new webrtc_stream_config();
    config->config = webrtc::StreamConfig(sample_rate_hz, num_channels);
    return config;
}

void webrtc_apm_stream_config_destroy(const webrtc_stream_config* config) {
    delete config;
}

int webrtc_apm_stream_config_sample_rate_hz(const webrtc_stream_config* config) {
    return config ? config->config.sample_rate_hz() : 0;
}

size_t webrtc_apm_stream_config_num_channels(const webrtc_stream_config* config) {
    return config ? config->config.num_channels() : 0;
}

// Processing configuration
webrtc_processing_config* webrtc_apm_processing_config_create() {
    return new webrtc_processing_config();
}

void webrtc_apm_processing_config_destroy(const webrtc_processing_config* config) {
    delete config;
}

webrtc_stream_config* webrtc_apm_processing_config_input_stream(webrtc_processing_config* config) {
    if (!config) return nullptr;
    auto* stream_config = new webrtc_stream_config();
    stream_config->config = config->config.input_stream();
    return stream_config;
}

webrtc_stream_config* webrtc_apm_processing_config_output_stream(webrtc_processing_config* config) {
    if (!config) return nullptr;
    auto* stream_config = new webrtc_stream_config();
    stream_config->config = config->config.output_stream();
    return stream_config;
}

webrtc_stream_config* webrtc_apm_processing_config_reverse_input_stream(webrtc_processing_config* config) {
    if (!config) return nullptr;
    auto* stream_config = new webrtc_stream_config();
    stream_config->config = config->config.reverse_input_stream();
    return stream_config;
}

webrtc_stream_config* webrtc_apm_processing_config_reverse_output_stream(webrtc_processing_config* config) {
    if (!config) return nullptr;
    auto* stream_config = new webrtc_stream_config();
    stream_config->config = config->config.reverse_output_stream();
    return stream_config;
}

// Initialization
webrtc_apm_error webrtc_apm_initialize(const webrtc_apm* apm) {
    if (!apm) {
        return WEBRTC_APM_BAD_PARAMETER;
    }
    return static_cast<webrtc_apm_error>(apm->apm->Initialize());
}

webrtc_apm_error webrtc_apm_initialize_with_config(const webrtc_apm* apm, const webrtc_processing_config* config) {
    if (!apm || !config) {
        return WEBRTC_APM_BAD_PARAMETER;
    }
    return static_cast<webrtc_apm_error>(apm->apm->Initialize(config->config));
}

// Audio processing
webrtc_apm_error webrtc_apm_process_stream(const webrtc_apm* apm, const float* const* src,
                                                  const webrtc_stream_config* input_config,
                                                  const webrtc_stream_config* output_config,
                                                  float* const* dest) {
    if (!apm || !input_config || !output_config) {
        return WEBRTC_APM_BAD_PARAMETER;
    }
    return static_cast<webrtc_apm_error>(
        apm->apm->ProcessStream(src, input_config->config, output_config->config, dest));
}

webrtc_apm_error webrtc_apm_process_reverse_stream(const webrtc_apm* apm, const float* const* src,
                                                          const webrtc_stream_config* input_config,
                                                          const webrtc_stream_config* output_config,
                                                          float* const* dest) {
    if (!apm || !input_config || !output_config) {
        return WEBRTC_APM_BAD_PARAMETER;
    }
    return static_cast<webrtc_apm_error>(
        apm->apm->ProcessReverseStream(src, input_config->config, output_config->config, dest));
}

webrtc_apm_error webrtc_apm_analyze_reverse_stream(const webrtc_apm* apm, const float* const* data,
                                                          const webrtc_stream_config* reverse_config) {
    if (!apm || !reverse_config) {
        return WEBRTC_APM_BAD_PARAMETER;
    }
    return static_cast<webrtc_apm_error>(
        apm->apm->AnalyzeReverseStream(data, reverse_config->config));
}

// Runtime controls
void webrtc_apm_set_stream_analog_level(const webrtc_apm* apm, int level) {
    if (apm) {
        apm->apm->set_stream_analog_level(level);
    }
}

int webrtc_apm_recommended_stream_analog_level(const webrtc_apm* apm) {
    return apm ? apm->apm->recommended_stream_analog_level() : 0;
}

void webrtc_apm_set_stream_delay_ms(const webrtc_apm* apm, int delay) {
    if (apm) {
        apm->apm->set_stream_delay_ms(delay);
    }
}

int webrtc_apm_stream_delay_ms(const webrtc_apm* apm) {
    return apm ? apm->apm->stream_delay_ms() : 0;
}

void webrtc_apm_set_stream_key_pressed(const webrtc_apm* apm, int key_pressed) {
    if (apm) {
        apm->apm->set_stream_key_pressed(key_pressed != 0);
    }
}

void webrtc_apm_set_output_will_be_muted(const webrtc_apm* apm, int muted) {
    if (apm) {
        apm->apm->set_output_will_be_muted(muted != 0);
    }
}

// Runtime settings
void webrtc_apm_set_runtime_setting_float(const webrtc_apm* apm, webrtc_apm_runtime_setting_type type, float value) {
    if (!apm) return;

    switch (type) {
        case WEBRTC_APM_RUNTIME_CAPTURE_PRE_GAIN:
            apm->apm->SetRuntimeSetting(webrtc::AudioProcessing::RuntimeSetting::CreateCapturePreGain(value));
            break;
        case WEBRTC_APM_RUNTIME_CAPTURE_POST_GAIN:
            apm->apm->SetRuntimeSetting(webrtc::AudioProcessing::RuntimeSetting::CreateCapturePostGain(value));
            break;
        case WEBRTC_APM_RUNTIME_CAPTURE_FIXED_POST_GAIN:
            apm->apm->SetRuntimeSetting(webrtc::AudioProcessing::RuntimeSetting::CreateCaptureFixedPostGain(value));
            break;
        case WEBRTC_APM_RUNTIME_CUSTOM_RENDER_SETTING:
            apm->apm->SetRuntimeSetting(webrtc::AudioProcessing::RuntimeSetting::CreateCustomRenderSetting(value));
            break;
        default:
            break;
    }
}

void webrtc_apm_set_runtime_setting_int(const webrtc_apm* apm, webrtc_apm_runtime_setting_type type, int value) {
    if (!apm) return;

    switch (type) {
        case WEBRTC_APM_RUNTIME_CAPTURE_COMPRESSION_GAIN:
            apm->apm->SetRuntimeSetting(webrtc::AudioProcessing::RuntimeSetting::CreateCompressionGainDb(value));
            break;
        case WEBRTC_APM_RUNTIME_PLAYOUT_VOLUME_CHANGE:
            apm->apm->SetRuntimeSetting(webrtc::AudioProcessing::RuntimeSetting::CreatePlayoutVolumeChange(value));
            break;
        default:
            break;
    }
}

// Statistics and info
int webrtc_apm_proc_sample_rate_hz(const webrtc_apm* apm) {
    return apm ? apm->apm->proc_sample_rate_hz() : 0;
}

int webrtc_apm_proc_split_sample_rate_hz(const webrtc_apm* apm) {
    return apm ? apm->apm->proc_split_sample_rate_hz() : 0;
}

size_t webrtc_apm_num_input_channels(const webrtc_apm* apm) {
    return apm ? apm->apm->num_input_channels() : 0;
}

size_t webrtc_apm_num_proc_channels(const webrtc_apm* apm) {
    return apm ? apm->apm->num_proc_channels() : 0;
}

size_t webrtc_apm_num_output_channels(const webrtc_apm* apm) {
    return apm ? apm->apm->num_output_channels() : 0;
}

size_t webrtc_apm_num_reverse_channels(const webrtc_apm* apm) {
    return apm ? apm->apm->num_reverse_channels() : 0;
}

// AEC dump
int webrtc_apm_create_aec_dump(const webrtc_apm* apm, const char* file_name, int64_t max_log_size_bytes) {
    if (!apm || !file_name) {
        return 0;
    }
    // The C API does not support passing a task queue, so we pass nullptr.
    // The underlying implementation needs to handle this gracefully.
    return apm->apm->CreateAndAttachAecDump(file_name, max_log_size_bytes, nullptr) ? 1 : 0;
}

void webrtc_apm_detach_aec_dump(const webrtc_apm* apm) {
    if (apm) {
        apm->apm->DetachAecDump();
    }
}

// Frame size calculation
size_t webrtc_apm_get_frame_size(int sample_rate_hz) {
    return webrtc::AudioProcessing::GetFrameSize(sample_rate_hz);
}