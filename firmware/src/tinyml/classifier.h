// =============================================================================
//  TinyML event classifier (TensorFlow Lite Micro, int8 fully-connected MLP).
//
//  Input : the FeatureVector (features/feature_spec.h), standardised with the
//          mean/std learned in training (embedded in model_data.cpp) and
//          quantised to int8 with the model's input scale/zero-point.
//  Output: softmax over the 8 PqClass values 0..7.
//  Memory: the model lives in flash (const array); the tensor arena is a
//          statically allocated, 16-byte aligned buffer (no heap at runtime).
// =============================================================================
#pragma once
#include <cstdint>

#include "common/pq_types.h"

namespace pq {

struct ClassificationResult {
    uint8_t  class_id;
    float    confidence;
    uint32_t inference_time_us;       // interpreter Invoke() only
    uint32_t total_time_us;           // normalise + quantise + invoke + dequantise
    float    probs[kNumMlClasses];
    bool     valid;
};

struct ClassifierInfo {
    bool        ready;
    const char* error;          // non-null when not ready
    uint32_t    model_bytes;    // flatbuffer size in flash
    uint32_t    arena_bytes;    // statically reserved arena
    uint32_t    arena_used;     // bytes actually used after AllocateTensors
    uint32_t    spec_hash_model;
    uint32_t    spec_hash_firmware;
    float       in_scale;
    int         in_zero_point;
    float       out_scale;
    int         out_zero_point;
    const char* model_version;
    float       reported_accuracy;   // test accuracy recorded by the trainer
    uint32_t    runs;
    uint32_t    max_inference_us;
};

bool                        Classifier_Init();
ClassificationResult        Classifier_Run(const float* features);
const ClassifierInfo&       Classifier_Info();

}  // namespace pq
