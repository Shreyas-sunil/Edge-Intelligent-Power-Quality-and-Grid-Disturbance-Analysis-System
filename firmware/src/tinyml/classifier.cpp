#include "tinyml/classifier.h"

#include <cmath>
#include <cstring>

#include "common/platform.h"
#include "features/feature_spec.h"
#include "tinyml/model_data.h"

#ifndef PQ_TENSOR_ARENA_SIZE
#define PQ_TENSOR_ARENA_SIZE (6 * 1024)
#endif

#if PQ_HAVE_TFLM
#include <new>

#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "system/log_sink.h"

extern "C" {
extern void (*pq_tflm_log_hook)(const char* msg);
extern uint32_t (*pq_tflm_time_hook)(void);
}
namespace {
void tflmLog(const char* m) { pq::logSink(pq::LogLevel::Warn, "tflm", m); }
uint32_t tflmTime() { return pq::plat::timeUs32(); }
}  // namespace
#endif

namespace pq {

namespace {
ClassifierInfo g_info{};

#if PQ_HAVE_TFLM
alignas(16) uint8_t g_arena[PQ_TENSOR_ARENA_SIZE];
alignas(tflite::MicroInterpreter) uint8_t g_interp_mem[sizeof(tflite::MicroInterpreter)];
tflite::MicroInterpreter* g_interp = nullptr;
TfLiteTensor* g_in = nullptr;
TfLiteTensor* g_out = nullptr;
#endif

bool fail(const char* msg) {
    g_info.ready = false;
    g_info.error = msg;
    return false;
}
}  // namespace

const ClassifierInfo& Classifier_Info() { return g_info; }

bool Classifier_Init() {
    std::memset(&g_info, 0, sizeof(g_info));
    g_info.model_bytes = model::kModelLen;
    g_info.arena_bytes = PQ_TENSOR_ARENA_SIZE;
    g_info.spec_hash_model = model::kFeatureSpecHash;
    g_info.spec_hash_firmware = featureSpecHash();
    g_info.model_version = model::kModelVersion;
    g_info.reported_accuracy = model::kTestAccuracy;

    if (model::kNumFeatures != kNumFeatures || model::kFeatureSpecHash != featureSpecHash())
        return fail("feature spec mismatch: retrain model for this firmware");
    if (model::kNumClasses != kNumMlClasses) return fail("class count mismatch");

#if PQ_HAVE_TFLM
    pq_tflm_log_hook = &tflmLog;
    pq_tflm_time_hook = &tflmTime;
    if (model::kModelLen == 0) return fail("no model compiled in (run ml/train_model.py)");
    const tflite::Model* m = tflite::GetModel(model::kModel);
    if (m->version() != TFLITE_SCHEMA_VERSION) return fail("model schema version mismatch");

    static tflite::MicroMutableOpResolver<5> resolver;
    static bool ops_added = false;
    if (!ops_added) {
        resolver.AddFullyConnected();
        resolver.AddSoftmax();
        resolver.AddRelu();
        resolver.AddQuantize();
        resolver.AddDequantize();
        ops_added = true;
    }
    if (g_interp) g_interp->~MicroInterpreter();
    g_interp = new (g_interp_mem) tflite::MicroInterpreter(m, resolver, g_arena, sizeof(g_arena));
    if (g_interp->AllocateTensors() != kTfLiteOk) return fail("AllocateTensors failed (arena too small?)");
    g_info.arena_used = static_cast<uint32_t>(g_interp->arena_used_bytes());

    g_in = g_interp->input(0);
    g_out = g_interp->output(0);
    if (!g_in || !g_out) return fail("missing input/output tensor");
    if (g_in->type != kTfLiteInt8 || g_out->type != kTfLiteInt8) return fail("model must be int8 in/out");
    if (g_in->dims->size != 2 || g_in->dims->data[1] != kNumFeatures) return fail("input shape mismatch");
    if (g_out->dims->size != 2 || g_out->dims->data[1] != kNumMlClasses) return fail("output shape mismatch");
    g_info.in_scale = g_in->params.scale;
    g_info.in_zero_point = g_in->params.zero_point;
    g_info.out_scale = g_out->params.scale;
    g_info.out_zero_point = g_out->params.zero_point;
    g_info.ready = true;
    g_info.error = nullptr;
    return true;
#else
    return fail("firmware built without TensorFlow Lite Micro (PQ_ENABLE_TFLM=OFF)");
#endif
}

ClassificationResult Classifier_Run(const float* features) {
    ClassificationResult r{};
    if (!g_info.ready) return r;
#if PQ_HAVE_TFLM
    const uint64_t t0 = plat::timeUs();
    int8_t* in = g_in->data.int8;
    const float inv = 1.0f / g_info.in_scale;
    for (int k = 0; k < kNumFeatures; ++k) {
        const float z = (features[k] - model::kFeatureMean[k]) / model::kFeatureStd[k];
        long q = std::lround(z * inv) + g_info.in_zero_point;
        in[k] = static_cast<int8_t>(q < -128 ? -128 : (q > 127 ? 127 : q));
    }
    const uint64_t t1 = plat::timeUs();
    const TfLiteStatus st = g_interp->Invoke();
    const uint64_t t2 = plat::timeUs();
    if (st != kTfLiteOk) return r;
    const int8_t* out = g_out->data.int8;
    int best = 0;
    for (int c = 0; c < kNumMlClasses; ++c) {
        r.probs[c] = (static_cast<int>(out[c]) - g_info.out_zero_point) * g_info.out_scale;
        if (r.probs[c] > r.probs[best]) best = c;
    }
    r.class_id = static_cast<uint8_t>(best);
    r.confidence = r.probs[best];
    r.inference_time_us = static_cast<uint32_t>(t2 - t1);
    r.total_time_us = static_cast<uint32_t>(plat::timeUs() - t0);
    r.valid = true;
    ++g_info.runs;
    if (r.inference_time_us > g_info.max_inference_us) g_info.max_inference_us = r.inference_time_us;
#else
    (void)features;
#endif
    return r;
}

}  // namespace pq
