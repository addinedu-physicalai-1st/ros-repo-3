#pragma once

#ifdef PINKY_HAS_ONNXRUNTIME

#include <array>
#include <memory>
#include <string>

#include <onnxruntime_cxx_api.h>

#include "pinky_core/common/constants.h"

namespace pinky {

// Loads an ONNX SAC actor model and runs inference.
// Input:  28D float32 observation vector
// Output: 2D float32 action in [-1, 1] (tanh-squashed)
class OnnxActor {
 public:
  explicit OnnxActor(const std::string& model_path);

  std::array<float, kActionDim> Infer(
      const std::array<float, kStateDim>& observation);

 private:
  Ort::Env env_;
  Ort::Session session_;
  Ort::MemoryInfo memory_info_;

  std::vector<const char*> input_names_;
  std::vector<const char*> output_names_;
  std::array<int64_t, 2> input_shape_;
  std::array<int64_t, 2> output_shape_;
};

}  // namespace pinky

#endif  // PINKY_HAS_ONNXRUNTIME
