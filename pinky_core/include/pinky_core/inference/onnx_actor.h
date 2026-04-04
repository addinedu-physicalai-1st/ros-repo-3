#pragma once

#include <array>
#include <string>
#include <vector>
#include "pinky_core/common/constants.h"

#ifdef PINKY_HAS_ONNXRUNTIME
#include <onnxruntime_cxx_api.h>
#endif

namespace pinky {

class OnnxActor {
 public:
  OnnxActor(const std::string& model_path);
  std::array<float, kActionDim> Infer(const std::array<float, kStateDim>& observation);

 private:
#ifdef PINKY_HAS_ONNXRUNTIME
  Ort::Env env_;
  Ort::Session session_;
  Ort::MemoryInfo memory_info_;
  std::vector<const char*> input_names_;
  std::vector<const char*> output_names_;
  std::vector<int64_t> input_shape_;
  std::vector<int64_t> output_shape_;
#endif
};

}  // namespace pinky
