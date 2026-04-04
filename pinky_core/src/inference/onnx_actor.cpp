#include "pinky_core/inference/onnx_actor.h"
#include <iostream>
#include <cmath>

#ifdef PINKY_HAS_ONNXRUNTIME
#include <stdexcept>
#endif

namespace pinky {

OnnxActor::OnnxActor(const std::string& model_path)
#ifdef PINKY_HAS_ONNXRUNTIME
    : env_(ORT_LOGGING_LEVEL_WARNING, "RlInference"),
      session_(env_, model_path.c_str(), []() {
        Ort::SessionOptions opts;
        opts.SetIntraOpNumThreads(1);
        opts.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
        return opts;
      }()),
      memory_info_(
          Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault)),
      input_names_{"state"},
      output_names_{"action"},
      input_shape_{1, kStateDim},
      output_shape_{1, kActionDim} {
  std::cout << "OnnxActor: Initialized with ONNX Runtime backend.\n";
}
#else
{
  std::cout << "OnnxActor: ONNX Runtime NOT FOUND. Initialized with Mock (P-Control) backend.\n";
}
#endif

std::array<float, kActionDim> OnnxActor::Infer(
    const std::array<float, kStateDim>& observation) {
#ifdef PINKY_HAS_ONNXRUNTIME
  // Real inference code (zero-copy from the array)
  auto input_tensor = Ort::Value::CreateTensor<float>(
      memory_info_, const_cast<float*>(observation.data()), kStateDim,
      input_shape_.data(), input_shape_.size());

  auto output = session_.Run(Ort::RunOptions{nullptr}, input_names_.data(),
                             &input_tensor, 1, output_names_.data(),
                             output_names_.size());

  const float* data = output[0].GetTensorData<float>();
  return {data[0], data[1]};
#else
  // Mock inference: Based on observation builder, the first few elements are goal relative position
  // state[0] = goal_x_rel, state[1] = goal_y_rel (usually normalized)
  float goal_x = observation[0];
  float goal_y = observation[1];
  
  // Very simple goal seeker: 
  // v (linear) = x, w (angular) = y
  // This is enough to verify that robot moves towards the target in the simulator/station
  return {std::min(1.0f, goal_x * 2.0f), std::min(1.0f, std::max(-1.0f, goal_y * 3.0f))};
#endif
}

}  // namespace pinky
