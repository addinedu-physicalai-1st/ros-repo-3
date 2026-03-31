#ifdef PINKY_HAS_ONNXRUNTIME

#include "pinky_core/inference/onnx_actor.h"

#include <stdexcept>

namespace pinky {

OnnxActor::OnnxActor(const std::string& model_path)
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
  // Validate model input/output shapes
  auto input_info = session_.GetInputTypeInfo(0);
  auto input_tensor_info = input_info.GetTensorTypeAndShapeInfo();
  auto shape = input_tensor_info.GetShape();
  if (shape.size() != 2 || shape[1] != kStateDim) {
    throw std::runtime_error("ONNX model input shape mismatch: expected (1, " +
                             std::to_string(kStateDim) + ")");
  }
}

std::array<float, kActionDim> OnnxActor::Infer(
    const std::array<float, kStateDim>& observation) {
  // Create input tensor (zero-copy from the array)
  auto input_tensor = Ort::Value::CreateTensor<float>(
      memory_info_, const_cast<float*>(observation.data()), kStateDim,
      input_shape_.data(), input_shape_.size());

  // Run inference
  auto output = session_.Run(Ort::RunOptions{nullptr}, input_names_.data(),
                             &input_tensor, 1, output_names_.data(),
                             output_names_.size());

  // Copy result
  const float* data = output[0].GetTensorData<float>();
  return {data[0], data[1]};
}

}  // namespace pinky

#endif  // PINKY_HAS_ONNXRUNTIME
