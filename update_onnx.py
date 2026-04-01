import onnx

model_path = "/home/hajun/ros2_ws/pinky_cpp/pinky_core/models/sac_actor.onnx"
try:
    model = onnx.load(model_path)
    model.ir_version = 9
    onnx.save(model, model_path)
    print("Successfully downgraded ONNX IR version to 9.")
except Exception as e:
    print("Error:", e)
