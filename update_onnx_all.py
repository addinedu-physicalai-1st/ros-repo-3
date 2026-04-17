import onnx
import glob

for model_path in glob.glob("/home/hajun/ros2_ws/pinky_cpp/**/*.onnx", recursive=True):
    try:
        model = onnx.load(model_path)
        model.ir_version = 9
        onnx.save(model, model_path)
        print(f"Successfully downgraded ONNX IR version to 9 for {model_path}")
    except Exception as e:
        print(f"Error for {model_path}:", e)
