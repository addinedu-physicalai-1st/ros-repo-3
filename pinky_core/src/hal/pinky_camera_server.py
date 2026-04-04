import zmq
import time
import cv2
import numpy as np
import sys

try:
    from picamera2 import Picamera2
    HAS_PICAMERA = True
except ImportError:
    HAS_PICAMERA = False

def main():
    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    # Use TCP for localhost communication
    socket.bind("tcp://127.0.0.1:5555")

    picam2 = None
    if HAS_PICAMERA:
        try:
            picam2 = Picamera2()
            config = picam2.create_preview_configuration(main={"format": "RGB888", "size": (640, 480)})
            picam2.configure(config)
            picam2.start()
            print("Picamera2 started successfully.")
        except Exception as e:
            print(f"Failed to start Picamera2: {e}")
            HAS_PICAMERA = False
    
    if not HAS_PICAMERA:
        print("Picamera2 not available or failed. Mocking camera with random noise...")

    print("Camera server publishing on tcp://127.0.0.1:5555")
    
    try:
        while True:
            if HAS_PICAMERA:
                # Capture frame as numpy array (RGB888)
                frame = picam2.capture_array()
                # Rotate 180 to match hardware mounting
                frame = cv2.rotate(frame, cv2.ROTATE_180)
                # Convert RGB to BGR for OpenCV encoding
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            else:
                # Mock frame: Blue rectangle moving on gray background
                frame = np.full((480, 640, 3), 128, dtype=np.uint8)
                t = time.time()
                x = int(320 + 200 * np.sin(t))
                y = int(240 + 100 * np.cos(t))
                cv2.rectangle(frame, (x-20, y-20), (x+20, y+20), (255, 0, 0), -1)
                time.sleep(0.05) # ~20 fps mock

            # Encode to JPEG
            ret, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 60])
            if ret:
                socket.send(buffer.tobytes())
            
            # Control rate (~20-30 fps)
            if HAS_PICAMERA:
                time.sleep(0.01)
                
    except KeyboardInterrupt:
        print("Stopping camera server...")
    finally:
        if picam2:
            picam2.stop()
        socket.close()
        context.term()

if __name__ == "__main__":
    main()
