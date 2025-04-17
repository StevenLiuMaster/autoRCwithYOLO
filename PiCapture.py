from picamera2 import Picamera2, Preview
import queue
import time
from PiShared import frame_queue, stop_flag, camera_lock

def captureFrame():
    camera = None
    try:
        with camera_lock:
            camera = Picamera2()

            # Configure camera settings
            camera_config = camera.create_video_configuration(main={"size": (640, 480), "format": 'RGB888'}) #essential format for yolov5n and correct color of recording
            camera.configure(camera_config)
            camera.start()
            print("Camera started successfully")

            while not stop_flag.is_set():
                try:
                    frame = camera.capture_array()

                    # if the retriever is slower and the supplier is faster, there would be backlogs, in which case 
                    #  the supplier would take out and discard the (only one, old) frame in the queue right 
                    #  before it stores the new frame.
                    try:
                        frame_queue.get_nowait()
                    except queue.Empty:
                        pass
                    frame_queue.put(frame)
                
                except Exception as e:
                    print(f"Frame capture error: {e}")
                    time.sleep(0.01)

            camera.stop()
    except Exception as e:
        print(f"Camera initialization error: {e}")

    finally:
        if camera:
            try:
                camera.stop()
                print("Camera stopped")
            except:
                pass
