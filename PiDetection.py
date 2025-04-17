import numpy as np
import queue
import time
import cv2
import onnxruntime as ort
from PiShared import frame_queue, detection_queue, stop_flag

def detectObject():
    #load trained model (already exported to ONNX type)
    model = ort.InferenceSession("best_exp20.onnx", providers=['CPUExecutionProvider'])
    print("PiDetection: model loaded successfully.")
    
    while not stop_flag.is_set():
        frame_available = False
        try:
            frame = frame_queue.get()
            frame_available = True
            print(f"{time.time()} PiDetection: get a frame from queue.")
        except queue.Empty:
            frame_available = False
            time.sleep(0.01)
            continue

        if frame_available:
            try:
                input = cv2.resize(frame, (640, 480))
                input = cv2.cvtColor(input, cv2.COLOR_BGR2RGB) #essential convertion for yolov5n
                input = input.astype(np.float32)/255.0
                input = np.transpose(input,(2,0,1))  #HWC to CHW
                input = np.expand_dims(input, axis=0) #add batch dimension

                outputs = model.run(None, {"images": input})
                detections = outputs[0][0]  # N detections * [x1, y1, x2, y2, conf, class]      
                
                print(f"PiDetection: detections shape: {detections.shape}.")
                            
                detection_package = [frame, detections] 
                # detection_package = detections
                            
                # if the retriever is slower and the supplier is faster, there would be backlogs, in which case 
                #  the supplier would take out and discard the (only one, old) frame in the queue right 
                #  before it stores the new frame.  
                try:
                    detection_queue.get_nowait()
                except queue.Empty:
                    pass
                detection_queue.put(detection_package)
            
            except Exception as e:
                print(f"Error in detection: {e}")
                time.sleep(0.01)
