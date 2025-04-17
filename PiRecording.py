# SEP742 Project Group2 Note:
# For this module:
# We kept the orignal AutoRC code offered by the course, for consideration of possible utilization of 
# existing functions
# New code added for our project mainly include:
# - def RecordResult()

from picamera2 import Picamera2, Preview
import picamera2.encoders 
import time
from datetime import datetime
import os
import cv2
import queue
from PiShared import record_queue, stop_flag

def RecordResult():
    try:
        output_file = generate_filename()
        print(f"PiRecording: Start recording file name {output_file}.")
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        outputVideo = cv2.VideoWriter(output_file,fourcc,3,(640,480))

        while not stop_flag.is_set():
            frame_available = False
            try:
                frame = record_queue.get()
                frame_available = True
                print(f"{time.time()} PiRecording: get a frame from queue.")
            except queue.Empty:
                frame_available = False
                time.sleep(0.01)
                continue

            if frame_available:
                try:
                    outputVideo.write(frame)
                    print(f"PiRecording: Wrote a frame to file name {output_file}.")
                
                except Exception as e:
                    print(f"Error writing frame: {e}")
                    time.sleep(0.01)

        outputVideo.release()
        print("PiRecording: Output video released")

    except Exception as e:
        print(f"Error in PiRecording: {e}")

    finally:
        if camera:
            try:
                outputVideo.release()
                print("Output video released")
            except:
                pass


# Initialize the camera
def camInit() :
    global camera, camera_config
    camera = Picamera2()

# Configure camera settings
    camera_config = camera.create_video_configuration(main={"size": (640, 480), "format": 'YUV420'})
    #camera.sensor_resolution = (640,480)
    camera.configure(camera_config)

# Define the directory where recordings will be saved
SAVE_DIRECTORY = os.path.expanduser("~/Videos")  # Change this to your desired directory

def generate_filename():
    """Generates a filename based on the current date and time, including the save directory."""
    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")  # e.g., "2025-01-24_15-45-30"
    return os.path.join(SAVE_DIRECTORY, f"recording_{timestamp}.h264")

def start_recording(output_file):
    """Starts recording to the specified file."""
    generate_filename()
    print(f"Starting recording: {output_file}")
    encoder = picamera2.encoders.H264Encoder() # Use the H264 encoder
    camera.start_recording( encoder ,output_file)  # Pass encoder and output file

def stop_recording():
    """Stops recording."""
    print("Stopping recording")
    camera.stop_recording()

def CameraSetupHandler() :
    camInit()
    global SAVE_DIRECTORY, camera, camera_config, output_file
    if not os.path.exists(SAVE_DIRECTORY):
        os.makedirs(SAVE_DIRECTORY)
        print(f"Created directory: {SAVE_DIRECTORY}")

    
    output_file = generate_filename()
    camera.start()
    time.sleep(0.5)

def CameraEnd() :
    stop_recording()
    camera.stop()

#CameraSetupHandler()
# Example Usage
#if __name__ == "__main__":
    # Ensure the save directory exists
 #   if not os.path.exists(SAVE_DIRECTORY):
  #      os.makedirs(SAVE_DIRECTORY)
   #     print(f"Created directory: {SAVE_DIRECTORY}")
    
    # Generate a timestamped filename
output_file = generate_filename()
    
    # Start the camera
#camera.start()
#time.sleep(2)  # Let the camera warm up

#try:
        # Start recording
        #start_recording(output_file)
        #time.sleep(10)  # Record for 10 seconds

        # Stop recording
        #stop_recording()
#finally:
        # Stop the camera
        #camera.stop()
