import queue
from queue import Queue
import threading

# Shared queues for data transfer between threads
# We choose queue max size to be 1 for the following consideration:
# It is critical for our project to achieve as low latency as possible (because it is real time video processing to control movement) 
#  on limited performance hardware. So if the threads process their individual tasks at different speed (in which case backlog would
#  occur in the queues), it does not make sense for a slower thread to choose a backlogged frame (outdated, from an older time) to
#  process, rather than to choose the latest frame from the queue.
# So our strategy is to discard older/backlogged/outdated frames and only keep the latest frame in the queue. If the retriever of the
#  queue is faster than the supplier, there would be no backlogs. On the other hand, if the retriever is slower and the supplier is
#  faster, there would be backlogs, in which case the supplier would take out and discard the (only one, old) frame in the queue right 
#  before it stores the new frame.
# This strategy is used by all retrievers and suppliers of the queues in our project.
# 
frame_queue = Queue(maxsize=1) # stores raw frame captured from camera
detection_queue = Queue(maxsize=1) # stores raw frame and detection results (not post-processed yet) from deep learning model
record_queue = Queue(maxsize=1) # Stores result frame with bounding boxes for sign detection and lines for route edge detection drawn

stop_flag = threading.Event() #flag to inform child threads if the main thread stops or interrupted
camera_lock = threading.Lock() #lock for camera to ensure only one user application uses it