1. PI setup：

1.1 # Install dependencies
sudo apt update
sudo apt install -y libatlas-base-dev libopenblas-dev
pip install opencv-python-headless onnxruntime numpy

1.2 Copy PiCapture.py, PiCarDriving.py, PiDetection.py, PiRecording.py, PiShared.py and best_exp20.onnx to ~/AutoRC/src

1.3 cd ~/AutoRC/src
python PiCarDriving.py

2. dataset setup

2.1 download coco train2017 and val2017 from coco website

2.2 use "Yolov5, Yolov8n dataset preparation and parameter comparison.ipynb"
run each block
2.2.1 merge background with signs
#set the correct sign3.csv, paths of coco images and result folder for :
"CSV_PATH = "signs.csv""
"dataset = TrafficSignDataset("signs.csv", synthetic_backgrounds_dir="train2017/")
dataset.process_images_4("result_train")" 

2.2.2 generat yolo annotation label text files

3. training

3.1 get yolov5n
git clone https://github.com/ultralytics/yolov5
cd yolov5
pip install -r requirements.txt

3.2 copy step 2 images and labels to yolo folder
put yolov5n_coco.yaml in yolo folder

3.3. use "Yolov5, Yolov8n dataset preparation and parameter comparison.ipynb"
bash command to train
bash command to export to ONNX


