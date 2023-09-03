# @time   : 2023/7/13 20:38
# @author : Xiang Lei

from ultralytics import YOLO

model = YOLO("weights/best_7_24.pt")

success = model.export(format="onnx", batch=1)
