# in Python
import torch
from ultralytics import YOLO

# load your trained .pt model
model = YOLO("best.pt")

# export to TorchScript
model.export(format="torchscript")

