import torch
import numpy as np
from ais_bench.infer.interface import InferSession
from src.utils.cv_utils import nms, scale_coords, preprocess_image_yolov5


class YoloV5:
    def __init__(self, model_path):
        self.model = InferSession(0, model_path)
        self.neth = 640
        self.netw = 640
        self.conf_threshold = 0.1
        dic = {0: 'park',
               1: 'stop',
               2: 'right',
               3: 'back',
               4: 'sideway',
               5: 'left'}
        # self.names = ['person', 'sports_ball', 'bicycle', 'motorcycle', 'car', 'bus', 'truck'] * 12
        # self.object_list = ['person', 'sports_ball', 'bicycle', 'motorcycle', 'car', 'bus', 'truck']
        self.names = list(dic.values())
        self.object_list = list(dic.values())
        self.cfg = {
            'conf_thres': 0.3,  
            'iou_thres': 0.5, 
            'input_shape': [640, 640],  
        }
        # self.model = InferSession(0, model_path)
        # self.cfg = {
        #     'conf_thres': 0.25,  
        #     'iou_thres': 0.5,
        #     'input_shape': [640, 640],
        # }
     
        # self.names = ['park', 'stop', 'right', 'back', 'left']

    def infer(self, img_bgr):
        img, scale_ratio, pad_size = preprocess_image_yolov5(img_bgr, self.cfg)
      

        if img.shape == (3, 640, 640):
            img = np.expand_dims(img, 0)
        if img.dtype != np.float32:
            img = img.astype(np.float32)
        output = self.model.infer([img])[0]

        output = torch.tensor(output)
      
        boxout = nms(output, conf_thres=self.cfg["conf_thres"], iou_thres=self.cfg["iou_thres"])
        pred_all = boxout[0].numpy()
      
        scale_coords(self.cfg['input_shape'], pred_all[:, :4], img_bgr.shape, ratio_pad=(scale_ratio, pad_size))
        pred_boxes = []

        for idx, class_id in enumerate(pred_all[:, 5]):
            if float(pred_all[idx][4] < float(0.05)):
                continue
            obj_name = self.names[int(pred_all[idx][5])]
            confidence = pred_all[idx][4]
            x1 = int(pred_all[idx][0])
            y1 = int(pred_all[idx][1])
            x2 = int(pred_all[idx][2])
            y2 = int(pred_all[idx][3])

            pred_boxes.append([x1, y1, x2, y2, obj_name, confidence])

        return pred_boxes