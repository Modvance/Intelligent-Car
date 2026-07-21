import cv2
import numpy as np
import time
from smart_car_nodes.models.base_model import Model
# 在 lfnet.py 中

# --- 预处理参数 (必须与导出 ONNX 时 dummy_input 的预处理一致) ---
RESIZE_SHAPE = (512, 256) # (宽度, 高度)

# 现在我们知道训练时的归一化方式了
NORMALIZATION_MODE = "minus_one_to_one" # 或者你自定义一个更贴切的名字

# 如果不再使用 ImageNet 归一化，下面这两行可以注释掉或删除
# IMAGENET_MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32)
# IMAGENET_STD = np.array([0.229, 0.224, 0.225], dtype=np.float32)

class LFNet(Model): # 假设 Model 基类和 self.execute 来自某个昇腾推理库
    def __init__(self, model_path, acl_init=True):
        super().__init__(model_path, acl_init)
        self.input_width = RESIZE_SHAPE[0] # 512
        self.input_height = RESIZE_SHAPE[1] # 256

    def infer(self, original_bgr_image):
        # start_t = time.time()
        # 1. Resize
        #    cv2.resize dsize is (width, height)
        resized_image = cv2.resize(original_bgr_image, (self.input_width, self.input_height), interpolation=cv2.INTER_LINEAR)

        # 2. BGR to RGB (训练时没有这一步，所以这里要注释掉或删除！)
        # image_rgb = cv2.cvtColor(resized_image, cv2.COLOR_BGR2RGB)
        # 直接使用 resized_image (它是 BGR 格式)

        # 3. Normalize to [-1, 1]
        image_float = resized_image.astype(np.float32) # resized_image 已经是 BGR
        normalized_image = image_float / 127.5 - 1.0   # <--- 匹配训练时的归一化

        # 4. Transpose HWC to CHW
        #    因为输入是 BGR (H, W, C)，所以转置后也是 (C, H, W)
        image_chw = np.transpose(normalized_image, (2, 0, 1))

        # 5. Add batch dimension: NCHW
        batched_img_nchw = np.expand_dims(image_chw, axis=0)
        #end_t = time.time()
        # 6. Execute inference
        result = self.execute([np.ascontiguousarray(batched_img_nchw)])
        f_t = time.time()
        # print("pre time : ", end_t - start_t)
        # print("mdel time: ", f_t - end_t)
        return result

# class LFNet(Model):
#     def __init__(self, model_path, acl_init=True):
#         super().__init__(model_path, acl_init)

#     def infer(self, inputs):
#         inputs = inputs[:, :, [2, 1, 0]]
#         inputs = cv2.resize(inputs, (512,256))
#         inputs = inputs.astype(np.float32) / 255
#         batched_img = np.expand_dims(inputs, axis=0)
#         result = self.execute([np.ascontiguousarray(batched_img)])
#         return result

