import os.path as ops
import re
import numpy as np
import torch
import cv2
import torchvision


_SAMPLE_PATH_PATTERN = re.compile(r"(.+?\.(?:png|jpg|jpeg|bmp))(?:\s+|$)", re.IGNORECASE)


def _parse_sample_line(raw_line):
    line = raw_line.strip()
    if not line:
        return []

    if "\t" in line:
        parts = [part.strip() for part in line.split("\t") if part.strip()]
    else:
        parts = line.split()
        if len(parts) != 3:
            parts = [match.group(1).strip() for match in _SAMPLE_PATH_PATTERN.finditer(line)]

    if len(parts) != 3:
        raise ValueError("expected 3 image paths, got {}: {}".format(len(parts), raw_line.rstrip()))
    return parts


def _read_sample_list(file_open):
    img_pathes = []
    with open(file_open, 'r', encoding='utf-8') as file:
        for line_number, line in enumerate(file, start=1):
            if not line.strip():
                continue
            try:
                img_pathes.append(_parse_sample_line(line))
            except ValueError as exc:
                raise ValueError("{}:{}: {}".format(file_open, line_number, exc)) from exc
    return img_pathes


def _read_image(path, flag, role):
    image = cv2.imread(path, flag)
    if image is None:
        raise FileNotFoundError("could not read {} image: {}".format(role, path))
    return image


class TUSIMPLE(torch.utils.data.Dataset):
    def __init__(self, root, transforms=None, resize=(512, 256), flag='train'):
        self.root = root
        self.transforms = transforms
        self.resize = resize
        self.flag = flag

        self.img_pathes = []

        self.train_file = ops.join(root, 'train.txt')
        self.val_file = ops.join(root, 'val.txt')
        self.test_file = ops.join(root, 'test.txt')

        if self.flag == 'train':
            file_open = self.train_file
        elif self.flag == 'valid':
            file_open = self.val_file
        else:
            file_open = self.test_file

        self.img_pathes = _read_sample_list(file_open)

    def __len__(self):
        return len(self.img_pathes)

    def __getitem__(self, idx):
        # print(self.img_pathes[idx][0])
        gt_image = _read_image(self.img_pathes[idx][0], cv2.IMREAD_UNCHANGED, "source")
        gt_binary_image = _read_image(self.img_pathes[idx][1], cv2.IMREAD_UNCHANGED, "binary")
        gt_instance = _read_image(self.img_pathes[idx][2], cv2.IMREAD_UNCHANGED, "instance")

        gt_image = cv2.resize(gt_image, dsize=self.resize, interpolation=cv2.INTER_LINEAR)
        gt_binary_image = cv2.resize(gt_binary_image, dsize=self.resize, interpolation=cv2.INTER_NEAREST)
        gt_instance = cv2.resize(gt_instance, dsize=self.resize, interpolation=cv2.INTER_NEAREST)

        gt_image = gt_image / 127.5 - 1.0
        gt_binary_image = np.array(gt_binary_image / 255.0, dtype=np.uint8)
        gt_binary_image = gt_binary_image[:, :, np.newaxis]
        gt_instance = gt_instance[:, :, np.newaxis]

        gt_binary_image = np.transpose(gt_binary_image, (2, 0, 1))
        gt_instance = np.transpose(gt_instance, (2, 0, 1))

        gt_image = torch.tensor(gt_image, dtype=torch.float)
        gt_image = np.transpose(gt_image, (2, 0, 1))
        # trsf = torchvision.transforms.Normalize(mean=(0.485, 0.456, 0.406), std=(0.229, 0.224, 0.225), inplace=False)
        # gt_image = trsf(gt_image)

        gt_binary_image = torch.tensor(gt_binary_image, dtype=torch.long).view(self.resize[1], self.resize[0])
        #gt_binary_image = torch.tensor(gt_binary_image, dtype=torch.float)
        # gt_instance = torch.tensor(gt_instance, dtype=torch.float)
        gt_instance = torch.tensor(gt_instance, dtype=torch.long).view(self.resize[1], self.resize[0])

        return gt_image, gt_binary_image, gt_instance

# class TUSIMPLE(torch.utils.data.Dataset):
#     def __init__(self, root, transforms=None, resize=(512, 256), flag='train'):
#         self.root = root
#         self.transforms = transforms
#         self.resize = resize
#         self.flag = flag
        
#         self.img_pathes = []
        
#         self.train_file = ops.join(root, 'train.txt')
#         self.val_file = ops.join(root, 'val.txt')
#         self.test_file = ops.join(root, 'test.txt')
        
#         if self.flag == 'train':
#             file_open = self.train_file
#         elif self.flag == 'valid':
#             file_open = self.val_file
#         else:
#             file_open = self.test_file
            
#         # 读取文件路径列表
#         with open(file_open, 'r') as file:
#             data = file.readlines()
#             for l in data:
#                 line = l.strip().split()  # 去除行末换行符
#                 if len(line) >= 3:  # 确保每行至少包含3个路径
#                     self.img_pathes.append(line)
#                 else:
#                     print(f"Warning: invalid line skipped - {line}")
    
#     def __len__(self):
#         return len(self.img_pathes)
    
#     def __getitem__(self, idx):
#         # 检查索引有效性
#         if idx < 0 or idx >= len(self):
#             raise IndexError(f"Index {idx} is out of bounds for dataset of size {len(self)}")
            
#         # 获取图像路径
#         img_path, binary_path, instance_path = self.img_pathes[idx]
        
#         # 读取图像并检查是否成功
#         gt_image = cv2.imread(img_path, cv2.IMREAD_UNCHANGED)
#         if gt_image is None:
#             raise FileNotFoundError(f"Could not read image at path: {img_path}")
            
#         gt_binary_image = cv2.imread(binary_path, cv2.IMREAD_UNCHANGED)
#         if gt_binary_image is None:
#             raise FileNotFoundError(f"Could not read binary image at path: {binary_path}")
            
#         gt_instance = cv2.imread(instance_path, cv2.IMREAD_UNCHANGED)
#         if gt_instance is None:
#             raise FileNotFoundError(f"Could not read instance image at path: {instance_path}")
        
#         # 调整图像大小
#         gt_image = cv2.resize(gt_image, dsize=self.resize, interpolation=cv2.INTER_LINEAR)
#         gt_binary_image = cv2.resize(gt_binary_image, dsize=self.resize, interpolation=cv2.INTER_NEAREST)
#         gt_instance = cv2.resize(gt_instance, dsize=self.resize, interpolation=cv2.INTER_NEAREST)
        
#         # 图像预处理
#         # 转换为RGB格式（OpenCV默认读取为BGR）
#         if len(gt_image.shape) == 3:
#             gt_image = cv2.cvtColor(gt_image, cv2.COLOR_BGR2RGB)
        
#         # 归一化处理 - 使用标准的ImageNet归一化参数
#         gt_image = gt_image.astype(np.float32) / 255.0
#         mean = np.array([0.485, 0.456, 0.406], dtype=np.float32)
#         std = np.array([0.229, 0.224, 0.225], dtype=np.float32)
#         gt_image = (gt_image - mean) / std
        
#         # 处理二值图像（确保为0/1）
#         gt_binary_image = (gt_binary_image > 127).astype(np.uint8)
        
#         # 调整维度顺序 (H, W, C) -> (C, H, W)
#         gt_image = np.transpose(gt_image, (2, 0, 1))
#         gt_binary_image = np.expand_dims(gt_binary_image, axis=0)
#         gt_instance = np.expand_dims(gt_instance, axis=0)
        
#         # 转换为PyTorch张量
#         gt_image = torch.tensor(gt_image, dtype=torch.float32)
#         gt_binary_image = torch.tensor(gt_binary_image, dtype=torch.long)
#         gt_instance = torch.tensor(gt_instance, dtype=torch.long)
        
#         # 应用额外的变换（如果有）
#         if self.transforms is not None:
#             gt_image = self.transforms(gt_image)
            
#         return gt_image, gt_binary_image, gt_instance
    
class TUSIMPLE_AUG(torch.utils.data.Dataset):
    def __init__(self, root, transforms=None, resize=(512, 256), flag='train'):
        self.root = root
        self.transforms = transforms
        self.resize = resize
        self.flag = flag

        self.img_pathes = []

        self.train_file = ops.join(root, 'train.txt')
        self.val_file = ops.join(root, 'val.txt')
        self.test_file = ops.join(root, 'test.txt')

        if self.flag == 'train':
            file_open = self.train_file
        elif self.flag == 'valid':
            file_open = self.val_file
        else:
            file_open = self.test_file

        self.img_pathes = _read_sample_list(file_open)

    def __len__(self):
        return len(self.img_pathes) * 2

    def __getitem__(self, idx):
        if idx % 2 == 0:
            sample = self.img_pathes[int(idx/2)]
            gt_image = _read_image(sample[0], cv2.IMREAD_UNCHANGED, "source")
            gt_binary_image = _read_image(sample[1], cv2.IMREAD_UNCHANGED, "binary")
            gt_instance = _read_image(sample[2], cv2.IMREAD_UNCHANGED, "instance")
        else:
            sample = self.img_pathes[int((idx-1)/2)]
            gt_image = _read_image(sample[0], cv2.IMREAD_UNCHANGED, "source")
            gt_binary_image = _read_image(sample[1], cv2.IMREAD_UNCHANGED, "binary")
            gt_instance = _read_image(sample[2], cv2.IMREAD_UNCHANGED, "instance")

            gt_image = cv2.flip(gt_image, 1)
            gt_binary_image = cv2.flip(gt_binary_image, 1)
            gt_instance = cv2.flip(gt_instance, 1)

        gt_image = cv2.resize(gt_image, dsize=self.resize, interpolation=cv2.INTER_LINEAR)
        gt_binary_image = cv2.resize(gt_binary_image, dsize=self.resize, interpolation=cv2.INTER_NEAREST)
        gt_instance = cv2.resize(gt_instance, dsize=self.resize, interpolation=cv2.INTER_NEAREST)

        gt_image = gt_image / 127.5 - 1.0
        gt_binary_image = np.array(gt_binary_image / 255.0, dtype=np.uint8)
        gt_binary_image = gt_binary_image[:, :, np.newaxis]
        gt_instance = gt_instance[:, :, np.newaxis]

        gt_binary_image = np.transpose(gt_binary_image, (2, 0, 1))
        gt_instance = np.transpose(gt_instance, (2, 0, 1))

        gt_image = torch.tensor(gt_image, dtype=torch.float)
        gt_image = np.transpose(gt_image, (2, 0, 1))
        # trsf = torchvision.transforms.Normalize(mean=(0.485, 0.456, 0.406), std=(0.229, 0.224, 0.225), inplace=False)
        # gt_image = trsf(gt_image)

        gt_binary_image = torch.tensor(gt_binary_image, dtype=torch.long).view(self.resize[1], self.resize[0])
        # gt_binary_image = torch.tensor(gt_binary_image, dtype=torch.float)
        # gt_instance = torch.tensor(gt_instance, dtype=torch.float)
        gt_instance = torch.tensor(gt_instance, dtype=torch.long).view(self.resize[1], self.resize[0])

        return gt_image, gt_binary_image, gt_instance
