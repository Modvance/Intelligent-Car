# FILE: src/models/quickLF.py (REVISED)

import cv2
import sys
import os
import numpy as np
import time

try:
    parent_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
    sys.path.append(parent_dir)
    from src.models.lfnet import LFNet
except ImportError as e:
    print(f"Failed to import LFNet model handler: {e}")
    sys.exit(1)

class LFModel:
    """
    Inference engine that returns lane parameters along with a confidence score.
    """
    def __init__(self, model_path: str):
        self.model_width, self.model_height = (512, 256)
        try:
            self.model_handler = LFNet(model_path)
            print(f"Successfully loaded OM model: {model_path}")
        except Exception as e:
            raise RuntimeError(f"Failed to load model: {e}")
    
    # def _extract_lane_coordinates(self, binary_pred, instance_pred, min_pixels=100):
    #     num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(
    #         binary_pred.astype(np.uint8), connectivity=4
    #     )
    #     lane_coordinates = []
    #     for label in range(1, num_labels):
    #         if stats[label, cv2.CC_STAT_AREA] < min_pixels:
    #             continue
    #         ys, xs = np.where(labels == label)
    #         lane_coordinates.append(list(zip(xs, ys)))
    #     print(f"Extracted {len(lane_coordinates)} lane coordinates with min_pixels={min_pixels}")
    #     return lane_coordinates

    def _extract_lane_coordinates_and_fit(self, binary_pred, min_pixels=1000):
        """
        提取车道线并拟合垂直直线 x = ky + b，返回 [(k, b, confidence), ...]
        """
        num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(
            binary_pred.astype(np.uint8), connectivity=4
        )
        lane_results = []
        for label in range(1, num_labels):
            if stats[label, cv2.CC_STAT_AREA] < min_pixels:
                continue
            ys, xs = np.where(labels == label)
            if len(xs) < 2:
                continue
            A = np.vstack([ys, np.ones(len(ys))]).T
            try:
                k, b = np.linalg.lstsq(A, xs, rcond=None)[0]
                confidence = len(xs)
                lane_results.append((k, b, confidence))
            except np.linalg.LinAlgError:
                continue
        return lane_results


    def pred(self, image: np.ndarray) -> tuple:
        """
        Performs inference and returns raw results with confidence.

        Args:
            image (np.ndarray): Input image of any size.

        Returns:
            tuple: (lane_results, inference_time_ms)
                   - lane_results: A list of (k, b, confidence) tuples.
                                   Confidence is the number of pixels in the lane.
                   - inference_time_ms: The inference time in milliseconds.
        """
        total_start = time.perf_counter()
        model_start = time.perf_counter()
        try:
            om_outputs = self.model_handler.infer(image)
        except Exception as e:
            print(f"An error occurred during model inference: {e}")
            om_outputs = None

        inference_time_ms = (time.perf_counter() - model_start) * 1000

        if om_outputs is None or len(om_outputs) == 0:
            self.last_profile = {
                'model_ms': inference_time_ms,
                'postprocess_ms': 0.0,
                'total_ms': (time.perf_counter() - total_start) * 1000,
            }
            return [], inference_time_ms

        binary_logits = om_outputs[0]
        binary_mask = np.argmax(binary_logits, axis=1).squeeze(0).astype(np.uint8)
        
        # lane_coordinates = self._extract_lane_coordinates(binary_mask, None, min_pixels=1000)
    
        # lane_results = []
        # for coords in lane_coordinates:
        #     if len(coords) < 2:
        #         continue
        #     xs, ys = zip(*coords)
        #     xs = np.array(xs)
        #     ys = np.array(ys)

        #     A = np.vstack([ys, np.ones(len(ys))]).T
        #     try:
        #         k_model, b_model = np.linalg.lstsq(A, xs, rcond=None)[0]
        #         confidence = len(coords)  
        #         lane_results.append((k_model, b_model, confidence))
        #     except np.linalg.LinAlgError:
        #         continue
        lane_results = self._extract_lane_coordinates_and_fit(binary_mask, min_pixels=500)
        self.last_profile = {
            'model_ms': inference_time_ms,
            'postprocess_ms': (time.perf_counter() - model_start) * 1000 - inference_time_ms,
            'total_ms': (time.perf_counter() - total_start) * 1000,
        }
        return lane_results, inference_time_ms
