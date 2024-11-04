"""
utils for calling SAM2 image segmentation

Author: Elaina
"""
import torch
import numpy as np
from sam2.build_sam import build_sam2
from sam2.sam2_image_predictor import SAM2ImagePredictor

class SAM:
    def __init__(self, config):
        self.config = config
        self.predictor = SAM2ImagePredictor(build_sam2(self.config['cfg'], self.config['checkpoint']))
        
    def infer(self, img):
        img = img.numpy() if isinstance(img, torch.Tensor) else img
        with torch.inference_mode(), torch.autocast("cuda", dtype=torch.bfloat16):
            self.predictor.set_image(img)
            masks, scores, _ = self.predictor.predict()
            masks = masks[np.argsort(scores)[::-1]]
            return masks, self.binary2category(masks)

    def binary2category(self, masks):
        seg = np.zeros(masks.shape[-2:], np.uint8)
        occupancy = np.zeros(masks.shape[-2:], np.uint8)
        for i, m in enumerate(masks.astype(np.uint8)):
            if (m * occupancy).sum() / m.sum() > 0.15:
                continue
            m[occupancy] = 0
            seg[m] = i + 1
            occupancy[m] = 1
        return seg
    