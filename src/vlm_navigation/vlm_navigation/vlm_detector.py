#!/usr/bin/env python3
"""Hybrid open-vocabulary object detector.

A small local vision-language model (YOLO-World, ``yolov8s-world.pt``) does the
semantic find: given a free-form phrase like "the red box", it decides whether
the target class is present in an RGB frame and roughly where. Because it only
fires at low confidence on small flat-shaded cubes, a cheap HSV pass then refines
the exact pixel centroid inside the VLM's region of interest. The VLM directs;
HSV anchors. CPU-only.
"""

import threading

import numpy as np
import cv2


TARGET_VOCAB = {
    'red':   {'prompts': ['red box', 'red cube', 'red block'],   'color': 'red'},
    'green': {'prompts': ['green box', 'green cube', 'green block'], 'color': 'green'},
    'blue':  {'prompts': ['blue box', 'blue cube', 'blue block'],  'color': 'blue'},
}

ALIASES = {
    'red cube': 'red', 'red box': 'red', 'red block': 'red', 'red': 'red',
    'green cube': 'green', 'green box': 'green', 'green block': 'green',
    'green': 'green',
    'blue cube': 'blue', 'blue box': 'blue', 'blue block': 'blue',
    'blue': 'blue',
}

HSV_RANGES = {
    'red':   [((0, 120, 70), (10, 255, 255)),
              ((170, 120, 70), (179, 255, 255))],
    'green': [((40, 80, 50), (85, 255, 255))],
    'blue':  [((100, 120, 50), (130, 255, 255))],
}

VLM_CONF = 0.03
MIN_ANCHOR_PX = 80


def resolve_target(phrase: str) -> str:
    """Map a free-form phrase to a color key, or raise KeyError."""
    key = phrase.strip().lower()
    if key in TARGET_VOCAB:
        return key
    if key in ALIASES:
        return ALIASES[key]
    for color in TARGET_VOCAB:
        if color in key:
            return color
    raise KeyError(phrase)


class Detection:
    """A confirmed sighting of the target in one RGB frame."""

    def __init__(self, color, cx, cy, area_px, vlm_conf, bbox):
        self.color = color
        self.cx = cx
        self.cy = cy
        self.area_px = area_px
        self.vlm_conf = vlm_conf
        self.bbox = bbox

    def __repr__(self):
        return (f'Detection({self.color}, px=({self.cx},{self.cy}), '
                f'area={self.area_px}, vlm_conf={self.vlm_conf:.3f})')


class VLMDetector:
    """YOLO-World (open-vocab) + HSV anchor. Thread-safe lazy model load."""

    def __init__(self, weights='yolov8s-world.pt', threads=0):
        self._weights = weights
        self._threads = threads
        self._model = None
        self._classes = None
        self._prompt_color = {}
        self._lock = threading.Lock()

    def _ensure_model(self, target_key):
        """Load YOLO-World once and set the class list for every known color."""
        with self._lock:
            if self._model is None:
                import torch
                if self._threads:
                    torch.set_num_threads(self._threads)
                from ultralytics import YOLO
                self._model = YOLO(self._weights)
                prompts, mapping = [], {}
                for color, spec in TARGET_VOCAB.items():
                    for p in spec['prompts']:
                        prompts.append(p)
                        mapping[p] = color
                self._model.set_classes(prompts)
                self._classes = prompts
                self._prompt_color = mapping
            return self._model

    def detect(self, bgr, target_key):
        """Return a Detection for ``target_key`` in this BGR frame, or None.

        The VLM proposes boxes for the target color (best first); inside the best
        box's padded ROI, the HSV anchor gives the precise pixel centroid.
        """
        model = self._ensure_model(target_key)
        want_color = TARGET_VOCAB[target_key]['color']

        res = model.predict(bgr, conf=VLM_CONF, verbose=False)[0]
        names = res.names

        cand = []
        for b in res.boxes:
            prompt = names[int(b.cls)]
            if self._prompt_color.get(prompt) != want_color:
                continue
            conf = float(b.conf)
            x0, y0, x1, y1 = (int(v) for v in b.xyxy[0].tolist())
            cand.append((conf, (x0, y0, x1, y1)))
        cand.sort(reverse=True)

        h, w = bgr.shape[:2]
        for conf, (x0, y0, x1, y1) in cand:
            pad = 12
            rx0 = max(0, x0 - pad); ry0 = max(0, y0 - pad)
            rx1 = min(w, x1 + pad); ry1 = min(h, y1 + pad)
            anchor = self._hsv_anchor(bgr, want_color, (rx0, ry0, rx1, ry1))
            if anchor is not None:
                cx, cy, area = anchor
                return Detection(want_color, cx, cy, area, conf,
                                 (x0, y0, x1, y1))
        return None

    def _hsv_anchor(self, bgr, color, roi):
        """HSV centroid of ``color`` inside ROI (x0,y0,x1,y1), or None."""
        x0, y0, x1, y1 = roi
        crop = bgr[y0:y1, x0:x1]
        if crop.size == 0:
            return None
        hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
        mask = np.zeros(hsv.shape[:2], np.uint8)
        for lo, hi in HSV_RANGES[color]:
            mask |= cv2.inRange(hsv, np.array(lo), np.array(hi))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        area = int(np.count_nonzero(mask))
        if area < MIN_ANCHOR_PX:
            return None
        ys, xs = np.nonzero(mask)
        cx = x0 + float(xs.mean())
        cy = y0 + float(ys.mean())
        return int(round(cx)), int(round(cy)), area
