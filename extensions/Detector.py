# IDEA: Directly implement SORT to the Detector class. Let's not lie to ourselves.
from torch import device as torchdevice, from_numpy, tensor, randn
from numba import njit
import numpy as np
import cv2
from os import environ
import tensorrt as trt
from collections import OrderedDict, namedtuple
from extensions.tools import coco
try:
    from .sort import Sort
except ImportError:
    from sort import Sort
from extensions.logs import logging
environ['CUDA_MODULE_LOADING'] = 'LAZY'
logger = logging.getLogger(__name__)

class Detections:
    def __init__(self, tracker, dets, track=True):
        self.dets = dets
        self.tracker = tracker
        if track:
            self.track()
        self.tracked = track

    def sort(self, min_conf=0.3, sort_axis=1, reverse=False):  # Sort dets depending on a certain list axis index
        sorted_tup = sorted(self.dets, key=lambda x: x[sort_axis], reverse=reverse)  # Sort list by axis

        # Filter by confidence
        if min_conf > 0.01:
            sorted_tup = [item for item in sorted_tup if item[1] > min_conf]
        # Iterate over the tuple
        self.dets = sorted_tup

    def track(self):
        tracks = self.tracker.update(np.array([i[0] for i in self.dets])).astype(int).tolist()  # Feed bboxes to tracker
        # box, id, conf, class
        final_det = [(track[:4], track[4], self.dets[i][1], self.dets[i][2]) for i, track in
                     enumerate(tracks)]  # Modify original bboxes to include ID
        self.dets = final_det

    def draw(self, img, start_det=0, end_det=None):  # Draw Detections
        for det in self.dets[slice(start_det, end_det, None)]:  # Index the desired BBoxes
            if self.tracked:
                bbox, id, conf, cls = det  # unpacks the detections
                text = f"{id}: {coco[cls]}"
            else:
                bbox, conf, cls = det
                text = f"{conf:.3f}: {coco[cls]}"
            cv2.rectangle(img, bbox[:2], bbox[2:], (0, 255, 0), 2)  # draw box
            cv2.putText(img, text, bbox[:2], cv2.FONT_HERSHEY_SIMPLEX, 0.75,  # write id and class
                        (255, 255, 0), thickness=2)


class Detector:
    def __init__(self, detector: str, tracker=None, device=torchdevice('cuda:0'), log=1, filter=None, track=True):
        """
        TensorRT-Powered YoloV7 Object Detector Class with SORT Object Tracking.
        :param detector: Path of the .trt or .engine file
        :param tracker: SORT Object Tracker object to pass onto Detections
        :param device: The device on which the Model should run
        :param log: Log Level (Verbose, Info, Warning)
        :param filter: Makes the Detector return only specific object types (person, car, phone, etc.)
        """
        self.detector = detector
        self.device = device
        self.tracker = tracker if tracker is not None else Sort(60)
        self.filter = filter
        self.track = track
        logs = (trt.Logger.VERBOSE, trt.Logger.INFO, trt.Logger.WARNING)  # tensorrt logger options
        Binding = namedtuple('Binding', ('name', 'dtype', 'shape', 'data', 'ptr'))  # namedtuple for model bindings
        logger = trt.Logger(logs[log if 0<=log<=2 else 2])  # Init logger
        trt.init_libnvinfer_plugins(logger, namespace="")  # Init plugins
        with open(self.detector, 'rb') as f, trt.Runtime(logger) as runtime:
            model = runtime.deserialize_cuda_engine(f.read())  # get engine
        bindings = OrderedDict()

        # Extract and organize model bindings
        for index in range(model.num_bindings):
            name = model.get_tensor_name(index)
            dtype = trt.nptype(model.get_tensor_dtype(name))
            shape = tuple(model.get_tensor_shape(name))
            data = from_numpy(np.empty(shape, dtype=np.dtype(dtype))).to(self.device)
            bindings[name] = Binding(name, dtype, shape, data, int(data.data_ptr()))

        binding_addrs = OrderedDict((n, d.ptr) for n, d in bindings.items())
        context = model.create_execution_context()

        # Warm up
        for _ in range(10):
            tmp = randn(1, 3, 640, 640).to(self.device)
            binding_addrs['images'] = int(tmp.data_ptr())
            context.execute_v2(list(binding_addrs.values()))

        del name, dtype, shape, data, tmp, logs  # Clear some memory

        self.binding_addrs, self.bindings, self.context = binding_addrs, bindings, context
        logger.log("[Detector/Detector] TRT Detector setup successfully.", logging.INFO)

    def _postprocess(self, boxes):  # Normalize bounding box from preprocessed image to actual image
        self.dwdh = tensor(self.dwdh * 2).to(boxes.device)
        boxes -= self.dwdh
        boxes /= self.r
        return boxes.int().tolist()

    def _preprocess(self, img, new_shape=(640, 640), color=(114, 114, 114), auto=True, scaleup=True, stride=32, div=True):  # Adapt image to feed to model
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        im = img.copy()

        ####
        # Resize and pad image while meeting stride-multiple constraints
        shape = im.shape[:2]  # current shape [height, width]
        if isinstance(new_shape, int):
            new_shape = (new_shape, new_shape)

        # Scale ratio (new / old)
        r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])
        if not scaleup:  # only scale down, do not scale up (for better val mAP)
            r = min(r, 1.0)

        # Compute padding
        new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))
        dw, dh = new_shape[1] - new_unpad[0], new_shape[0] - new_unpad[1]  # wh padding

        if auto:
            dw, dh = np.mod(dw, stride), np.mod(dh, stride)  # wh padding

        dw /= 2  # divide padding into 2 sides
        dh /= 2

        if shape[::-1] != new_unpad:  # resize
            im = cv2.resize(im, new_unpad, interpolation=cv2.INTER_LINEAR)
        top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
        left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
        im = cv2.copyMakeBorder(im, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)  # add border
        ####

        im = im.transpose((2, 0, 1))
        im = np.expand_dims(im, 0)
        im = np.ascontiguousarray(im)
        im = im.astype(np.float32)
        im = from_numpy(im).to(self.device)
        if div:
            return im/255, r, (dw, dh)
        else:
            return im

    def __call__(self, frame) -> Detections:  # Inference
        frame, self.r, self.dwdh = self._preprocess(frame, auto=False)  # Process frame

        # Insert image into model
        layers = list(self.binding_addrs.keys())
        self.binding_addrs[layers[0]] = int(frame.data_ptr())
        self.context.execute_v2(list(self.binding_addrs.values()))  # Run model

        nums, boxes, scores, classes = (self.bindings[i].data for i in layers[1:])  # Get outputs
        boxes = self._postprocess(boxes[0, :nums[0][0]])  # Normalize bboxes
        classes = classes[0, :nums[0][0]].tolist()
        scores = [round(i, 4) for i in scores[0, :nums[0][0]].tolist()]

        out = tuple(zip(boxes, scores, classes))
        if nums:
            if self.filter is not None:
                return Detections(self.tracker, tuple(filter(lambda x: x[2] == self.filter, out)), self.track)
            else:
                return Detections(self.tracker, out, self.track)
        else:
            return None

    def get_dets(self, cap, draw=True, end_det=None):
        dets = self(cap)  # Detect
        if dets is not None:
            dets.sort()  # sort detections by ID
            if draw:
                dets.draw(cap, end_det=end_det)
            return dets.dets  # Extract from class
        else:
            return None


def frame_debug(frame, px=25):  # Draw crosshairs
    cy, cx = frame.shape[:2]
    ccx = cx//2
    ccy = cy//2
    cv2.rectangle(frame, (ccx-px, ccy-px), (ccx+px, ccy+px), (255, 0, 0), 2, cv2.LINE_AA)

    cv2.line(frame, (0, ccy), (ccx-px, ccy), (0, 0, 255), 2, cv2.LINE_AA)  # left line
    cv2.line(frame, (ccx, 0), (ccx, ccy-px), (0, 0, 255), 2, cv2.LINE_AA)  # upper line
    cv2.line(frame, (ccx+px, ccy), (cx, ccy), (0, 0, 255), 2, cv2.LINE_AA)  # right line
    cv2.line(frame, (ccx, ccy+px), (ccx, cy), (0, 0, 255), 2, cv2.LINE_AA)  # lower line


if __name__ == "__main__":
    from FastCam import Camera
    from time import time
    model = Detector("../Resources/yolov7-tiny-nms.trt", log=2, track=False)

    cam = Camera().start()
    frame = cam.read()

    while True:
        start = time()
        frame = cam.read()
        dets = model(frame)
        if dets is not None:
            dets.draw(frame)
        print(f"FPS: {1/(time()-start)}")
        cv2.imshow("hi", frame)
        cv2.waitKey(1)
