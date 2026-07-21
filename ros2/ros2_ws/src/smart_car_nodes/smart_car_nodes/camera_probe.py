import time


def _to_int(value, default):
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def candidate_camera_indices(preferred_index=0, max_camera_index=5):
    preferred_index = _to_int(preferred_index, 0)
    max_camera_index = max(0, _to_int(max_camera_index, 5))
    candidates = []
    if preferred_index >= 0:
        candidates.append(preferred_index)
    for index in range(max_camera_index + 1):
        if index not in candidates:
            candidates.append(index)
    return candidates


def _open_capture(cap, camera_index, cv2_module):
    try:
        opened = cap.open(camera_index, apiPreference=getattr(cv2_module, "CAP_V4L2", 0))
    except TypeError:
        opened = cap.open(camera_index)
    if not opened and hasattr(cap, "isOpened"):
        opened = cap.isOpened()
    return bool(opened)


def open_camera(camera_info=None, cv2_module=None, logger=None):
    camera_info = camera_info or {}
    if cv2_module is None:
        import cv2

        cv2_module = cv2

    width = _to_int(camera_info.get("width"), 640)
    height = _to_int(camera_info.get("height"), 480)
    fps = _to_int(camera_info.get("fps"), 30)
    preferred_index = camera_info.get("camera", camera_info.get("camera_index", 0))
    max_camera_index = camera_info.get("max_camera_index", 5)
    read_attempts = _to_int(camera_info.get("read_attempts"), 3)
    read_retry_interval = float(camera_info.get("read_retry_interval", 0.05))

    candidates = candidate_camera_indices(preferred_index, max_camera_index)
    for camera_index in candidates:
        cap = cv2_module.VideoCapture()
        if not _open_capture(cap, camera_index, cv2_module):
            cap.release()
            continue

        fourcc_func = getattr(cv2_module, "VideoWriter_fourcc", None)
        fourcc_prop = getattr(cv2_module, "CAP_PROP_FOURCC", None)
        if fourcc_func is not None and fourcc_prop is not None:
            cap.set(fourcc_prop, fourcc_func("M", "J", "P", "G"))
        cap.set(getattr(cv2_module, "CAP_PROP_FRAME_WIDTH"), width)
        cap.set(getattr(cv2_module, "CAP_PROP_FRAME_HEIGHT"), height)
        cap.set(getattr(cv2_module, "CAP_PROP_FPS"), fps)

        for _ in range(max(1, read_attempts)):
            ret, frame = cap.read()
            if ret and frame is not None:
                if logger is not None:
                    logger.info(f"Camera opened on index {camera_index}.")
                return cap, camera_index
            if read_retry_interval > 0:
                time.sleep(read_retry_interval)

        if logger is not None:
            logger.warning(f"Camera index {camera_index} opened but did not return frames.")
        cap.release()

    raise RuntimeError(f"No readable camera found. Tried camera indices: {candidates}")
