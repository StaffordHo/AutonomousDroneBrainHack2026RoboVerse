import cv2
import numpy as np

def is_inside_large_decorative_region(bbox, color_mask):
    """
    Reject small detections that sit on top of large decorative red/yellow barrels.
    """
    x, y, w, h = bbox
    image_h, image_w = color_mask.shape[:2]
    
    # Expand search window to check surrounding color density
    pad = 50
    x1, y1 = max(0, x - pad), max(0, y - pad)
    x2, y2 = min(image_w, x + w + pad), min(image_h, y + h + pad)
    
    local_mask = color_mask[y1:y2, x1:x2]
    color_pixels = cv2.countNonZero(local_mask)
    local_area = max((x2 - x1) * (y2 - y1), 1)
    
    # If more than 15% of the surrounding area is the same color, 
    # it's likely a decorative barrel, not a standalone canister.
    return (color_pixels / local_area) > 0.18

def is_standalone_candidate(bbox, contour_area, color_mask):
    """
    Ensures the detection is a standalone object, not a patch on a wall or ladder.
    """
    x, y, w, h = bbox
    image_h, image_w = color_mask.shape[:2]
    
    pad = 30
    x1, y1 = max(0, x - pad), max(0, y - pad)
    x2, y2 = min(image_w, x + w + pad), min(image_h, y + h + pad)
    
    local = color_mask[y1:y2, x1:x2]
    contours, _ = cv2.findContours(local, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if not contours: return False
    
    largest_local_area = max(cv2.contourArea(c) for c in contours)
    # If the surrounding color mass is 8x bigger than our candidate, reject it.
    if largest_local_area > contour_area * 8:
        return False
    return True

def detect_small_fuel_barrels(frame):
    if frame is None: return [], None, None, []
    
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    img_h, img_w = frame.shape[:2]
    
    # --- MASKS ---
    # Red (two ranges for wrap-around)
    lower_red1 = np.array([0, 150, 50])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 150, 50])
    upper_red2 = np.array([180, 255, 255])
    red_mask = cv2.bitwise_or(cv2.inRange(hsv, lower_red1, upper_red1),
                              cv2.inRange(hsv, lower_red2, upper_red2))
    
    # Yellow
    lower_yellow = np.array([20, 100, 100])
    upper_yellow = np.array([35, 255, 255])
    yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    
    detections = []
    
    for color_name, mask in [("red", red_mask), ("yellow", yellow_mask)]:
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if 50 < area < 5000: # Small fuel canister size range
                x, y, w, h = cv2.boundingRect(cnt)
                aspect_ratio = w / float(h)
                
                # Filter 1: Aspect Ratio (should be vertical-ish)
                if not (0.2 < aspect_ratio < 1.5): continue
                
                # Filter 2: Height-based Context
                if color_name == "red":
                    if y > img_h * 0.70: continue # Red canisters are usually elevated
                if color_name == "yellow":
                    if y + h > img_h * 0.95 and h < 50: continue # Ignore ground-rust noise
                
                # Filter 3: Standalone & Decoration Rejection
                if is_inside_large_decorative_region((x,y,w,h), mask): continue
                if not is_standalone_candidate((x,y,w,h), area, mask): continue
                
                detections.append({
                    "colour": color_name,
                    "bbox": (x, y, w, h),
                    "center": (x + w // 2, y + h // 2),
                    "area": area
                })
                
    return detections, yellow_mask, red_mask, detections
