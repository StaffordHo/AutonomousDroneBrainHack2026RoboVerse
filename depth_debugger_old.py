import cv2
import numpy as np
import os
from datetime import datetime

def save_depth_debug(depth_map, label="impact"):
    """Saves a visual representation of the depth map for debugging."""
    if depth_map is None: return
    
    # Ensure directory exists
    os.makedirs("competition_evidence", exist_ok=True)
    
    # 1. Normalize depth (0-10m) to 0-255 grayscale
    # We cap at 10m for better contrast
    display_depth = np.clip(depth_map, 0, 10)
    display_depth = (display_depth / 10.0 * 255).astype(np.uint8)
    
    # 2. Colorize for better visibility (JET colormap)
    # Blue = Far, Red = Very Close
    color_depth = cv2.applyColorMap(255 - display_depth, cv2.COLORMAP_JET)
    
    # 3. Add metadata
    timestamp = datetime.now().strftime("%H%M%S")
    filename = f"competition_evidence/depth_{label}_{timestamp}.png"
    
    # Annotate with center distance
    h, w = depth_map.shape
    center_dist = depth_map[h//2, w//2]
    cv2.putText(color_depth, f"Center Dist: {center_dist:.2f}m", (20, 40), 
                cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    
    cv2.imwrite(filename, color_depth)
    return filename
