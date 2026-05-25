import subprocess
import time
import sys

def check_topic(topic):
    print(f"Checking {topic}...", end="", flush=True)
    try:
        # Try to get 1 message with a 2-second timeout
        res = subprocess.check_output(['gz', 'topic', '-e', '-t', topic, '-n', '1'], timeout=3.0)
        if res:
            print(" ✅ DATA RECEIVED")
            return True
    except subprocess.TimeoutExpired:
        print(" ❌ TIMEOUT (No data)")
    except Exception as e:
        print(f" ❌ ERROR ({e})")
    return False

def main():
    print("--- ROBOVERSE SENSOR DIAGNOSTIC ---")
    
    # 1. Discover active topics
    try:
        all_topics = subprocess.check_output(['gz', 'topic', '-l']).decode().split()
    except:
        print("CRITICAL: Could not connect to Gazebo. Is it running?")
        return

    # 2. Pick the best topics to check
    camera_topic = next((t for t in all_topics if "IMX214/image" in t), None)
    depth_topic = "/depth_camera" if "/depth_camera" in all_topics else next((t for t in all_topics if "depth" in t), None)
    lidar_topic = next((t for t in all_topics if "lidar" in t and "scan" in t), None)
    odom_topic = next((t for t in all_topics if "odometry" in t), None)

    topics_to_check = {
        "Camera (RGB)": camera_topic,
        "Depth (Obstacles)": depth_topic,
        "Lidar": lidar_topic,
        "Odometry (Position)": odom_topic
    }
    
    print(f"Discovered topics: {list(topics_to_check.values())}")
    
    results = {}
    for label, t in topics_to_check.items():
        if t:
            results[label] = check_topic(t)
        else:
            print(f"Checking {label}... ❌ NOT FOUND IN TOPIC LIST")
            results[label] = False
    
    print("\n--- SUMMARY ---")
    if not results.get("Camera (RGB)"):
        print("WARNING: Camera not detected or silent. Target detection will fail.")
    if not results.get("Depth (Obstacles)"):
        print("CRITICAL: No depth sensor found. THE DRONE IS BLIND TO WALLS.")
    
    if all(results.values()):
        print("✅ ALL SYSTEMS GO. You are ready to fly!")
    else:
        print("⚠️ Some sensors are missing. Obstacle avoidance may be unreliable.")

if __name__ == "__main__":
    main()
