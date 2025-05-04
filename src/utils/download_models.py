import os
import requests
from tqdm import tqdm
import time

def download_file(url, filename):
    """Download a file from URL to filename with progress bar."""
    print(f"Downloading {filename}...")
    
    # Create directory if it doesn't exist
    os.makedirs(os.path.dirname(filename), exist_ok=True)
    
    # Try to download with retries
    max_retries = 3
    for attempt in range(max_retries):
        try:
            response = requests.get(url, stream=True)
            response.raise_for_status()
            
            # Get file size
            total_size = int(response.headers.get('content-length', 0))
            
            # Download with progress bar
            with open(filename, 'wb') as f, tqdm(
                desc=os.path.basename(filename),
                total=total_size,
                unit='iB',
                unit_scale=True,
                unit_divisor=1024,
            ) as pbar:
                for data in response.iter_content(chunk_size=1024):
                    size = f.write(data)
                    pbar.update(size)
            
            print(f"Successfully downloaded {filename}")
            return True
            
        except requests.exceptions.RequestException as e:
            print(f"Attempt {attempt + 1} failed: {str(e)}")
            if attempt < max_retries - 1:
                print("Retrying in 5 seconds...")
                time.sleep(5)
            else:
                print(f"Failed to download {filename} after {max_retries} attempts")
                return False

def download_models():
    """Download required model files."""
    # Create models directory if it doesn't exist
    os.makedirs("models", exist_ok=True)
    
    # YOLO model files with alternative URLs
    yolo_files = {
        "yolov3.weights": [
            "https://pjreddie.com/media/files/yolov3.weights",
            "https://github.com/pjreddie/darknet/raw/master/cfg/yolov3.weights"
        ],
        "yolov3.cfg": [
            "https://raw.githubusercontent.com/pjreddie/darknet/master/cfg/yolov3.cfg",
            "https://github.com/pjreddie/darknet/raw/master/cfg/yolov3.cfg"
        ],
        "coco.names": [
            "https://raw.githubusercontent.com/pjreddie/darknet/master/data/coco.names",
            "https://github.com/pjreddie/darknet/raw/master/data/coco.names"
        ]
    }
    
    # Download YOLO files
    for filename, urls in yolo_files.items():
        if not os.path.exists(f"models/{filename}"):
            success = False
            for url in urls:
                if download_file(url, f"models/{filename}"):
                    success = True
                    break
            if not success:
                print(f"Warning: Could not download {filename} from any source")

if __name__ == "__main__":
    download_models() 