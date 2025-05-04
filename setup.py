import os
import subprocess
import sys
from pathlib import Path
import time

def check_python_version():
    """Check if Python version is 3.8 or higher."""
    if sys.version_info < (3, 8):
        print("Error: Python 3.8 or higher is required")
        sys.exit(1)
    print(f"Python version {sys.version_info.major}.{sys.version_info.minor} detected - OK")

def create_virtual_env():
    """Create and activate virtual environment."""
    venv_path = Path(".venv")
    if not venv_path.exists():
        print("Creating virtual environment...")
        try:
            subprocess.run([sys.executable, "-m", "venv", str(venv_path)], check=True)
            print("Virtual environment created successfully")
        except subprocess.CalledProcessError as e:
            print(f"Error creating virtual environment: {e}")
            sys.exit(1)
    
    # Activate virtual environment
    try:
        if sys.platform == "win32":
            activate_script = venv_path / "Scripts" / "activate.bat"
            subprocess.run([str(activate_script)], shell=True, check=True)
        else:
            activate_script = venv_path / "bin" / "activate"
            subprocess.run(f"source {activate_script}", shell=True, check=True)
        print("Virtual environment activated")
    except subprocess.CalledProcessError as e:
        print(f"Error activating virtual environment: {e}")
        sys.exit(1)

def install_dependencies():
    """Install required dependencies."""
    print("\nInstalling dependencies...")
    max_retries = 3
    for attempt in range(max_retries):
        try:
            subprocess.run([sys.executable, "-m", "pip", "install", "-r", "requirements.txt"], check=True)
            print("Dependencies installed successfully")
            return
        except subprocess.CalledProcessError as e:
            print(f"Attempt {attempt + 1} failed: {e}")
            if attempt < max_retries - 1:
                print("Retrying in 5 seconds...")
                time.sleep(5)
            else:
                print("Failed to install dependencies after multiple attempts")
                sys.exit(1)

def download_models():
    """Download required model files."""
    print("\nDownloading model files...")
    try:
        subprocess.run([sys.executable, "src/utils/download_models.py"], check=True)
        print("Model files downloaded successfully")
    except subprocess.CalledProcessError as e:
        print(f"Error downloading model files: {e}")
        print("Warning: Some model files may be missing. The simulation may not work properly.")

def main():
    """Main setup function."""
    print("Starting setup process...")
    
    try:
        # Check Python version
        check_python_version()
        
        # Create and activate virtual environment
        create_virtual_env()
        
        # Install dependencies
        install_dependencies()
        
        # Download model files
        download_models()
        
        print("\nSetup completed successfully!")
        print("\nTo run the simulation:")
        print("1. Activate the virtual environment:")
        print("   - Windows: .venv\\Scripts\\activate")
        print("   - Linux/Mac: source .venv/bin/activate")
        print("2. Run the simulation: python src/main.py")
        
    except Exception as e:
        print(f"\nSetup failed: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main() 