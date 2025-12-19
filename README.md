# TUSS4470-arduino-shield

# Echo Sounder Interface Setup Guide

This guide provides step-by-step instructions to set up the environment for the Echo Sounder display interface.

## 1. Environment Preparation
If you have previously installed the dark theme module, please remove it first to avoid conflicts.

```powershell
# Remove old version
pip uninstall -y pyqtdarktheme

2. Virtual Environment Management
Check your current directory for existing environments, then create and activate a new one.

# Check existing folders
ls

# Create the virtual environment
python -m venv venv

# Activate the virtual environment
.\venv\Scripts\Activate.ps1

Note: If you get an error saying "scripts are disabled", run this command: Set-ExecutionPolicy -Scope CurrentUser -ExecutionPolicy RemoteSigned

3. Dependency Installation
Ensure you have a requirements.txt file in your folder with these contents:

numpy

pyserial==3.4

PyQt5

pyqtgraph

pyqtdarktheme

Run the following commands to install everything correctly:

# Install standard dependencies
pip install -r requirements.txt

# Force install specific dark theme version
pip install pyqtdarktheme==2.1.0 --ignore-requires-python

4. Run Application
Execute the main interface script:

python .\echo_interface.py

Project Requirements Summary: | Package | Version | | :--- | :--- | | numpy | latest | | pyserial | 3.4 | | PyQt5 | latest | | pyqtgraph | latest | | pyqtdarktheme | 2.1.0 |
