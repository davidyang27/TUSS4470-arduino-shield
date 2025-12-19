# TUSS4470 Arduino Shield
## Echo Sounder Interface Setup Guide

This repository contains an **Echo Sounder interface** using a **TUSS4470 Arduino shield** and a **Python-based waterfall / echogram display**.

This document describes how to set up the Python environment and run the display interface.

---

## 1. Environment Preparation

If you have previously installed a dark theme module, please remove it first to avoid conflicts.

```powershell
pip uninstall -y pyqtdarktheme

```
## 2. Virtual Environment Management

Check your current directory for existing folders, then create and activate a new virtual environment.
```powershell
# List current directory
ls

# Create virtual environment
python -m venv venv


# Activate virtual environment (Windows PowerShell)
.\venv\Scripts\Activate.ps1
```

PowerShell Execution Policy

If you see the error:

> running scripts is disabled on this system

Run this command once:
```powershell
Set-ExecutionPolicy -Scope CurrentUser -ExecutionPolicy RemoteSigned
```
## 3. Dependency Installation

Install dependencies:
```powershell
pip install -r requirements.txt
```
Force install the supported dark theme version:
```powershell
pip install pyqtdarktheme==2.1.0 --ignore-requires-python
```
## 4. Run the Application

Start the Echo Sounder interface:
```powershell
python echo_interface.py
```





