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

# List current directory
ls

# Create virtual environment
python -m venv venv

# Activate virtual environment (Windows PowerShell)
.\venv\Scripts\Activate.ps1
