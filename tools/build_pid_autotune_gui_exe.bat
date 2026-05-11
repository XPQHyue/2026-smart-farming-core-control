@echo off
setlocal

cd /d "%~dp0.."

echo [1/3] Installing packaging dependency...
python -m pip install -r tools\requirements_packaging.txt
if errorlevel 1 goto :error

echo [2/3] Building Windows executable...
python -m PyInstaller --noconfirm --clean --windowed --onedir --name PID_Autotune_GUI ^
	--hidden-import pyqtgraph ^
	--exclude-module PyQt5 --exclude-module PyQt6 --exclude-module PySide2 ^
	--exclude-module pyqtgraph.examples --exclude-module pyqtgraph.opengl ^
	--exclude-module matplotlib --exclude-module pandas --exclude-module scipy ^
	--exclude-module sklearn --exclude-module skimage --exclude-module PIL ^
	--exclude-module cv2 --exclude-module imageio --exclude-module moviepy ^
	--exclude-module torch --exclude-module torchvision --exclude-module torchaudio ^
	--exclude-module onnxruntime --exclude-module tensorflow --exclude-module jupyter ^
	--exclude-module IPython --exclude-module openpyxl --exclude-module lxml ^
	tools\pid_autotune_gui.py
if errorlevel 1 goto :error

echo [2.5/3] Building CLI helper executable...
python -m PyInstaller --noconfirm --clean --onefile --console --name PID_Autotune_CLI ^
	--exclude-module PyQt5 --exclude-module PyQt6 --exclude-module PySide2 --exclude-module PySide6 ^
	--exclude-module matplotlib --exclude-module pandas --exclude-module scipy ^
	--exclude-module sklearn --exclude-module skimage --exclude-module PIL ^
	--exclude-module cv2 --exclude-module imageio --exclude-module moviepy ^
	--exclude-module torch --exclude-module torchvision --exclude-module torchaudio ^
	--exclude-module onnxruntime --exclude-module tensorflow --exclude-module jupyter ^
	--exclude-module IPython --exclude-module openpyxl --exclude-module lxml ^
	tools\pid_autotune_serial.py
if errorlevel 1 goto :error

copy /Y dist\PID_Autotune_CLI.exe dist\PID_Autotune_GUI\PID_Autotune_CLI.exe >nul
if errorlevel 1 goto :error

echo [3/3] Done.
echo Output folder: dist\PID_Autotune_GUI
echo Main executable: dist\PID_Autotune_GUI\PID_Autotune_GUI.exe
echo CLI helper: dist\PID_Autotune_GUI\PID_Autotune_CLI.exe
exit /b 0

:error
echo Build failed.
exit /b 1
