# -*- mode: python ; coding: utf-8 -*-


a = Analysis(
    ['tools\\pid_autotune_gui.py'],
    pathex=[],
    binaries=[],
    datas=[],
    hiddenimports=['pyqtgraph'],
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=['PyQt5', 'PyQt6', 'PySide2', 'pyqtgraph.examples', 'pyqtgraph.opengl', 'matplotlib', 'pandas', 'scipy', 'sklearn', 'skimage', 'PIL', 'cv2', 'imageio', 'moviepy', 'torch', 'torchvision', 'torchaudio', 'onnxruntime', 'tensorflow', 'jupyter', 'IPython', 'openpyxl', 'lxml'],
    noarchive=False,
    optimize=0,
)
pyz = PYZ(a.pure)

exe = EXE(
    pyz,
    a.scripts,
    [],
    exclude_binaries=True,
    name='PID_Autotune_GUI',
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=True,
    console=False,
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
)
coll = COLLECT(
    exe,
    a.binaries,
    a.datas,
    strip=False,
    upx=True,
    upx_exclude=[],
    name='PID_Autotune_GUI',
)
