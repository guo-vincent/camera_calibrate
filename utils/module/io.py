# --------------------------------------------------------------------------
# Helper: utils/io.py (small helpers used by scripts)
# --------------------------------------------------------------------------
"""
File: utils/io.py
Purpose: Small helper utilities for reading/writing OpenCV FileStorage YAML files.
"""
import cv2
from typing import Union
from cv2.typing import MatLike
import os

def write_yaml_matrix(path: Union[str, os.PathLike], name: str, mat: MatLike) -> None:
    fs = cv2.FileStorage(path, cv2.FILE_STORAGE_WRITE)
    fs.write(name, mat)
    fs.release()

def read_yaml_matrix(path: Union[str, os.PathLike], name: str) -> MatLike:
    fs = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)
    if not fs.isOpened():
        raise RuntimeError(f"Failed to open {path}")
    mat = fs.getNode(name).mat()
    fs.release()
    return mat

# TODO: refactor scripts to use this helper instead of duplicating code
def load_yaml_matrix(path: Union[str, os.PathLike], name: str) -> MatLike:
    fs = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)
    if not fs.isOpened():
        raise RuntimeError(f"Failed to open {path}")
    mat = fs.getNode(name).mat()
    fs.release()
    if mat is None:
        raise RuntimeError(f"Node '{name}' not found in {path}")
    return mat