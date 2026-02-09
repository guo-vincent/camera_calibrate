from typing import Union
import os

# TODO: have scripts use these helpers instead of duplicating code
def ensure_dir(path: Union[str, os.PathLike]) -> None:
    if not os.path.exists(path):
        os.makedirs(path, exist_ok=True)