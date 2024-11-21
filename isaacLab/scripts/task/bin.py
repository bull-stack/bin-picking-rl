from dataclasses import dataclass
from omni.isaac.core.prims import XFormPrim

@dataclass
class Bin:
    prim_path: str
    bin_prim: XFormPrim
    occupied: bool
    