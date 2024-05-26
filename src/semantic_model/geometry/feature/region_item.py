from src.common.types import *
from src.semantic_model.geometry.element.point import Vertex
from .bound_box import BoundBox


class RegionItem(BoundBox):
    def within_region(self, point: 'Vertex', tolerance: Number = 0) -> bool:
        raise NotImplementedError

