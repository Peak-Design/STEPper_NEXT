# Helpers bridging pythonocc-core conveniences that do not exist in the
# OCP (pybind11) bindings.

from OCP.TCollection import TCollection_AsciiString
from OCP.TDataStd import TDataStd_Name
from OCP.TDF import TDF_Tool


def get_label_name(label):
    """Replacement for pythonocc's TDF_Label.GetLabelName() SWIG extension."""
    n = TDataStd_Name()
    if label.FindAttribute(TDataStd_Name.GetID_s(), n):
        return TCollection_AsciiString(n.Get(), "?").ToCString()
    return ""


def label_entry(label):
    """Replacement for pythonocc's TDF_Label.EntryDumpToString()."""
    a = TCollection_AsciiString()
    TDF_Tool.Entry_s(label, a)
    return a.ToCString()


class ShapeKey:
    """Dict/set key wrapper restoring pythonocc's TopoDS_Shape semantics.

    OCP's TopoDS_Shape.__hash__ is content-based but __eq__ is wrapper
    identity, so two wrappers of the same shape hash alike yet compare
    unequal, silently breaking every shape-keyed container.
    """

    __slots__ = ("shape", "_h")

    def __init__(self, shape):
        self.shape = shape
        self._h = hash(shape)

    def __hash__(self):
        return self._h

    def __eq__(self, other):
        if not isinstance(other, ShapeKey):
            return NotImplemented
        return self.shape.IsEqual(other.shape)
