import pytest
import os
from franca2ros.translator.translator import FIDLTranslator

# ---------------------------------------------------------------------------
# Inline FIDL fixtures
# ---------------------------------------------------------------------------

BASIC_FIDL = """\
package test

typeCollection common {
    version {major 0 minor 1}

    struct Position {
        Double latitude
        Double longitude
    }
}
"""

MULTI_STRUCT_FIDL = """\
package test

typeCollection common {
    version {major 0 minor 1}

    struct Dop {
        Float hdop
        Float vdop
        Float pdop
    }

    struct Position {
        UInt32 satellites_visible
        UInt32 satellites_used
        Double latitude
        Double longitude
    }
}
"""

METHOD_FIDL = """\
package calculator

interface Calculator {
    version {major 0 minor 1}

    method add {
        in {
            Int32 a
            Int32 b
        }
        out {
            Int32 result
        }
    }
}
"""

MALFORMED_FIDL = """\
this is not valid fidl content {{{
"""

ALL_TYPES_FIDL = """\
package types

typeCollection all {
    version {major 0 minor 1}

    struct AllTypes {
        Float f
        Double d
        Int8 i8
        Int16 i16
        Int32 i32
        Int64 i64
        UInt8 u8
        UInt16 u16
        UInt32 u32
        UInt64 u64
        Boolean b
        String s
        ByteBuffer buf
    }
}
"""


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def make_translator(tmp_path, content, filename="test.fidl"):
    fidl_file = tmp_path / filename
    fidl_file.write_text(content)
    return FIDLTranslator(str(fidl_file), None)


# ---------------------------------------------------------------------------
# Type-mapping unit tests
# ---------------------------------------------------------------------------

class TestFidlTypes2RosTypes:

    def _translator(self, tmp_path):
        # Need an initialized instance; __init__ loads templates from the
        # installed package, so we pass any existing file path.
        fidl_file = tmp_path / "dummy.fidl"
        fidl_file.write_text("package p\n")
        return FIDLTranslator(str(fidl_file), None)

    def test_float_maps_to_float32(self, tmp_path):
        assert self._translator(tmp_path).fidlTypes2RosTypes("Float") == "float32"

    def test_double_maps_to_float64(self, tmp_path):
        assert self._translator(tmp_path).fidlTypes2RosTypes("Double") == "float64"

    def test_uint32_maps_to_uint32(self, tmp_path):
        assert self._translator(tmp_path).fidlTypes2RosTypes("UInt32") == "uint32"

    def test_int32_maps_to_int32(self, tmp_path):
        assert self._translator(tmp_path).fidlTypes2RosTypes("Int32") == "int32"

    def test_int8_maps_to_int8(self, tmp_path):
        assert self._translator(tmp_path).fidlTypes2RosTypes("Int8") == "int8"

    def test_int16_maps_to_int16(self, tmp_path):
        assert self._translator(tmp_path).fidlTypes2RosTypes("Int16") == "int16"

    def test_int64_maps_to_int64(self, tmp_path):
        assert self._translator(tmp_path).fidlTypes2RosTypes("Int64") == "int64"

    def test_uint8_maps_to_uint8(self, tmp_path):
        assert self._translator(tmp_path).fidlTypes2RosTypes("UInt8") == "uint8"

    def test_uint16_maps_to_uint16(self, tmp_path):
        assert self._translator(tmp_path).fidlTypes2RosTypes("UInt16") == "uint16"

    def test_uint64_maps_to_uint64(self, tmp_path):
        assert self._translator(tmp_path).fidlTypes2RosTypes("UInt64") == "uint64"

    def test_boolean_maps_to_bool(self, tmp_path):
        assert self._translator(tmp_path).fidlTypes2RosTypes("Boolean") == "bool"

    def test_string_maps_to_string(self, tmp_path):
        assert self._translator(tmp_path).fidlTypes2RosTypes("String") == "string"

    def test_bytebuffer_maps_to_uint8_array(self, tmp_path):
        assert self._translator(tmp_path).fidlTypes2RosTypes("ByteBuffer") == "uint8[]"

    def test_unknown_type_passes_through(self, tmp_path):
        assert self._translator(tmp_path).fidlTypes2RosTypes("CustomStruct") == "CustomStruct"


# ---------------------------------------------------------------------------
# Output file naming
# ---------------------------------------------------------------------------

class TestOutputFileNaming:

    def test_single_struct_creates_msg_file(self, tmp_path):
        out = tmp_path / "out"
        out.mkdir()
        make_translator(tmp_path, BASIC_FIDL).save(str(out))
        assert (out / "Position.msg").exists()

    def test_multiple_structs_create_multiple_msg_files(self, tmp_path):
        out = tmp_path / "out"
        out.mkdir()
        make_translator(tmp_path, MULTI_STRUCT_FIDL).save(str(out))
        assert (out / "Dop.msg").exists()
        assert (out / "Position.msg").exists()


# ---------------------------------------------------------------------------
# Generated .msg content
# ---------------------------------------------------------------------------

class TestGeneratedMsgContent:

    def test_double_generates_float64_not_float32(self, tmp_path):
        out = tmp_path / "out"
        out.mkdir()
        make_translator(tmp_path, BASIC_FIDL).save(str(out))
        content = (out / "Position.msg").read_text()
        assert "float64 latitude" in content
        assert "float64 longitude" in content
        # Ensure the old wrong mapping is NOT present for Double fields
        assert "float32 latitude" not in content
        assert "float32 longitude" not in content

    def test_uint32_generates_uint32_not_int32(self, tmp_path):
        out = tmp_path / "out"
        out.mkdir()
        make_translator(tmp_path, MULTI_STRUCT_FIDL).save(str(out))
        content = (out / "Position.msg").read_text()
        assert "uint32 satellites_visible" in content
        assert "uint32 satellites_used" in content

    def test_float_generates_float32(self, tmp_path):
        out = tmp_path / "out"
        out.mkdir()
        make_translator(tmp_path, MULTI_STRUCT_FIDL).save(str(out))
        content = (out / "Dop.msg").read_text()
        assert "float32 hdop" in content
        assert "float32 vdop" in content
        assert "float32 pdop" in content

    def test_all_types_generated_correctly(self, tmp_path):
        out = tmp_path / "out"
        out.mkdir()
        make_translator(tmp_path, ALL_TYPES_FIDL).save(str(out))
        content = (out / "AllTypes.msg").read_text()
        assert "float32 f" in content
        assert "float64 d" in content
        assert "int8 i8" in content
        assert "int16 i16" in content
        assert "int32 i32" in content
        assert "int64 i64" in content
        assert "uint8 u8" in content
        assert "uint16 u16" in content
        assert "uint32 u32" in content
        assert "uint64 u64" in content
        assert "bool b" in content
        assert "string s" in content
        assert "uint8[] buf" in content


# ---------------------------------------------------------------------------
# Service (.srv) generation
# ---------------------------------------------------------------------------

class TestServiceGeneration:

    def test_method_creates_srv_file(self, tmp_path):
        out = tmp_path / "out"
        out.mkdir()
        make_translator(tmp_path, METHOD_FIDL, "calc.fidl").save(str(out))
        assert (out / "add.srv").exists()

    def test_srv_contains_separator(self, tmp_path):
        out = tmp_path / "out"
        out.mkdir()
        make_translator(tmp_path, METHOD_FIDL, "calc.fidl").save(str(out))
        content = (out / "add.srv").read_text()
        assert "---" in content


# ---------------------------------------------------------------------------
# Error handling
# ---------------------------------------------------------------------------

class TestErrorHandling:

    def test_malformed_fidl_returns_empty_lists(self, tmp_path):
        translator = make_translator(tmp_path, MALFORMED_FIDL, "bad.fidl")
        types, services = translator.parse()
        assert types == []
        assert services == []

    def test_malformed_fidl_does_not_raise(self, tmp_path):
        out = tmp_path / "out"
        out.mkdir()
        # Should not raise — errors are caught internally
        make_translator(tmp_path, MALFORMED_FIDL, "bad.fidl").save(str(out))
        # No output files should have been created
        assert list(out.iterdir()) == []

    def test_translator_reusable_after_parse_error(self, tmp_path):
        """A translator that fails on one file should not affect a fresh instance."""
        # First translator encounters malformed FIDL
        bad_translator = make_translator(tmp_path, MALFORMED_FIDL, "bad.fidl")
        types_bad, services_bad = bad_translator.parse()
        assert types_bad == []
        assert services_bad == []

        # A second, independent translator on valid FIDL still works correctly
        good_out = tmp_path / "good_out"
        good_out.mkdir()
        make_translator(tmp_path, BASIC_FIDL, "good.fidl").save(str(good_out))
        assert (good_out / "Position.msg").exists()
