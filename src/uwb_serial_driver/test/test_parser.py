#!/usr/bin/env python3
import unittest

from uwb_serial_driver.parser import RangeFilter, UwbParser


class UwbParserTest(unittest.TestCase):
    def test_one_line_multiple_anchors_is_one_parser_round(self):
        parser = UwbParser(parser_mode="pairs", range_bias={0: 0.1, 1: -0.2})
        values = parser.parse("0,8.5,1,9.0")
        self.assertEqual(2, len(values))
        self.assertAlmostEqual(8.4, values[0].corrected_range_m)
        self.assertAlmostEqual(9.2, values[1].corrected_range_m)

    def test_independent_lines_are_not_merged(self):
        parser = UwbParser(parser_mode="auto", anchor_order=[0, 1])
        first = parser.parse("distance[0],8.5")
        second = parser.parse("distance[1],9.0")
        self.assertEqual([0], [item.anchor_id for item in first])
        self.assertEqual([1], [item.anchor_id for item in second])

    def test_values_debug_and_invalid_ranges(self):
        parser = UwbParser(parser_mode="auto", anchor_order=[3, 4],
                           min_range_m=0.5, max_range_m=20.0)
        values = parser.parse("8.0 9.0 10.0")
        self.assertEqual([3, 4, 2], [item.anchor_id for item in values])
        debug = parser.parse("[UWBDBG] target=4 ok=1 dist=7.5 diag=12")
        self.assertEqual(12, debug[0].diag)
        self.assertEqual("uwbdbg", debug[0].source_format)
        invalid = parser.parse("distance[0],0.1")[0]
        self.assertFalse(invalid.valid)
        self.assertEqual(UwbParser.RANGE_LIMIT, invalid.reject_reason)

    def test_repeat_filter(self):
        parser = UwbParser(parser_mode="distance")
        value = parser.parse("distance[0],8.0")
        range_filter = RangeFilter(epsilon_m=0.001, max_count=2, max_duration_s=1.0)
        self.assertEqual(1, len(range_filter.filter(value, 0.0)))
        self.assertEqual(1, len(range_filter.filter(value, 0.5)))
        self.assertEqual(0, len(range_filter.filter(value, 1.0)))
        self.assertEqual(1, range_filter.drop_count)

    def test_serial_file_parser_identity(self):
        parser = UwbParser(parser_mode="auto", anchor_order=[0, 1])
        line = "0,8.5,1,9.0"
        serial = parser.parse(line)
        replay = parser.parse(line)
        self.assertEqual(serial, replay)


if __name__ == "__main__":
    unittest.main()
