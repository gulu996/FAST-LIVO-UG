#!/usr/bin/env python3
import os
import unittest

from uwb_serial_driver.parser import (
    DistanceRoundAssembler,
    RangeFilter,
    UwbParser,
)


SAMPLE_PATH = os.path.join(
    os.path.dirname(__file__), "data", "distance_round5_sample.txt"
)


def new_assembler(**parser_kwargs):
    parser = UwbParser(parser_mode="distance_round5", **parser_kwargs)
    return DistanceRoundAssembler(
        parser, lines_per_round=5, round_timeout_s=2.0
    )


def feed_round(assembler, distance_lines, contexts=None, start_s=0.0):
    contexts = contexts or [None] * len(distance_lines)
    assembler.process("[UWBDBG] diag=1 dist=99.0", now_s=start_s)
    assembler.process("[TWR] Ra=123 Rb=456 num=789", now_s=start_s + 0.01)
    event = None
    for index, line in enumerate(distance_lines):
        event = assembler.process(
            line, context=contexts[index], now_s=start_s + 0.02 + index * 0.01
        )
    return event


class DistanceRoundAssemblerTest(unittest.TestCase):
    def test_uwbdbg_is_debug_only(self):
        assembler = new_assembler()
        event = assembler.process(
            "[UWBDBG] target=1 ok=1 dist=1.405 diag=8", now_s=1.0
        )
        self.assertEqual(DistanceRoundAssembler.DEBUG, event.kind)
        self.assertEqual([], event.ranges)
        self.assertFalse(event.parse_error)
        self.assertEqual(1, assembler.ignored_debug_line_count)

    def test_twr_is_debug_only_and_never_becomes_fake_ranges(self):
        assembler = new_assembler()
        event = assembler.process(
            "[TWR] Ra=191712001 Rb=281165994 Da=281167103 "
            "Db=191710237 num=283370160583 den=945755335",
            now_s=1.0,
        )
        self.assertEqual(DistanceRoundAssembler.TWR, event.kind)
        self.assertEqual([], event.ranges)
        self.assertFalse(event.parse_error)
        self.assertEqual(1, assembler.ignored_debug_line_count)

    def test_five_distance_lines_are_one_round_without_zero_anchor(self):
        assembler = new_assembler()
        event = feed_round(
            assembler,
            [
                "distance[0], 0.000, [ABORT][RES] poll wait deadline",
                "distance[1], 1.405,",
                "distance[0], 0.000, [ABORT] Response wait",
                "distance[0], 0.000,",
                "distance[0], 0.000,",
            ],
        )
        self.assertEqual(DistanceRoundAssembler.ROUND_COMPLETE, event.kind)
        self.assertEqual([1], [value.anchor_id for value in event.ranges])
        self.assertAlmostEqual(1.405, event.ranges[0].raw_range_m)
        self.assertEqual("distance_round5", event.ranges[0].source_format)
        self.assertEqual(4, assembler.zero_slot_count)
        self.assertEqual(1, assembler.complete_round_count)

    def test_debug_suffix_is_valid_zero_slot(self):
        assembler = new_assembler()
        assembler.process("[UWBDBG] diag=1", now_s=0.0)
        event = assembler.process(
            "distance[0], 0.000, [ABORT][RES] poll wait deadline",
            now_s=0.1,
        )
        self.assertEqual(DistanceRoundAssembler.DISTANCE_PENDING, event.kind)
        self.assertFalse(event.parse_error)
        self.assertEqual(1, assembler.zero_slot_count)
        self.assertEqual(0, assembler.malformed_distance_count)

    def test_uwbdbg_distance_never_fills_an_empty_round(self):
        assembler = new_assembler()
        event = feed_round(
            assembler,
            [
                "distance[0],0.000",
                "distance[1],0.000",
                "distance[0],0.000",
                "distance[0],0.000",
                "distance[0],0.000",
            ],
        )
        self.assertEqual(DistanceRoundAssembler.EMPTY_ROUND, event.kind)
        self.assertEqual([], event.ranges)
        self.assertEqual(1, assembler.empty_round_count)
        self.assertEqual(5, assembler.zero_slot_count)

    def test_real_sample_has_seven_rounds_six_published_and_one_empty(self):
        assembler = new_assembler()
        completed = []
        empty = 0
        with open(SAMPLE_PATH, "r", encoding="utf-8") as stream:
            lines = [line.strip() for line in stream if line.strip()]
        self.assertEqual(49, len(lines))
        for index, line in enumerate(lines):
            event = assembler.process(line, context=index, now_s=index * 0.01)
            if event.kind == DistanceRoundAssembler.ROUND_COMPLETE:
                completed.append(event)
            elif event.kind == DistanceRoundAssembler.EMPTY_ROUND:
                empty += 1

        self.assertEqual(7, assembler.complete_round_count)
        self.assertEqual(6, len(completed))
        self.assertEqual(1, empty)
        self.assertEqual(1, assembler.empty_round_count)
        self.assertEqual(29, assembler.zero_slot_count)
        self.assertEqual(14, assembler.ignored_debug_line_count)
        self.assertEqual(
            [1.405, 1.362, 1.391, 1.374, 1.308, 1.342],
            [round_.ranges[0].corrected_range_m for round_ in completed],
        )
        self.assertTrue(
            all(
                [1] == [value.anchor_id for value in round_.ranges]
                for round_ in completed
            )
        )

    def test_new_debug_discards_incomplete_round(self):
        assembler = new_assembler()
        assembler.process("[UWBDBG] diag=1", now_s=0.0)
        for index in range(3):
            assembler.process(
                "distance[0],0.000", now_s=0.1 + index * 0.1
            )
        event = assembler.process("[UWBDBG] diag=2", now_s=0.5)
        self.assertTrue(event.incomplete_round)
        self.assertEqual(1, assembler.incomplete_round_count)
        self.assertEqual(0, assembler.complete_round_count)
        self.assertEqual(0, assembler.pending_distance_line_count)

    def test_starting_mid_stream_waits_for_debug_boundary(self):
        assembler = new_assembler()
        event = assembler.process(
            "distance[1],1.2", context="must-not-publish", now_s=0.0
        )
        self.assertEqual(
            DistanceRoundAssembler.OUT_OF_SYNC_DISTANCE, event.kind
        )
        self.assertEqual(0, assembler.complete_round_count)
        completed = feed_round(
            assembler,
            [
                "distance[1],1.2",
                "distance[0],0",
                "distance[0],0",
                "distance[0],0",
                "distance[0],0",
            ],
            start_s=1.0,
        )
        self.assertEqual(
            DistanceRoundAssembler.ROUND_COMPLETE, completed.kind
        )

    def test_multiple_anchors_remain_in_one_round(self):
        assembler = new_assembler()
        event = feed_round(
            assembler,
            [
                "distance[1],1.2",
                "distance[2],2.3",
                "distance[0],0",
                "distance[0],0",
                "distance[0],0",
            ],
        )
        self.assertEqual(
            [1, 2], [value.anchor_id for value in event.ranges]
        )
        self.assertEqual(1, assembler.complete_round_count)

    def test_duplicate_nonzero_anchor_keeps_first(self):
        assembler = new_assembler()
        event = feed_round(
            assembler,
            [
                "distance[1],1.2",
                "distance[1],9.9",
                "distance[0],0",
                "distance[0],0",
                "distance[0],0",
            ],
        )
        self.assertEqual(1, len(event.ranges))
        self.assertAlmostEqual(1.2, event.ranges[0].raw_range_m)
        self.assertEqual(1, assembler.duplicate_anchor_count)

    def test_repeated_zero_anchor_is_not_duplicate(self):
        assembler = new_assembler()
        event = feed_round(
            assembler, ["distance[0],0"] * 5
        )
        self.assertEqual(DistanceRoundAssembler.EMPTY_ROUND, event.kind)
        self.assertEqual(0, assembler.duplicate_anchor_count)

    def test_timestamp_context_is_first_nonzero_distance(self):
        assembler = new_assembler()
        contexts = ["zero-1", "first-nonzero", "second-nonzero", "zero-2", "zero-3"]
        event = feed_round(
            assembler,
            [
                "distance[0],0",
                "distance[1],1.2",
                "distance[2],2.3",
                "distance[0],0",
                "distance[0],0",
            ],
            contexts=contexts,
        )
        self.assertEqual("first-nonzero", event.timestamp_context)

    def test_timeout_discards_pending_round(self):
        assembler = new_assembler()
        assembler.process("[UWBDBG] diag=1", now_s=1.0)
        assembler.process("distance[0],0", now_s=1.5)
        self.assertTrue(assembler.expire(now_s=3.0))
        self.assertEqual(1, assembler.incomplete_round_count)
        self.assertEqual(DistanceRoundAssembler.WAIT_DEBUG, assembler.state)

    def test_malformed_distance_and_unknown_are_errors_only(self):
        assembler = new_assembler()
        malformed = assembler.process("distance[1] 1.2", now_s=1.0)
        unknown = assembler.process("firmware booted 123", now_s=1.1)
        self.assertEqual(
            DistanceRoundAssembler.MALFORMED_DISTANCE, malformed.kind
        )
        self.assertEqual(DistanceRoundAssembler.UNKNOWN, unknown.kind)
        self.assertTrue(malformed.parse_error)
        self.assertTrue(unknown.parse_error)
        self.assertEqual(1, assembler.malformed_distance_count)

    def test_scale_bias_and_invalid_nonzero_are_preserved(self):
        assembler = new_assembler(
            range_scale=2.0, range_bias={1: 0.5}, min_range_m=0.05
        )
        event = feed_round(
            assembler,
            [
                "distance[1],2.0",
                "distance[2],1e309",
                "distance[0],0",
                "distance[0],0",
                "distance[0],0",
            ],
        )
        self.assertAlmostEqual(4.0, event.ranges[0].raw_range_m)
        self.assertAlmostEqual(3.5, event.ranges[0].corrected_range_m)
        self.assertFalse(event.ranges[1].valid)
        self.assertEqual(
            UwbParser.INVALID_RAW_RANGE, event.ranges[1].reject_reason
        )

    def test_serial_and_file_text_paths_assemble_identically(self):
        with open(SAMPLE_PATH, "r", encoding="utf-8") as stream:
            lines = [line.strip() for line in stream if line.strip()]
        serial = new_assembler()
        replay = new_assembler()
        serial_output = []
        replay_output = []
        for index, line in enumerate(lines):
            serial_event = serial.process(line, context=index, now_s=index * 0.01)
            replay_event = replay.process(line, context=index, now_s=index * 0.01)
            if serial_event.kind == DistanceRoundAssembler.ROUND_COMPLETE:
                serial_output.append(serial_event)
            if replay_event.kind == DistanceRoundAssembler.ROUND_COMPLETE:
                replay_output.append(replay_event)
        self.assertEqual(serial_output, replay_output)


class LegacyParserAndFilterTest(unittest.TestCase):
    def test_pairs_and_auto_numeric_records_remain_supported(self):
        pairs = UwbParser(
            parser_mode="pairs", range_bias={0: 0.1, 1: -0.2}
        ).parse("0,8.5,1,9.0")
        self.assertEqual(2, len(pairs))
        self.assertAlmostEqual(8.4, pairs[0].corrected_range_m)
        self.assertAlmostEqual(9.2, pairs[1].corrected_range_m)
        auto = UwbParser(parser_mode="auto", anchor_order=[0, 1])
        self.assertEqual([0], [item.anchor_id for item in auto.parse("distance[0],8.5")])
        self.assertEqual([1], [item.anchor_id for item in auto.parse("distance[1],9.0")])

    def test_auto_never_parses_debug_or_arbitrary_numbers(self):
        parser = UwbParser(parser_mode="auto", anchor_order=[0, 1])
        self.assertEqual(
            [],
            parser.parse("[UWBDBG] target=1 ok=1 dist=7.5 diag=12"),
        )
        self.assertEqual(
            [],
            parser.parse("[TWR] Ra=191712001 Rb=281165994 num=3 den=4"),
        )
        self.assertEqual([], parser.parse("firmware booted 1 2 3 4"))

    def test_repeat_filter_semantics_are_unchanged(self):
        parser = UwbParser(parser_mode="distance")
        value = parser.parse("distance[0],8.0")
        range_filter = RangeFilter(
            epsilon_m=0.001, max_count=2, max_duration_s=1.0
        )
        self.assertEqual(1, len(range_filter.filter(value, 0.0)))
        self.assertEqual(1, len(range_filter.filter(value, 0.5)))
        self.assertEqual(0, len(range_filter.filter(value, 1.0)))
        self.assertEqual(1, range_filter.drop_count)


if __name__ == "__main__":
    unittest.main()
