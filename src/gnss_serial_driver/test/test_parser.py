#!/usr/bin/env python3
import datetime
import math
import unittest

from gnss_serial_driver.parser import GnssParser
from gnss_serial_driver.time_policy import utc_to_local, valid_for_fusion


def nmea(body):
    checksum = 0
    for character in body:
        checksum ^= ord(character)
    return "${}*{:02X}".format(body, checksum)


class ParserTest(unittest.TestCase):
    def setUp(self):
        self.parser = GnssParser()

    def test_ksxt_fixed_and_float(self):
        fixed = self.parser.parse(nmea(
            "KSXT,20210906104914.00,120.15516000,30.27413000,16.0000,"
            "72.315,1.236,72.315,1.000,-0.418,4,4,12,12,0,0,0,0.1,0.0,0.0,50,50"
        ))[0]
        self.assertTrue(fixed.checksum_valid)
        self.assertTrue(fixed.utc_valid)
        self.assertTrue(fixed.position_valid)
        self.assertEqual(4, fixed.quality)
        self.assertEqual(2, fixed.carr_soln)
        floating = self.parser.parse(fixed.raw_line.replace(",4,4,12,12", ",2,4,12,12"))
        self.assertFalse(floating[0].checksum_valid)

    def test_rmc_gga_epoch_aggregation_and_invalid_startup(self):
        startup = self.parser.parse(nmea(
            "GNGGA,104914.00,3016.447800,N,12009.309600,E,0,00,9.9,0.0,M,0.0,M,,"
        ))[0]
        self.assertFalse(startup.utc_valid)
        self.assertFalse(startup.position_valid)
        rmc = self.parser.parse(nmea(
            "GNRMC,104914.00,A,3016.447800,N,12009.309600,E,0.0,0.0,060921,,,A"
        ))[0]
        self.assertTrue(rmc.utc_valid)
        gga = self.parser.parse(nmea(
            "GNGGA,104914.00,3016.447800,N,12009.309600,E,4,12,0.8,16.0,M,0.0,M,0.0,0000"
        ))[0]
        self.assertTrue(gga.utc_valid)
        self.assertTrue(gga.position_valid)
        expected = datetime.datetime(2021, 9, 6, 10, 49, 14,
                                     tzinfo=datetime.timezone.utc)
        self.assertEqual(int(expected.timestamp() * 1e9), gga.utc_ns)

    def test_gsa_gst_zda_and_bad_checksum(self):
        self.assertEqual("ok", self.parser.parse(nmea(
            "GNGSA,A,3,01,02,03,,,,,,,,,,1.2,0.8,0.9"
        ))[0].reject_reason)
        gst = self.parser.parse(nmea(
            "GNGST,104914.00,0.1,0.1,0.1,0.0,0.2,0.3,0.4"
        ))[0]
        self.assertAlmostEqual(0.3, gst.h_acc)
        zda = self.parser.parse(nmea("GNZDA,104914.00,06,09,2021,00,00"))[0]
        self.assertTrue(zda.utc_valid)
        bad = self.parser.parse("$GNRMC,000000.00,V,,,,,,,010100,,,N*00")[0]
        self.assertFalse(bad.checksum_valid)

    def test_agrica_and_legacy_json(self):
        fields = ["0"] * 55
        fields[0] = "GNSS"
        fields[8] = "4"
        fields[10], fields[11], fields[12], fields[53] = "5", "4", "3", "2"
        fields[23], fields[24], fields[25] = "0.2", "0.1", "-0.1"
        fields[29], fields[30], fields[31] = "30.2", "120.1", "16.0"
        fields[35], fields[36], fields[37] = "0.1", "0.2", "0.3"
        agrica = self.parser.parse("#AGRICA,COM1,0,0;{}*00000000".format(
            ",".join(fields)
        ))[0]
        self.assertTrue(agrica.position_valid)
        self.assertEqual(14, agrica.num_sv)
        legacy = self.parser.parse(
            '@IMUGNSS:{"seq":1,"lat":30.2,"lon":120.1,"alt":16.0,'
            '"ve":0.1,"vn":0.2,"vu":0.0,"state":4}'
        )[0]
        self.assertTrue(legacy.position_valid)

    def test_glued_records_and_serial_file_parser_identity(self):
        ksxt = nmea(
            "KSXT,20210906104914.00,120.15516000,30.27413000,16.0000,"
            "72.315,1.236,72.315,1.000,-0.418,4,4,12,12,0,0,0,0.1,0.0,0.0,50,50"
        )
        gga = nmea(
            "GNGGA,104914.00,3016.447800,N,12009.309600,E,4,12,0.8,16.0,M,0.0,M,0.0,0000"
        )
        parsed = self.parser.parse("noise" + ksxt + gga)
        self.assertEqual(["KSXT", "GGA"], [item.source for item in parsed])
        self.assertTrue(all(math.isfinite(item.latitude) for item in parsed))

    def test_single_differential_float_fixed_quality_classes(self):
        for quality, expected in (
            (1, (True, False, 0)),
            (2, (True, True, 0)),
            (5, (True, True, 1)),
            (4, (True, True, 2)),
        ):
            body = (
                "GNGGA,104914.00,3016.447800,N,12009.309600,E,{},12,0.8,"
                "16.0,M,0.0,M,0.0,0000"
            ).format(quality)
            value = self.parser.parse(nmea(body))[0]
            self.assertEqual(expected, (
                value.valid_fix, value.diff_soln, value.carr_soln
            ))

    def test_mapping_and_position_quality_are_independent(self):
        class Mapping:
            mapping_version = 0
            time_state = 0
            utc_ns_per_local_ns = 1.0
            utc_reference_ns = 1_700_000_000_000_000_000
            local_reference_ns = 946_684_800_000_000_000

        mapping = Mapping()
        local_ns, valid = utc_to_local(mapping.utc_reference_ns, mapping, 0)
        self.assertEqual((0, False), (local_ns, valid))
        self.assertFalse(valid_for_fusion(False, True, 4, {4}))

        mapping.mapping_version = 1
        mapping.time_state = 2
        local_ns, valid = utc_to_local(
            mapping.utc_reference_ns + 100_000_000, mapping, 0
        )
        self.assertTrue(valid)
        self.assertEqual(mapping.local_reference_ns + 100_000_000, local_ns)
        self.assertFalse(valid_for_fusion(True, False, 4, {4}))
        self.assertFalse(valid_for_fusion(True, True, 1, {4}))
        self.assertTrue(valid_for_fusion(True, True, 4, {4}))
        self.assertEqual(
            (0, False),
            utc_to_local(mapping.utc_reference_ns, mapping, local_ns),
        )


if __name__ == "__main__":
    unittest.main()
