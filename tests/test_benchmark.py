"""Test suite for Benchmark Harness."""

import unittest
from apex_autonomous_logistics_platform.benchmark import run_all_benchmarks


class TestBenchmarkHarness(unittest.TestCase):
    """Verify that all 8 engine benchmarks execute reliably."""

    def test_all_benchmarks_execute(self):
        results = run_all_benchmarks(iterations=3)
        self.assertEqual(len(results), 8)
        for r in results:
            self.assertGreater(r.p50_us, 0.0)
            self.assertGreater(len(r.summary), 0)


if __name__ == "__main__":
    unittest.main()
