"""The stage clock itself -- the arithmetic every measured number rests on.

No CUDA and no pipeline: `Stopwatch` is a clock and a dict, and the properties
the breakdown depends on (stages accumulate, marks do not overlap, the total is
the span and not the sum) are exactly the ones that would go unnoticed if they
broke. A wrong duration does not raise; it just quietly sends someone to
optimise the wrong stage.
"""

from __future__ import annotations

import pytest
from weld_motion_planner.utils.timing import Stopwatch, cuda_sync, report


def test_a_stage_entered_twice_accumulates():
    """What makes one watch usable for a whole run: `plan` enters `seams` once
    per seam and reads the total off the end."""
    watch = Stopwatch()
    for _ in range(3):
        with watch.stage("seams"):
            sum(range(10_000))
    assert list(watch.stages) == ["seams"]
    assert watch.stages["seams"] > 0


def test_stages_keep_the_order_they_first_ran():
    """Insertion order IS pipeline order, and the UI draws its breakdown from
    it without being told separately what follows what."""
    watch = Stopwatch()
    for name in ("scene", "seams", "respond"):
        with watch.stage(name):
            pass
    with watch.stage("scene"):
        pass
    assert list(watch.ms()) == ["scene", "seams", "respond", "total"]


def test_marks_partition_the_run_rather_than_overlapping():
    """A mark closes the stretch since the last one, so two marks never count
    the same millisecond -- which is what lets `nodes.build` report five stages
    that sum to the build."""
    watch = Stopwatch()
    sum(range(200_000))
    watch.mark("first")
    sum(range(200_000))
    watch.mark("second")
    stages = watch.ms()
    assert stages["first"] > 0 and stages["second"] > 0
    assert stages["first"] + stages["second"] <= stages["total"] + 1.0


def test_the_total_is_the_span_and_not_the_sum():
    """The gap between them is time no stage claimed, and a total computed by
    adding the stages up could never show it."""
    watch = Stopwatch()
    with watch.stage("counted"):
        sum(range(100_000))
    sum(range(500_000))  # nobody's stage
    stages = watch.ms()
    assert stages["total"] > stages["counted"]


def test_a_stage_that_raised_still_reports_what_it_spent():
    """The most interesting line in a performance log is the seam that spent
    nine seconds and then failed. `plan_seam` reports exactly that."""
    watch = Stopwatch()
    with pytest.raises(ValueError):
        with watch.stage("doomed"):
            sum(range(100_000))
            raise ValueError("as it happens")
    assert watch.stages["doomed"] > 0


def test_a_cpu_device_pays_for_no_synchronise():
    """A planner told to run on the CPU on a machine that HAS a GPU must not
    pay for a device sync -- and must not look like it did."""
    called = cuda_sync(object())
    assert called() is None
    assert cuda_sync(None)() is None


def test_report_names_every_stage_and_orders_by_cost():
    lines = report({"ik": 388.0, "screen": 62.0, "total": 500.0})
    assert [line.split()[0] for line in lines] == ["ik", "screen", "total"]
    assert "78%" in lines[0], lines[0]
    # An omitted stage would read as a stage that did not run.
    assert len(lines) == 3
