"""No ROS/GUI timing; exercise accounting including mixed failures and missing reports."""
import argparse
import importlib.util
from pathlib import Path
import pytest
PATH=Path(__file__).resolve().parents[2]/'ros2_ws/src/home_robotics_manipulation/scripts/benchmark_pick_place.py'
spec=importlib.util.spec_from_file_location('phase4_benchmark',PATH)
benchmark=importlib.util.module_from_spec(spec)
spec.loader.exec_module(benchmark)

@pytest.mark.parametrize('value',['0','-1','101'])
def test_invalid_repeats(value):
    with pytest.raises(argparse.ArgumentTypeError): benchmark.repeat_count(value)

def test_accounting_keeps_all_failures():
    trials=[dict(success=True)]*8+[dict(success=False,failure_stage='PLACING'),{}]
    assert benchmark.repeat_count('10')==10
    assert benchmark.summarize(trials)==dict(success_count=8,failure_count=2,success_rate=0.8,
                                            failures_by_stage={'PLACING':1,'UNKNOWN':1})
    assert benchmark.summarize([])['success_rate']==0
    assert benchmark.summarize([dict(success=False,planning_only_pass=True)])['success_count']==0

def test_execution_defaults_false():
    root=PATH.parent.parent
    for name in ('cube_pick_place_demo','cube_pick_lift_return_demo'):
        assert 'DeclareLaunchArgument("execute", default_value="false")' in (root/'launch'/f'{name}.launch.py').read_text()
