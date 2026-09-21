#!/usr/bin/env python3
"""Headless Phase 4 trials. Sequential fresh stack reset; never two physics owners.
Run with the ROS overlay sourced. Logs go to --log-dir (not the evidence tree).
"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import yaml


def repeat_count(value):
    number = int(value)
    if number < 1 or number > 100:
        raise argparse.ArgumentTypeError('repeat count must be 1..100')
    return number


def summarize(trials):
    successes = sum(t.get('success') is True for t in trials)
    failures = {}
    for trial in trials:
        if trial.get('success') is not True:
            stage = trial.get('failure_stage', 'UNKNOWN')
            failures[stage] = failures.get(stage, 0) + 1
    return dict(success_count=successes, failure_count=len(trials)-successes,
                success_rate=successes/len(trials) if trials else 0,
                failures_by_stage=failures)


def stop(process):
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=12)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()


def wait_text(process, path, text, timeout=60):
    deadline = time.monotonic()+timeout
    while time.monotonic()<deadline:
        if text in path.read_text(errors='replace'):
            return
        if process.poll() is not None:
            raise RuntimeError(f'Process exited before readiness: {path}')
        time.sleep(0.2)
    raise RuntimeError(f'Readiness timeout: {path}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trials', type=repeat_count, default=10)
    parser.add_argument('--domain', type=int, default=64)
    parser.add_argument('--object', choices=['cube','purple_ball'], default='cube')
    parser.add_argument('--target', default='surface_left_2')
    parser.add_argument('--planning-only', action='store_true')
    parser.add_argument('--regression', action='store_true', help='run original lift-return instead')
    parser.add_argument('--output', type=Path, default=Path('docs/validation/phase4/benchmark.json'))
    parser.add_argument('--log-dir', type=Path, default=Path('/tmp/phase4-benchmark'))
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.log_dir.mkdir(parents=True, exist_ok=True)
    env = dict(os.environ, ROS_DOMAIN_ID=str(args.domain), ROS2CLI_NO_DAEMON='1')
    # Refuse reuse of an occupied test domain. Never stop user-owned processes.
    discovery = subprocess.run(['ros2','node','list','--no-daemon','--spin-time','3'],env=env,
                               capture_output=True,text=True,timeout=15)
    if discovery.returncode or discovery.stdout.strip():
        raise RuntimeError('Benchmark requires an empty ROS domain: '+discovery.stdout+discovery.stderr)
    trials=[]
    for number in range(1,args.trials+1):
        processes=[]; handles=[]; trial=dict(trial=number,success=False,failure_stage='STARTUP',
            planning_time=None,grasp_verified=False,lift_success=False,transport_success=False,
            placement_success=False,final_position_error=None,total_duration=None)
        start=time.monotonic()
        def launch(package, launch_file, label, *parameters):
            path=args.log_dir/f'{number:02d}-{label}.log'
            handle=path.open('w');handles.append(handle)
            process=subprocess.Popen(['ros2','launch',package,launch_file,*parameters],env=env,
                                     stdout=handle,stderr=subprocess.STDOUT,start_new_session=True)
            processes.append(process)
            return process,path
        try:
            p,path=launch('home_robotics_bringup','phase2_control.launch.py','physics','use_viewer:=false')
            wait_text(p,path,'All Phase 2 controllers are active')
            p,path=launch('home_robotics_moveit_config','move_group.launch.py','moveit')
            wait_text(p,path,'You can start planning now')
            p,path=launch('home_robotics_moveit_config','planning_scene_environment.launch.py','static')
            if p.wait(timeout=45)!=0: raise RuntimeError('Static scene loader failed')
            p,path=launch('home_robotics_manipulation','dynamic_object_scene_sync.launch.py','sync')
            wait_text(p,path,'Dynamic world diff accepted')
            result_path=args.log_dir/f'{number:02d}-result.yaml'
            if result_path.exists(): result_path.unlink()
            demo='cube_pick_lift_return_demo' if args.regression else 'cube_pick_place_demo'
            parameters=[f'object:={args.object}',f'execute:={str(not args.planning_only).lower()}',f'result_file:={result_path.absolute()}']
            if not args.regression: parameters.append(f'target:={args.target}')
            p,path=launch('home_robotics_manipulation',demo+'.launch.py','demo',*parameters)
            p.wait(timeout=360)
            if not result_path.exists(): raise RuntimeError('Demo produced no result')
            trial.update(yaml.safe_load(result_path.read_text()))
            trial['trial']=number
            trial['planning_only_pass']=args.planning_only and 'PLANNING_ONLY PASS' in path.read_text()
            if trial.get('success') or trial.get('planning_only_pass'): trial.pop('failure_stage',None)
        except Exception as exc:
            trial['failure_reason']=str(exc)
        finally:
            for process in reversed(processes): stop(process)
            for handle in handles: handle.close()
        trial['wall_duration']=time.monotonic()-start
        trials.append(trial)
        result=dict(mode='planning' if args.planning_only else ('regression' if args.regression else 'pick_place'),
                    reset='sequential full stack restart from authoritative configuration',trials=trials,**summarize(trials))
        args.output.write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps(trial),flush=True)
        time.sleep(2)  # DDS teardown; no simulator exists during this interval.
    summary=summarize(trials)
    args.output.with_suffix('.md').write_text(f"# Phase 4 measured results\n\n{summary['success_count']} / {len(trials)} complete successes.\n\nMode: {result['mode']}. Planning-only trials never count as physical success.\n\nFailures by stage: {summary['failures_by_stage']}\n")
    print(json.dumps(summary),flush=True)
    return 0 if (all(t.get('planning_only_pass') for t in trials) if args.planning_only else summary['failure_count']==0) else 1


if __name__ == '__main__':
    raise SystemExit(main())
