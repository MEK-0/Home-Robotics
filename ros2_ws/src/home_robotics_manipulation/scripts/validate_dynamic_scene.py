#!/usr/bin/env python3
"""Read-only scene validation plus an explicitly requested, resettable cube fixture."""
import argparse
import json
import math
from pathlib import Path
import time

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPlanningScene
import rclpy
from rclpy.node import Node
from rosidl_runtime_py.convert import message_to_ordereddict
from std_srvs.srv import Trigger
import yaml


def pose_values(pose):
    return [pose.position.x, pose.position.y, pose.position.z], [
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]


def multiply(a, b):
    x, y, z, w = a
    X, Y, Z, W = b
    return [w*X+x*W+y*Z-z*Y, w*Y-x*Z+y*W+z*X,
            w*Z+x*Y-y*X+z*W, w*W-x*X-y*Y-z*Z]


def compose(a, b):
    p, q = a
    r, s = b
    rotated = multiply(multiply(q, [*r, 0.0]), [-q[0], -q[1], -q[2], q[3]])
    return [p[i] + rotated[i] for i in range(3)], multiply(q, s)


def errors(a, b):
    pos = math.dist(a[0], b[0])
    # q and -q encode the same orientation; chord error avoids acos roundoff.
    orientation = min(math.dist(a[1], b[1]), math.dist(a[1], [-v for v in b[1]]))
    return pos, orientation


class Validation(Node):
    def __init__(self):
        super().__init__('dynamic_scene_validation')
        config = Path(get_package_share_directory('home_robotics_control')) / 'config'
        self.objects = {key: value for key, value in yaml.safe_load(
            (config / 'objects.yaml').read_text())['objects'].items()
            if value['planning_scene_enabled']}
        scene = yaml.safe_load((config / 'scene.yaml').read_text())['scene']
        self.static_ids = {scene['floor']['id'], 'arena_front', 'arena_back',
                           'arena_left', 'arena_right', *scene['surfaces'],
                           *scene['shared_rail']['supports']['names']}
        self.poses = {}
        self.subscriptions_keep = [self.create_subscription(
            PoseStamped, f'/mujoco/objects/{key}/pose',
            lambda msg, key=key: self.poses.__setitem__(key, msg), 1)
            for key in self.objects]
        self.scene_client = self.create_client(GetPlanningScene, '/get_planning_scene')

    def call(self, client, request):
        if not client.wait_for_service(timeout_sec=5):
            raise RuntimeError('Service unavailable')
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5)
        if not future.done():
            raise RuntimeError('Service timeout')
        return future.result()

    def scene(self):
        request = GetPlanningScene.Request()
        request.components.components = 1023
        return self.call(self.scene_client, request).scene

    def trigger(self, name):
        client = self.create_client(Trigger, name)
        result = self.call(client, Trigger.Request())
        self.destroy_client(client)
        if not result.success:
            raise RuntimeError(result.message)
        print(result.message, flush=True)

    def validate(self, baseline):
        deadline = time.monotonic() + 5
        while len(self.poses) != len(self.objects) and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        assert set(self.poses) == set(self.objects), 'Missing pose topic'
        scene = self.scene()
        ids = [obj.id for obj in scene.world.collision_objects]
        assert len(ids) == len(set(ids)), 'Duplicate object ID'
        assert set(ids) == self.static_ids | set(self.objects), 'Missing/unexpected world ID'
        assert not scene.robot_state.attached_collision_objects, 'Objects unexpectedly attached'
        by_id = {obj.id: obj for obj in scene.world.collision_objects}
        static = {key: message_to_ordereddict(by_id[key]) for key in sorted(self.static_ids)}
        assert static == baseline, 'Static geometry changed'
        report = {}
        for key, metadata in self.objects.items():
            source = self.poses[key]
            obj = by_id[key]
            assert obj.header.frame_id == 'world' and source.header.frame_id == 'world'
            parts = metadata['collision'].get('primitives', [metadata['collision']])
            assert len(obj.primitives) == len(parts) == len(obj.primitive_poses)
            position_error, orientation_error = errors(pose_values(source.pose), pose_values(obj.pose))
            primitive_errors = []
            for index, part in enumerate(parts):
                shape = obj.primitives[index]
                if part['type'] == 'box':
                    assert shape.type == shape.BOX
                    dimensions = part['dimensions']
                elif part['type'] == 'sphere':
                    assert shape.type == shape.SPHERE
                    dimensions = [part['radius']]
                else:
                    assert shape.type == shape.CYLINDER
                    dimensions = [part['height'], part['radius']]
                assert list(shape.dimensions) == dimensions
                yaw = part.get('yaw', 0)
                local = part.get('position', [0, 0, 0]), [0, 0, math.sin(yaw/2), math.cos(yaw/2)]
                expected = compose(pose_values(source.pose), local)
                observed = compose(pose_values(obj.pose), pose_values(obj.primitive_poses[index]))
                pe, qe = errors(expected, observed)
                assert pe <= 0.001 and qe <= 1e-6, f'{key} primitive mismatch: {pe}, {qe}'
                primitive_errors.append([pe, qe])
            assert position_error <= 0.001 and orientation_error <= 1e-6, key
            report[key] = dict(frame=obj.header.frame_id, mujoco_pose=pose_values(source.pose),
                               planning_scene_pose=pose_values(obj.pose),
                               position_error_m=position_error, quaternion_error=orientation_error,
                               geometry=metadata['collision'], primitive_errors=primitive_errors,
                               presence=True, timestamp=message_to_ordereddict(source.header.stamp))
        return dict(objects=report, static_count=len(static), dynamic_count=len(self.objects),
                    total_count=len(ids), static_unchanged=True, attached_count=0)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--baseline', required=True)
    parser.add_argument('--capture-static', action='store_true')
    parser.add_argument('--movement', action='store_true')
    parser.add_argument('--output')
    args = parser.parse_args()
    rclpy.init()
    node = Validation()
    try:
        if args.capture_static:
            scene = node.scene()
            objects = {obj.id: message_to_ordereddict(obj) for obj in scene.world.collision_objects}
            assert set(objects) == node.static_ids
            Path(args.baseline).write_text(json.dumps(objects, indent=2))
            print('PASS static baseline captured', flush=True)
            return
        baseline = json.loads(Path(args.baseline).read_text())
        report = node.validate(baseline)
        print(json.dumps(report, indent=2), flush=True)
        if args.movement:
            initial = report['objects']['cube']['mujoco_pose']
            try:
                node.trigger('/validation/shift_cube')
                deadline = time.monotonic() + 5
                while True:
                    try:
                        moved = node.validate(baseline)
                        dx = moved['objects']['cube']['mujoco_pose'][0][0] - initial[0][0]
                        assert abs(dx - 0.05) <= 0.001
                        break
                    except AssertionError:
                        if time.monotonic() >= deadline:
                            raise
                        rclpy.spin_once(node, timeout_sec=0.1)
                report['movement'] = dict(delta_x_m=dx, after=moved['objects']['cube'], unique_cube=True)
                print('PASS cube +0.05 m authoritative movement and scene follow', flush=True)
            finally:
                node.trigger('/reset_simulation')
            deadline = time.monotonic() + 5
            while True:
                try:
                    restored = node.validate(baseline)
                    assert errors(initial, restored['objects']['cube']['mujoco_pose'])[0] <= 0.001
                    break
                except AssertionError:
                    if time.monotonic() >= deadline:
                        raise
                    rclpy.spin_once(node, timeout_sec=0.1)
            report['restored'] = restored['objects']['cube']
            print('PASS reset restored cube and scene', flush=True)
        if args.output:
            Path(args.output).write_text(json.dumps(report, indent=2))
        print('PASS exact IDs, poses, primitive geometry, static preservation and no attachments', flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
