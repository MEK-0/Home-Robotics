"""Verify pose forwarding and the opt-in fixture without creating any simulation."""
from types import ModuleType, SimpleNamespace
from unittest.mock import patch

# This unit test replaces only the simulator-owning base class. Runtime integration
# is validated separately against the real running MuJoCo instance.
bridge = ModuleType("home_robotics_control.mujoco_joint_state_bridge")
bridge.MujocoJointStateBridge = object
bridge.main = lambda args=None: None
with patch.dict("sys.modules", {bridge.__name__: bridge}):
    from home_robotics_bringup.mujoco_runtime import Phase2MujocoRuntime


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def test_pose_interface_forwards_named_body_and_simulation_timestamp():
    bodies = {
        name: SimpleNamespace(xpos=[index, 0.2, 0.3], xquat=[0.0, 1.0, 0.0, 0.0])
        for index, name in enumerate(('cube', 'apple', 'purple_ball', 'bowl', 'pan'))
    }
    runtime = SimpleNamespace(
        data=SimpleNamespace(time=12.5, body=lambda name: bodies[name]),
        object_publishers={name: Publisher() for name in bodies},
    )
    Phase2MujocoRuntime._publish_object_poses(runtime)
    for name, publisher in runtime.object_publishers.items():
        message = publisher.messages[0]
        assert message.header.frame_id == 'world'
        assert message.header.stamp.sec == 12
        assert message.header.stamp.nanosec == 500000000
        assert message.pose.position.x == bodies[name].xpos[0]
        assert message.pose.orientation.x == 1.0
        assert message.pose.orientation.w == 0.0


def test_fixed_cube_fixture_is_one_shot_and_uses_existing_simulator():
    calls = []
    runtime = SimpleNamespace(
        debug_shifted=False,
        data=SimpleNamespace(body=lambda _: SimpleNamespace(xpos=[0.1, 0.2, 0.3], xquat=[1, 0, 0, 0])),
        sim=SimpleNamespace(
            set_free_joint_pose=lambda *args: calls.append(('pose', args)),
            set_free_joint_velocity=lambda *args: calls.append(('velocity', args)),
            forward=lambda: calls.append(('forward', ())),
        ),
        _publish_object_poses=lambda: calls.append(('publish', ())),
    )
    response = SimpleNamespace(success=False, message='')
    Phase2MujocoRuntime._shift_cube_for_validation(runtime, None, response)
    assert response.success
    assert calls[0][1][0] == 'cube_free_joint'
    assert abs(calls[0][1][1][0] - 0.15) < 1e-12
    assert calls[0][1][1][1:] == (0.2, 0.3)
    count = len(calls)
    Phase2MujocoRuntime._shift_cube_for_validation(runtime, None, response)
    assert not response.success
    assert len(calls) == count
