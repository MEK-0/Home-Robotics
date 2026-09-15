"""Deterministic contact decisions and authoritative runtime guard tests."""
from pathlib import Path
import numpy as np
import pytest
from simulation.config_loader import ConfigLoader
from simulation.scene_builder import SceneBuilder
from simulation.grasp_contact import CubeContact, verification_reason

ROOT = Path(__file__).resolve().parents[2]

@pytest.mark.parametrize('left,right,expected', [(True,True,'SUCCESS'),(True,False,'NO_CONTACT'),(False,True,'NO_CONTACT'),(False,False,'NO_CONTACT')])
def test_bilateral_required(left,right,expected):
    state=dict(width=0.05,left_finger_contact=left,right_finger_contact=right,
               between_fingers=True,position=[0,0,0])
    assert verification_reason(state,0.064,np.zeros(3))==expected

@pytest.mark.parametrize('change,expected', [({'width':0.063},'GRIPPER_FAILED'),
    ({'between_fingers':False},'GRASP_UNSTABLE'),({'position':[0,0,0.04]},'OBJECT_DROPPED')])
def test_verification_rejects_bad_geometry_and_no_closure(change,expected):
    state=dict(width=0.05,left_finger_contact=True,right_finger_contact=True,
               between_fingers=True,position=[0,0,0]);state.update(change)
    assert verification_reason(state,0.064,np.zeros(3))==expected


def test_authoritative_mapping_guard_and_reset():
    config=ConfigLoader(ROOT/'config').load()
    with SceneBuilder(config).build(headless=True) as sim:
        observer=CubeContact(sim.model,sim.data,config)
        assert sim.model.body('cube').id==sim.model.geom_bodyid[sim.model.geom('cube_collision').id]
        for side in ('left','right'):
            assert {sim.model.geom(i).name for i in observer.geom_sets[side]}=={
                f'panda1_{side}_finger_geom_{i}' for i in range(2,8)}
        before=sim.data.qpos.copy()
        assert not sim.data.eq_active[observer.eq]
        with pytest.raises(ValueError,match='verification required'):
            observer.stabilize(True)
        observer.begin()
        with pytest.raises(ValueError): observer.verify()
        np.testing.assert_array_equal(sim.data.qpos,before)
        assert not sim.data.eq_active[observer.eq]
        sim.data.eq_active[observer.eq]=True  # Reset fixture, never a demo activation.
        observer.reset()
        assert not sim.data.eq_active[observer.eq]
        assert not observer.verified and observer.open_width is None


def test_physical_bilateral_close_then_preserving_weld():
    """A fixed IK fixture seeds the robot, then only actuator commands close fingers."""
    from simulation.arm_control import PandaArmPositionController
    from simulation.gripper_control import GripperWidthController
    from simulation.rail_control import RailTargetController
    config=ConfigLoader(ROOT/'config').load()
    with SceneBuilder(config).build(headless=True) as sim:
        fixture={'panda1_rail_joint':-1.130090370603,
            **{f'panda1_joint{i}':v for i,v in enumerate([
                -0.108779046147,1.180531967380,-0.125578495744,-2.130764684596,
                -0.612007358702,3.344600796951,1.230913903560],1)},
            'panda1_finger_joint1':0.032,'panda1_finger_joint2':0.032}
        sim.set_joint_positions(fixture)  # Offline initial-condition fixture only.
        rail=RailTargetController(config,sim);rail.accept_target('panda1',fixture['panda1_rail_joint'])
        rail.actuator_targets['panda1']=fixture['panda1_rail_joint']
        arms=[PandaArmPositionController(config,sim,r) for r in ('panda1','panda2')]
        arms[0].accept_command(arms[0].joint_names,[fixture[n] for n in arms[0].joint_names])
        arms[0].actuator_targets=dict(arms[0].targets)
        fingers=[GripperWidthController(config,sim,r) for r in ('panda1','panda2')]
        fingers[0].accept_target(0.064);fingers[0].actuator_width=0.064
        sim.forward();observer=CubeContact(sim.model,sim.data,config)
        def step(count):
            for _ in range(count):
                rail.apply_targets()
                for arm in arms: arm.apply_targets()
                for finger in fingers: finger.apply_target()
                sim.step();observer.observe()
        step(300);observer.begin();fingers[0].accept_target(0.0);step(1000)
        state=observer.verify()
        assert state['left_finger_contact'] and state['right_finger_contact']
        assert state['width']<0.060 and state['between_fingers']
        before=sim.data.qpos.copy();observer.stabilize(True)
        np.testing.assert_array_equal(sim.data.qpos,before)
        step(100)
        assert observer.observe()['activation_jump']<0.005
        assert observer.observe()['relative_drift']<0.001
        assert observer.observe()['support_contact']
        observer.stabilize(False)
        assert not sim.data.eq_active[observer.eq]
