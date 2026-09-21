"""Known-primitive contact verification; both observers borrow one runtime model/data."""
import numpy as np
import mujoco


def verification_reason(state, open_width, original):
    if open_width is None or open_width - state['width'] < 0.004:
        return 'GRIPPER_FAILED'
    if not state['left_finger_contact'] or not state['right_finger_contact']:
        return 'NO_CONTACT'
    if not state['between_fingers']:
        return 'GRASP_UNSTABLE'
    if np.linalg.norm(np.array(state['position']) - original) > 0.025:
        return 'OBJECT_DROPPED'
    return 'SUCCESS'


class CubeContact:
    def __init__(self, model, data, config, object_id="cube"):
        self.object_id = object_id
        self.model, self.data = model, data
        collision = config.objects[object_id]['collision']
        self.spherical = collision['type'] == 'sphere'
        self.width = 2 * float(collision['radius']) if self.spherical else float(collision['dimensions'][0])
        self.support = config.objects[object_id]['initial']['support_surface'] + '_top'
        self.surfaces = config.scene['surfaces']
        self.eq = model.equality(f'panda1_{object_id}_grasp').id
        self.geom_sets = {}
        for side in ('left', 'right'):
            body = model.body(f'panda1_{side}_finger').id
            self.geom_sets[side] = {i for i in range(model.ngeom)
                if model.geom_bodyid[i] == body and (model.geom_contype[i] or model.geom_conaffinity[i])}
        self.reset()

    def reset(self):
        self.data.eq_active[self.eq] = False
        self.open_width = None
        self.original = None
        self.verified = False
        self.since = None
        self.reference = None
        self.max_drift = 0.0
        self.activation_jump = 0.0
        self.activation_position = None
        self.activation_time = None

    def observe(self):
        d, m = self.data, self.model
        cube, hand = d.body(self.object_id), d.body('panda1_hand')
        rotation = hand.xmat.reshape(3, 3)
        relative = rotation.T @ (cube.xpos - hand.xpos)
        pairs, sides, support = [], set(), False
        cube_id = m.geom(self.object_id + '_collision').id
        unexpected, support_surfaces = [], []
        for c in d.contact[:d.ncon]:
            if c.dist > 0 or c.efc_address < 0:
                continue
            ids = {int(c.geom1), int(c.geom2)}
            if cube_id not in ids:
                continue
            other = next(iter(ids - {cube_id}))
            name = m.geom(other).name
            pairs.append([m.geom(int(c.geom1)).name, m.geom(int(c.geom2)).name])
            found = False
            for side, geoms in self.geom_sets.items():
                if other in geoms:
                    sides.add(side); found = True
            if name in {key + '_top' for key in self.surfaces}:
                support = True
                support_surfaces.append(name[:-4])
            elif not found:
                unexpected.append(name)
        width = sum(float(d.joint(f'panda1_finger_joint{i}').qpos[0]) for i in (1, 2))
        # Hand-local pad centre is read from the actual main pad geom.
        pad = m.geom('panda1_left_finger_geom_3').id
        pad_local = rotation.T @ (d.geom_xpos[pad] - hand.xpos)
        between = (abs(relative[1]) < self.width / 2 and
                   abs(relative[0] - pad_local[0]) < self.width / 2 and
                   abs(relative[2] - pad_local[2]) < self.width / 2)
        active = bool(d.eq_active[self.eq])
        if active and self.reference is not None:
            self.max_drift = max(self.max_drift, float(np.linalg.norm(relative - self.reference)))
            if d.time - self.activation_time <= 0.10:
                self.activation_jump = max(self.activation_jump, float(np.linalg.norm(cube.xpos-self.activation_position)))
        near_support = []
        half = np.full(3, self.width / 2) if self.spherical else np.abs(cube.xmat.reshape(3, 3)) @ np.full(3, self.width / 2)
        for key, surface in self.surfaces.items():
            center = np.array(surface['pose']['position'][:2]) + np.array(surface['safe_place_region']['center'][:2])
            fits = np.all(np.abs(cube.xpos[:2] - center) + half[:2] <= np.array(surface['safe_place_region']['size']) / 2)
            gap = cube.xpos[2] - half[2] - surface['top_height']
            if fits and -0.0005 <= gap <= 0.002:
                near_support.append(key)
        result = dict(object_id=self.object_id, timestamp=float(d.time), left_finger_contact='left' in sides,
            right_finger_contact='right' in sides, contact_count=len(pairs), contact_pairs=pairs,
            support_contact=support, support_surfaces=sorted(set(support_surfaces)),
            near_support_surfaces=near_support, unexpected_contacts=unexpected, width=width,
            between_fingers=bool(between), position=cube.xpos.tolist(), quaternion_wxyz=cube.xquat.tolist(),
            relative_position=relative.tolist(), stabilization_active=active,
            relative_drift=self.max_drift, activation_jump=self.activation_jump)
        reason = verification_reason(result, self.open_width, self.original) if self.original is not None else 'GRIPPER_FAILED'
        if reason == 'SUCCESS':
            if self.since is None: self.since = float(d.time)
        else:
            self.since = None
        result['verification'] = reason
        result['bilateral_duration'] = 0.0 if self.since is None else float(d.time)-self.since
        return result

    def begin(self):
        state = self.observe()
        if state['stabilization_active'] or state['width'] < self.width + 0.006:
            raise ValueError('GRIPPER_FAILED: open gripper required before arming')
        self.open_width = state['width']
        self.original = np.array(state['position'])
        self.verified = False
        self.since = None

    def verify(self):
        state = self.observe()
        if state['verification'] != 'SUCCESS' or state['bilateral_duration'] < 0.10:
            raise ValueError(state['verification'] if state['verification'] != 'SUCCESS' else 'GRASP_UNSTABLE')
        self.verified = True
        return state

    def stabilize(self, active):
        if active:
            if not self.verified:
                raise ValueError('GRASP_UNSTABLE: verification required before stabilization')
            self.verify()  # Recheck live physics; a historical success is insufficient.
            if self.data.eq_active[self.eq]:
                raise ValueError('Already stabilized')
            hand, cube = self.data.body('panda1_hand'), self.data.body(self.object_id)
            inv = np.empty(4); quat = np.empty(4)
            mujoco.mju_negQuat(inv, hand.xquat)
            mujoco.mju_mulQuat(quat, inv, cube.xquat)
            relative = hand.xmat.reshape(3, 3).T @ (cube.xpos - hand.xpos)
            self.model.eq_data[self.eq, :3] = 0
            self.model.eq_data[self.eq, 3:6] = relative
            self.model.eq_data[self.eq, 6:10] = quat
            self.reference = relative.copy()
            self.activation_position = cube.xpos.copy()
            self.activation_time = float(self.data.time)
            self.max_drift = self.activation_jump = 0.0
        else:
            state = self.observe()
            if self.data.eq_active[self.eq] and not (state['support_contact'] or state['near_support_surfaces']):
                raise ValueError('PLACE_FAILED: support contact required before release')
            self.verified = False
        self.data.eq_active[self.eq] = active
        # No qpos writes or extra physics steps.

    def clear_dropped(self):
        """Clear only a demonstrably stale weld; retain a healthy airborne grasp."""
        state = self.observe()
        if state['stabilization_active'] and state['relative_drift'] <= 0.01:
            raise ValueError('GRASP_UNSTABLE: healthy weld must be retained')
        self.data.eq_active[self.eq] = False
        self.verified = False
