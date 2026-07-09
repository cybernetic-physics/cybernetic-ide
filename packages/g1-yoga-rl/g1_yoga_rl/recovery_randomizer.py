"""Small domain randomizer for yoga policy recovery training."""

from __future__ import annotations

from types import ModuleType
from typing import Any, Tuple, Union

import jax
import jax.numpy as jnp
import numpy as np
from mujoco import MjData, MjModel
from mujoco.mjx import Data, Model

from loco_mujoco.core.domain_randomizer import DomainRandomizer
from loco_mujoco.core.utils.backend import assert_backend_is_supported


class YogaObservationNoiseRandomizer(DomainRandomizer):
    """Add mild observation noise without changing the MuJoCo model.

    The stock LocoMuJoCo DefaultRandomizer is broader than we need here and
    currently crashes in the NumPy reset path for this custom 29-DOF setup. This
    randomizer only targets the base observation fields that exist in the yoga
    env, leaving the trajectory goal/reference half untouched.
    """

    def reset(
        self,
        env: Any,
        model: Union[MjModel, Model],
        data: Union[MjData, Data],
        carry: Any,
        backend: ModuleType,
    ) -> Tuple[Union[MjData, Data], Any]:
        assert_backend_is_supported(backend)
        return data, carry

    def update(
        self,
        env: Any,
        model: Union[MjModel, Model],
        data: Union[MjData, Data],
        carry: Any,
        backend: ModuleType,
    ) -> Tuple[Union[MjModel, Model], Union[MjData, Data], Any]:
        assert_backend_is_supported(backend)
        return model, data, carry

    def update_action(
        self,
        env: Any,
        action: Union[np.ndarray, jax.Array],
        model: Union[MjModel, Model],
        data: Union[MjData, Data],
        carry: Any,
        backend: ModuleType,
    ) -> Tuple[Union[np.ndarray, jax.Array], Any]:
        assert_backend_is_supported(backend)
        return action, carry

    def update_observation(
        self,
        env: Any,
        obs: Union[np.ndarray, jax.Array],
        model: Union[MjModel, Model],
        data: Union[MjData, Data],
        carry: Any,
        backend: ModuleType,
    ) -> Tuple[Union[np.ndarray, jax.Array], Any]:
        assert_backend_is_supported(backend)

        joint_pos_scale = float(self.rand_conf.get("joint_pos_noise_scale", 0.0))
        joint_vel_scale = float(self.rand_conf.get("joint_vel_noise_scale", 0.0))
        free_lin_scale = float(self.rand_conf.get("free_joint_lin_vel_noise_scale", 0.0))
        free_ang_scale = float(self.rand_conf.get("free_joint_ang_vel_noise_scale", 0.0))

        joint_pos = np.asarray(getattr(env._obs_indices, "JointPos", []), dtype=int)
        joint_vel = np.asarray(getattr(env._obs_indices, "JointVel", []), dtype=int)
        free_joint_vel = np.asarray(getattr(env._obs_indices, "FreeJointVel", []), dtype=int)
        free_lin_vel = free_joint_vel[:3]
        free_ang_vel = free_joint_vel[3:]

        if backend == jnp:
            key = carry.key
            keys = jax.random.split(key, 5)
            key = keys[0]
            obs = self._add_jax_noise(obs, joint_pos, joint_pos_scale, keys[1])
            obs = self._add_jax_noise(obs, joint_vel, joint_vel_scale, keys[2])
            obs = self._add_jax_noise(obs, free_lin_vel, free_lin_scale, keys[3])
            obs = self._add_jax_noise(obs, free_ang_vel, free_ang_scale, keys[4])
            return obs, carry.replace(key=key)

        obs = self._add_numpy_noise(obs, joint_pos, joint_pos_scale)
        obs = self._add_numpy_noise(obs, joint_vel, joint_vel_scale)
        obs = self._add_numpy_noise(obs, free_lin_vel, free_lin_scale)
        obs = self._add_numpy_noise(obs, free_ang_vel, free_ang_scale)
        return obs, carry

    @staticmethod
    def _add_jax_noise(obs: jax.Array, indices: np.ndarray, scale: float, key: jax.Array) -> jax.Array:
        if scale <= 0.0 or indices.size == 0:
            return obs
        noise = jax.random.normal(key, shape=(indices.size,)) * scale
        return obs.at[indices].add(noise)

    @staticmethod
    def _add_numpy_noise(obs: np.ndarray, indices: np.ndarray, scale: float) -> np.ndarray:
        if scale <= 0.0 or indices.size == 0:
            return obs
        obs = obs.copy()
        obs[indices] += np.random.normal(size=indices.size) * scale
        return obs


if YogaObservationNoiseRandomizer.get_name() not in DomainRandomizer.registered:
    YogaObservationNoiseRandomizer.register()
