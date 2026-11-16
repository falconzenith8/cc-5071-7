"""Wheel-ground contact boundary.

This package is the single place where a wheel's *support* is decided and
where suspension and tyre forces are produced.  The boundaries are explicit:

``frames``
    Contact coordinate interface: relative velocity (body translation plus
    rotational lever) and forces are exchanged as contact-frame components
    (normal / forward / lateral), never as ad-hoc world tuples at each call
    site.

``support``
    Explicit ray / sphere shape query.  A query *hit* is only a candidate
    support: the module still applies the finite suspension reach rule, so a
    hit beyond ``rest_length + max_travel`` is not treated as stable support.

``suspension``
    Spring/damper evaluation and the compression state that follows from a
    support probe.

``tire``
    Longitudinal slip, friction-circle limiting, lateral grip and wheel spin.

``module``
    Per-wheel orchestration plus the wheel state.  It consumes a body-like
    object (position, velocity, quaternion, angular velocity, mass) and
    returns force vectors; it never moves the chassis itself.

The legacy vehicle-contact entry point (``engine.vehicle.Vehicle``) is a
compatibility facade over :class:`WheelGroundModule`; its observable
behaviour is frozen and checked against the pre-refactor raw records in
``data/evidence/``.
"""

from .frames import ContactFrame, relative_velocity
from .support import SupportProbe, SupportQuery
from .suspension import (SuspensionParameters, SuspensionResult,
                         evaluate_suspension, support_compression)
from .tire import TireParameters, TireResult, evaluate_tire, update_steer
from .module import Wheel, WheelGroundModule
from .interface import classify_warnings, wheel_ground_state
from .legacy import LegacyVehicleAdapter

__all__ = [
    "ContactFrame", "relative_velocity",
    "SupportProbe", "SupportQuery",
    "SuspensionParameters", "SuspensionResult", "evaluate_suspension",
    "support_compression",
    "TireParameters", "TireResult", "evaluate_tire", "update_steer",
    "Wheel", "WheelGroundModule",
    "classify_warnings", "wheel_ground_state", "LegacyVehicleAdapter",
]
