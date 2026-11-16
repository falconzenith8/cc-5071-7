"""Suspension spring/damper boundary.

This module owns the compression state rule and the force evaluation.  It does
not know about terrain queries or tyre forces; it is handed a compression and
its velocity and returns the normal-direction force.
"""

from math import sqrt

from .. import math3d as V


class SuspensionParameters:
    __slots__ = ("rest_length", "max_travel", "spring_k", "tau")

    def __init__(self, rest_length=0.35, max_travel=0.22, spring_k=32000.0,
                 tau=0.12):
        self.rest_length = rest_length
        self.max_travel = max_travel
        self.spring_k = spring_k
        self.tau = tau

    def reach(self):
        return self.rest_length + self.max_travel

    def support_compression(self, distance):
        """Compression from the rest position, clamped to the travel limit."""
        return V.clamp(self.rest_length - distance, 0.0, self.max_travel)

    def effective_damping(self, sprung_mass):
        """Damper coefficient ``c`` from spring ``k`` and time constant ``tau``.

        ``zeta = c / (2 sqrt(k m))``; the requested time constant is mapped onto
        the damping ratio and clamped to a stable positive range.
        """
        omega_n = sqrt(self.spring_k / max(sprung_mass, 1e-6))
        zeta = 1.0 / max(self.tau * omega_n, 0.05)
        zeta = V.clamp(zeta, 0.15, 3.0)
        c = 2.0 * zeta * sqrt(self.spring_k * max(sprung_mass, 1e-6))
        return c, zeta


class SuspensionResult:
    __slots__ = ("c", "zeta", "spring_force", "damper_force", "magnitude",
                 "force")

    def __init__(self, c, zeta, spring_force, damper_force, magnitude, force):
        self.c = c
        self.zeta = zeta
        self.spring_force = spring_force
        self.damper_force = damper_force
        self.magnitude = magnitude
        self.force = force


def support_compression(params, distance):
    return params.support_compression(distance)


def evaluate_suspension(params, compression, compression_vel, normal,
                        sprung_mass):
    """Spring + damper force along the contact normal.

    ``compression_vel > 0`` means the wheel is moving into the spring; the
    damper must then push back harder, so the combined magnitude is clamped at
    zero rather than allowed to cancel the spring.
    """
    c, zeta = params.effective_damping(sprung_mass)
    spring_force = params.spring_k * compression
    damper_force = c * compression_vel
    magnitude = max(spring_force + damper_force, 0.0)
    return SuspensionResult(c, zeta, spring_force, damper_force, magnitude,
                            V.scale(normal, magnitude))


def relax_airborne(params, compression, dt):
    """Relax an unsupported wheel toward full extension.

    Returns ``(new_compression, compression_velocity)``.  The previous surface
    is never retained as support.
    """
    rate = -compression / max(params.tau, 1e-3)
    return max(0.0, compression + rate * dt), rate
