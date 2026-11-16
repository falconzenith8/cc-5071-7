"""Contact coordinate interface.

Both the relative velocity at a contact point and the force assembled at that
point are expressed in one orthonormal contact frame (normal / forward /
lateral).  Callers no longer pass raw world tuples around for the two sides of
the wheel-ground exchange; they decompose a world vector into frame components
and rebuild a world force from components with the same basis.
"""

from .. import math3d as V


def relative_velocity(body_velocity, body_angular_velocity, contact_offset):
    """Velocity of the body material point at ``contact_offset``.

    ``contact_offset`` is the contact point relative to the centre of mass, so
    the rotational lever ``omega x r`` is added to the body translation exactly
    once.
    """
    return V.add(body_velocity,
                 V.cross(body_angular_velocity, contact_offset))


class ContactFrame:
    """Orthonormal normal/forward/lateral basis at one wheel contact.

    ``normal`` is the terrain contact normal, ``forward`` the steered wheel
    heading.  ``lateral`` closes the right-handed triad and is returned
    normalised so force assembly is a plain component sum.
    """

    __slots__ = ("normal", "forward", "lateral")

    def __init__(self, normal, forward):
        self.normal = normal
        self.forward = forward
        self.lateral = V.norm(V.cross(normal, forward))

    def decompose(self, world_vector):
        """Return ``(normal, forward, lateral)`` components of a world vector."""
        return (V.dot(world_vector, self.normal),
                V.dot(world_vector, self.forward),
                V.dot(world_vector, self.lateral))

    def force(self, forward_component, lateral_component):
        """Rebuild a world-space tangential force from frame components."""
        return V.add(V.scale(self.forward, forward_component),
                     V.scale(self.lateral, lateral_component))
