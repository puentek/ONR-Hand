class BulletPhysicsEngine(PhysicsEngine):
    """Physics engine API wrapper for Bullet."""

    def __init__(self):
        self._gravity = None
