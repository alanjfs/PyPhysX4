"""Throw a single capsule in the air, with some angular velocity"""

import time
import math
import PhysX4 as px

if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--steps", type=int, default=100)

    args = parser.parse_args()

    px.initPhysics()

    material = px.createMaterial(1, 1, 0)

    px.createStatic(
        px.PxTransform(
            px.PxVec3(0, 0, 0),
            px.PxQuat(px.PxHalfPi, px.PxVec3(0, 0, 1))
        ),
        shape=px.createShape(
            geometry=px.PxPlaneGeometry(),
            material=material,
        )
    )

    # Throw an actor at it
    ball = px.createDynamic(
        px.PxTransform(px.PxVec3(0, 5, 0), px.PxQuat(
            # Give it an angle
            angleRadians=math.radians(30),
            unitAxis=px.PxVec3(0, 0, 1))
        ),
        shape=px.createShape(
            geometry=px.PxCapsuleGeometry(radius=0.5, halfHeight=0.5),
            material=material
        )
    )

    ball.setLinearVelocity(px.PxVec3(0, 5, 1))
    ball.setAngularVelocity(px.PxVec3(0, 0, math.radians(200)))

    t0 = time.time()
    timestep = 1.0 / 30
    for i in range(args.steps):
        px.stepPhysics(timestep)

        tm = ball.getGlobalPose()
        print(tm.p, tm.q)

    t1 = time.time()
    duration = t1 - t0

    px.cleanupPhysics()

    print(
        "Finished in %.2f ms (%d fps)"
        % (duration * 1000, args.steps / duration)
    )
