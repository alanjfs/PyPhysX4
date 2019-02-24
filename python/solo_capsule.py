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
    px.createPlane()

    # Throw an actor at it
    ball = px.createDynamic(
        px.PxTransform(px.PxVec3(0, 5, 0), px.PxQuat(
            # Give it an angle
            angleRadians=math.radians(30),
            unitAxis=px.PxVec3(0, 0, 1))
        ),
        geometry=px.PxCapsuleGeometry(radius=0.5, halfHeight=0.5),
        linearVelocity=px.PxVec3(0, 5, 1),
        angularVelocity=px.PxVec3(0, 0, math.radians(200)),
    )

    t0 = time.time()
    timestep = 1.0 / 30
    for i in range(args.steps):
        px.stepPhysics(timestep)

        tm = px.getGlobalPose(ball)
        print(tm.p, tm.q)

    t1 = time.time()
    duration = t1 - t0

    px.cleanupPhysics()

    print(
        "Finished in %.2f ms (%d fps)"
        % (duration * 1000, args.steps / duration)
    )
