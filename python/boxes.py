import time
import PhysX4 as px


def createStack(t, size, halfExtent):
    """Create a stack of boxes"""

    for i in range(size):
        for j in range(size - i):
            pos = px.PxVec3(
                j * 2 - size + i,
                i * 2 + 1,
                0
            ) * halfExtent

            px.createDynamic(
                t.transform(px.PxTransform(pos)),
                geometry=px.PxBoxGeometry(halfExtent, halfExtent, halfExtent),
            )


if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--steps", type=int, default=100)
    parser.add_argument("--size", type=int, default=10)

    args = parser.parse_args()

    px.initPhysics()
    px.createPlane()

    createStack(
        px.PxTransform(px.PxVec3(0, 0, -30.0)),
        size=args.size,
        halfExtent=2.0
    )

    # Throw an actor at it
    px.createDynamic(
        px.PxTransform(px.PxVec3(0, 50, 100)),
        geometry=px.PxCapsuleGeometry(radius=5, halfHeight=5),
        velocity=px.PxVec3(0, -50, -100)
    )

    count = (args.size ** 2) / 2 + args.size / 2
    print("Simulating %d boxes in %d steps.." % (count, args.steps))

    t0 = time.time()
    timestep = 1.0 / 30
    for i in range(args.steps):
        px.stepPhysics(timestep)

    t1 = time.time()
    duration = t1 - t0

    px.cleanupPhysics()

    print(
        "Finished in %.2f ms (%d fps)"
        % (duration * 1000, args.steps / duration)
    )
