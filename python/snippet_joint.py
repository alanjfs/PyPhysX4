import time
import PhysX4 as px


def createChain(t, length, geometry, separation, func):
    offset = px.PxVec3(separation / 2, 0, 0)
    localTm = px.PxTransform(offset)
    prev = 0

    for i in range(length):
        current = px.createDynamic(
            transform=t * localTm,
            geometry=geometry
        )

        func(
            parent=prev,
            parentFrame=px.PxTransform(offset) if prev != 0 else t,
            child=current,
            childFrame=px.PxTransform(-offset)
        )

        prev = current
        localTm.p.x += separation

    return current


if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--steps", type=int, default=100)
    parser.add_argument("--length", type=int, default=5)

    args = parser.parse_args()

    px.initPhysics()
    px.createPlane()

    createChain(
        px.PxTransform(px.PxVec3(0.0, 20.0, 0.0)),
        length=args.length,
        geometry=px.PxBoxGeometry(2.0, 0.5, 0.5),
        separation=4.0,
        func=px.createLimitedSpherical
    )
    createChain(
        px.PxTransform(px.PxVec3(0.0, 20.0, -10.0)),
        length=args.length,
        geometry=px.PxBoxGeometry(2.0, 0.5, 0.5),
        separation=4.0,
        func=px.createBreakableFixed
    )
    createChain(
        px.PxTransform(px.PxVec3(0.0, 20.0, -20.0)),
        length=args.length,
        geometry=px.PxBoxGeometry(2.0, 0.5, 0.5),
        separation=4.0,
        func=px.createDampedD6
    )

    count = args.length
    print("Simulating %d joints in %d steps.." % (count, args.steps))

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
