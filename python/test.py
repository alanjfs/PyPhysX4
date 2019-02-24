import time
import PhysX4 as px

if __name__ == '__main__':
    px.initPhysics()

    px.createPlane(
        px.PxPlane(0, 1, 0, 0)
    )

    px.createStack(
        px.PxTransform(px.PxVec3(0, 0, 10.0)),
        size=5,
        halfExtent=2.0
    )

    px.createDynamic(
        px.PxTransform(px.PxVec3(0, 50, 100)),
        geometry=px.PxCapsuleGeometry(radius=5, halfHeight=5),
        velocity=px.PxVec3(0, -50, -100)
    )

    print("Simulating..")

    t0 = time.time()
    steps = 100
    timestep = 1.0 / 30
    for i in range(steps):
        px.stepPhysics(timestep)

    t1 = time.time()
    duration = t1 - t0

    px.cleanupPhysics()

    print(
        "Finished in %.2f ms (%d fps)"
        % (duration * 1000, steps / duration)
    )
