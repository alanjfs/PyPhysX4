import PhysX4 as px

if __name__ == '__main__':
    px.initPhysics()
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
    steps = 100
    timestep = 1.0 / 60
    for i in range(steps):
        print("Step %d" % i)
        px.stepPhysics(timestep)

    px.cleanupPhysics()
    print("Finished")
