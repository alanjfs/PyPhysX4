import time
import PhysX4 as px


def createD6DampedJoint(parent, parentFrame, child, childFrame):
    joint = px.createD6Joint(parent, parentFrame, child, childFrame)
    joint.setMotion(px.PxD6Axis.eSWING1, px.PxD6Motion.eFREE)
    joint.setMotion(px.PxD6Axis.eSWING2, px.PxD6Motion.eFREE)
    joint.setMotion(px.PxD6Axis.eTWIST, px.PxD6Motion.eFREE)

    drive = px.PxD6JointDrive(0, 1000, px.FLT_MAX, True)
    joint.setDrive(px.PxD6Drive.eSLERP, drive)

    return joint


def createLimitedSphericalJoint(parent, parentFrame, child, childFrame):
    joint = px.createSphericalJoint(parent, parentFrame, child, childFrame)

    joint.setLimitCone(px.PxJointLimitCone(px.PxPi / 4, px.PxPi / 4, 0.05))
    joint.setSphericalJointFlag(px.PxSphericalJointFlag.eLIMIT_ENABLED, True)

    return joint


def createBreakableFixedJoint(parent, parentFrame, child, childFrame):
    joint = px.createFixedJoint(parent, parentFrame, child, childFrame)

    joint.setBreakForce(1000, 100000)
    joint.setConstraintFlag(px.PxConstraintFlag.eDRIVE_LIMITS_ARE_FORCES, True)
    joint.setConstraintFlag(px.PxConstraintFlag.eDISABLE_PREPROCESSING, True)

    return joint


def createChain(t, length, geometry, separation, func):
    offset = px.PxVec3(separation / 2, 0, 0)
    localTm = px.PxTransform(offset)
    prev = px.PxRigidDynamic()

    for i in range(length):
        current = px.createDynamic(
            transform=t * localTm,
            shape=px.createShape(
                geometry=geometry,
                material=px.createMaterial()
            )
        )

        func(
            parent=prev,
            parentFrame=px.PxTransform(offset) if prev else t,
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
        func=createLimitedSphericalJoint
    )
    createChain(
        px.PxTransform(px.PxVec3(0.0, 20.0, -10.0)),
        length=args.length,
        geometry=px.PxBoxGeometry(2.0, 0.5, 0.5),
        separation=4.0,
        func=createBreakableFixedJoint
    )
    createChain(
        px.PxTransform(px.PxVec3(0.0, 20.0, -20.0)),
        length=args.length,
        geometry=px.PxBoxGeometry(2.0, 0.5, 0.5),
        separation=4.0,
        func=createD6DampedJoint
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
