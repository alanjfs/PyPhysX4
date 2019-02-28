#include <map>
#include <string>

#include <pybind11/pybind11.h>
#include <pybind11/operators.h>

#include "PxPhysicsAPI.h"
#include "foundation/PxPreprocessor.h"
#include "foundation/PxSimpleTypes.h"

#define PVD_HOST "127.0.0.1"

using namespace physx;

typedef uint32_t UniqueId;
typedef std::map<uint32_t, PxRigidActor*> ActorMap;
typedef std::pair<uint32_t, PxRigidActor*> ActorPair;
typedef std::map<uint32_t, PxJoint*> JointMap;
typedef std::pair<uint32_t, PxJoint*> JointPair;

PxDefaultAllocator      gAllocator;
PxDefaultErrorCallback  gErrorCallback;

PxFoundation*           gFoundation = NULL;
PxPhysics*              gPhysics = NULL;

PxDefaultCpuDispatcher* gDispatcher = NULL;
PxScene*                gScene = NULL;

PxMaterial*             gMaterial = NULL;

PxPvd*                  gPvd = NULL;

// State
bool                    gInitialised = false;

void initPhysics(const PxTolerancesScale tolerances = PxTolerancesScale())
{

    if (gInitialised)
    {
        throw std::exception("Already initialised");
    }

    gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);

    gPvd = PxCreatePvd(*gFoundation);
    PxPvdTransport* transport = PxDefaultPvdSocketTransportCreate(PVD_HOST, 5425, 10);
    gPvd->connect(*transport, PxPvdInstrumentationFlag::eALL);

    gPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, tolerances, true, gPvd);
    PxInitExtensions(*gPhysics, gPvd);

    PxSceneDesc sceneDesc(gPhysics->getTolerancesScale());
    sceneDesc.gravity = PxVec3(0.0f, -9.81f, 0.0f);
    gDispatcher = PxDefaultCpuDispatcherCreate(2);
    sceneDesc.cpuDispatcher = gDispatcher;
    sceneDesc.filterShader = PxDefaultSimulationFilterShader;
    gScene = gPhysics->createScene(sceneDesc);

    PxPvdSceneClient* pvdClient = gScene->getScenePvdClient();
    if (pvdClient)
    {
        pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS, true);
        pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONTACTS, true);
        pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES, true);
    }
    gMaterial = gPhysics->createMaterial(0.5f, 0.5f, 0.6f);

    gInitialised = true;
}

void stepPhysics(const PxReal timestep)
{
    gScene->simulate(timestep);
    gScene->fetchResults(true);
}

void cleanupPhysics()
{
    gScene->release();
    gDispatcher->release();
    gPhysics->release();
    PxPvdTransport* transport = gPvd->getTransport();
    gPvd->release();
    transport->release();
    PxCloseExtensions();
    gFoundation->release();
}

class PyMaterial
{
public:
    PyMaterial() {}
    PyMaterial(PxMaterial* material) : mRef(material) {}
    void getStaticFriction() { mRef->getStaticFriction(); }

    const PxMaterial* mRef{ NULL };
};

class PyShape
{
public:
    PyShape() {}
    PyShape(PxShape* shape) : mRef(shape) {}

    PxShape* mRef{ NULL };
};


class PyRigidDynamic
{
public:
    PyRigidDynamic() {}
    PyRigidDynamic(PxRigidDynamic* actor) : mRef(actor) {}

    PxTransform getGlobalPose() { return mRef->getGlobalPose(); }
    void setAngularDamping(PxReal value) { mRef->setAngularDamping(value); }
    void setLinearVelocity(PxVec3 value) { mRef->setLinearVelocity(value); }
    void setAngularVelocity(PxVec3 value) { mRef->setAngularVelocity(value); }
    bool isValid() { return mRef != NULL; }

    PxRigidDynamic* mRef{ NULL };
};


class PyRigidStatic
{
public:
    PyRigidStatic() {}
    PyRigidStatic(PxRigidStatic* actor) : mRef(actor) {}

    PxRigidStatic* mRef{ NULL };
};

// class PyJoint
// {
// public:
//     void setBreakForce (PxReal force, PxReal torque) { mRef->setBreakForce(force, torque); }
//     void setConstraintFlag(PxConstraintFlag::Enum flag, bool value) { mRef->setConstraintFlag(flag, value); }
// };

class PyD6Joint
{
public:
    PyD6Joint() {}
    PyD6Joint(PxD6Joint* joint) : mRef(joint) {}
    void setMotion(PxD6Axis::Enum axis, PxD6Motion::Enum type) { mRef->setMotion(axis, type); }
    void setDrive(PxD6Drive::Enum index, const PxD6JointDrive& drive) {mRef->setDrive(index, drive); }

    PxD6Joint* mRef{ NULL };
};

class PySphericalJoint
{
public:
    PySphericalJoint() {}
    PySphericalJoint(PxSphericalJoint* joint) : mRef(joint) {}
    void setLimitCone(const PxJointLimitCone &limit) { mRef->setLimitCone(limit); }
    void setSphericalJointFlag (PxSphericalJointFlag::Enum flag, bool value) { mRef->setSphericalJointFlag(flag, value); }

    PxSphericalJoint* mRef{ NULL };
};


class PyFixedJoint
{
public:
    PyFixedJoint() {}
    PyFixedJoint(PxFixedJoint* joint) : mRef(joint) {}

    void setBreakForce (PxReal force, PxReal torque) { mRef->setBreakForce(force, torque); }
    void setConstraintFlag(PxConstraintFlag::Enum flag, bool value) { mRef->setConstraintFlag(flag, value); }

    PxFixedJoint* mRef{ NULL };
};


PyMaterial createMaterial(PxReal staticFriction,
                          PxReal dynamicFriction,
                          PxReal restitution)
{
    PxMaterial* material = gPhysics->createMaterial(staticFriction, dynamicFriction, restitution);
    return PyMaterial(material);
}

PyShape createShape(const PxGeometry& geometry,
                    const PyMaterial& material,
                    bool isExclusive = false)
{
    PxShapeFlags shapeFlags = PxShapeFlag::eVISUALIZATION |
                              PxShapeFlag::eSCENE_QUERY_SHAPE |
                              PxShapeFlag::eSIMULATION_SHAPE;
    PxShape* shape = gPhysics->createShape(geometry, *material.mRef, isExclusive, shapeFlags);
    return PyShape(shape);
}

PyRigidDynamic createDynamic(const PxTransform& t,
                             const PyShape& shape,
                             const PxReal density = PxReal(1.0f))
{
    PxRigidDynamic* actor = PxCreateDynamic(*gPhysics, t, *shape.mRef, density);
    gScene->addActor(*actor);

    return PyRigidDynamic(actor);
}

PyRigidStatic createStatic(const PxTransform& t,
                           const PyShape& shape)
{
    PxRigidStatic* actor = PxCreateStatic(*gPhysics, t, *shape.mRef);
    gScene->addActor(*actor);

    return PyRigidStatic(actor);
}


void createPlane(PxPlane& plane)
{
    PxRigidStatic* groundPlane = PxCreatePlane(*gPhysics, plane, *gMaterial);
    gScene->addActor(*groundPlane);
}

PyD6Joint createD6Joint(const PyRigidDynamic& parent,
                        const PxTransform& parentFrame,
                        const PyRigidDynamic& child,
                        const PxTransform& childFrame)
{
    PxD6Joint* joint = PxD6JointCreate(*gPhysics,
                                       parent.mRef,
                                       parentFrame,
                                       child.mRef,
                                       childFrame);

    return PyD6Joint(joint);
}

PySphericalJoint createSphericalJoint(const PyRigidDynamic& parent,
                                      const PxTransform& parentFrame,
                                      const PyRigidDynamic& child,
                                      const PxTransform& childFrame)
{
    PxSphericalJoint* joint = PxSphericalJointCreate(*gPhysics,
                                                     parent.mRef,
                                                     parentFrame,
                                                     child.mRef,
                                                     childFrame);
    return PySphericalJoint(joint);

}

PyFixedJoint createFixedJoint(const PyRigidDynamic& parent,
                                      const PxTransform& parentFrame,
                                      const PyRigidDynamic& child,
                                      const PxTransform& childFrame)
{
    PxFixedJoint* joint = PxFixedJointCreate(*gPhysics,
                                             parent.mRef,
                                             parentFrame,
                                             child.mRef,
                                             childFrame);

    return PyFixedJoint(joint);
}

namespace py = pybind11;

PYBIND11_MODULE(PhysX4, m) {
    m.doc() = "PhysX4 Python Bindings";

    // ====================================================================
    // 
    // Globals
    //
    // ====================================================================

    m.attr("PxPi") = PxPi;
    m.attr("PxHalfPi") = PxHalfPi;
    m.attr("FLT_MAX") = FLT_MAX;

    // ====================================================================
    // 
    // Enums
    //
    // ====================================================================

    py::enum_<PxShapeFlag::Enum>(m, "PxShapeFlag")
        .value("eSIMULATION_SHAPE", PxShapeFlag::Enum::eSIMULATION_SHAPE)
        .value("eSCENE_QUERY_SHAPE", PxShapeFlag::Enum::eSCENE_QUERY_SHAPE)
        .value("eTRIGGER_SHAPE", PxShapeFlag::Enum::eTRIGGER_SHAPE)
        .value("eVISUALIZATION", PxShapeFlag::Enum::eVISUALIZATION);

    py::enum_<PxD6Axis::Enum>(m, "PxD6Axis")
        .value("eX", PxD6Axis::Enum::eX)
        .value("eY", PxD6Axis::Enum::eY)
        .value("eZ", PxD6Axis::Enum::eZ)
        .value("eTWIST", PxD6Axis::Enum::eTWIST)
        .value("eSWING1", PxD6Axis::Enum::eSWING1)
        .value("eSWING2", PxD6Axis::Enum::eSWING2)
        .value("eCOUNT", PxD6Axis::Enum::eCOUNT);

    py::enum_<PxD6Drive::Enum>(m, "PxD6Drive")
        .value("eX", PxD6Drive::Enum::eX)
        .value("eY", PxD6Drive::Enum::eY)
        .value("eZ", PxD6Drive::Enum::eZ)
        .value("eSWING", PxD6Drive::Enum::eSWING)
        .value("eTWIST", PxD6Drive::Enum::eTWIST)
        .value("eSLERP", PxD6Drive::Enum::eSLERP)
        .value("eCOUNT", PxD6Drive::Enum::eCOUNT);

    py::enum_<PxD6Motion::Enum>(m, "PxD6Motion")
        .value("eLOCKED", PxD6Motion::Enum::eLOCKED)
        .value("eLIMITED", PxD6Motion::Enum::eLIMITED)
        .value("eFREE", PxD6Motion::Enum::eFREE);

    py::enum_<PxConstraintFlag::Enum>(m, "PxConstraintFlag")
        .value("eBROKEN", PxConstraintFlag::eBROKEN)
        .value("ePROJECT_TO_ACTOR0", PxConstraintFlag::ePROJECT_TO_ACTOR0)
        .value("ePROJECT_TO_ACTOR1", PxConstraintFlag::ePROJECT_TO_ACTOR1)
        .value("ePROJECTION", PxConstraintFlag::ePROJECTION)
        .value("eCOLLISION_ENABLED", PxConstraintFlag::eCOLLISION_ENABLED)
        .value("eVISUALIZATION", PxConstraintFlag::eVISUALIZATION)
        .value("eDRIVE_LIMITS_ARE_FORCES", PxConstraintFlag::eDRIVE_LIMITS_ARE_FORCES)
        .value("eIMPROVED_SLERP", PxConstraintFlag::eIMPROVED_SLERP)
        .value("eDISABLE_PREPROCESSING", PxConstraintFlag::eDISABLE_PREPROCESSING)
        .value("eENABLE_EXTENDED_LIMITS", PxConstraintFlag::eENABLE_EXTENDED_LIMITS)
        .value("eGPU_COMPATIBLE", PxConstraintFlag::eGPU_COMPATIBLE);

    py::enum_<PxSphericalJointFlag::Enum>(m, "PxSphericalJointFlag")
        .value("eLIMIT_ENABLED", PxSphericalJointFlag::Enum::eLIMIT_ENABLED);

    // ====================================================================
    // 
    // Types
    //
    // ====================================================================

    py::class_<PxVec3>(m, "PxVec3")
        .def(py::init<>())
        .def(py::init<const PxReal,
                      const PxReal,
                      const PxReal>())
        .def(py::init<const PxVec3 &>())
        .def_readwrite("x", &PxVec3::x)
        .def_readwrite("y", &PxVec3::y)
        .def_readwrite("z", &PxVec3::z)

        .def(py::self + py::self)
        .def(py::self - py::self)
        .def(py::self += py::self)
        .def(py::self -= py::self)
        .def(py::self *= float())
        .def(py::self /= float())
        .def(py::self * float())
        .def(py::self / float())

        .def("__neg__",   [](PxVec3 v) { return -v; }, py::is_operator())

        // Support print(vec)
        .def("__repr__", [](const PxVec3 &a) {
            return "<PhysX4.PxVec3( " +
                   std::to_string(a.x) + ", " +
                   std::to_string(a.y) + ", " +
                   std::to_string(a.z) + " )>";
        }
    );

    py::class_<PxQuat>(m, "PxQuat")
        .def(py::init<const PxReal,
                      const PxReal,
                      const PxReal,
                      const PxReal>())
        .def(py::init<const PxQuat &>())
        .def(py::init<const float, const PxVec3>(),
             py::arg("angleRadians"),
             py::arg("unitAxis"))
        .def_readwrite("x", &PxQuat::x)
        .def_readwrite("y", &PxQuat::y)
        .def_readwrite("z", &PxQuat::z)
        .def_readwrite("w", &PxQuat::w)

        .def("__repr__", [](const PxQuat &q) {
            return "<PhysX4.PxQuat( " +
                   std::to_string(q.x) + ", " +
                   std::to_string(q.y) + ", " +
                   std::to_string(q.z) + ", " +
                   std::to_string(q.w) + " )>";
        }
    );

    py::class_<PxPlane>(m, "PxPlane")
        .def(py::init<const PxReal, const PxReal, const PxReal, const PxReal>(),
             py::arg("nx"),
             py::arg("ny"),
             py::arg("nz"),
             py::arg("distance"))
        .def(py::init<const PxPlane &>());

    py::class_<PxJointLimitCone>(m, "PxJointLimitCone")
        .def(py::init<const PxReal, const PxReal, const PxReal>(),
             py::arg("yLimitAngle"),
             py::arg("zLimitAngle"),
             py::arg("contactDist") = -1.0f)
        .def(py::init<const PxJointLimitCone &>())

        .def_readwrite("yAngle", &PxJointLimitCone::yAngle)
        .def_readwrite("zAngle", &PxJointLimitCone::zAngle);

    py::class_<PxTransform>(m, "PxTransform")
        .def(py::init([]() { return PxTransform(PxVec3(0, 0, 0)); }))
        .def(py::init<const PxVec3 &>())
        .def(py::init<const PxVec3 &, const PxQuat &>())

        .def(py::self * py::self)

        // Support passing position
        .def(py::init([](const PxReal x,
                         const PxReal y,
                         const PxReal z) {
            return PxTransform(PxVec3(x, y, z));
        }))
        .def("transform", [](const PxTransform &self, const PxTransform &other) {
            return self.transform(other);
        })
        .def_readwrite("p", &PxTransform::p)
        .def_readwrite("q", &PxTransform::q);

    py::class_<PxTolerancesScale>(m, "PxTolerancesScale")
        .def(py::init<>())
        .def_readwrite("length", &PxTolerancesScale::length)
        .def_readwrite("speed", &PxTolerancesScale::speed);

    py::class_<PxD6JointDrive>(m, "PxD6JointDrive")
        .def(py::init<PxReal,
                      PxReal,
                      PxReal,
                      bool>(),
                      py::arg("driveStiffness"),
                      py::arg("driveDamping"),
                      py::arg("driveForceLimit"),
                      py::arg("isAcceleration"));

    // Needed for below subclasses
    py::class_<PxGeometry>(m, "PxGeometry");

    py::class_<PxSphereGeometry, PxGeometry>(m, "PxSphereGeometry")
        .def(py::init<const PxReal>(),
             py::arg("radius"));

    py::class_<PxCapsuleGeometry, PxGeometry>(m, "PxCapsuleGeometry")
        .def(py::init<const PxReal, const PxReal>(),
             py::arg("radius"),
             py::arg("halfHeight"));

    py::class_<PxPlaneGeometry, PxGeometry>(m, "PxPlaneGeometry")
        .def(py::init<>());

    py::class_<PxBoxGeometry, PxGeometry>(m, "PxBoxGeometry")
        .def(py::init<const PxReal,
                      const PxReal,
                      const PxReal>(),
                      py::arg("x") = 1,
                      py::arg("y") = 1,
                      py::arg("z") = 1);

    py::class_<PyMaterial>(m, "PxMaterial")
        .def(py::init<>())
        .def("getStaticFriction", &PyMaterial::getStaticFriction);

    py::class_<PyShape>(m, "PxShape")
        .def(py::init<>());

    py::class_<PyRigidDynamic>(m, "PxRigidDynamic")
        .def(py::init<>())
        .def("isValid", &PyRigidDynamic::isValid)
        .def("setAngularDamping", &PyRigidDynamic::setAngularDamping)
        .def("setLinearVelocity", &PyRigidDynamic::setLinearVelocity)
        .def("setAngularVelocity", &PyRigidDynamic::setAngularVelocity)
        .def("getGlobalPose", &PyRigidDynamic::getGlobalPose)

        .def("__bool__", [](PyRigidDynamic self) {
            return self.isValid();
        })
    ;

    py::class_<PyRigidStatic>(m, "PxRigidStatic")
        .def(py::init<>());

    py::class_<PyD6Joint>(m, "PxD6Joint")
        .def(py::init<>())
        .def("setMotion", &PyD6Joint::setMotion)
        .def("setDrive", &PyD6Joint::setDrive)
    ;

    py::class_<PySphericalJoint>(m, "PxSphericalJoint")
        .def(py::init<>())
        .def("setLimitCone", &PySphericalJoint::setLimitCone,
             py::arg("limit"))
        .def("setSphericalJointFlag", &PySphericalJoint::setSphericalJointFlag,
             py::arg("flag"),
             py::arg("value"))
    ;

    py::class_<PyFixedJoint>(m, "PxFixedJoint")
        .def(py::init<>())
        .def("setBreakForce", &PyFixedJoint::setBreakForce)
        .def("setConstraintFlag", &PyFixedJoint::setConstraintFlag)
    ;

    // ===============================================================
    // 
    // API
    // 
    // ===============================================================

    m.def("initPhysics", &initPhysics, "Init physics",
          py::arg("tolerances") = PxTolerancesScale());

    m.def("cleanupPhysics", &cleanupPhysics, "Cleanup physics");

    m.def("stepPhysics", &stepPhysics, "Step physics",
          py::arg("timestep"));

    m.def("createPlane", &createPlane, "Create a plane",
          py::arg("plane") = PxPlane(0, 1, 0, 0));

    m.def("createMaterial", &createMaterial,
         py::arg("staticFriction") = 1,
         py::arg("dynamicFriction") = 1,
         py::arg("restitution") = 0);

    m.def("createShape", &createShape,
         py::arg("geometry"),
         py::arg("material"),
         py::arg("isExclusive") = false
    );

    m.def("createDynamic", &createDynamic, "Create a dynamic actor",
          py::arg("transform"),
          py::arg("shape"),
          py::arg("density") = PxReal(1)
    );

    m.def("createStatic", &createStatic, "Create a static actor",
          py::arg("transform"),
          py::arg("shape")
    );

    m.def("createD6Joint", &createD6Joint, "Create a D6 joint between two actors",
          py::arg("parent"),
          py::arg("parentFrame"),
          py::arg("child"),
          py::arg("childFrame")
    );

    m.def("createSphericalJoint", &createSphericalJoint, "Create a spherical joint between two actors",
          py::arg("parent"),
          py::arg("parentFrame"),
          py::arg("child"),
          py::arg("childFrame")
    );

    m.def("createFixedJoint", &createFixedJoint, "Create a fixed joint between two actors",
          py::arg("parent"),
          py::arg("parentFrame"),
          py::arg("child"),
          py::arg("childFrame")
    );
}
