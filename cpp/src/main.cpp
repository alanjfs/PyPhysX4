#include <ctype.h>
#include <string>

#include <pybind11/pybind11.h>
#include <pybind11/operators.h>

#include "PxPhysicsAPI.h"
#include "foundation/PxPreprocessor.h"
#include "foundation/PxSimpleTypes.h"

#define PVD_HOST "127.0.0.1"    // Set this to the IP address of the system running the PhysX Visual Debugger that you want to connect to.

using namespace physx;

PxDefaultAllocator      gAllocator;
PxDefaultErrorCallback  gErrorCallback;

PxFoundation*           gFoundation = NULL;
PxPhysics*              gPhysics = NULL;

PxDefaultCpuDispatcher* gDispatcher = NULL;
PxScene*                gScene = NULL;

PxMaterial*             gMaterial = NULL;

PxPvd*                  gPvd = NULL;



void createDynamic(const PxTransform& t,
                   const PxGeometry& geometry,
                   const PxVec3& velocity = PxVec3(0))
{
    PxRigidDynamic* dynamic = PxCreateDynamic(*gPhysics, t, geometry, *gMaterial, 10.0f);
    dynamic->setAngularDamping(0.5f);
    dynamic->setLinearVelocity(velocity);
    gScene->addActor(*dynamic);
}

void createPlane(PxPlane& plane)
{
    PxRigidStatic* groundPlane = PxCreatePlane(*gPhysics, plane, *gMaterial);
    gScene->addActor(*groundPlane);
}

void initPhysics()
{
    gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);

    gPvd = PxCreatePvd(*gFoundation);
    PxPvdTransport* transport = PxDefaultPvdSocketTransportCreate(PVD_HOST, 5425, 10);
    gPvd->connect(*transport, PxPvdInstrumentationFlag::eALL);

    gPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, PxTolerancesScale(), true, gPvd);

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
    gFoundation->release();
}

namespace py = pybind11;

PYBIND11_MODULE(PhysX4, m) {
    m.doc() = "PhysX4 Python Bindings"; // optional module docstring

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
        .def(py::self += py::self)
        .def(py::self -= py::self)
        .def(py::self *= float())
        .def(py::self /= float())
        .def(py::self * float())
        .def(py::self / float())

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
        .def_readwrite("x", &PxQuat::x)
        .def_readwrite("y", &PxQuat::y)
        .def_readwrite("z", &PxQuat::z)
        .def_readwrite("w", &PxQuat::w)
        .def("__repr__", [](const PxQuat &a) {
            return "<PhysX4.PxQuat( " +
                   std::to_string(a.x) + ", " +
                   std::to_string(a.y) + ", " +
                   std::to_string(a.z) + ", " +
                   std::to_string(a.w) + " )>";
        }
    );

    py::class_<PxPlane>(m, "PxPlane")
        .def(py::init<const PxReal, const PxReal, const PxReal, const PxReal>(),
             py::arg("nx"),
             py::arg("ny"),
             py::arg("nz"),
             py::arg("distance"))
        .def(py::init<const PxPlane &>());

    py::class_<PxTransform>(m, "PxTransform")
        .def(py::init<>())
        .def(py::init<const PxVec3 &>())

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

    // Needed for below subclasses
    py::class_<PxGeometry>(m, "PxGeometry");

    py::class_<PxSphereGeometry, PxGeometry>(m, "PxSphereGeometry")
        .def(py::init<const PxReal>(),
             py::arg("radius"));

    py::class_<PxCapsuleGeometry, PxGeometry>(m, "PxCapsuleGeometry")
        .def(py::init<const PxReal, const PxReal>(),
             py::arg("radius"),
             py::arg("halfHeight"));

    py::class_<PxBoxGeometry, PxGeometry>(m, "PxBoxGeometry")
        .def(py::init<const PxReal,
                      const PxReal,
                      const PxReal>(),
                      py::arg("x") = 1,
                      py::arg("y") = 1,
                      py::arg("z") = 1);

    // API
    m.def("initPhysics", &initPhysics, "Init physics");
    m.def("cleanupPhysics", &cleanupPhysics, "Cleanup physics");

    m.def("stepPhysics", &stepPhysics, "Step physics",
          py::arg("timestep"));

    m.def("createPlane", &createPlane, "Create a plane",
          py::arg("plane") = PxPlane(0, 1, 0, 0));

    m.def("createDynamic", &createDynamic, "Create a dynamic actor",
          py::arg("transform"),
          py::arg("geometry") = PxBoxGeometry(1, 1, 1),
          py::arg("velocity") = PxVec3(0, 0, 0));
}
