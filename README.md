### Python Bindings for PhysX4

This project aims to expose all of PhysX 4.0 in Python as similarly to the C++ equivalent as possible, akin to how PyQt5 provides bindings for Qt. The advantage of such is that the C++ documentation applies to both C++ and Python users of the library.

```python
import PhysX4 as px
px.initPhysics()
px.stepPhysics(timestep=1.0 / 60)
px.cleanupPhysics()
```

<br>

### Building on Windows

A Visual Studio 2015 project is provided for reference with the repository.

**Prerequisities**

Prior to running Visual Studio, the following environment variables must be set.

- `set PROJECTROOT=c:\path\to\repo`
- `set PROJECTEXTERNALS=c:\path\to\externals`

Where `\externals` contains a [PhysX](https://github.com/NVIDIAGameWorks/PhysX) project build with the `checked` profile, along with the [pybind11](https://github.com/pybind/pybind11) headers.

For example.

```bash
$ externals\PhysX\physx\bin\win.x86_64.vc140.mt\checked\PhysX4_64.dll
$ externals\pybind11\include\pybind11\pybind11.h
```

**Building**

The project generates a `PhysX4.pyd` file in the `bin\` directory of the repository, and copies relevant DLLs from the PhysX4 project.

```bash
$ cd PyPhysX4\compiler\vc140win64\PyPhysX4
$ msbuild PyPhysX4.sln /p:Configuration=Release;Platform=x64
```

### Usage

Launch the PhysX Visual Debugger and run the below snippet to try things out.

**test.py**

```python
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
```