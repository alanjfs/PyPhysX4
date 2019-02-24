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

It also assumes Python 3.6 is installed at `c:\Python36`.

**Building**

The project generates a `PhysX4.pyd` file in the `bin\` directory of the repository, and copies relevant DLLs from the PhysX4 project.

```bash
$ cd PyPhysX4\compiler\vc140win64\PyPhysX4
$ msbuild PyPhysX4.sln /p:Configuration=Release;Platform=x64
```

<br>

### Usage

Launch the PhysX Visual Debugger and run the below snippet to try things out.

![pyphysx](https://user-images.githubusercontent.com/2152766/53298860-6b82e380-382b-11e9-86aa-dd6ececa3c99.gif)

**test.py**

```python
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

```