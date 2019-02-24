
### Python Bindings for PhysX4

This project aims to expose all of PhysX 4.0 in Python as similarly to the C++ equivalent as possible, akin to how PyQt and PySide provides bindings for Qt. The advantage of such is that the C++ documentation applies to both C++ and Python users of the library.

```python
import PhysX4 as px
px.initPhysics()
px.stepPhysics(timestep=1.0 / 60)
px.cleanupPhysics()
```

**Examples**

- [`snippet_helloworld.py`](blob/master/python/snippet_helloworld.py)
- [`snippet_joint.py`](blob/master/python/snippet_joint.py)

![](https://user-images.githubusercontent.com/2152766/53303656-1ebbfe80-3865-11e9-9464-9777d6dbbdde.gif)

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

Launch the PhysX Visual Debugger and run any of the following.

```bash
$ cd PyPhysX4\python
$ python snippet_helloworld.py --steps 100 --size 10
$ python snippet_joint.py --steps 100 --length 5
```
