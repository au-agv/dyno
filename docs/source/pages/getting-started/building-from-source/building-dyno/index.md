# Building DYNO

## Building the library and the binary

Clone the DYNO repository and create a build folder inside of it

```shell
git clone https://github.com/aarhus-robotics/dyno.git dyno
mkdir -p dyno/build && cd dyno/build
```

Then, configure the project using plain CMake or the CMake curses GUI (shown in
the following):

```shell
ccmake ../
```

The following options are available:

* `BUILD_DEMOS`: Whether or not the included demos are built alongside the library.
* `VSG_INCLUDE_PATH`: The fully qualified path to the Vulkan Scene Graph (VSG)
  library include folder.

Once you are satisfied with your selections, generate your CMake configuration
file (using [G] in the case of the CMake Curses GUI) and build the DYNO targets:

```shell
cmake --build . -j$(nproc)
```

The DYNO CMake configuration file provides an install target which may be used to install DYNO globally on your system.
When the build is completed, you may also perform a system-wide installation by running the `install` target:

```shell
sudo cmake --build . --target install
```

Once DYNO is installed, you may use DYNO in your own CMake project by linking directly to the shared library:

```cmake
find_package(dyno REQUIRED)
target_link_libraries(my_project dyno)
```

## Building the Python modules (pydyno)

DYNO includes a set of Python modules, PyDyno, for post-processing results.

Install the [Poetry](https://python-poetry.org/) dependency manager for Python.
On Ubuntu 22.04, this will require installing the `pipx` package and using
`pipx` to install Poetry in an isolated environment:

```shell
sudo apt install -y pipx
pipx install poetry
```

You may then create and activate the bundled Python virtual environment for the
documentation:

```shell
poetry install --with documentation
```

A Python virtual environment (venv) specification is included in the DYNO
repository. You may instantiate and activate the virtual environment from the
root of the repository as follows:

```shell
python -m venv .venv/
./venv/bin/activate
pip install -r requirements.txt
```

You may now run any Python scripts in the repository to perform your
post-processing or uncertainty quantification.
