# Building the Docker image 

:::{warning}
Be aware that building the CUDA Docker images (or the ROS images deriving from
it) requires the NVIDIA OptiX 7.7.0 package for `x86_64` architectures to be
located under the `docker/build/standalone/devel/third_party` directory in order
for the Chrono::Sensor dependency to be built correctly.

NVIDIA OptiX cannot be bundled directly within this repository as access to the
archive is restricted to users with a valid (free) NVIDIA developer login.
:::

Navigate to the `docker/build` folder and run the build script to generate all
tagged images:

```shell
cd docker/build
./build.sh
```

To generate a specific image, you can use the command below, by selecting an
option between {a/b}:

```shell
docker build --build-arg TARGET={cpu/cuda} \
             --tag <your-tag>
             {devel/runtime/ros}
```