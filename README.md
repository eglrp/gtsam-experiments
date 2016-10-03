# gtsam-experiments

Experimenting with GTSAM


## Compiling and Running the Examples

For every example, a `CMakeLists.txt` file has been provided for compilation.

For instance, if you want to compile the `simpleRotation` example, you could do the following. These instructions assume that you are already in the root directory of the repository (the directory which contains this README).

```
cd simpleRotation
mkdir build && cd build
```

Edit the following line in `CMakeLists.txt` and replace the value of the variable `GTSAM_DIR` with the location of your GTSAM installation. For example, on line 4 of `CMakeLists.txt` inside the `simpleRotation` directory,

```
set(GTSAM_DIR "/home/km/libs/gtsam-3.2.1/build")
```
is the location on my computer.

Now, to compile the code, run the following.

```
cmake ..
make
```

To execute the `simpleRotation` example, run
```
./simpleExperiments
```
