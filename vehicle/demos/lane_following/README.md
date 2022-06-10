# Demo: Lane Following

## Introduction

This demo is designed to work properly with the map `loop_0` you find 
in this repository.
Using this demo with a vehicle with different camera/kinematics 
parameters from those in `map_0/vehicle_0` in `loop_0` will affect 
the behavior.


## Step 1: Build the demo

Run the following command from inside this directory to build the demo.

```shell
dts devel build
```

## Step 2: Run the duckiematrix

Run the following command from inside the directory `maps/` you can 
find at the root of this repository to launch the duckiematrix on the
map `loop_0`.

```shell
dts matrix run --standalone --map ./loop_0
```

You should see a duckiematrix renderer pop up.


## Step 3: Run the demo

Run the following command from inside this directory to run the demo.

```shell
dts devel run -X
```

You should see a matplotlib window pop up and the vehicle inside the 
duckiematrix start moving.
