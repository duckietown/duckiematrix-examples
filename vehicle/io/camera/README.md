# I/O: Camera

## Introduction

This demo is designed to work properly with the map `loop_0` you find 
in this repository.
Using this demo with a different map might not produce the 
expected result as the requested vehicle might not be present
in the new map.


## Step 1: Build the demo

Run the following command from inside this directory to build the demo.

```shell
dts devel build --pull
```

## Step 2: Run the duckiematrix

Run the following command from inside this directory to launch the duckiematrix on the
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

You should see a matplotlib window pop up rendering the image captured 
by the vehicle's camera in the duckiematrix.
