# Camera Calibration: Extrinsics

## Introduction

This project is an example of how to perform camera calibration
of a robot inside the duckiematrix.
In this case, we are performing extrinsics calibration.

This example is designed to work properly with the map `loop_0` you find
in this repository.
Using this demo with a vehicle with different intrinsics camera
parameters from those in `map_0/vehicle_0` in `loop_0` will affect
the result.


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

You should see a matplotlib window pop up showing the image from the
robot's front camera.


## Step 4: Go to the laboratory

From the duckiematrix window: 
- select Mayor's View (top-right corner of the window);
- walk to the vehicle you see on the road;
- press [E] to mount the vehicle;
- use the keys [W], [A], [S], [D] to drive;
- drive to the laboratory's driveway, then press [F] to enter the building;
- select the Extrinsics Calibration tool (bottom-center of the window);


## Step 5: Wait for a good detection

The window we opened during Step 3 will show both the raw camera frame and the corners detected on the board. 
Let it sit until the best reprojection error (in the "_Reprojection Error(Best so far)_") reaches a value 
of less than `0.03cm`. Then click on [Save].


## Step 6: Verify the calibration

The validation window will now appear. The homography we saved in the previous step is now used
to warp the entire chessboard into a top-view. Verify that the entire chessboard is visible on the right.
