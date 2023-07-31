# Demo: Lane Following

## Introduction

This demo is designed to work properly with the map `loop_0` you find 
in this repository.
Using this demo with a vehicle with different camera/kinematics 
parameters from those in `map_0/vehicle_0` in `loop_0` will affect 
the behavior.

### Step 1: Build the demo

Run the following command from inside this directory to build the demo.

```shell
dts devel build --pull
```

### Step 2: Run the Duckiematrix

Run the following command from inside the directory `maps/` you can 
find at the root of this repository to launch the Duckiematrix on the
map `loop_0`.

```shell
dts matrix run --standalone --map --gym ../../../maps/loop_0
```

```{warning}
The gym mode is still in alpha and it might not work. You can still use the gym
api to control the matrix entities in `realtime` mode by omitting the `--gym` flag.
```

You should see a Duckiematrix renderer pop up.


### Step 3: Run the demo

Run the following command from inside this directory to run the demo.

```shell
dts devel run -X
```

You should see a matplotlib window pop up and the vehicle inside the 
Duckiematrix start moving at random.

## Using the Duckiematrix Gymnasium API

The Duckiematrix Gymnasium API is an interface based on the [Gymnasium API](https://gymnasium.farama.org/) to control the Duckiematrix. This API is a de-facto standard for RL research and provides a simple way
to interact with the environment.

When the Gymnasium environment is instantiated it connects to a running Duckiematrix instance.

### Instantiating an environment

The environment can be instantiated from the `DuckietownEnv` class. You can control the instance of the 
Duckiematrix to connect to by providing the `matrix_hostname` argument at instantiation. The `entities` argument
tells the `DuckietownEnv` which entities to control in the Duckiematrix and it can be a tuple of `str` or `DifferentialDriveRobot` types. In this way you can connect to the entities by both referring to them by their 
entity name or by providing a `DifferentialDriveRobot` object connected to them.

You can provide a `render_mode` to the constructor method, according to the Gymnasium specifications.

```python
env = DuckietownEnv(entities=('map_0/vehicle_0',), render_mode='human')
```

```{warning}
Support for multiple enitities is still in alpha, expect it to have bugs.
```
### Controlling the environment

The first call to a Gymnasium environment has to be the `.reset()` method. This method resets the environment 
and returns an initial observation and info dictionary.

The observations returned by the environment are `Tuple[np.ndarray]` (i.e. a tuple of numpy arrays) with as many elements as the entities we have connected to. The arrays contain the image from the ego-centric perspective of the Duckiebot entity. 

To control the environment we can provide a `Tuple[np.ndarray]` of actions for each entity. Each numpy array contains two normalized values to control the parallel and angular velocity.

### Rendering

The rendering mode is selected at instantiation time and can be of type `None`, `rgb_array` or `human`.

The `rgb_array` mode returns a tuple of rgb images, while the `human` mode renders them in a matplotlib figure.