# rmf_mock_adapter_python
Python bindings for https://github.com/osrf/rmf_mock_adapter



## Introduction

The `rmf_mock_adapter` package is simply that.

A mock adapter with interfaces to `rmf_core`, but with no effect on the ROS2 graph. That means, no ROS2 publishing or subscribing, allowing for quick and easy prototyping of Python implemented fleet adapters as they interact with `rmf_core`.

If you want to, you can skip using this package and go directly into `rmf_fleet_adapter_python` to play around with the ROS2 side as well in addition to `rmf_core`.



## Installation

- Place the `pybind_ament` and `rmf_mock_adapter_python` packages in your ROS2 workspace, and run `colcon build` as per normal.



## Using the Bindings

Ensure that you have built this package and sourced its environment. Then it is as simple as importing the module containing the Python bindings!

```python
import rmf_mock_adapter as adpt

# You may then check the available bindings
print(dir(adpt))
```



## Running the Integration Test

```shell
ros2 run rmf_mock_adapter_python test_adapter
```



## Creating your own RobotCommandHandle

### Introduction

A RobotCommandHandle allows you to define routines that should be executed when `rmf_core` issues a command to the robot it represents to:

- `dock()`: Dock
- `follow_new_path()`: Follow a new path
- `stop()` Stop

Python bindings have been written for the `rmf_mock_adapter::RobotCommandHandle` abstract class that allows you to implement it in Python and have it communicate with the C++ code, as well as other Python bound `rmf_mock_adapter` classes and methods.

That is, **you can write Python code as overrides to C++ RobotCommandHandle methods and have `rmf_core` execute it in the C++ side**! This allows you to effectively write robot routines in Python that are callable from `rmf_core`!

All the standard functionalities for Python classes are open to you. You **may define new attributes or members** as you wish.



### Caveats

- The class methods `follow_new_path`, `dock`, and `stop` methods must be implemented if you want to use them, since the underlying C++ definitions are pure `virtual` method which **requires** an override
  - These methods will be called in from the C++ side by `rmf_core`! `rmf_core` will call the methods with the relevant arguments, so it is **absolutely essential that you ensure that the same number of arguments are exposed in any of the core methods that you define**.
- You **must not** declare an `__init__` method, as that will override the binding
  - If you still need to, look at the **Using `__init__`** section

Also:

- You may pass instances of your implemented RobotCommandHandles into bound C++ methods that take in `std::shared_ptr<rmf_mock_adapter::RobotCommandHandle>` arguments. This is because the bound class inherits from that type! It's very convenient.



### Very Minimal Template

```python
import rmf_mock_adapter as adpt

class RobotCommandHandle(adpt.RobotCommandHandle):
    # If you want to declare class attributes you do it here or in an init
    new_member = "Rawr"
    
    # The argument names do not need to be the same
    # But they are declared here to match the C++ interface for consistency
    def follow_new_path(self,
                        waypoints: str,
                        path_finished_callback: Callable) -> None:
        # Your implementation goes here.
        # You may replace the following lines!
        print(self.new_member)  # We use the instance variable here!
        path_finished_callback()
    
    def dock(self,
             dock_name: str,
             docking_finished_callback: Callable) -> None:
        # Implementation here too!
        print(dock_name)
        docking_finished_callback()
        
# Then you may simply instantiate it like any other Python class!
command_handler = RobotCommandHandle()

## Testing Instance Attributes
print(command_handler.new_member)  # Directly
command_handler.follow_new_path("", lambda: None)  # Via class method

command_handler.newer_member = "Rer"  # A new one!
print(command_handler.newer_member)

## Testing Class Methods
# And the methods can get called from the Python side
command_handler.dock(
    "", lambda: print("Dock callback works!")
) 

# But also to a C++ method that takes in a std::shared_ptr argument!
adpt.test_shared_ptr(command_handler,
                     "wow",
                     lambda: print("wow"))
# With default args!
adpt.test_shared_ptr(command_handler,
                     docking_finish_callback=lambda: print("wow"))

# adpt.test_shared_ptr binds:
# [](std::shared_ptr<rmf_mock_adapter::RobotCommandHandle> handle,
#    std::string dock_name = "DUMMY_DOCK_NAME",
#    std::function<void()> docking_finished_callback = [&](){})
# {
#   handle->dock(dock_name, docking_finished_callback);
# }
```



## Using `__init__`

If, however, you still want to define an `__init__` magic method, ensure that you **explicitly** call the required bound C++ constructor.

```python
class RobotCommandHandleInit(adpt.RobotCommandHandle):
    def __init__(self, new_member="rawr"):
        adpt.RobotCommandHandle.__init__(self)
        self.new_member = new_member
    
    # The argument names do not need to be the same
    # But they are declared here to match the C++ interface for consistency
    def follow_new_path(self,
                        waypoints: str,
                        path_finished_callback: Callable) -> None:
        # Your implementation goes here.
        # You may replace the following lines!
        print(self.new_member)  # We use the instance variable here!
        path_finished_callback()
    
    def dock(self,
             dock_name: str,
             docking_finished_callback: Callable) -> None:
        # Implementation here too!
        print(dock_name)
        docking_finished_callback()
```



## Interacting with `rmf_core`

Check out the `test_adapter.py` script in `scripts` for an integration example with `rmf_core`! These steps are **much easier understood in context**.

However, if you have already done that, relevant points are to:

1. Define your own `ChildRobotCommandHandle` subclass of `adpt.RobotCommandHandle`
   - Making sure to define any of the core methods you intend for the `rmf_core`  to be able to call
   - **Ensure that the method argument signatures are the same as the template though**!
   - If you want the ability to update `rmf_core` on the robot's status, also be sure to expose a class member for a bound `RobotUpdateHandle` instance, that can be used to notify `rmf_core` of any **updates to robot position** (`update_position()`), **interruptions** (`interrupted()`), or additional **delays** (`add_delay()`)
2. Define your map's graph with `graph.Graph()`
3. Adding **map waypoints**: `add_waypoint()`
4. And **map lanes**: `add_lane()`, `add_bidir_lane()`
5. Define your robot
   - By defining its profile: `traits.Profile()`
   - And instantiating its traits: `traits.VehicleTraits()`
6. Then create a scenario: `adpt.TestScenario()`
7. Attach a new fleet: `add_fleet()`
8. And add your robots to it: `add_robot()`
9. Then simply run your tests!: `test()`

T​hen you're done! :tada:

![](https://media.giphy.com/media/woDjSSLgqys8yLiJaP/giphy.gif)



## Running tests

Unit tests have been written for the individual components.

You may invoke `pytest` directly in the appropriate directory.

```shell
# Ensure environment is sourced and you are in the right directory
$ source <workspace_dir>/install/setup.bash
$ cd <rmf_mock_adapter_python_dir>/tests

# Invoke the tests
$ pytest -v
```

Or use `colcon test` with console output redirection.

```shell
# Ensure environment is sourced and you are in the right directory
$ source <workspace_dir>/install/setup.bash
$ cd <workspace_dir>

# Invoke the tests
$ colcon test --packages-select rmf_mock_adapter_python --event-handlers console_direct+
```



## Gotchas

### Pointer Indirection Gotchas

- `clone_ptr` indirection does not seem to work correctly currently! Be **very careful**!
- The only way surefire way to do pointer indirection is to do it via the objects that manage them. Unfortunately there isn't much of a workaround given that most of the pointers point to abstract classes.
- For most of the other pointers, you must make them using the various factory functions. Do not instantiate them directly since you will not be able to configure their internal members, even if those members are public.
  - No explicit bound methods exist for them as they are meant to be pointers to implementations that might vary widely.

### Missing Implementations

- Unfortunately, since there is no way to instantiate `VelocityConstraint` pointers yet, you cannot pass them into Node` objects yet.
  - `VelocityConstraint::clone` is not bound. And overriding it in Python doesn't expose the underlying pointer.

### Update Handles Should NOT be Directly Instantiated (Segfault risk!)

- The `RobotUpdateHandle` and `FleetUpdateHandle` classes must **only be instantiated via their factory methods**! (Their init members have been disabled as a result.)
  - `FleetUpdateHandle` should be instantiated via `TestScenario` (via `add_fleet`)
  - `RobotUpdateHandle` should be instantiated via `FleetUpdateHandle` (via `add_robot`)
  - Doing otherwise will cause their members to contain null values which will **lead to segmentation faults**!!!

### Different Kinds of Waypoints

- The `graph` and `plan` submodules have their own internal `Waypoint` classes with different, but related interfaces!
