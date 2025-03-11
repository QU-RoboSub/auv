# AUV Navigation System

This document provides an overview of the AUV navigation system components, core functions, and how to add new tasks.

## System Overview

The navigation system consists of two main components:

1. **Mission Control Node** (`missionControl.py`): Makes high-level decisions about tasks based on sensor inputs
2. **Navigation Node** (`navigation.py`): Executes navigation tasks and sends control signals to thrusters

## Core Functions in Mission Control

### 1. Detection Processing

| Function | Purpose |
|----------|---------|
| `listener_callback_front_yolo` | Processes front camera detections, filters target class, switches to tracking task |
| `listener_callback_bottom_yolo` | Processes bottom camera detections, triggers hold position when target detected |
| `listener_callback_sensors` | Processes orientation and depth data |

### 2. Task Management

| Function | Purpose |
|----------|---------|
| `send_task_command` | Sends task commands to the navigation node |
| `task_evaluation_callback` | Regularly evaluates if tasks need to change based on timeouts |

### 3. Payload Control

| Function | Purpose |
|----------|---------|
| `timer_callback_payload_signal` | Sends periodic signals to payload systems |

## Core Functions in Navigation

### 1. Input Processing

| Function | Purpose |
|----------|---------|
| `listener_callback_target` | Processes target information messages from Mission Control |
| `listener_callback_yolo` | Processes YOLO detections and updates tracking parameters |
| `listener_callback_sensors` | Processes sensor data for orientation and depth |
| `listener_callback_task_command` | Processes task commands from Mission Control |

### 2. Tracking and Control

| Function | Purpose |
|----------|---------|
| `update_tracking_parameters` | Updates angle and distance tracking based on detections |
| `calculate_surge` | Converts distance to a surge value (-2 to 2) |
| `send_orientation_deviations` | Updates and sends orientation deviations to thrusters |
| `execute_current_task` | Calls the appropriate task handler based on current task |

### 3. Task Implementations

| Function | Purpose |
|----------|---------|
| `task_track_target` | Tracks detected targets using vision-based control |
| `task_hold_position` | Maintains current position and orientation |
| `task_depth_control` | Maintains specific depth while allowing other movements |
| `task_search_pattern` | Performs search pattern to find targets |
| `task_return_home` | Returns to a predefined home position |

### 4. Output Generation

| Function | Purpose |
|----------|---------|
| `timer_callback_orientation` | Publishes orientation deviations to thrusters at regular intervals |

## Adding a New Task

### Step 1: Add Task to Navigation Node

1. Create a new task method in the Navigation class within the TASK IMPLEMENTATIONS section:

```python
def task_my_new_task(self):
    """Description of what this task does"""
    self.get_logger().info('TASK MY_NEW_TASK: Description of execution')
    
    # Implement task logic here
    
    # Send appropriate deviations to thrusters
    self.send_orientation_deviations(
        roll_dev=0.0,
        pitch_dev=0.0,
        yaw_dev=0.0,
        surge=0.0,
        sway=0.0
    )
```

2. Register the task in the `task_handlers` dictionary in the `__init__` method:

```python
self.task_handlers = {
    # ... existing tasks ...
    "my_new_task": self.task_my_new_task,
}
```

### Step 2: Add Task to Mission Control

1. Add the task to the `available_tasks` dictionary in the `__init__` method:

```python
self.available_tasks = {
    # ... existing tasks ...
    "my_new_task": "Description of my new task",
}
```

2. Add logic to trigger the new task in Mission Control, for example:

```python
# In a relevant callback or timer
if some_condition:
    self.current_task = "my_new_task"
    self.send_task_command("my_new_task")
```

### Step 3: Test Your New Task

1. Make sure both nodes are running
2. Trigger your task via mission control logic or manual command
3. Check the logs to ensure the task is executing as expected

## Best Practices

1. Keep task implementations clean and focused on a single purpose
2. Use appropriate logging to track task execution
3. Make sure task transitions are handled properly
4. Always call `send_orientation_deviations` with appropriate values at the end of each task
5. Validate all inputs and handle edge cases (missing detections, sensor failures) 