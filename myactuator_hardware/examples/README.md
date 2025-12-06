# Motor Control Examples

This folder contains example scripts for controlling the 4-motor system.

## Prerequisites

Make sure the simulation is running:
```bash
ros2 launch myactuator_hardware rviz_simulation.launch.py
```

## Examples

### 1. Sine Wave Trajectory (Action-based)

Uses the JointTrajectoryController action interface (same as MoveIt2):

```bash
# Default wave pattern (phase shifted)
ros2 run myactuator_hardware sine_wave_trajectory.py

# Synchronized (all motors same position)
ros2 run myactuator_hardware sine_wave_trajectory.py --mode sync

# Alternating (diagonal pairs opposite)
ros2 run myactuator_hardware sine_wave_trajectory.py --mode alternate

# Custom parameters
ros2 run myactuator_hardware sine_wave_trajectory.py -a 0.5 -f 0.5 -d 20.0
```

**Options:**
- `-a, --amplitude`: Amplitude in radians (default: 1.0)
- `-f, --frequency`: Frequency in Hz (default: 0.3)  
- `-d, --duration`: Duration in seconds (default: 10.0)
- `-m, --mode`: Trajectory mode - `wave`, `sync`, or `alternate`

### 2. Simple Position Publisher (Topic-based)

Continuous real-time position updates:

```bash
ros2 run myactuator_hardware simple_position_pub.py
```

This publishes directly to the trajectory topic at 20Hz for real-time control.

## Trajectory Modes

| Mode | Description | Motor Pattern |
|------|-------------|---------------|
| `wave` | Phase shifted sine waves | 1→2→3→4 with 90° phase difference |
| `sync` | Synchronized motion | All motors move together |
| `alternate` | Diagonal pairs | Motors 1,4 vs 2,3 opposite direction |

## Topics Used

- **Action**: `/joint_trajectory_controller/follow_joint_trajectory`
- **Topic**: `/joint_trajectory_controller/joint_trajectory`
- **State**: `/joint_states`
