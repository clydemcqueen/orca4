# MADDPG+RBF Formation Control Algorithm - Pseudocode

This document provides detailed pseudocode for the Multi-Agent Deep Deterministic Policy Gradient with Radial Basis Function (MADDPG+RBF) formation control algorithm implemented in the multi-AUV system.

## Table of Contents
- [Main Algorithm Structure](#main-algorithm-structure)
- [MADDPG Components](#maddpg-components)
- [RBF Network Implementation](#rbf-network-implementation)
- [Formation Control Logic](#formation-control-logic)
- [Mission Coordination](#mission-coordination)
- [System Integration](#system-integration)

## Main Algorithm Structure

```pseudocode
ALGORITHM: MADDPG+RBF Formation Control
INPUT: 
    - N AUVs (N=3: 1 leader + 2 followers)
    - Formation configuration Δ = {Δ₁₂, Δ₁₃}
    - Control parameters K_p, max_speeds, update_rate
OUTPUT: 
    - Velocity commands for follower AUVs
    - Formation maintenance with error < threshold

BEGIN MainFormationControl
    // System Initialization
    INITIALIZE RBF_networks for each follower AUV
    INITIALIZE formation_parameters(Δ₁₂ = (-5, 2), Δ₁₃ = (-5, -2))
    INITIALIZE control_gains(K_p = 0.3, max_linear = 2.0, max_angular = 1.0)
    
    // Main Control Loop (20Hz)
    WHILE system_active DO
        // Step 1: State Observation
        leader_state ← GET_ODOMETRY("/model/auv1/odometry")
        auv2_state ← GET_ODOMETRY("/model/auv2/odometry")
        auv3_state ← GET_ODOMETRY("/model/auv3/odometry")
        
        // Step 2: Formation Control for Each Follower
        FOR each follower_auv IN {auv2, auv3} DO
            // Calculate desired formation position
            desired_position ← CALCULATE_FORMATION_POSITION(leader_state, formation_offset[follower_auv])
            
            // MADDPG+RBF Control Decision
            control_action ← MADDPG_RBF_CONTROL(
                current_state = follower_state,
                desired_state = desired_position,
                leader_velocity = leader_state.velocity,
                formation_error = desired_position - follower_state.position
            )
            
            // Apply safety constraints
            safe_action ← APPLY_SAFETY_LIMITS(control_action)
            
            // Send control command
            PUBLISH_VELOCITY_COMMAND(follower_auv, safe_action)
        END FOR
        
        // Step 3: Update RBF Networks (Learning)
        UPDATE_RBF_NETWORKS(experiences)
        
        SLEEP(1/20)  // 20Hz control rate
    END WHILE
END MainFormationControl
```

## MADDPG Components

### Actor Network (RBF-based)

```pseudocode
FUNCTION MADDPG_RBF_CONTROL(current_state, desired_state, leader_velocity, formation_error)
INPUT:
    - current_state: [x, y, vx, vy, yaw, vyaw]
    - desired_state: [x_des, y_des, yaw_des]
    - leader_velocity: [vx_leader, vy_leader, vyaw_leader]
    - formation_error: [ex, ey, eyaw]
OUTPUT:
    - control_action: [linear_velocity, angular_velocity]

BEGIN
    // Step 1: Construct state vector for RBF network
    state_vector ← [
        current_state.x,
        current_state.y,
        current_state.vx,
        current_state.vy,
        formation_error.x,     // relative position to desired
        formation_error.y,
        leader_velocity.vx,    // leader motion information
        leader_velocity.vy
    ]
    
    // Step 2: RBF Network Forward Pass
    rbf_output ← RBF_FORWARD_PASS(state_vector)
    
    // Step 3: Combine with proportional control (simplified MADDPG)
    proportional_control ← K_p * formation_error
    
    // Step 4: Action selection
    raw_action ← rbf_output + proportional_control + leader_velocity
    
    // Step 5: Action bounds and safety
    control_action ← CLIP_ACTIONS(raw_action, max_linear_speed, max_angular_speed)
    
    RETURN control_action
END FUNCTION
```

### Critic Network (Value Estimation)

```pseudocode
FUNCTION MADDPG_CRITIC_NETWORK(joint_state, joint_action)
INPUT:
    - joint_state: Combined state of all AUVs
    - joint_action: Combined actions of all AUVs
OUTPUT:
    - q_value: Estimated Q-value for state-action pair

BEGIN
    // Centralized training, decentralized execution
    global_state ← CONCATENATE([auv1_state, auv2_state, auv3_state])
    global_action ← CONCATENATE([auv1_action, auv2_action, auv3_action])
    
    // Multi-layer perceptron for Q-value estimation
    hidden1 ← RELU(W1 * global_state + b1)
    hidden2 ← RELU(W2 * CONCATENATE([hidden1, global_action]) + b2)
    q_value ← W3 * hidden2 + b3
    
    RETURN q_value
END FUNCTION
```

## RBF Network Implementation

### RBF Forward Pass

```pseudocode
FUNCTION RBF_FORWARD_PASS(input_state)
INPUT:
    - input_state: State vector [x, y, vx, vy, ex, ey, vx_leader, vy_leader]
OUTPUT:
    - action: [linear_velocity, angular_velocity]

BEGIN
    // RBF Network Parameters
    num_centers ← 5
    rbf_centers ← [[c1_x, c1_y, ...], [c2_x, c2_y, ...], ..., [c5_x, c5_y, ...]]
    rbf_weights ← [W1, W2, W3, W4, W5]  // learned weights
    rbf_gamma ← 1.0  // width parameter
    
    // Step 1: Calculate RBF activations
    FOR i = 1 TO num_centers DO
        distance_squared ← ||input_state - rbf_centers[i]||²
        rbf_activation[i] ← EXP(-rbf_gamma * distance_squared)
    END FOR
    
    // Step 2: Weighted sum
    linear_output ← SUM(rbf_weights[i] * rbf_activation[i]) FOR i = 1 to num_centers
    angular_output ← SUM(rbf_weights_angular[i] * rbf_activation[i]) FOR i = 1 to num_centers
    
    action ← [linear_output, angular_output]
    
    RETURN action
END FUNCTION
```

### RBF Learning Update

```pseudocode
FUNCTION UPDATE_RBF_NETWORKS(experience_buffer)
INPUT:
    - experience_buffer: Collection of (state, action, reward, next_state) tuples
OUTPUT:
    - Updated RBF network weights

BEGIN
    // Experience Replay
    batch ← SAMPLE_RANDOM_BATCH(experience_buffer, batch_size = 32)
    
    FOR each experience IN batch DO
        state, action, reward, next_state ← experience
        
        // Calculate TD target
        next_action ← RBF_FORWARD_PASS(next_state)
        target_q ← reward + gamma * CRITIC_NETWORK(next_state, next_action)
        
        // Current Q-value
        current_q ← CRITIC_NETWORK(state, action)
        
        // TD error
        td_error ← target_q - current_q
        
        // Update RBF weights using gradient descent
        rbf_activations ← CALCULATE_RBF_ACTIVATIONS(state)
        weight_gradient ← td_error * rbf_activations
        rbf_weights ← rbf_weights + learning_rate * weight_gradient
    END FOR
    
    // Update target networks (soft update)
    target_weights ← tau * rbf_weights + (1 - tau) * target_weights
END FUNCTION
```

## Formation Control Logic

### Formation Position Calculation

```pseudocode
FUNCTION CALCULATE_FORMATION_POSITION(leader_state, formation_offset)
INPUT:
    - leader_state: {position: [x, y, z], orientation: yaw}
    - formation_offset: [dx, dy] relative to leader
OUTPUT:
    - desired_position: [x_des, y_des, yaw_des]

BEGIN
    // Transform formation offset to global coordinates
    leader_yaw ← leader_state.orientation.yaw
    
    // Rotation matrix for formation offset
    cos_yaw ← COS(leader_yaw)
    sin_yaw ← SIN(leader_yaw)
    
    // Global formation position
    x_global ← leader_state.x + formation_offset.dx * cos_yaw - formation_offset.dy * sin_yaw
    y_global ← leader_state.y + formation_offset.dx * sin_yaw + formation_offset.dy * cos_yaw
    yaw_global ← leader_yaw  // Maintain same orientation as leader
    
    desired_position ← [x_global, y_global, yaw_global]
    
    RETURN desired_position
END FUNCTION
```

### Reward Function Calculation

```pseudocode
FUNCTION CALCULATE_REWARD(auv_state, desired_state, action, collision_detected)
INPUT:
    - auv_state: Current AUV state
    - desired_state: Target formation position
    - action: Control action taken
    - collision_detected: Boolean collision flag
OUTPUT:
    - reward: Scalar reward value

BEGIN
    // Formation error penalty
    position_error ← ||auv_state.position - desired_state.position||
    formation_penalty ← -alpha * position_error²
    
    // Control effort penalty
    control_penalty ← -beta * (action.linear² + action.angular²)
    
    // Collision penalty
    IF collision_detected THEN
        collision_penalty ← -1000.0  // Large negative reward
    ELSE
        collision_penalty ← 0.0
    END IF
    
    // Bonus for maintaining formation
    IF position_error < formation_tolerance THEN
        formation_bonus ← 10.0
    ELSE
        formation_bonus ← 0.0
    END IF
    
    reward ← formation_penalty + control_penalty + collision_penalty + formation_bonus
    
    RETURN reward
END FUNCTION
```

## Mission Coordination

### Mission Runner Algorithm

```pseudocode
ALGORITHM: Formation Mission Coordination
INPUT: Mission type, trajectory parameters, formation configuration
OUTPUT: Coordinated multi-AUV formation execution

BEGIN FormationMissionRunner
    // Step 1: System Initialization
    INITIALIZE_BASE_CONTROLLERS([auv1, auv2, auv3])
    WAIT_FOR_MAVROS_CONNECTION(timeout = 30_seconds)
    ENABLE_FORMATION_CONTROLLER()
    
    // Step 2: Mission Selection and Execution
    SWITCH mission_type DO
        CASE "basic_maneuvers":
            EXECUTE_BASIC_MANEUVERS()
        CASE "waypoint_trajectory":
            EXECUTE_WAYPOINT_MISSION()
        CASE "sine_wave":
            EXECUTE_SINE_WAVE_TRAJECTORY()
        DEFAULT:
            LOG_ERROR("Unknown mission type")
    END SWITCH
    
    // Step 3: Mission Monitoring
    WHILE mission_active DO
        // Monitor formation quality
        formation_error ← CHECK_FORMATION_ERROR()
        
        IF formation_error > error_threshold THEN
            TRIGGER_FORMATION_RECOVERY()
        END IF
        
        // Check mission completion
        IF MISSION_COMPLETE() THEN
            STOP_ALL_AUVS()
            BREAK
        END IF
        
        SLEEP(0.1)  // 10Hz monitoring rate
    END WHILE
END FormationMissionRunner
```

### Waypoint Mission Implementation

```pseudocode
FUNCTION EXECUTE_WAYPOINT_MISSION()
BEGIN
    // Waypoints from mophong.txt
    waypoints ← [
        [0, 0, 0],      // Start
        [20, -13, -2],  // Waypoint 1
        [10, -23, -5],  // Waypoint 2
        [-10, -8, -3],  // Waypoint 3
        [0, 0, 0]       // Return home
    ]
    
    current_waypoint ← 0
    waypoint_tolerance ← 1.0  // meters
    
    WHILE current_waypoint < LENGTH(waypoints) DO
        target ← waypoints[current_waypoint]
        
        // Move leader towards target
        MOVE_LEADER_TO_WAYPOINT(target)
        
        // Check if waypoint reached
        leader_position ← GET_LEADER_POSITION()
        distance_to_target ← ||leader_position - target||
        
        IF distance_to_target < waypoint_tolerance THEN
            current_waypoint ← current_waypoint + 1
            LOG_INFO("Waypoint " + current_waypoint + " reached")
        END IF
        
        SLEEP(0.1)
    END WHILE
    
    LOG_INFO("Waypoint mission completed")
END FUNCTION
```

### Sine Wave Trajectory

```pseudocode
FUNCTION EXECUTE_SINE_WAVE_TRAJECTORY()
INPUT: amplitude = 5.0, frequency = 0.1, speed = 0.5
BEGIN
    start_time ← GET_CURRENT_TIME()
    
    WHILE mission_active DO
        current_time ← GET_CURRENT_TIME()
        t ← current_time - start_time
        
        // Sine wave trajectory
        x_target ← speed * t
        y_target ← amplitude * SIN(2 * PI * frequency * t)
        
        // Calculate required velocity
        vx_target ← speed
        vy_target ← amplitude * 2 * PI * frequency * COS(2 * PI * frequency * t)
        
        // Send velocity command to leader
        leader_cmd ← CREATE_TWIST_MESSAGE(vx_target, vy_target, 0)
        PUBLISH_LEADER_COMMAND(leader_cmd)
        
        SLEEP(1/20)  // 20Hz update rate
    END WHILE
END FUNCTION
```

## System Integration

### Safety and Collision Avoidance

```pseudocode
FUNCTION APPLY_SAFETY_LIMITS(control_action, auv_states, safety_distance = 2.0)
INPUT:
    - control_action: Desired control action
    - auv_states: Positions of all AUVs
    - safety_distance: Minimum allowed distance between AUVs
OUTPUT:
    - safe_action: Modified action with safety constraints

BEGIN
    safe_action ← control_action
    
    // Check for potential collisions
    FOR each other_auv IN auv_states DO
        distance ← ||current_auv.position - other_auv.position||
        
        IF distance < safety_distance THEN
            // Calculate avoidance vector
            avoidance_direction ← NORMALIZE(current_auv.position - other_auv.position)
            avoidance_force ← avoidance_direction * collision_avoidance_gain
            
            // Modify action to avoid collision
            safe_action.linear ← safe_action.linear + avoidance_force
        END IF
    END FOR
    
    // Apply speed limits
    safe_action.linear ← CLIP(safe_action.linear, -max_linear_speed, max_linear_speed)
    safe_action.angular ← CLIP(safe_action.angular, -max_angular_speed, max_angular_speed)
    
    RETURN safe_action
END FUNCTION
```

### Error Recovery and Fault Tolerance

```pseudocode
FUNCTION TRIGGER_FORMATION_RECOVERY()
BEGIN
    LOG_WARNING("Formation error detected, initiating recovery")
    
    // Step 1: Stop all AUVs
    FOR each auv IN [auv1, auv2, auv3] DO
        SEND_STOP_COMMAND(auv)
    END FOR
    
    SLEEP(2.0)  // Allow AUVs to stop
    
    // Step 2: Reset formation controller
    RESTART_FORMATION_CONTROLLER()
    
    // Step 3: Gradual formation restoration
    FOR each follower IN [auv2, auv3] DO
        GRADUALLY_RESTORE_FORMATION(follower, restore_speed = 0.3)
    END FOR
    
    LOG_INFO("Formation recovery completed")
END FUNCTION
```

## Algorithm Parameters Summary

```pseudocode
CONSTANTS:
    // Formation Configuration
    FORMATION_DISTANCE ← 5.0        // meters behind leader
    FORMATION_OFFSET ← 2.0          // meters port/starboard
    
    // Control Parameters
    K_p ← 0.3                       // Proportional gain
    MAX_LINEAR_SPEED ← 2.0          // m/s
    MAX_ANGULAR_SPEED ← 1.0         // rad/s
    CONTROL_FREQUENCY ← 20          // Hz
    
    // RBF Network Parameters
    RBF_CENTERS ← 5                 // Number of RBF centers
    RBF_GAMMA ← 1.0                 // RBF width parameter
    LEARNING_RATE ← 0.001           // Weight update rate
    
    // Reward Function Weights
    ALPHA ← 1.0                     // Formation error weight
    BETA ← 0.1                      // Control effort weight
    GAMMA ← 0.99                    // Discount factor
    
    // Safety Parameters
    SAFETY_DISTANCE ← 2.0           // Minimum AUV separation
    FORMATION_TOLERANCE ← 1.0       // Acceptable formation error
    WAYPOINT_TOLERANCE ← 1.0        // Waypoint arrival tolerance
END CONSTANTS
```

This pseudocode provides a comprehensive description of the MADDPG+RBF formation control algorithm, covering all major components from low-level RBF network operations to high-level mission coordination. The algorithm combines the advantages of reinforcement learning (MADDPG) with the function approximation capabilities of RBF networks to achieve robust, adaptive formation control for multiple AUVs.
