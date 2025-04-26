import math

def reward_function(params):
    """
    Optimized reward function for AWS DeepRacer on re:Invent 2018 track.
    Goals:
    - Follow optimal racing line (~15.4m length).
    - Maximize speed on straights (target: 4.0 m/s).
    - Control speed in curves (target: 2.5 m/s).
    - Penalize off-track and edge cases harshly.
    - Reward smooth steering, heading alignment, and lap efficiency.
    - Provide significant bonus for lap completion.

    Args:
        params (dict): AWS DeepRacer environment parameters.

    Returns:
        float: Reward value for the current state.
    """
    # Input parameters
    track_width = params['track_width']
    distance_from_center = params['distance_from_center']
    steering_angle = abs(params['steering_angle'])
    speed = params['speed']
    progress = params['progress']
    all_wheels_on_track = params['all_wheels_on_track']
    waypoints = params['waypoints']
    closest_waypoints = params['closest_waypoints']
    heading = params['heading']
    steps = params['steps']

    # Constants
    MAX_SPEED = 4.0  # Target speed for straights (m/s)
    CURVE_SPEED = 2.5  # Target speed for curves (m/s)
    STEERING_LIMIT = 30.0  # Maximum steering angle (degrees)
    MIN_REWARD = 1e-5  # Minimum reward for off-track or edge cases

    # Initialize reward
    reward = 1.0

    # Hard penalty for off-track or near edge
    if not all_wheels_on_track or distance_from_center >= track_width / 2:
        return float(MIN_REWARD)

    # Center-line reward: Encourage staying close to the optimal racing line
    marker_1 = 0.1 * track_width
    marker_2 = 0.25 * track_width
    marker_3 = 0.4 * track_width
    if distance_from_center <= marker_1:
        reward *= 2.5  # High reward for being very close to center
    elif distance_from_center <= marker_2:
        reward *= 1.8  # Moderate reward for staying near center
    elif distance_from_center <= marker_3:
        reward *= 1.2  # Small reward for being within bounds
    else:
        reward *= 0.5  # Penalty for being too far from center

    # Steering penalty: Discourage excessive steering for smoother driving
    steering_penalty = 1.0 - (steering_angle / STEERING_LIMIT)
    steering_penalty = max(0.6, steering_penalty)  # Ensure penalty doesn't go too low
    reward *= steering_penalty

    # Track curvature detection: Identify curves to adjust speed
    next_point = waypoints[closest_waypoints[1]]
    prev_point = waypoints[closest_waypoints[0]]
    track_direction = math.degrees(math.atan2(next_point[1] - prev_point[1], next_point[0] - prev_point[0]))
    direction_diff = abs(track_direction - heading)
    if direction_diff > 180:
        direction_diff = 360 - direction_diff

    # Calculate angle difference between consecutive track segments
    angle_diff = direction_diff
    if closest_waypoints[1] + 1 < len(waypoints):
        next_next_point = waypoints[closest_waypoints[1] + 1]
        dx1 = next_point[0] - prev_point[0]
        dy1 = next_point[1] - prev_point[1]
        dx2 = next_next_point[0] - next_point[0]
        dy2 = next_next_point[1] - next_point[1]
        angle_diff = abs(math.degrees(math.atan2(dy2, dx2) - math.atan2(dy1, dx1)))
        if angle_diff > 180:
            angle_diff = 360 - angle_diff

    # Speed optimization: Adjust speed based on track curvature
    is_curve = angle_diff > 12 or direction_diff > 8
    if is_curve:
        if CURVE_SPEED - 0.2 <= speed <= CURVE_SPEED + 0.2:
            reward *= 2.2  # Reward optimal curve speed
        elif speed > CURVE_SPEED + 0.2:
            reward *= 0.6  # Penalize overspeeding in curves
        else:
            reward *= 0.8  # Penalize underspeeding in curves
    else:
        if speed >= MAX_SPEED - 0.2:
            reward *= 3.0  # Reward high speed on straights
        elif speed >= MAX_SPEED - 0.5:
            reward *= 1.7  # Moderate reward for near-max speed
        else:
            reward *= 0.7  # Penalize low speed on straights

    # Heading alignment: Reward alignment with track direction
    if direction_diff < 3:
        reward *= 1.5  # Bonus for precise alignment
    elif direction_diff > 15:
        reward *= 0.6  # Penalty for poor alignment

    # Lap efficiency: Reward progress relative to steps taken
    if steps > 0:
        efficiency = progress / steps
        reward += efficiency * 20.0  # Scale reward by efficiency

    # Lap completion bonus: Significant reward for finishing the lap
    if progress >= 100:
        reward += 150.0

    # Ensure reward is positive and within bounds
    reward = max(MIN_REWARD, reward)
    return float(reward)