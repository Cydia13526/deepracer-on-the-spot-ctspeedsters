import math

class Reward:
    def __init__(self, verbose=False):
        self.verbose = verbose
        self.racing_track = self._load_racing_track()
        self.first_racingpoint_index = 0
        self.straight_waypoints = [
            0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21,
            43, 44, 45, 46, 47, 48, 49, 50, 56, 57, 58, 59, 60, 61, 62, 63, 64,
            71, 72, 73, 74, 75, 76, 77, 78, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99,
            100, 101, 102, 103, 112, 113, 114, 115, 116, 117
        ]

    def _load_racing_track(self):
        # Hypothetical racing line for re:Invent 2018 track (118 waypoints, ~15.4m optimized length)
        # Format: [x, y, speed, timeFromPreviousPoint, curvature]
        # Curvature: High (>50) for straights, low (≤50) for curves
        # Note: Replace with actual K1999 data if available
        racing_track = [
            [-2.29464, -5.88662, 4.5, 0.06628, 100.0],  # Straight
            [-2.03416, -5.93312, 4.5, 0.06615, 100.0],
            [-1.77324, -5.97635, 4.5, 0.06612, 100.0],
            [-1.51150, -6.01641, 4.5, 0.06620, 100.0],
            [-1.24857, -6.05332, 4.5, 0.06638, 100.0],
            [-0.98419, -6.08703, 4.5, 0.06663, 100.0],
            [-0.71816, -6.11743, 4.5, 0.06694, 100.0],
            [-0.45051, -6.14452, 4.5, 0.06726, 100.0],
            [-0.18120, -6.16799, 4.5, 0.06758, 100.0],
            [0.08969, -6.18741, 4.5, 0.06790, 100.0],
            [0.36205, -6.20218, 4.5, 0.06819, 100.0],
            [0.63563, -6.21144, 4.5, 0.06843, 100.0],
            [0.91003, -6.21403, 4.5, 0.06860, 100.0],
            [1.18467, -6.20850, 4.0, 0.06973, 50.0],   # Transition to curve
            [1.45868, -6.19302, 3.5, 0.07842, 20.0],   # Curve
            [1.73083, -6.16529, 3.0, 0.08818, 15.0],
            [1.99945, -6.12237, 2.7, 0.10147, 10.0],
            [2.26224, -6.06074, 2.3, 0.11550, 10.0],
            [2.51549, -5.97515, 2.3, 0.11439, 10.0],
            [2.75339, -5.86013, 2.6, 0.10111, 15.0],
            [2.97699, -5.72341, 2.8, 0.09498, 20.0],
            [3.18659, -5.56848, 3.0, 0.08912, 30.0],
            [3.38292, -5.39827, 3.2, 0.08433, 50.0],
            [3.56670, -5.21494, 3.5, 0.08142, 100.0],  # Straight
            [3.73825, -5.01988, 4.0, 0.07938, 100.0],
            [3.89775, -4.81407, 4.5, 0.07781, 100.0],
            [4.04527, -4.59832, 4.5, 0.08112, 100.0],
            [4.18081, -4.37326, 4.0, 0.08787, 50.0],   # Transition
            [4.30237, -4.13812, 3.5, 0.09150, 20.0],   # Curve
            [4.40682, -3.89181, 3.0, 0.09248, 15.0],
            [4.49180, -3.63437, 3.8, 0.07195, 50.0],
            [4.56471, -3.37077, 4.5, 0.06838, 100.0],  # Straight
            [4.62780, -3.10244, 4.5, 0.06891, 100.0],
            [4.68312, -2.83055, 4.5, 0.06936, 100.0],
            [4.73224, -2.55594, 4.5, 0.06974, 100.0],
            [4.77662, -2.27971, 4.5, 0.06994, 100.0],
            [4.81686, -2.00759, 4.5, 0.06877, 100.0],
            [4.85961, -1.73686, 4.5, 0.06852, 100.0],
            [4.90695, -1.46797, 4.5, 0.06826, 100.0],
            [4.96161, -1.20204, 4.0, 0.07728, 50.0],   # Transition
            [5.02675, -0.94041, 3.5, 0.08795, 20.0],   # Curve
            [5.10579, -0.68460, 3.0, 0.09979, 15.0],
            [5.20264, -0.43654, 2.7, 0.09925, 10.0],
            [5.32233, -0.19900, 3.0, 0.08674, 20.0],
            [5.45937, 0.03006, 3.5, 0.07802, 50.0],
            [5.61037, 0.25193, 4.0, 0.07244, 100.0],  # Straight
            [5.77332, 0.46733, 4.5, 0.06752, 100.0],
            [5.94644, 0.67680, 4.5, 0.06794, 100.0],
            [6.12822, 0.88062, 4.5, 0.06828, 100.0],
            [6.31722, 1.07890, 4.5, 0.06848, 100.0],
            [6.51205, 1.27164, 4.5, 0.06851, 100.0],
            [6.69662, 1.46477, 4.0, 0.06964, 50.0],   # Transition
            [6.87283, 1.66210, 3.5, 0.07992, 20.0],   # Curve
            [7.03791, 1.86487, 3.0, 0.08940, 15.0],
            [7.18837, 2.07443, 2.7, 0.09922, 10.0],
            [7.32053, 2.29185, 2.3, 0.11029, 10.0],
            [7.42991, 2.51794, 2.0, 0.12558, 10.0],
            [7.51060, 2.75306, 2.0, 0.12429, 10.0],
            [7.55291, 2.99640, 2.0, 0.12339, 15.0],
            [7.55651, 3.24318, 2.3, 0.11817, 20.0],
            [7.52435, 3.48859, 2.5, 0.10703, 30.0],
            [7.46312, 3.72954, 2.7, 0.09844, 50.0],
            [7.37770, 3.96444, 3.0, 0.09396, 100.0],  # Straight
            [7.27157, 4.19234, 3.5, 0.09450, 100.0],
            [7.14432, 4.41105, 4.0, 0.08947, 100.0],
            [6.99896, 4.61992, 4.5, 0.08408, 100.0],
            [6.83829, 4.81894, 4.5, 0.08220, 100.0],
            [6.66364, 5.00760, 4.5, 0.08041, 100.0],
            [6.47625, 5.18554, 4.5, 0.07921, 100.0],
            [6.27713, 5.35235, 4.0, 0.07966, 50.0],   # Transition
            [6.06685, 5.50722, 3.5, 0.09285, 20.0],   # Curve
            [5.84606, 5.64934, 3.0, 0.09465, 15.0],
            [5.61253, 5.77295, 2.7, 0.09524, 10.0],
            [5.36807, 5.87599, 3.0, 0.08774, 20.0],
            [5.11616, 5.96078, 3.5, 0.08240, 50.0],
            [4.85890, 6.02904, 4.0, 0.07893, 100.0],  # Straight
            [4.59771, 6.08185, 4.5, 0.07542, 100.0],
            [4.33369, 6.12039, 4.5, 0.07265, 100.0],
            [4.06762, 6.14557, 4.5, 0.07082, 100.0],
            [3.80009, 6.15798, 4.5, 0.06976, 100.0],
            [3.53159, 6.15794, 4.5, 0.06909, 100.0],
            [3.26254, 6.14572, 4.5, 0.07079, 100.0],
            [2.99334, 6.12115, 4.5, 0.07278, 100.0],
            [2.72447, 6.08358, 4.0, 0.08116, 50.0],   # Transition
            [2.45647, 6.03224, 3.5, 0.08912, 20.0],   # Curve
            [2.19034, 5.96378, 3.0, 0.08975, 15.0],
            [1.92776, 5.87471, 3.5, 0.07946, 20.0],
            [1.66884, 5.76966, 4.0, 0.07024, 50.0],
            [1.41315, 5.65217, 4.5, 0.07035, 100.0],  # Straight
            [1.15993, 5.52562, 4.5, 0.07077, 100.0],
            [0.90855, 5.39249, 4.5, 0.07111, 100.0],
            [0.65838, 5.25506, 4.5, 0.07136, 100.0],
            [0.40881, 5.11551, 4.5, 0.07148, 100.0],
            [0.15308, 4.97464, 4.5, 0.07299, 100.0],
            [-0.10390, 4.83589, 4.5, 0.07301, 100.0],
            [-0.36233, 4.69975, 4.5, 0.07302, 100.0],
            [-0.62241, 4.56675, 4.5, 0.07303, 100.0],
            [-0.88441, 4.43759, 4.5, 0.07303, 100.0],
            [-1.14855, 4.31299, 4.5, 0.07302, 100.0],
            [-1.41498, 4.19346, 4.5, 0.07300, 100.0],
            [-1.68372, 4.07935, 4.5, 0.07299, 100.0],
            [-1.95470, 3.97068, 4.5, 0.07299, 100.0],
            [-2.22782, 3.86743, 4.5, 0.07300, 100.0],
            [-2.50290, 3.76934, 4.5, 0.07301, 100.0],
            [-2.77960, 3.67572, 4.5, 0.07303, 100.0],
            [-3.05760, 3.58583, 4.5, 0.07304, 100.0],
            [-3.33656, 3.49877, 4.5, 0.07306, 100.0],
            [-3.61617, 3.41372, 4.5, 0.07307, 100.0],
            [-3.89621, 3.32998, 4.5, 0.07307, 100.0]
        ]
        return racing_track

    def reward_function(self, params):
        # Helper functions
        def dist_2_points(x1, x2, y1, y2):
            return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

        def closest_2_racing_points_index(racing_coords, car_coords):
            distances = [dist_2_points(racing_coords[i][0], car_coords[0], racing_coords[i][1], car_coords[1])
                         for i in range(len(racing_coords))]
            closest_index = distances.index(min(distances))
            distances_no_closest = distances.copy()
            distances_no_closest[closest_index] = float('inf')
            second_closest_index = distances_no_closest.index(min(distances_no_closest))
            return [closest_index, second_closest_index]

        def dist_to_racing_line(closest_coords, second_closest_coords, car_coords):
            a = dist_2_points(closest_coords[0], second_closest_coords[0], closest_coords[1], second_closest_coords[1])
            b = dist_2_points(car_coords[0], closest_coords[0], car_coords[1], closest_coords[1])
            c = dist_2_points(car_coords[0], second_closest_coords[0], car_coords[1], second_closest_coords[1])
            if a < 1e-6:
                return min(b, c)
            try:
                vx, vy = second_closest_coords[0] - closest_coords[0], second_closest_coords[1] - closest_coords[1]
                wx, wy = car_coords[0] - closest_coords[0], car_coords[1] - closest_coords[1]
                distance = abs(vx * wy - vy * wx) / a
            except:
                distance = min(b, c)
            return distance

        def racing_direction_diff(closest_coords, second_closest_coords, car_coords, heading):
            next_point = second_closest_coords
            prev_point = closest_coords
            track_direction = math.degrees(math.atan2(next_point[1] - prev_point[1], next_point[0] - prev_point[0]))
            direction_diff = abs(track_direction - heading)
            if direction_diff > 180:
                direction_diff = 360 - direction_diff
            return direction_diff

        def projected_time(first_index, closest_index, step_count, times_list):
            if not times_list or step_count < 1:
                return 1.0
            current_actual_time = (step_count - 1) / 15
            indexes_traveled = [(first_index + i) % len(times_list) for i in range(closest_index - first_index + 1)]
            current_expected_time = sum(times_list[i] for i in indexes_traveled)
            total_expected_time = sum(times_list)
            if current_expected_time < 1e-6:
                return total_expected_time
            return max(1.0, (current_actual_time / current_expected_time) * total_expected_time)

        # Input validation
        required_params = ['all_wheels_on_track', 'x', 'y', 'distance_from_center', 'heading', 'progress', 'steps',
                           'speed', 'steering_angle', 'track_width', 'waypoints', 'closest_waypoints', 'is_offtrack']
        for param in required_params:
            if param not in params:
                return 1e-3

        try:
            all_wheels_on_track = bool(params['all_wheels_on_track'])
            x, y = float(params['x']), float(params['y'])
            distance_from_center = float(params['distance_from_center'])
            heading = float(params['heading'])
            progress = float(params['progress'])
            steps = int(params['steps'])
            speed = float(params['speed'])
            steering_angle = float(params['steering_angle'])
            track_width = float(params['track_width'])
            closest_waypoints = params['closest_waypoints']
            is_offtrack = bool(params['is_offtrack'])
        except:
            return 1e-3

        # Edge case: Validate racing track and indices
        if not self.racing_track or closest_waypoints[0] >= len(self.racing_track) or closest_waypoints[1] >= len(self.racing_track):
            return 1e-3

        # Initialize reward
        reward = 1.0

        # Get closest racing line points
        closest_index, second_closest_index = closest_2_racing_points_index(self.racing_track, [x, y])
        optimals = self.racing_track[closest_index]
        optimals_second = self.racing_track[second_closest_index]

        if steps == 1:
            self.first_racingpoint_index = closest_index

        # Reward for staying close to racing line
        dist = dist_to_racing_line(optimals[0:2], optimals_second[0:2], [x, y])
        distance_reward = max(1e-3, (1 - (dist / (track_width * 0.3))) ** 2)  # Quadratic for stability
        reward += distance_reward * 2.0

        # Dynamic speed reward based on curvature
        curvature = optimals[4]  # Curvature from racing line data
        if curvature > 50:  # Straight or gentle curve
            target_speed = min(4.5, optimals[2])  # Push to 4.5 m/s
            speed_diff = abs(target_speed - speed)
            if speed_diff <= 0.3 and speed >= 4.0:
                speed_reward = (1 - (speed_diff / 0.3)) * (speed / 4.5)
                reward += speed_reward * 3.0  # Strong incentive
            else:
                speed_reward = 0.05
                reward += speed_reward * 1.5
        else:  # Tight curve
            target_speed = max(1.8, optimals[2])  # Controlled speed
            speed_diff = abs(target_speed - speed)
            if speed_diff <= 0.3 and speed <= 2.5:
                speed_reward = (1 - (speed_diff / 0.3)) * (2.5 / speed)
                reward += speed_reward * 2.5
            else:
                speed_reward = 0.05
                reward += speed_reward * 1.5

        # Direction alignment (tighter)
        direction_diff = racing_direction_diff(optimals[0:2], optimals_second[0:2], [x, y], heading)
        if direction_diff > 5:
            reward *= max(0.1, 1 - (direction_diff - 5) / 20)  # Stricter penalty
        elif direction_diff < 2:
            reward += 1.0  # Strong bonus for precision

        # Steering penalty in curves
        if curvature < 20 and abs(steering_angle) > 15:
            reward *= 0.6  # Penalize aggressive steering in tight corners
        elif curvature > 50 and abs(steering_angle) > 5:
            reward *= 0.8  # Encourage straighter paths in high-speed sections

        # Progress and time-based reward
        BENCHMARK_TIME = 9.5  # Match Chen’s 9.5s target
        BENCHMARK_STEPS = int(BENCHMARK_TIME * 15)  # ~142 steps
        times_list = [row[3] for row in self.racing_track]
        if times_list:
            projected_time_val = projected_time(self.first_racingpoint_index, closest_index, steps, times_list)
            if (steps % 25) == 0 and progress >= (steps / BENCHMARK_STEPS) * 100:
                reward += 10.0  # Frequent progress checks
            elif (steps % 25) == 0:
                reward *= 0.7

        # Finish reward
        if progress == 100:
            finish_reward = 200 * (BENCHMARK_TIME / max(BENCHMARK_TIME, steps / 15))  # Linear scaling
            reward += finish_reward
        else:
            finish_reward = 0

        # Adaptive off-track penalty
        if is_offtrack:
            penalty = 0.1 if speed > 3.0 else 0.3  # Harsher at high speed
            reward *= penalty
        elif not all_wheels_on_track:
            reward *= 0.5

        # Verbose logging
        if self.verbose:
            print(f"Distance reward: {distance_reward}")
            print(f"Speed reward: {speed_reward}")
            print(f"Direction diff: {direction_diff}")
            print(f"Curvature: {curvature}")
            print(f"Finish reward: {finish_reward}")
            print(f"Total reward: {reward}")

        return float(max(1e-3, reward))

def reward_function(params):
    reward_object = Reward(verbose=False)
    return reward_object.reward_function(params)