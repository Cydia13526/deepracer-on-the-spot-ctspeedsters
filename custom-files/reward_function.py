import math

class Reward:
    def __init__(self, verbose=False):
        self.verbose = verbose
        self.racing_track = self._load_racing_track()
        self.first_racingpoint_index = 0
        self.straight_waypoints = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21,
                                   43, 44, 45, 46, 47, 48, 49, 50, 56, 57, 58, 59, 60, 61, 62, 63, 64,
                                   71, 72, 73, 74, 75, 76, 77, 78, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99,
                                   100, 101, 102, 103, 112, 113, 114, 115, 116, 117]

    def _load_racing_track(self):
        # Placeholder: Must use K1999 race-line calculator for re:Invent 2018
        # Format: [x, y, speed, timeFromPreviousPoint, curvature]
        # Curvature: Approximate radius of turn (high for straights, low for curves)
        return [
            [0.0, 0.0, 4.5, 0.05, 100.0],  # Straight, high speed, low curvature
            [0.5, 0.1, 4.5, 0.05, 100.0],
            # ... (Add ~118 waypoints with x, y, speed, time, curvature)
        ] * 118  # Replace with actual K1999 data

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
        distance_reward = max(1e-3, 1 - (dist / (track_width * 0.3))) ** 3  # Tighter, cubic reward
        reward += distance_reward * 2.5

        # Dynamic speed reward based on curvature
        curvature = optimals[4]  # Curvature from racing line data
        if curvature > 50:  # Straight or gentle curve
            target_speed = min(4.5, optimals[2])  # Push to 4.5 m/s
            speed_diff = abs(target_speed - speed)
            if speed_diff <= 0.3 and speed >= 4.0:
                speed_reward = (1 - (speed_diff / 0.3)) * (speed / 4.5)
                reward += speed_reward * 4.0  # Strong incentive
            else:
                speed_reward = 0.05
                reward += speed_reward * 2.0
        else:  # Tight curve
            target_speed = max(1.8, optimals[2])  # Controlled speed
            speed_diff = abs(target_speed - speed)
            if speed_diff <= 0.3 and speed <= 2.5:
                speed_reward = (1 - (speed_diff / 0.3)) * (2.5 / speed)
                reward += speed_reward * 3.0
            else:
                speed_reward = 0.05
                reward += speed_reward * 2.0

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
        BENCHMARK_TIME = 9.0  # Aggressive target (below Chen’s 9.5s)
        BENCHMARK_STEPS = int(BENCHMARK_TIME * 15)  # ~135 steps
        times_list = [row[3] for row in self.racing_track]
        projected_time_val = projected_time(self.first_racingpoint_index, closest_index, steps, times_list)
        if (steps % 25) == 0 and progress >= (steps / BENCHMARK_STEPS) * 100:
            reward += 15.0  # Frequent progress checks
        elif (steps % 25) == 0:
            reward *= 0.7

        # Finish reward
        if progress == 100:
            finish_reward = 2000 * (BENCHMARK_TIME / max(BENCHMARK_TIME, steps / 15)) ** 2  # Quadratic scaling
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