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
        return [[-2.29464, -5.88662, 4.0, 0.06628],
                 [-2.03416, -5.93312, 4.0, 0.06615],
                 [-1.77324, -5.97635, 4.0, 0.06612],
                 [-1.5115, -6.01641, 4.0, 0.0662],
                 [-1.24857, -6.05332, 4.0, 0.06638],
                 [-0.98419, -6.08703, 4.0, 0.06663],
                 [-0.71816, -6.11743, 4.0, 0.06694],
                 [-0.45051, -6.14452, 4.0, 0.06726],
                 [-0.1812, -6.16799, 4.0, 0.06758],
                 [0.08969, -6.18741, 4.0, 0.0679],
                 [0.36205, -6.20218, 4.0, 0.06819],
                 [0.63563, -6.21144, 4.0, 0.06843],
                 [0.91003, -6.21403, 4.0, 0.0686],
                 [1.18467, -6.2085, 3.9394, 0.06973],
                 [1.45868, -6.19302, 3.49975, 0.07842],
                 [1.73083, -6.16529, 3.10242, 0.08818],
                 [1.99945, -6.12237, 2.68076, 0.10147],
                 [2.26224, -6.06074, 2.33689, 0.1155],
                 [2.51549, -5.97515, 2.33689, 0.11439],
                 [2.75339, -5.86013, 2.61337, 0.10111],
                 [2.97699, -5.72341, 2.75943, 0.09498],
                 [3.18659, -5.56848, 2.92464, 0.08912],
                 [3.38292, -5.39827, 3.0813, 0.08433],
                 [3.5667, -5.21494, 3.18818, 0.08142],
                 [3.73825, -5.01988, 3.27252, 0.07938],
                 [3.89775, -4.81407, 3.34647, 0.07781],
                 [4.04527, -4.59832, 3.22187, 0.08112],
                 [4.18081, -4.37326, 2.98995, 0.08787],
                 [4.30237, -4.13812, 2.89294, 0.0915],
                 [4.40682, -3.89181, 2.89294, 0.09248],
                 [4.4918, -3.63437, 3.76779, 0.07195],
                 [4.56471, -3.37077, 4.0, 0.06838],
                 [4.6278, -3.10244, 4.0, 0.06891],
                 [4.68312, -2.83055, 4.0, 0.06936],
                 [4.73224, -2.55594, 4.0, 0.06974],
                 [4.77662, -2.27971, 4.0, 0.06994],
                 [4.81686, -2.00759, 4.0, 0.06877],
                 [4.85961, -1.73686, 4.0, 0.06852],
                 [4.90695, -1.46797, 4.0, 0.06826],
                 [4.96161, -1.20204, 3.51293, 0.07728],
                 [5.02675, -0.94041, 3.06571, 0.08795],
                 [5.10579, -0.6846, 2.68313, 0.09979],
                 [5.20264, -0.43654, 2.68313, 0.09925],
                 [5.32233, -0.199, 3.06672, 0.08674],
                 [5.45937, 0.03006, 3.42132, 0.07802],
                 [5.61037, 0.25193, 3.70512, 0.07244],
                 [5.77332, 0.46733, 4.0, 0.06752],
                 [5.94644, 0.6768, 4.0, 0.06794],
                 [6.12822, 0.88062, 4.0, 0.06828],
                 [6.31722, 1.0789, 4.0, 0.06848],
                 [6.51205, 1.27164, 4.0, 0.06851],
                 [6.69662, 1.46477, 3.8359, 0.06964],
                 [6.87283, 1.6621, 3.31012, 0.07992],
                 [7.03791, 1.86487, 2.92472, 0.0894],
                 [7.18837, 2.07443, 2.60005, 0.09922],
                 [7.32053, 2.29185, 2.307, 0.11029],
                 [7.42991, 2.51794, 2.0, 0.12558],
                 [7.5106, 2.75306, 2.0, 0.12429],
                 [7.55291, 2.9964, 2.00177, 0.12339],
                 [7.55651, 3.24318, 2.08858, 0.11817],
                 [7.52435, 3.48859, 2.31239, 0.10703],
                 [7.46312, 3.72954, 2.52562, 0.09844],
                 [7.3777, 3.96444, 2.66024, 0.09396],
                 [7.27157, 4.19234, 2.66024, 0.0945],
                 [7.14432, 4.41105, 2.82817, 0.08947],
                 [6.99896, 4.61992, 3.02662, 0.08408],
                 [6.83829, 4.81894, 3.11163, 0.0822],
                 [6.66364, 5.0076, 3.1973, 0.08041],
                 [6.47625, 5.18554, 3.26229, 0.07921],
                 [6.27713, 5.35235, 3.26097, 0.07966],
                 [6.06685, 5.50722, 2.81253, 0.09285],
                 [5.84606, 5.64934, 2.77425, 0.09465],
                 [5.61253, 5.77295, 2.77425, 0.09524],
                 [5.36807, 5.87599, 3.02345, 0.08774],
                 [5.11616, 5.96078, 3.22575, 0.0824],
                 [4.8589, 6.02904, 3.37189, 0.07893],
                 [4.59771, 6.08185, 3.53321, 0.07542],
                 [4.33369, 6.12039, 3.67288, 0.07265],
                 [4.06762, 6.14557, 3.7736, 0.07082],
                 [3.80009, 6.15798, 3.83884, 0.06976],
                 [3.53159, 6.15794, 3.88654, 0.06909],
                 [3.26254, 6.14572, 3.80465, 0.07079],
                 [2.99334, 6.12115, 3.71393, 0.07278],
                 [2.72447, 6.08358, 3.34501, 0.08116],
                 [2.45647, 6.03224, 3.06192, 0.08912],
                 [2.19034, 5.96378, 3.06192, 0.08975],
                 [1.92776, 5.87471, 3.48955, 0.07946],
                 [1.66884, 5.76966, 3.97781, 0.07024],
                 [1.41315, 5.65217, 4.0, 0.07035],
                 [1.15993, 5.52562, 4.0, 0.07077],
                 [0.90855, 5.39249, 4.0, 0.07111],
                 [0.65838, 5.25506, 4.0, 0.07136],
                 [0.40881, 5.11551, 4.0, 0.07148],
                 [0.15308, 4.97464, 4.0, 0.07299],
                 [-0.1039, 4.83589, 4.0, 0.07301],
                 [-0.36233, 4.69975, 4.0, 0.07302],
                 [-0.62241, 4.56675, 4.0, 0.07303],
                 [-0.88441, 4.43759, 4.0, 0.07303],
                 [-1.14855, 4.31299, 4.0, 0.07302],
                 [-1.41498, 4.19346, 4.0, 0.073],
                 [-1.68372, 4.07935, 4.0, 0.07299],
                 [-1.9547, 3.97068, 4.0, 0.07299],
                 [-2.22782, 3.86743, 4.0, 0.073],
                 [-2.5029, 3.76934, 4.0, 0.07301],
                 [-2.7796, 3.67572, 4.0, 0.07303],
                 [-3.0576, 3.58583, 4.0, 0.07304],
                 [-3.33656, 3.49877, 4.0, 0.07306],
                 [-3.61617, 3.41372, 4.0, 0.07307],
                 [-3.89621, 3.32998, 4.0, 0.07307],
                 [-4.16772, 3.24968, 4.0, 0.07078],
                 [-4.43809, 3.16784, 4.0, 0.07062],
                 [-4.70642, 3.08314, 4.0, 0.07034],
                 [-4.97169, 2.9941, 4.0, 0.06995],
                 [-5.23291, 2.89915, 4.0, 0.06949],
                 [-5.48901, 2.79655, 3.49107, 0.07903],
                 [-5.73834, 2.68383, 3.08018, 0.08883],
                 [-5.97875, 2.55813, 2.68424, 0.10107],
                 [-6.20761, 2.41639, 2.3219, 0.11594],
                 [-6.4207, 2.25449, 2.3219, 0.11526],
                 [-6.61098, 2.06759, 2.5613, 0.10413],
                 [-6.7808, 1.86203, 2.77961, 0.09593],
                 [-6.93208, 1.64195, 3.00324, 0.08892],
                 [-7.0666, 1.4105, 3.2251, 0.08301],
                 [-7.18591, 1.17005, 3.35362, 0.08004],
                 [-7.29061, 0.92209, 3.45809, 0.07783],
                 [-7.3811, 0.66787, 3.48925, 0.07734],
                 [-7.45721, 0.40833, 3.5954, 0.07523],
                 [-7.51945, 0.14463, 3.62813, 0.07468],
                 [-7.56779, -0.12234, 3.687, 0.07358],
                 [-7.60256, -0.39164, 3.66905, 0.07401],
                 [-7.62356, -0.66248, 3.6484, 0.07446],
                 [-7.63057, -0.93401, 3.6395, 0.07463],
                 [-7.62344, -1.20537, 3.6395, 0.07459],
                 [-7.60213, -1.47571, 3.65084, 0.07428],
                 [-7.56687, -1.74424, 3.75413, 0.07214],
                 [-7.51853, -2.0104, 3.83622, 0.07052],
                 [-7.45782, -2.27373, 3.9032, 0.06923],
                 [-7.3857, -2.53394, 3.73849, 0.07223],
                 [-7.30195, -2.79044, 3.495, 0.07721],
                 [-7.20571, -3.04236, 3.19338, 0.08445],
                 [-7.09548, -3.28832, 3.19338, 0.0844],
                 [-6.96899, -3.52613, 3.11339, 0.08651],
                 [-6.82834, -3.75573, 2.7798, 0.09686],
                 [-6.67173, -3.97483, 2.63043, 0.10239],
                 [-6.49599, -4.17939, 2.63043, 0.10252],
                 [-6.30032, -4.36587, 2.73611, 0.09879],
                 [-6.08776, -4.53391, 3.08324, 0.08788],
                 [-5.86301, -4.68636, 3.37246, 0.08053],
                 [-5.62886, -4.82512, 3.66924, 0.07418],
                 [-5.38743, -4.95181, 3.97843, 0.06853],
                 [-5.14041, -5.06787, 4.0, 0.06823],
                 [-4.8891, -5.17452, 4.0, 0.06825],
                 [-4.63451, -5.27273, 4.0, 0.06822],
                 [-4.37752, -5.36345, 4.0, 0.06813],
                 [-4.11878, -5.44734, 4.0, 0.068],
                 [-3.85885, -5.52499, 4.0, 0.06782],
                 [-3.59822, -5.59691, 4.0, 0.06759],
                 [-3.33727, -5.6636, 4.0, 0.06733],
                 [-3.07632, -5.72553, 4.0, 0.06705],
                 [-2.81556, -5.78311, 4.0, 0.06676],
                 [-2.55503, -5.83671, 4.0, 0.0665]] * 118  # Replace with actual K1999 data

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