import math

class Reward:
    def __init__(self, verbose=False):
        self.first_racingpoint_index = None
        self.verbose = verbose

    def reward_function(self, params):
        ################## HELPER FUNCTIONS ###################

        def dist_2_points(x1, x2, y1, y2):
            if not all(isinstance(coord, (int, float)) for coord in [x1, x2, y1, y2]):
                return float('inf')
            return abs(abs(x1 - x2) ** 2 + abs(y1 - y2) ** 2) ** 0.5

        def closest_2_racing_points_index(racing_coords, car_coords):
            if not racing_coords or not car_coords or len(car_coords) < 2:
                return [0, 1]
            distances = []
            for i in range(len(racing_coords)):
                if len(racing_coords[i]) < 2:
                    distances.append(float('inf'))
                    continue
                distance = dist_2_points(x1=racing_coords[i][0], x2=car_coords[0],
                                         y1=racing_coords[i][1], y2=car_coords[1])
                distances.append(distance)
            if not distances:
                return [0, 1]
            closest_index = distances.index(min(distances))
            distances_no_closest = distances.copy()
            distances_no_closest[closest_index] = float('inf')
            second_closest_index = distances_no_closest.index(min(distances_no_closest))
            return [closest_index, second_closest_index]

        def dist_to_racing_line(closest_coords, second_closest_coords, car_coords):
            a = abs(dist_2_points(x1=closest_coords[0],
                                  x2=second_closest_coords[0],
                                  y1=closest_coords[1],
                                  y2=second_closest_coords[1]))
            b = abs(dist_2_points(x1=car_coords[0],
                                  x2=closest_coords[0],
                                  y1=car_coords[1],
                                  y2=closest_coords[1]))
            c = abs(dist_2_points(x1=car_coords[0],
                                  x2=second_closest_coords[0],
                                  y1=car_coords[1],
                                  y2=second_closest_coords[1]))
            try:
                distance = abs(-(a**4) + 2*(a**2)*(b**2) + 2*(a**2)*(c**2) -
                               (b**4) + 2*(b**2)*(c**2) - (c**4))**0.5 / (2*a)
            except:
                distance = b
            return distance

        def next_prev_racing_point(closest_coords, second_closest_coords, car_coords, heading):
            heading_vector = [math.cos(math.radians(heading)), math.sin(math.radians(heading))]
            new_car_coords = [car_coords[0] + heading_vector[0], car_coords[1] + heading_vector[1]]
            distance_closest_coords_new = dist_2_points(x1=new_car_coords[0],
                                                        x2=closest_coords[0],
                                                        y1=new_car_coords[1],
                                                        y2=closest_coords[1])
            distance_second_closest_coords_new = dist_2_points(x1=new_car_coords[0],
                                                               x2=second_closest_coords[0],
                                                               y1=new_car_coords[1],
                                                               y2=second_closest_coords[1])
            if distance_closest_coords_new <= distance_second_closest_coords_new:
                next_point_coords = closest_coords
                prev_point_coords = second_closest_coords
            else:
                next_point_coords = second_closest_coords
                prev_point_coords = closest_coords
            return [next_point_coords, prev_point_coords]

        def racing_direction_diff(closest_coords, second_closest_coords, car_coords, heading):
            next_point, prev_point = next_prev_racing_point(closest_coords,
                                                            second_closest_coords,
                                                            car_coords,
                                                            heading)
            track_direction = math.atan2(next_point[1] - prev_point[1], next_point[0] - prev_point[0])
            track_direction = math.degrees(track_direction)
            direction_diff = abs(track_direction - heading)
            if direction_diff > 180:
                direction_diff = 360 - direction_diff
            return direction_diff

        def indexes_cyclical(start, end, array_len):
            if end is None or start is None or array_len <= 0:
                return []
            if end < start:
                end += array_len
            return [index % array_len for index in range(start, end)]

        def projected_time(first_index, closest_index, step_count, times_list):
            if first_index is None or closest_index is None or not times_list:
                return 9999
            current_actual_time = (step_count - 1) / 15
            indexes_traveled = indexes_cyclical(first_index, closest_index, len(times_list))
            current_expected_time = sum([times_list[i] for i in indexes_traveled]) if indexes_traveled else 0
            total_expected_time = sum(times_list)
            try:
                projected_time = (current_actual_time / current_expected_time) * total_expected_time
            except ZeroDivisionError:
                projected_time = 9999
            return projected_time

        #################### RACING LINE ######################

        # Replace with actual waypoints for 2022_march_open_ccw if necessary
        racing_track = [[-2.29464, -5.88662, 4.0, 0.06628],
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
                        [0.63563, -6.21144, 3.65266, 0.07494],
                        [0.91003, -6.21403, 3.29749, 0.08322],
                        [1.18467, -6.2085, 2.95455, 0.09297],
                        [1.45868, -6.19302, 2.62481, 0.10456],
                        [1.73083, -6.16529, 2.32681, 0.11757],
                        [1.99945, -6.12237, 2.01057, 0.1353],
                        [2.26224, -6.06074, 1.75266, 0.15401],
                        [2.51549, -5.97515, 1.75266, 0.15253],
                        [2.75339, -5.86013, 1.96002, 0.13482],
                        [2.97699, -5.72341, 2.06958, 0.12664],
                        [3.18659, -5.56848, 2.19348, 0.11883],
                        [3.38292, -5.39827, 2.31098, 0.11244],
                        [3.5667, -5.21494, 2.39114, 0.10856],
                        [3.73825, -5.01988, 2.45439, 0.10584],
                        [3.89775, -4.81407, 2.50985, 0.10374],
                        [4.04527, -4.59832, 2.4164, 0.10816],
                        [4.18081, -4.37326, 2.24246, 0.11716],
                        [4.30237, -4.13812, 2.16971, 0.122],
                        [4.40682, -3.89181, 2.16971, 0.12331],
                        [4.4918, -3.63437, 2.82584, 0.09594],
                        [4.56471, -3.37077, 3.18601, 0.08584],
                        [4.6278, -3.10244, 3.62629, 0.07601],
                        [4.68312, -2.83055, 4.0, 0.06936],
                        [4.73224, -2.55594, 4.0, 0.06974],
                        [4.77662, -2.27971, 4.0, 0.06994],
                        [4.81686, -2.00759, 4.0, 0.06877],
                        [4.85961, -1.73686, 3.70888, 0.0739],
                        [4.90695, -1.46797, 3.06795, 0.08899],
                        [4.96161, -1.20204, 2.63469, 0.10304],
                        [5.02675, -0.94041, 2.29928, 0.11726],
                        [5.10579, -0.6846, 2.01234, 0.13305],
                        [5.20264, -0.43654, 2.01234, 0.13233],
                        [5.32233, -0.199, 2.30004, 0.11565],
                        [5.45937, 0.03006, 2.56599, 0.10402],
                        [5.61037, 0.25193, 2.77884, 0.09658],
                        [5.77332, 0.46733, 3.00922, 0.08976],
                        [5.94644, 0.6768, 3.22468, 0.08427],
                        [6.12822, 0.88062, 3.44571, 0.07926],
                        [6.31722, 1.0789, 3.661, 0.07482],
                        [6.51205, 1.27164, 3.36239, 0.08151],
                        [6.69662, 1.46477, 2.87693, 0.09286],
                        [6.87283, 1.6621, 2.48259, 0.10656],
                        [7.03791, 1.86487, 2.19354, 0.1192],
                        [7.18837, 2.07443, 1.95004, 0.13229],
                        [7.32053, 2.29185, 1.73025, 0.14705],
                        [7.42991, 2.51794, 1.5, 0.16744],
                        [7.5106, 2.75306, 1.5, 0.16572],
                        [7.55291, 2.9964, 1.50133, 0.16452],
                        [7.55651, 3.24318, 1.56644, 0.15756],
                        [7.52435, 3.48859, 1.73429, 0.14271],
                        [7.46312, 3.72954, 1.89422, 0.13125],
                        [7.3777, 3.96444, 1.99518, 0.12528],
                        [7.27157, 4.19234, 1.99518, 0.126],
                        [7.14432, 4.41105, 2.12113, 0.11929],
                        [6.99896, 4.61992, 2.26996, 0.1121],
                        [6.83829, 4.81894, 2.33372, 0.1096],
                        [6.66364, 5.0076, 2.39797, 0.10721],
                        [6.47625, 5.18554, 2.44672, 0.10562],
                        [6.27713, 5.35235, 2.44572, 0.10621],
                        [6.06685, 5.50722, 2.1094, 0.1238],
                        [5.84606, 5.64934, 2.08069, 0.1262],
                        [5.61253, 5.77295, 2.08069, 0.12699],
                        [5.36807, 5.87599, 2.26759, 0.11699],
                        [5.11616, 5.96078, 2.41932, 0.10987],
                        [4.8589, 6.02904, 2.52892, 0.10525],
                        [4.59771, 6.08185, 2.64991, 0.10056],
                        [4.33369, 6.12039, 2.75466, 0.09686],
                        [4.06762, 6.14557, 2.8302, 0.09443],
                        [3.80009, 6.15798, 2.87913, 0.09302],
                        [3.53159, 6.15794, 2.9149, 0.09211],
                        [3.26254, 6.14572, 2.85348, 0.09439],
                        [2.99334, 6.12115, 2.78545, 0.09704],
                        [2.72447, 6.08358, 2.50876, 0.10821],
                        [2.45647, 6.03224, 2.29644, 0.11883],
                        [2.19034, 5.96378, 2.29644, 0.11966],
                        [1.92776, 5.87471, 2.61716, 0.10594],
                        [1.66884, 5.76966, 2.98336, 0.09366],
                        [1.41315, 5.65217, 3.51848, 0.07998],
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
                        [-4.97169, 2.9941, 3.50513, 0.07983],
                        [-5.23291, 2.89915, 3.0097, 0.09235],
                        [-5.48901, 2.79655, 2.6183, 0.10537],
                        [-5.73834, 2.68383, 2.31014, 0.11844],
                        [-5.97875, 2.55813, 2.01318, 0.13476],
                        [-6.20761, 2.41639, 1.74142, 0.15458],
                        [-6.4207, 2.25449, 1.74142, 0.15368],
                        [-6.61098, 2.06759, 1.92097, 0.13884],
                        [-6.7808, 1.86203, 2.08471, 0.1279],
                        [-6.93208, 1.64195, 2.25243, 0.11856],
                        [-7.0666, 1.4105, 2.41882, 0.11068],
                        [-7.18591, 1.17005, 2.51522, 0.10672],
                        [-7.29061, 0.92209, 2.59357, 0.10378],
                        [-7.3811, 0.66787, 2.61694, 0.10312],
                        [-7.45721, 0.40833, 2.69655, 0.1003],
                        [-7.51945, 0.14463, 2.7211, 0.09957],
                        [-7.56779, -0.12234, 2.76525, 0.09811],
                        [-7.60256, -0.39164, 2.75179, 0.09868],
                        [-7.62356, -0.66248, 2.7363, 0.09928],
                        [-7.63057, -0.93401, 2.72963, 0.09951],
                        [-7.62344, -1.20537, 2.72963, 0.09945],
                        [-7.60213, -1.47571, 2.73813, 0.09904],
                        [-7.56687, -1.74424, 2.8156, 0.09619],
                        [-7.51853, -2.0104, 2.87717, 0.09402],
                        [-7.45782, -2.27373, 2.9274, 0.09231],
                        [-7.3857, -2.53394, 2.80387, 0.0963],
                        [-7.30195, -2.79044, 2.62125, 0.10294],
                        [-7.20571, -3.04236, 2.39504, 0.1126],
                        [-7.09548, -3.28832, 2.39504, 0.11254],
                        [-6.96899, -3.52613, 2.33504, 0.11535],
                        [-6.82834, -3.75573, 2.08485, 0.12914],
                        [-6.67173, -3.97483, 1.97282, 0.13652],
                        [-6.49599, -4.17939, 1.97282, 0.1367],
                        [-6.30032, -4.36587, 2.05208, 0.13172],
                        [-6.08776, -4.53391, 2.31243, 0.11717],
                        [-5.86301, -4.68636, 2.52934, 0.10737],
                        [-5.62886, -4.82512, 2.75193, 0.0989],
                        [-5.38743, -4.95181, 2.98383, 0.09137],
                        [-5.14041, -5.06787, 3.21725, 0.08483],
                        [-4.8891, -5.17452, 3.44145, 0.07933],
                        [-4.63451, -5.27273, 3.68538, 0.07404],
                        [-4.37752, -5.36345, 3.89299, 0.07001],
                        [-4.11878, -5.44734, 4.0, 0.068],
                        [-3.85885, -5.52499, 4.0, 0.06782],
                        [-3.59822, -5.59691, 4.0, 0.06759],
                        [-3.33727, -5.6636, 4.0, 0.06733],
                        [-3.07632, -5.72553, 4.0, 0.06705],
                        [-2.81556, -5.78311, 4.0, 0.06676],
                        [-2.55503, -5.83671, 4.0, 0.0665]]


        # Validate input parameters
        if not all(isinstance(p, (int, float)) for p in [params.get('x'), params.get('y'), params.get('heading'),
                                                         params.get('speed'), params.get('steering_angle'),
                                                         params.get('track_width')]):
            return 1e-3

        all_wheels_on_track = params['all_wheels_on_track']
        x = params['x']
        y = params['y']
        distance_from_center = params['distance_from_center']
        is_left_of_center = params['is_left_of_center']
        heading = params['heading']
        progress = params['progress']
        steps = params['steps']
        speed = params['speed']
        steering_angle = params['steering_angle']
        track_width = params['track_width']
        waypoints = params['waypoints']
        closest_waypoints = params['closest_waypoints']
        is_offtrack = params['is_offtrack']

        ############### OPTIMAL X,Y,SPEED,TIME ################

        closest_index, second_closest_index = closest_2_racing_points_index(racing_track, [x, y])
        optimals = racing_track[closest_index]
        optimals_second = racing_track[second_closest_index]

        if steps == 1:
            self.first_racingpoint_index = closest_index if closest_index is not None else 0
        if self.verbose:
            self.first_racingpoint_index = 0

        ################ REWARD AND PUNISHMENT ################

        reward = 1

        DISTANCE_MULTIPLE = 1
        dist = dist_to_racing_line(optimals[0:2], optimals_second[0:2], [x, y])
        distance_reward = max(1e-3, 1 - (dist / (track_width * 0.5)))
        reward += distance_reward * DISTANCE_MULTIPLE

        SPEED_DIFF_NO_REWARD = 1
        SPEED_MULTIPLE = 2
        speed_diff = abs(optimals[2] - speed)
        if speed_diff <= SPEED_DIFF_NO_REWARD:
            speed_reward = (1 - (speed_diff / SPEED_DIFF_NO_REWARD) ** 2) ** 2
        else:
            speed_reward = 0
        reward += speed_reward * SPEED_MULTIPLE

        REWARD_PER_STEP_FOR_FASTEST_TIME = 1
        STANDARD_TIME = 18
        FASTEST_TIME = 12
        times_list = [row[3] for row in racing_track]
        projected_time_val = projected_time(self.first_racingpoint_index, closest_index, steps, times_list)
        try:
            steps_prediction = projected_time_val * 15 + 1
            reward_prediction = max(1e-3, (-REWARD_PER_STEP_FOR_FASTEST_TIME * FASTEST_TIME /
                                           (STANDARD_TIME - FASTEST_TIME)) * (steps_prediction - (STANDARD_TIME * 15 + 1)))
            steps_reward = min(REWARD_PER_STEP_FOR_FASTEST_TIME, reward_prediction / steps_prediction)
        except:
            steps_reward = 0
        reward += steps_reward

        direction_diff = racing_direction_diff(optimals[0:2], optimals_second[0:2], [x, y], heading)
        if direction_diff > 30:
            reward = 1e-3

        speed_diff_zero = optimals[2] - speed
        if speed_diff_zero > 0.5:
            reward = 1e-3

        if reward > 1e-3:
            reward = max(reward * reward, 1e-3)

        REWARD_FOR_FASTEST_TIME = 1500
        if progress == 100:
            finish_reward = max(1e-3, (-REWARD_FOR_FASTEST_TIME /
                                       (15 * (STANDARD_TIME - FASTEST_TIME))) * (steps - STANDARD_TIME * 15))
        else:
            finish_reward = 0
        reward += finish_reward

        if not all_wheels_on_track:
            reward = 1e-3

        ####################### VERBOSE #######################

        if self.verbose:
            print("Closest index: %i" % closest_index)
            print("Distance to racing line: %f" % dist)
            print("=== Distance reward (w/out multiple): %f ===" % distance_reward)
            print("Optimal speed: %f" % optimals[2])
            print("Speed difference: %f" % speed_diff)
            print("=== Speed reward (w/out multiple): %f ===" % speed_reward)
            print("Direction difference: %f" % direction_diff)
            print("Predicted time: %f" % projected_time_val)
            print("=== Steps reward: %f ===" % steps_reward)
            print("=== Finish reward: %f ===" % finish_reward)

        #################### RETURN REWARD ####################

        return float(reward)

reward_object = Reward(verbose=True)  # Enable verbose for debugging

def reward_function(params):
    return reward_object.reward_function(params)