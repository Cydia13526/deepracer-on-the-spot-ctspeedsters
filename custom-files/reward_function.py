# Fix the bug of version 3, adding more loggings
import math


class Reward:
    def __init__(self, verbose=False, debug=False):
        self.first_racingpoint_index = 0
        self.verbose = verbose
        self.debug = debug

    def reward_function(self, params):

        ################## HELPER FUNCTIONS ###################

        def dist_2_points(x1, x2, y1, y2):
            return abs(abs(x1-x2)**2 + abs(y1-y2)**2)**0.5

        def closest_2_racing_points_index(racing_coords, car_coords):

            # Calculate all distances to racing points
            distances = []
            for i in range(len(racing_coords)):
                distance = dist_2_points(x1=racing_coords[i][0], x2=car_coords[0],
                                         y1=racing_coords[i][1], y2=car_coords[1])
                distances.append(distance)

            # Get index of the closest racing point
            closest_index = distances.index(min(distances))

            # Get index of the second closest racing point
            distances_no_closest = distances.copy()
            distances_no_closest[closest_index] = 999
            second_closest_index = distances_no_closest.index(
                min(distances_no_closest))

            return [closest_index, second_closest_index]

        def dist_to_racing_line(closest_coords, second_closest_coords, car_coords):

            # Calculate the distances between 2 closest racing points
            a = abs(dist_2_points(x1=closest_coords[0],
                                  x2=second_closest_coords[0],
                                  y1=closest_coords[1],
                                  y2=second_closest_coords[1]))

            # Distances between car and closest and second closest racing point
            b = abs(dist_2_points(x1=car_coords[0],
                                  x2=closest_coords[0],
                                  y1=car_coords[1],
                                  y2=closest_coords[1]))
            c = abs(dist_2_points(x1=car_coords[0],
                                  x2=second_closest_coords[0],
                                  y1=car_coords[1],
                                  y2=second_closest_coords[1]))

            # Calculate distance between car and racing line (goes through 2 closest racing points)
            # try-except in case a=0 (rare bug in DeepRacer)
            try:
                distance = abs(-(a**4) + 2*(a**2)*(b**2) + 2*(a**2)*(c**2) -
                               (b**4) + 2*(b**2)*(c**2) - (c**4))**0.5 / (2*a)
            except:
                distance = b

            return distance

        # Calculate which one of the closest racing points is the next one and which one the previous one
        def next_prev_racing_point(closest_coords, second_closest_coords, car_coords, heading):

            # Virtually set the car more into the heading direction
            heading_vector = [math.cos(math.radians(
                heading)), math.sin(math.radians(heading))]
            new_car_coords = [car_coords[0]+heading_vector[0],
                              car_coords[1]+heading_vector[1]]

            # Calculate distance from new car coords to 2 closest racing points
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

            # Calculate the direction of the center line based on the closest waypoints
            next_point, prev_point = next_prev_racing_point(closest_coords,
                                                            second_closest_coords,
                                                            car_coords,
                                                            heading)

            # Calculate the direction in radius, arctan2(dy, dx), the result is (-pi, pi) in radians
            track_direction = math.atan2(
                next_point[1] - prev_point[1], next_point[0] - prev_point[0])

            # Convert to degree
            track_direction = math.degrees(track_direction)

            # Calculate the difference between the track direction and the heading direction of the car
            direction_diff = abs(track_direction - heading)
            if direction_diff > 180:
                direction_diff = 360 - direction_diff

            return direction_diff

        # Gives back indexes that lie between start and end index of a cyclical list
        # (start index is included, end index is not)
        def indexes_cyclical(start, end, array_len):

            if end < start:
                end += array_len

            return [index % array_len for index in range(start, end)]

        # Calculate how long car would take for entire lap, if it continued like it did until now
        def get_projected_time(first_index, closest_index, step_count):

            times_list = [row[3] for row in racing_track]

            # Calculate how much time has passed since start
            current_actual_time = (step_count-1) / 15

            if self.verbose:
                print("first_index: %i" % first_index)
            # Calculate which indexes were already passed
            indexes_traveled = indexes_cyclical(
                first_index, closest_index, len(times_list))

            # Calculate how much time should have passed if car would have followed optimals
            current_expected_time = sum(
                [times_list[i] for i in indexes_traveled])

            # Calculate how long one entire lap takes if car follows optimals
            total_expected_time = sum(times_list)

            # Calculate how long car would take for entire lap, if it continued like it did until now
            try:
                projected_time = (current_actual_time /
                                  current_expected_time) * total_expected_time
            except:
                projected_time = 9999

            return projected_time

        def get_distance_reward():
            ## Reward if car goes close to optimal racing line ##
            DISTANCE_MULTIPLE = 1
            dist = dist_to_racing_line(
                optimals[0:2], optimals_second[0:2], [x, y])
            distance_reward = max(1e-3, 1 - (dist/(track_width*0.5)))
            final_distance_reward = distance_reward * DISTANCE_MULTIPLE
            return dist, final_distance_reward

        def get_direction_reward():
            # Zero reward if obviously wrong direction (e.g. spin)
            direction_diff = racing_direction_diff(
                optimals[0:2], optimals_second[0:2], [x, y], heading)

            STEERING_ANGLE_THRESHOLD = 30
            if direction_diff > STEERING_ANGLE_THRESHOLD:
                reward = 1e-3
            else:
                reward = 1.0 - direction_diff / STEERING_ANGLE_THRESHOLD
            return direction_diff, reward

        def get_speed_reward():
            ## Reward if speed is close to optimal speed ##
            SPEED_MULTIPLE = 1  # change the multiple to emphasize on speed
            SPEED_DIFF_NO_REWARD = 1
            speed_diff = abs(optimals[2]-speed)
            if speed_diff <= SPEED_DIFF_NO_REWARD:
                # we use quadratic punishment (not linear) bc we're not as confident with the optimal speed
                # so, we do not punish small deviations from optimal speed
                speed_reward = (1 - (speed_diff/(SPEED_DIFF_NO_REWARD))**2)**2
            else:
                speed_reward = 1e-3
            final_speed_reward = speed_reward * SPEED_MULTIPLE
            return speed_diff, final_speed_reward

        def get_step_reward():
            # Reward if less steps
            REWARD_PER_STEP_FOR_FASTEST_TIME = 1
            FASTEST_TIME = 15  # seconds (best time of 1st place on the track)
            STANDARD_TIME = 17
            projected_time = get_projected_time(
                self.first_racingpoint_index, closest_index, steps)
            try:
                steps_prediction = projected_time * 15 + 1
                reward_prediction = max(1e-3, (-REWARD_PER_STEP_FOR_FASTEST_TIME*(FASTEST_TIME) /
                                               (STANDARD_TIME-FASTEST_TIME))*(steps_prediction-(STANDARD_TIME*15+1)))
                steps_reward = min(
                    REWARD_PER_STEP_FOR_FASTEST_TIME, reward_prediction / steps_prediction)
                if self.verbose == True:
                    print('projected_time {}, steps_prediction: {}, reward_prediction: {}, steps_reward: {}'.format(
                        projected_time, steps_prediction, reward_prediction, steps_reward))
            except:
                steps_reward = 0
            return projected_time, steps_reward

        def get_finish_reward():
            ## Incentive for finishing the lap in less steps ##
            STANDARD_TIME = 16  # seconds (time that is easily done by model)
            if progress == 100:
                finish_reward = max(1e-3, (STANDARD_TIME*15 - steps)**2)
                if self.verbose == True:
                    print('progress is 100, steps: {}, finish_reward: {}'.format(steps, finish_reward))
            else:
                finish_reward = 0
            return finish_reward

        #################### RACING LINE ######################

        # Optimal racing line
        # Each row: [x,y,speed,timeFromPreviousPoint]
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
                        [0.63563, -6.21144, 4.0, 0.06843],
                        [0.91003, -6.21403, 3.73716, 0.07343],
                        [1.18467, -6.2085, 3.34849, 0.08204],
                        [1.45868, -6.19302, 2.97479, 0.09226],
                        [1.73083, -6.16529, 2.63705, 0.10374],
                        [1.99945, -6.12237, 2.27864, 0.11938],
                        [2.26224, -6.06074, 1.98635, 0.13589],
                        [2.51549, -5.97515, 1.98635, 0.13458],
                        [2.75339, -5.86013, 2.22136, 0.11896],
                        [2.97699, -5.72341, 2.34552, 0.11174],
                        [3.18659, -5.56848, 2.48595, 0.10485],
                        [3.38292, -5.39827, 2.61911, 0.09921],
                        [3.5667, -5.21494, 2.70996, 0.09579],
                        [3.73825, -5.01988, 2.78164, 0.09339],
                        [3.89775, -4.81407, 2.8445, 0.09154],
                        [4.04527, -4.59832, 2.73859, 0.09544],
                        [4.18081, -4.37326, 2.54145, 0.10337],
                        [4.30237, -4.13812, 2.459, 0.10765],
                        [4.40682, -3.89181, 2.459, 0.1088],
                        [4.4918, -3.63437, 3.20262, 0.08465],
                        [4.56471, -3.37077, 3.61081, 0.07574],
                        [4.6278, -3.10244, 4.0, 0.06891],
                        [4.68312, -2.83055, 4.0, 0.06936],
                        [4.73224, -2.55594, 4.0, 0.06974],
                        [4.77662, -2.27971, 4.0, 0.06994],
                        [4.81686, -2.00759, 4.0, 0.06877],
                        [4.85961, -1.73686, 4.0, 0.06852],
                        [4.90695, -1.46797, 3.47701, 0.07852],
                        [4.96161, -1.20204, 2.98599, 0.09092],
                        [5.02675, -0.94041, 2.60585, 0.10347],
                        [5.10579, -0.6846, 2.28066, 0.11739],
                        [5.20264, -0.43654, 2.28066, 0.11676],
                        [5.32233, -0.199, 2.60671, 0.10204],
                        [5.45937, 0.03006, 2.90812, 0.09178],
                        [5.61037, 0.25193, 3.14936, 0.08522],
                        [5.77332, 0.46733, 3.41045, 0.0792],
                        [5.94644, 0.6768, 3.65464, 0.07436],
                        [6.12822, 0.88062, 3.90513, 0.06993],
                        [6.31722, 1.0789, 4.0, 0.06848],
                        [6.51205, 1.27164, 3.81071, 0.07192],
                        [6.69662, 1.46477, 3.26052, 0.08193],
                        [6.87283, 1.6621, 2.8136, 0.09403],
                        [7.03791, 1.86487, 2.48601, 0.10517],
                        [7.18837, 2.07443, 2.21005, 0.11673],
                        [7.32053, 2.29185, 1.96095, 0.12975],
                        [7.42991, 2.51794, 1.7, 0.14774],
                        [7.5106, 2.75306, 1.7, 0.14622],
                        [7.55291, 2.9964, 1.7015, 0.14517],
                        [7.55651, 3.24318, 1.77529, 0.13902],
                        [7.52435, 3.48859, 1.96553, 0.12592],
                        [7.46312, 3.72954, 2.14678, 0.11581],
                        [7.3777, 3.96444, 2.26121, 0.11054],
                        [7.27157, 4.19234, 2.26121, 0.11118],
                        [7.14432, 4.41105, 2.40394, 0.10526],
                        [6.99896, 4.61992, 2.57262, 0.09891],
                        [6.83829, 4.81894, 2.64489, 0.09671],
                        [6.66364, 5.0076, 2.7177, 0.0946],
                        [6.47625, 5.18554, 2.77295, 0.09319],
                        [6.27713, 5.35235, 2.77182, 0.09371],
                        [6.06685, 5.50722, 2.39065, 0.10924],
                        [5.84606, 5.64934, 2.35811, 0.11135],
                        [5.61253, 5.77295, 2.35811, 0.11205],
                        [5.36807, 5.87599, 2.56993, 0.10323],
                        [5.11616, 5.96078, 2.74189, 0.09694],
                        [4.8589, 6.02904, 2.86611, 0.09286],
                        [4.59771, 6.08185, 3.00323, 0.08873],
                        [4.33369, 6.12039, 3.12194, 0.08547],
                        [4.06762, 6.14557, 3.20756, 0.08332],
                        [3.80009, 6.15798, 3.26302, 0.08207],
                        [3.53159, 6.15794, 3.30355, 0.08128],
                        [3.26254, 6.14572, 3.23395, 0.08328],
                        [2.99334, 6.12115, 3.15684, 0.08563],
                        [2.72447, 6.08358, 2.84326, 0.09548],
                        [2.45647, 6.03224, 2.60263, 0.10485],
                        [2.19034, 5.96378, 2.60263, 0.10558],
                        [1.92776, 5.87471, 2.96611, 0.09348],
                        [1.66884, 5.76966, 3.38114, 0.08264],
                        [1.41315, 5.65217, 3.98761, 0.07057],
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
                        [-4.97169, 2.9941, 3.97248, 0.07044],
                        [-5.23291, 2.89915, 3.411, 0.08149],
                        [-5.48901, 2.79655, 2.96741, 0.09297],
                        [-5.73834, 2.68383, 2.61815, 0.10451],
                        [-5.97875, 2.55813, 2.2816, 0.11891],
                        [-6.20761, 2.41639, 1.97361, 0.1364],
                        [-6.4207, 2.25449, 1.97361, 0.1356],
                        [-6.61098, 2.06759, 2.1771, 0.12251],
                        [-6.7808, 1.86203, 2.36267, 0.11285],
                        [-6.93208, 1.64195, 2.55275, 0.10462],
                        [-7.0666, 1.4105, 2.74133, 0.09766],
                        [-7.18591, 1.17005, 2.85058, 0.09416],
                        [-7.29061, 0.92209, 2.93937, 0.09157],
                        [-7.3811, 0.66787, 2.96586, 0.09099],
                        [-7.45721, 0.40833, 3.05609, 0.0885],
                        [-7.51945, 0.14463, 3.08391, 0.08786],
                        [-7.56779, -0.12234, 3.13395, 0.08657],
                        [-7.60256, -0.39164, 3.11869, 0.08707],
                        [-7.62356, -0.66248, 3.10114, 0.0876],
                        [-7.63057, -0.93401, 3.09358, 0.0878],
                        [-7.62344, -1.20537, 3.09358, 0.08775],
                        [-7.60213, -1.47571, 3.10321, 0.08739],
                        [-7.56687, -1.74424, 3.19101, 0.08488],
                        [-7.51853, -2.0104, 3.26079, 0.08296],
                        [-7.45782, -2.27373, 3.31772, 0.08145],
                        [-7.3857, -2.53394, 3.17772, 0.08497],
                        [-7.30195, -2.79044, 2.97075, 0.09083],
                        [-7.20571, -3.04236, 2.71437, 0.09935],
                        [-7.09548, -3.28832, 2.71437, 0.0993],
                        [-6.96899, -3.52613, 2.64638, 0.10178],
                        [-6.82834, -3.75573, 2.36283, 0.11395],
                        [-6.67173, -3.97483, 2.23587, 0.12046],
                        [-6.49599, -4.17939, 2.23587, 0.12062],
                        [-6.30032, -4.36587, 2.32569, 0.11622],
                        [-6.08776, -4.53391, 2.62075, 0.10339],
                        [-5.86301, -4.68636, 2.86659, 0.09474],
                        [-5.62886, -4.82512, 3.11885, 0.08727],
                        [-5.38743, -4.95181, 3.38167, 0.08062],
                        [-5.14041, -5.06787, 3.64621, 0.07485],
                        [-4.8891, -5.17452, 3.90031, 0.07],
                        [-4.63451, -5.27273, 4.0, 0.06822],
                        [-4.37752, -5.36345, 4.0, 0.06813],
                        [-4.11878, -5.44734, 4.0, 0.068],
                        [-3.85885, -5.52499, 4.0, 0.06782],
                        [-3.59822, -5.59691, 4.0, 0.06759],
                        [-3.33727, -5.6636, 4.0, 0.06733],
                        [-3.07632, -5.72553, 4.0, 0.06705],
                        [-2.81556, -5.78311, 4.0, 0.06676],
                        [-2.55503, -5.83671, 4.0, 0.0665]]

        ################## INPUT PARAMETERS ###################

        # Read all input parameters
        if self.verbose:
            new_params = {}
            for key, value in params.items():
                if key != 'waypoints':
                    new_params[key] = [value]
            # print out params without 'waypoints' key since it is duplicate
            print("params: ", new_params)

        is_offtrack = params['is_offtrack']
        x = params['x']
        y = params['y']
        heading = params['heading']
        progress = params['progress']
        steps = params['steps']
        speed = params['speed']
        track_width = params['track_width']
        # steering_angle = params['steering_angle']
        # waypoints = params['waypoints']
        # closest_waypoints = params['closest_waypoints']
        # is_offtrack = params['is_offtrack']
        # distance_from_center = params['distance_from_center']
        # is_left_of_center = params['is_left_of_center']

        ############### OPTIMAL X,Y,SPEED,TIME ################

        # Get closest indexes for racing line (and distances to all points on racing line)
        closest_index, second_closest_index = closest_2_racing_points_index(
            racing_track, [x, y])

        # Get optimal [x, y, speed, time] for closest and second closest index
        optimals = racing_track[closest_index]
        optimals_second = racing_track[second_closest_index]

        # Save first racingpoint of episode for later
        if self.debug:
            self.first_racingpoint_index = 0  # this is just for testing purposes

        if steps == 2.0:
            # when steps == 1, status is prepare, thus reset the first_racingpoint_index when steps = 2
            print('set first_racingpoint_index to %i' % closest_index)
            self.first_racingpoint_index = closest_index
        else:
            # print('first_racingpoint_index: %i' % self.first_racingpoint_index)
            pass

        ################ REWARD AND PUNISHMENT ################

        ## Define the default reward ##
        STANDARD_REWARD = 1
        reward = STANDARD_REWARD

        dist, distance_reward = get_distance_reward()
        reward *= distance_reward

        speed_diff, speed_reward = get_speed_reward()
        reward *= speed_reward

        direction_diff, direction_reward = get_direction_reward()
        reward *= direction_reward

        # projected_time, steps_reward = get_step_reward()
        # reward += steps_reward

        finish_reward = get_finish_reward()
        reward += finish_reward

        ## Zero reward if off track ##
        if is_offtrack:
            reward = 1e-3

        # if not params['all_wheels_on_track']:
        #     reward = 1e-3
        ####################### VERBOSE #######################

        if self.verbose == True:
            printStr = ("REWARD: {:3.4f}, DIS_REW: {:3.4f}, SPD_REW: {:3.4f}, DIR_REW: {:3.4f}, FIN_REW: {:3.4f}, "
                        "ACT_SPD: {:3.4f}, EXP_SPD: {:3.4f}, SPD_DIFF: {:3.4f}, "
                        "CLOSET_INDEX: {}, DIST: {:3.4f}, DIR_DIFF: {:3.4f}, STEPS: {}, PROGRESS: {}").format(
                reward, distance_reward, speed_reward, direction_reward, finish_reward,
                speed, optimals[2], speed_diff,
                closest_index, dist, direction_diff, steps, progress
            )
            print(printStr)

        #################### RETURN REWARD ####################

        # Always return a float value
        return float(reward)


# add parameter verbose=True to get noisy output for testing
reward_object = Reward(verbose=True, debug=False)


def reward_function(params):
    return reward_object.reward_function(params)
