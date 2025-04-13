import math


class Reward:
    def __init__(self, verbose=False):
        self.first_racingpoint_index = None
        self.verbose = verbose

    def reward_function(self, params):
        # Import package (needed for heading)
        import math

        ################## HELPER FUNCTIONS ###################
        def dist_2_points(x1, x2, y1, y2):
            return abs(abs(x1-x2)**2 + abs(y1-y2)**2)**0.5

        def closest_2_racing_points_index(racing_coords, car_coords):
            distances = []
            for i in range(len(racing_coords)):
                distance = dist_2_points(x1=racing_coords[i][0], x2=car_coords[0],
                                         y1=racing_coords[i][1], y2=car_coords[1])
                distances.append(distance)

            closest_index = distances.index(min(distances))
            distances_no_closest = distances.copy()
            distances_no_closest[closest_index] = 999
            second_closest_index = distances_no_closest.index(min(distances_no_closest))

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
            heading_vector = [math.cos(math.radians(heading)), math.sin(math.radians(heading))]
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
            next_point, prev_point = next_prev_racing_point(closest_coords,
                                                            second_closest_coords,
                                                            car_coords,
                                                            heading)

            # Calculate the direction in radius, arctan2(dy, dx), the result is (-pi, pi) in radians
            track_direction = math.atan2(
                next_point[1] - prev_point[1], next_point[0] - prev_point[0])

            # Convert to degree
            track_direction = math.atan2(next_point[1] - prev_point[1], next_point[0] - prev_point[0])
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
        def projected_time(first_index, closest_index, step_count, times_list):

            # Calculate how much time has passed since start
            current_actual_time = (step_count-1) / 15

            # Calculate which indexes were already passed
            indexes_traveled = indexes_cyclical(first_index, closest_index, len(times_list))

            # Calculate how much time should have passed if car would have followed optimals
            current_expected_time = sum([times_list[i] for i in indexes_traveled])

            # Calculate how long one entire lap takes if car follows optimals
            total_expected_time = sum(times_list)

            # Calculate how long car would take for entire lap, if it continued like it did until now
            try:
                projected_time = (current_actual_time/current_expected_time) * total_expected_time
            except:
                projected_time = 9999

            return projected_time

        #################### RACING LINE ######################

        # Optimal racing line for the Spain track
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
                        [0.91003, -6.21403, 3.9394, 0.06966],
                        [1.18467, -6.2085, 3.49975, 0.07849],
                        [1.45868, -6.19302, 3.10242, 0.08846],
                        [1.73083, -6.16529, 2.68076, 0.10205],
                        [1.99945, -6.12237, 2.33689, 0.11641],
                        [2.26224, -6.06074, 2.33689, 0.1155],
                        [2.51549, -5.97515, 2.33689, 0.11439],
                        [2.75339, -5.86013, 2.61337, 0.10111],
                        [2.97699, -5.72341, 2.75943, 0.09498],
                        [3.18659, -5.56848, 2.92464, 0.08912],
                        [3.38292, -5.39827, 3.0813, 0.08433],
                        [3.5667, -5.21494, 3.18818, 0.08142],
                        [3.73825, -5.01988, 3.27252, 0.07938],
                        [3.89775, -4.81407, 3.22187, 0.08082],
                        [4.04527, -4.59832, 2.98995, 0.08741],
                        [4.18081, -4.37326, 2.89294, 0.09081],
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
                        [4.90695, -1.46797, 3.51293, 0.07772],
                        [4.96161, -1.20204, 3.06571, 0.08856],
                        [5.02675, -0.94041, 2.68313, 0.10049],
                        [5.10579, -0.6846, 2.68313, 0.09979],
                        [5.20264, -0.43654, 2.68313, 0.09925],
                        [5.32233, -0.199, 3.06672, 0.08674],
                        [5.45937, 0.03006, 3.42132, 0.07802],
                        [5.61037, 0.25193, 3.70512, 0.07244],
                        [5.77332, 0.46733, 4.0, 0.06752],
                        [5.94644, 0.6768, 4.0, 0.06794],
                        [6.12822, 0.88062, 4.0, 0.06828],
                        [6.31722, 1.0789, 4.0, 0.06848],
                        [6.51205, 1.27164, 3.8359, 0.07145],
                        [6.69662, 1.46477, 3.31012, 0.0807],
                        [6.87283, 1.6621, 2.92472, 0.09046],
                        [7.03791, 1.86487, 2.60005, 0.10056],
                        [7.18837, 2.07443, 2.307, 0.11182],
                        [7.32053, 2.29185, 2.0, 0.12722],
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
                        [6.47625, 5.18554, 3.26097, 0.07925],
                        [6.27713, 5.35235, 2.81253, 0.09236],
                        [6.06685, 5.50722, 2.77425, 0.09413],
                        [5.84606, 5.64934, 2.77425, 0.09465],
                        [5.61253, 5.77295, 2.77425, 0.09524],
                        [5.36807, 5.87599, 3.02345, 0.08774],
                        [5.11616, 5.96078, 3.22575, 0.0824],
                        [4.8589, 6.02904, 3.37189, 0.07893],
                        [4.59771, 6.08185, 3.53321, 0.07542],
                        [4.33369, 6.12039, 3.67288, 0.07265],
                        [4.06762, 6.14557, 3.7736, 0.07082],
                        [3.80009, 6.15798, 3.83884, 0.06976],
                        [3.53159, 6.15794, 3.80465, 0.07057],
                        [3.26254, 6.14572, 3.71393, 0.07252],
                        [2.99334, 6.12115, 3.34501, 0.08081],
                        [2.72447, 6.08358, 3.06192, 0.08866],
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
                        [-5.23291, 2.89915, 3.49107, 0.07962],
                        [-5.48901, 2.79655, 3.08018, 0.08957],
                        [-5.73834, 2.68383, 2.68424, 0.10194],
                        [-5.97875, 2.55813, 2.3219, 0.11684],
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
                        [-7.56779, -0.12234, 3.66905, 0.07394],
                        [-7.60256, -0.39164, 3.6484, 0.07443],
                        [-7.62356, -0.66248, 3.6395, 0.07464],
                        [-7.63057, -0.93401, 3.6395, 0.07463],
                        [-7.62344, -1.20537, 3.6395, 0.07459],
                        [-7.60213, -1.47571, 3.65084, 0.07428],
                        [-7.56687, -1.74424, 3.75413, 0.07214],
                        [-7.51853, -2.0104, 3.83622, 0.07052],
                        [-7.45782, -2.27373, 3.73849, 0.07229],
                        [-7.3857, -2.53394, 3.495, 0.07726],
                        [-7.30195, -2.79044, 3.19338, 0.0845],
                        [-7.20571, -3.04236, 3.19338, 0.08445],
                        [-7.09548, -3.28832, 3.11339, 0.08657],
                        [-6.96899, -3.52613, 2.7798, 0.0969],
                        [-6.82834, -3.75573, 2.63043, 0.10236],
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
                        [-2.55503, -5.83671, 4.0, 0.0665],
                        [1.99945, -6.12237, 1.75266, 0.15521],
                        [2.26224, -6.06074, 1.75266, 0.15401],
                        [2.51549, -5.97515, 1.75266, 0.15253],
                        [2.75339, -5.86013, 1.96002, 0.13482],
                        [7.03791, 1.86487, 1.95004, 0.13408],
                        [7.18837, 2.07443, 1.73025, 0.1491],
                        [7.32053, 2.29185, 1.5, 0.16962],
                        [7.42991, 2.51794, 1.5, 0.16744],
                        [7.5106, 2.75306, 1.5, 0.16572],
                        [7.55291, 2.9964, 1.50133, 0.16452],
                        [7.55651, 3.24318, 1.56644, 0.15756],
                        [7.52435, 3.48859, 1.73429, 0.14271],
                        [7.46312, 3.72954, 1.89422, 0.13125],
                        [7.3777, 3.96444, 1.99518, 0.12528],
                        [7.27157, 4.19234, 1.99518, 0.126],
                        [-5.97875, 2.55813, 1.74142, 0.15579],
                        [-6.20761, 2.41639, 1.74142, 0.15458],
                        [-6.4207, 2.25449, 1.74142, 0.15368],
                        [-6.61098, 2.06759, 1.92097, 0.13884],
                        [-6.82834, -3.75573, 1.97282, 0.13648],
                        [-6.67173, -3.97483, 1.97282, 0.13652],
                        [-6.49599, -4.17939, 1.97282, 0.1367]]

        ################## INPUT PARAMETERS ###################

        # Read all input parameters
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

        # Get closest indexes for racing line (and distances to all points on racing line)
        closest_index, second_closest_index = closest_2_racing_points_index(
            racing_track, [x, y])

        # Get optimal [x, y, speed, time] for closest and second closest index
        optimals = racing_track[closest_index]
        optimals_second = racing_track[second_closest_index]

        # Save first racingpoint of episode for later
        if self.verbose == True:
            self.first_racingpoint_index = 0 # this is just for testing purposes
        if steps == 1:
            self.first_racingpoint_index = closest_index

        ################ REWARD AND PUNISHMENT ################

        ## Define the default reward ##
        reward = 1

        ## Reward if car goes close to optimal racing line ##
        DISTANCE_MULTIPLE = 1
        dist = dist_to_racing_line(optimals[0:2], optimals_second[0:2], [x, y])
        distance_reward = max(1e-3, 1 - (dist/(track_width*0.5)))
        reward += distance_reward * DISTANCE_MULTIPLE

        ## Reward if speed is close to optimal speed ##
        SPEED_DIFF_NO_REWARD = 1
        SPEED_MULTIPLE = 2
        speed_diff = abs(optimals[2]-speed)
        if speed_diff <= SPEED_DIFF_NO_REWARD:
            # we use quadratic punishment (not linear) bc we're not as confident with the optimal speed
            # so, we do not punish small deviations from optimal speed
            speed_reward = (1 - (speed_diff/(SPEED_DIFF_NO_REWARD))**2)**2
        else:
            speed_reward = 0
        reward += speed_reward * SPEED_MULTIPLE

        # Reward if less steps
        REWARD_PER_STEP_FOR_FASTEST_TIME = 1
        STANDARD_TIME = 37
        FASTEST_TIME = 27
        times_list = [row[3] for row in racing_track]
        projected_time = projected_time(self.first_racingpoint_index, closest_index, steps, times_list)
        try:
            steps_prediction = projected_time * 15 + 1
            reward_prediction = max(1e-3, (-REWARD_PER_STEP_FOR_FASTEST_TIME*(FASTEST_TIME) /
                                           (STANDARD_TIME-FASTEST_TIME))*(steps_prediction-(STANDARD_TIME*15+1)))
            steps_reward = min(REWARD_PER_STEP_FOR_FASTEST_TIME, reward_prediction / steps_prediction)
        except:
            steps_reward = 0
        reward += steps_reward

        # Zero reward if obviously wrong direction (e.g. spin)
        direction_diff = racing_direction_diff(
            optimals[0:2], optimals_second[0:2], [x, y], heading)
        if direction_diff > 30:
            reward = 1e-3

        # Zero reward of obviously too slow
        speed_diff_zero = optimals[2]-speed
        if speed_diff_zero > 0.5:
            reward = 1e-3

        ## Incentive for finishing the lap in less steps ##
        REWARD_FOR_FASTEST_TIME = 1500 # should be adapted to track length and other rewards
        STANDARD_TIME = 37  # seconds (time that is easily done by model)
        FASTEST_TIME = 27  # seconds (best time of 1st place on the track)
        if progress == 100:
            finish_reward = max(1e-3, (-REWARD_FOR_FASTEST_TIME /
                                       (15*(STANDARD_TIME-FASTEST_TIME)))*(steps-STANDARD_TIME*15))
        else:
            finish_reward = 0
        reward += finish_reward

        ## Zero reward if off track ##
        if all_wheels_on_track == False:
            reward = 1e-3

        ####################### VERBOSE #######################

        if self.verbose == True:
            print("Closest index: %i" % closest_index)
            print("Distance to racing line: %f" % dist)
            print("=== Distance reward (w/out multiple): %f ===" % (distance_reward))
            print("Optimal speed: %f" % optimals[2])
            print("Speed difference: %f" % speed_diff)
            print("=== Speed reward (w/out multiple): %f ===" % speed_reward)
            print("Direction difference: %f" % direction_diff)
            print("Predicted time: %f" % projected_time)
            print("=== Steps reward: %f ===" % steps_reward)
            print("=== Finish reward: %f ===" % finish_reward)

        #################### RETURN REWARD ####################

        # Always return a float value
        return float(reward)

reward_object = Reward() # add parameter verbose=True to get noisy output for testing


def reward_function(params):
    return reward_object.reward_function(params)