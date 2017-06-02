import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import numpy
from scipy.stats.kde import gaussian_kde

# implements a 1D monte carlo localization class,
# each particle is a value pair consisting of a
# position and a velocity along the line
class monte_carlo_localizer:

    # initialises the localizer, stores the map and the number of particles
    # as well as generates the initial random distribution
    def __init__(self, num_particles, env_map):

        self.npart = num_particles

        self.map = env_map
        self.map_length = env_map.size

        self.pos = self.map_length/2

        # generate initial random distribution of particles
        self.x_t = numpy.zeros((self.npart, 2))

        self.w = numpy.ones((self.npart, 1))

        for i in range(self.npart):

            # generate random starting location
            self.x_t[i,0] = numpy.random.randint(0, self.map_length)

            # generate random heading
            self.x_t[i,1] = numpy.random.randint(-3, 3)


    # updates the state of each particle based on it's state, essentialy moves
    # the particle position by it's velocity value
    def motion_update(self, speed):

        for i in range(self.npart):

            # maybe add some noise here?
            self.x_t[i, 0] = self.x_t[i, 0] + self.x_t[i, 1]


    # updates the weights of each particle, weight is inversely proportional to 
    # the difference between the measurement and the map value at the particle's
    # current position
    def sensor_update(self, measurement):

        for i in range(self.npart):

            # if particle has left the map, discard
            if self.x_t[i, 0] < 0 or self.x_t[i, 0] > self.map_length-1:
                self.w[i] = 0
            else:
                # weight is inverse of difference between map and measurement
                map_val = self.map[int(self.x_t[i, 0])]
                self.w[i] = 1/(1+abs(measurement - map_val)**3)

    def resample_particles(self):

        # initialise the new sample array
        new_x_t = numpy.zeros((self.npart, 2))

        # normalise weights
        w_total = sum(self.w)
        w_pick = []
        indices = []
        for i in range(self.npart):

            self.w[i] = self.w[i] / w_total

            if self.w[i] > 0:
                w_pick.append(self.w[i, 0])
                indices.append(i)

        # resample based on weights
        for i in range(int(self.npart*0.9)):

            # choose new particle based on weights
            index = numpy.random.choice(indices, p=w_pick)
            new_x_t[i] = self.x_t[index]

            # add some noise to the selection
            new_x_t[i, 0] = new_x_t[i, 0] + numpy.random.randint(-2, 2)
            new_x_t[i, 1] = new_x_t[i, 1] + numpy.random.randint(-2, 2)

        # always regenerate some particles entirely randomly to prevent particle deprivation
        for i in range(int(self.npart*0.1)):

            # generate random starting location
            new_x_t[int(self.npart*0.9) + i, 0] = numpy.random.randint(0, self.map_length)

            # generate random heading
            new_x_t[int(self.npart*0.9) + i, 1] = numpy.random.randint(-3, 3)


        # store new particles
        self.x_t = new_x_t


    # updates the distribution based on the motion model and the weight function
    def update(self, measurement, speed):

        # perform the update steps

        # project particles in time
        self.motion_update(speed)

        # calculate weights based on sensor measurement
        self.sensor_update(measurement)

        # resample particles
        self.resample_particles()

        # calculate new mean pos
        pos = 0
        for i in range(self.npart):
            pos = pos + self.x_t[i, 0]
        mcl.pos = pos / self.npart


if __name__ == "__main__":

    # map of hallway
    hall_map = numpy.random.randint(-10, 10, 200)

    print('Map')
    print(hall_map)
    print(' ')

    # sequence of measurements
    sensor_seq = hall_map[50:150]

    for i in range(30):
        sensor_seq = numpy.append(sensor_seq, hall_map[149])

    sensor_seq = numpy.append(sensor_seq, hall_map[20:50])

    #list of locations
    seq_length = 160
    start_pos = 50

    print('Sequence')
    print(sensor_seq)
    print(' ')

    # velocity estimate
    speed = 1

    num_particles = 500

    # particle filter initialise
    mcl = monte_carlo_localizer(num_particles, hall_map)

    fig = plt.figure()

    # simulation
    for i in range(seq_length):

        plt.clf()

        # get reading
        measurement = sensor_seq[i]

        # update
        mcl.update(measurement, speed)

        # get the location using kde, rather than simple mean

        # this create the kernel, given an array it will estimate the probability over that values
        kde = gaussian_kde( numpy.transpose(mcl.x_t[:, 0]) )
        # these are the values over wich your kernel will be evaluated
        pdf = kde(range(mcl.map_length))
        inds = numpy.argmax(pdf)
        est = numpy.mean(inds)

        print('Estimate')
        print(est)

        if i < 100:
            true_location = start_pos + i
        elif i < 130:
            true_location = start_pos + 99
        else:
            true_location = 20 + (i-130)
            
        print('True Location')
        print(true_location)
        print(' ')

        
        # plot the results
        plt.scatter(mcl.x_t[:,0], range(mcl.npart), s=1, c='k', marker='o', label="particles")
        plt.scatter(est, 0, s=100, c='red', marker='o', label="estimated location")
        plt.scatter(true_location, -10, s=100, c='g', marker='o', edgecolors='g', label="true location")
        plt.plot(range(mcl.map_length),pdf*mcl.npart*10, label='kde')
        plt.xlabel('x [m]')
        plt.ylabel('y [m]')
        plt.ylim([-10, num_particles])
        plt.xlim([-1, mcl.map_length])
        plt.pause(0.05)
        

