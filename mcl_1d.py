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

        # generate initial random distribution of particles
        self.x_t = numpy.zeros((self.npart, 2))
        self.w = numpy.ones((self.npart, 1))

        # generate random starting location
        self.x_t[:, 0] = numpy.random.rand(self.npart) * (self.map_length-1)
        # generate random heading
        self.x_t[:, 1] = (numpy.random.rand(self.npart) * 6) - 3

    # updates the state of each particle based on it's state, essentialy moves
    # the particle position by it's velocity value
    def motion_update(self, speed):

        self.x_t[:, 0] = self.x_t[:, 0] + self.x_t[:, 1]

    # updates the weights of each particle, weight is inversely proportional to 
    # the difference between the measurement and the map value at the particle's
    # current position
    def sensor_update(self, measurement):

        # get all invalid indices
        ind1 = numpy.where(self.x_t[:, 0] < 0) 
        ind2 = numpy.where(self.x_t[:, 0] > self.map_length-1)   
        invalid = numpy.union1d(ind1[0],ind2[0])    

        # discard by setting weight to 0
        self.w[invalid] = 0

        # get all valid particles
        valid = numpy.setdiff1d(range(self.npart),invalid)

        x = numpy.rint(self.x_t[valid,0].ravel())
        map_vals = self.map[x.astype(int)]

        # calculate weights
        weights = 1/(0.00001 + abs(measurement - map_vals)**3)

        # update weights
        self.w[valid,0] = weights


    # Performs the resampling of the new particle set from the old particle set
    def resample_particles(self):

        # initialise the new sample array
        new_x_t = numpy.zeros((self.npart, 2))

        # normalise weights
        w_total = sum(self.w)
        self.w = self.w / w_total

        # get all non-zero weights and matching indices
        indices = numpy.where(self.w[:,0] != 0)
        indices = indices[0]
        w_pick = self.w[indices,0]

        # randomly resample from old particles based on probability
        index = numpy.random.choice(indices, size=int(self.npart*0.9), p=w_pick)
        new_x_t[range(int(self.npart*0.9))] = self.x_t[index]

        # add some noise to the newly sampled particles
        new_x_t[range(int(self.npart*0.9)), 0] = new_x_t[range(int(self.npart*0.9)), 0] + (numpy.random.rand(int(self.npart*0.9)) * 2) - 1
        new_x_t[range(int(self.npart*0.9)), 1] = new_x_t[range(int(self.npart*0.9)), 1] + (numpy.random.rand(int(self.npart*0.9)) * 2) - 1

        #always regenerate some particles entirely randomly to prevent particle deprivation

        # generate random starting location
        new_x_t[int(self.npart*0.9):self.npart-1, 0] = numpy.random.rand(int(self.npart*0.1)-1) * (self.map_length-1)
        # generate random heading
        new_x_t[int(self.npart*0.9):self.npart-1, 1] = (numpy.random.rand(int(self.npart*0.1)-1) * 6) - 3

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



if __name__ == "__main__":

    # map of hallway
    hall_map = numpy.random.randint(1, 100, 200)

    print('Map')
    print(hall_map)
    print(' ')

    # sequence of measurements
    sensor_seq = hall_map[50:150]

    for i in range(30):
        sensor_seq = numpy.append(sensor_seq, hall_map[149])

    sensor_seq = numpy.append(sensor_seq, hall_map[20:50])

    # add some noise to the measurements
    sensor_seq = sensor_seq + numpy.random.randn(sensor_seq.size)

    #list of locations
    seq_length = 160
    start_pos = 50
    num_particles = 2000
    speed = 1

    # particle filter initialise
    mcl = monte_carlo_localizer(num_particles, hall_map)

    plt.scatter(mcl.x_t[:,0], range(mcl.npart), s=1, c='k', marker='o', label="particles")
    plt.scatter(start_pos, -10, s=100, c='g', marker='o', edgecolors='g', label="true location")
    plt.xlabel('x [m]')
    plt.ylabel('y [m]')
    plt.ylim([-10, num_particles])
    plt.xlim([-1, mcl.map_length])
    plt.pause(1)
        

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
        

