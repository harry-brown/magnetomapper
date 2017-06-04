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
        self.map_y, self.map_x = env_map.shape
        
        self.pos = numpy.array([self.map_x/2, self.map_y/2])

        # generate initial random distribution of particles
        self.x_t = numpy.zeros((self.npart,4))

        self.w = numpy.ones((self.npart,1))

        # generate random starting location
        self.x_t[:, 0] = numpy.random.rand(self.npart) * (self.map_x-1)
        self.x_t[:, 1] = numpy.random.rand(self.npart) * (self.map_y-1)

        # generate random heading
        self.x_t[:, 2] = (numpy.random.rand(self.npart) * 2) -1
        self.x_t[:, 3] = (numpy.random.rand(self.npart) * 2) -1


    # updates the state of each particle based on it's state, essentialy moves
    # the particle position by it's velocity value
    def motion_update(self, speed):
    
        self.x_t[:, 0] = self.x_t[:, 0] + self.x_t[:, 2]
        self.x_t[:, 1] = self.x_t[:, 1] + self.x_t[:, 3]


    # updates the weights of each particle, weight is inversely proportional to 
    # the difference between the measurement and the map value at the particle's
    # current position
    def sensor_update(self, measurement):

        # get indices of particles that have left the map
        ind1 = numpy.where(self.x_t[:, 0] < 0) 
        ind2 = numpy.where(self.x_t[:, 0] > self.map_x-1) 
        ind3 = numpy.where(self.x_t[:, 1] < 0)
        ind4 = numpy.where(self.x_t[:, 1] > self.map_y-1)
        invalid = numpy.union1d(numpy.union1d(ind1[0],ind2[0]),numpy.union1d(ind3[0],ind4[0]))

        # discard by setting weight to 0
        self.w[invalid] = 0
        
        # get all valid particles
        valid = numpy.setdiff1d(range(self.npart),invalid)

        # get corresponding map values
        x = numpy.rint(self.x_t[valid,0].ravel())
        y = numpy.rint(self.x_t[valid,1].ravel())
        map_vals = self.map[x.astype(int),y.astype(int)]

        # calculate weights
        weights = 1/(0.00001 + abs(measurement - map_vals)**2)

        # update weights
        self.w[valid,0] = weights


    # Performs the resampling of the new particle set from the old particle set   
    def resample_particles(self):

        # initialise the new sample array
        new_x_t = numpy.zeros((self.npart, 4))

        # normalise weights
        w_total = sum(self.w)
        self.w = self.w / w_total

        # get all non-zero weights and matching indices
        indices = numpy.where(self.w[:,0] != 0)
        indices = indices[0]
        w_pick = self.w[indices,0]
        
        # randomly resample from old particles based on probability
        index = numpy.random.choice(indices, size=int(self.npart*0.8), p=w_pick)
        new_x_t[range(int(self.npart*0.8))] = self.x_t[index]

        # add some noise to the newly sampled particles
        new_x_t[range(int(self.npart*0.8)), 0] = new_x_t[range(int(self.npart*0.8)), 0] + (numpy.random.rand(int(self.npart*0.8)) * 2) - 1
        new_x_t[range(int(self.npart*0.8)), 1] = new_x_t[range(int(self.npart*0.8)), 1] + (numpy.random.rand(int(self.npart*0.8)) * 2) - 1

        new_x_t[range(int(self.npart*0.8)), 2] = new_x_t[range(int(self.npart*0.8)), 2] + (numpy.random.rand(int(self.npart*0.8)) * 2) -1
        new_x_t[range(int(self.npart*0.8)), 3] = new_x_t[range(int(self.npart*0.8)), 3] + (numpy.random.rand(int(self.npart*0.8)) * 2) -1

        #always regenerate some particles entirely randomly to prevent particle deprivation

        # generate random starting location
        new_x_t[int(self.npart*0.8):self.npart-1, 0] = numpy.random.rand(int(self.npart*0.2)-1) * (self.map_x-1)
        new_x_t[int(self.npart*0.8):self.npart-1, 1] = numpy.random.rand(int(self.npart*0.2)-1) * (self.map_y-1)

        # generate random heading
        new_x_t[int(self.npart*0.8):self.npart-1, 2] = (numpy.random.rand(int(self.npart*0.2)-1) * 2) -1
        new_x_t[int(self.npart*0.8):self.npart-1, 3] = (numpy.random.rand(int(self.npart*0.2)-1) * 2) -1

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
        pos = numpy.array([0, 0]);
        pos[0] = sum(self.x_t[:, 0])
        pos[1] = sum(self.x_t[:, 1])
        mcl.pos = pos / self.npart


if __name__ == "__main__":

    # map of hallway
    hall_map = numpy.random.randint(0,100,size=[100, 100])

    # sequence of measurements
    sensor_seq = hall_map[20, 20:80];
    sensor_seq = numpy.append(sensor_seq,hall_map[20:50, 80])
    sensor_seq = numpy.append(sensor_seq,hall_map[50, 10:80])
    for i in range(50):
        sensor_seq = numpy.append(sensor_seq, hall_map[50,80])

    # add some noise to the measurements
    # sensor_seq = sensor_seq + numpy.random.randn(sensor_seq.size)
        
    #list of locations
    seq_length = 210
    start_pos = numpy.array([20,20])
    true_location = start_pos
    # velocity estimate
    speed = 1

    # number of particles (more = more accurate but also slower)
    num_particles = 10000

    # particle filter initialise
    mcl = monte_carlo_localizer(num_particles, hall_map)

    fig = plt.figure()

    # plot the initial distribution of particles
    plt.scatter(mcl.x_t[:,0], mcl.x_t[:,1], s=0.2, c='k', marker='o', label="particles")
    plt.scatter(20, 20, s=10, c='g', marker='o', edgecolors='g', label="true location")
    #plt.plot(range(mcl.map_length),pdf*mcl.npart*10, label='kde')
    plt.xlabel('x [m]')
    plt.ylabel('y [m]')
    plt.ylim([0, mcl.map_y])
    plt.xlim([0, mcl.map_x])
    plt.pause(0.5)


    # simulation
    for i in range(seq_length):

        plt.clf()

        # get reading
        measurement = sensor_seq[i]

        # update
        mcl.update(measurement,speed)

        # get the location using kde, rather than simple mean

        # this create the kernel, given an array it will estimate the probability over that values
        kde = gaussian_kde( numpy.stack([mcl.x_t[:,0].ravel(), mcl.x_t[:,1].ravel()]) )

        # these are the values over wich your kernel will be evaluated
        X,Y = numpy.mgrid[range(mcl.map_x), range(mcl.map_y)]
        positions = numpy.stack([X.ravel(),Y.ravel()])

        # evaluate kernel
        pdf = kde(positions);

        # get index of maximum
        inds = pdf.argmax()

        # get map position of maximum
        est = positions[:, int(numpy.mean(inds))]

        if (i < 60):
            true_location[0] = 20
            true_location[1] = 20+i
        elif (i < 90):
            true_location[0] = 20+(i-60)
            true_location[1] = 80
        elif (i < 160):
            true_location[0] = 50
            true_location[1] = 10+(i-90)
        else:
            true_location[0] = 50
            true_location[1] = 80

        # plot the results
        plt.imshow(numpy.rot90(numpy.reshape(pdf.T, X.shape)), cmap=plt.cm.gist_earth_r, extent=[0, mcl.map_x, 0, mcl.map_y],  label="kernel")
        plt.scatter(mcl.x_t[:,0], mcl.x_t[:,1], s=0.2, c='k', marker='o', label="particles")
        plt.scatter(est[0], est[1], s=10, c='red', marker='o', label="estimated location")
        plt.scatter(true_location[0], true_location[1], s=10, c='g', marker='o', edgecolors='g', label="true location")
        #plt.plot(range(mcl.map_length),pdf*mcl.npart*10, label='kde')
        plt.xlabel('x [m]')
        plt.ylabel('y [m]')
        plt.ylim([0, mcl.map_y])
        plt.xlim([0, mcl.map_x])
        plt.pause(0.05)

