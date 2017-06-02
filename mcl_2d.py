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

        for i in range(self.npart):
            # generate random starting location
            self.x_t[i, 0] = numpy.random.rand() * (self.map_x-1)
            self.x_t[i, 1] = numpy.random.rand() * (self.map_y-1)

            # generate random heading
            self.x_t[i, 2] = (numpy.random.rand() * 2) -1
            self.x_t[i, 3] = (numpy.random.rand() * 2) -1

    # updates the state of each particle based on it's state, essentialy moves
    # the particle position by it's velocity value
    def motion_update(self, speed):
        
        for i in range(self.npart):
            self.x_t[i, 0] = self.x_t[i, 0] + self.x_t[i, 2]
            self.x_t[i, 1] = self.x_t[i, 1] + self.x_t[i, 3]


    # updates the weights of each particle, weight is inversely proportional to 
    # the difference between the measurement and the map value at the particle's
    # current position
    def sensor_update(self, measurement):


        for i in range(self.npart):

            # if particle has left the map, discard
            if (self.x_t[i, 0] < 0) or (self.x_t[i, 0] > self.map_x-1) or (self.x_t[i, 1] < 0) or (self.x_t[i, 1] > self.map_y-1):
                self.w[i] = 0
            else:
                # weight is inverse of difference between map and measurement
                map_val = self.map[int(self.x_t[i, 0]),int(self.x_t[i, 1])]
                self.w[i] = 1/(0.0001+abs(measurement - map_val)**3)

    def resample_particles(self):

        # initialise the new sample array
        new_x_t = numpy.zeros((self.npart, 4))

        # normalise weights
        w_total = sum(self.w)
        w_pick = []
        indices = []

        for i in range(self.npart):

            self.w[i] = self.w[i] / w_total

            if self.w[i] > 0:
                w_pick.append(self.w[i, 0])
                indices.append(i)

        # print(w_pick)
        # print(indices)
        
        # resample based on weights
        for i in range(int(self.npart*0.8)):

            # choose new particle based on weights
            index = numpy.random.choice(indices, p=w_pick)
            new_x_t[i] = self.x_t[index]

            #add some noise to the selection
            new_x_t[i, 0] = new_x_t[i, 0] + (numpy.random.rand() * 2) - 1
            new_x_t[i, 1] = new_x_t[i, 1] + (numpy.random.rand() * 2) - 1

            new_x_t[i, 2] = new_x_t[i, 2] + (numpy.random.rand() * 2) -1
            new_x_t[i, 3] = new_x_t[i, 3] + (numpy.random.rand() * 2) -1

        #always regenerate some particles entirely randomly to prevent particle deprivation
        for i in range(int(self.npart*0.2)):

            #generate random starting location
            new_x_t[int(self.npart*0.8) + i, 0] = numpy.random.rand() * (self.map_x-1)
            new_x_t[int(self.npart*0.8) + i, 1] = numpy.random.rand() * (self.map_y-1)

            #generate random heading
            new_x_t[int(self.npart*0.8) + i, 2] = (numpy.random.rand() * 2) -1
            new_x_t[int(self.npart*0.8) + i, 3] = (numpy.random.rand() * 2) -1

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
        for i in range(self.npart):
            pos[0] = pos[0] + self.x_t[i, 0]
            pos[1] = pos[1] + self.x_t[i, 1]
        mcl.pos = pos / self.npart


if __name__ == "__main__":

    # map of hallway
    hall_map = numpy.random.randint(0,100,size=[100, 100])
    #hall_map = numpy.ones([50,50])

    print('Map')
    print(hall_map)
    print(' ')

    # sequence of measurements
    sensor_seq = hall_map[20, 20:80];

    sensor_seq = numpy.append(sensor_seq,hall_map[20:50, 80])

    sensor_seq = numpy.append(sensor_seq,hall_map[50, 10:80])
        
    #list of locations
    seq_length = 160
    start_pos = numpy.array([20,20])

    print('Sequence')
    print(sensor_seq)
    print(' ')

    # velocity estimate
    speed = 1

    num_particles = 4000

    # particle filter initialise
    mcl = monte_carlo_localizer(num_particles, hall_map)

    fig = plt.figure()

    # plot the results
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

        # # this create the kernel, given an array it will estimate the probability over that values
        # kde = gaussian_kde( numpy.transpose(numpy.transpose(mcl.x_t[:,1:2])) )
        # # these are the values over wich your kernel will be evaluated
        # X,Y = np.mgrid[range(50)-1,range(50)-1]
        # positions = np.vstack([X.ravel(),Y.ravel()])
        # pdf = kde(positions);
        # inds = numpy.argmax(pdf) #redo this
        # est = numpy.mean(inds) #redo this

        # print('Estimate')
        # print(mcl.pos)

        if (i < 60):
            true_location = numpy.array([20, 20+i])
        elif (i < 90):
            true_location = numpy.array([20+(i-60),80])
        else:
            true_location = numpy.array([50,10 + (i-90)])
            
        # print('True Location')
        # print(true_location)
        # print(' ')

        # plot the results
        plt.scatter(mcl.x_t[:,0], mcl.x_t[:,1], s=0.2, c='k', marker='o', label="particles")
        plt.scatter(mcl.pos[0], mcl.pos[1], s=10, c='red', marker='o', label="estimated location")
        plt.scatter(true_location[0], true_location[1], s=10, c='g', marker='o', edgecolors='g', label="true location")
        #plt.plot(range(mcl.map_length),pdf*mcl.npart*10, label='kde')
        plt.xlabel('x [m]')
        plt.ylabel('y [m]')
        plt.ylim([0, mcl.map_y])
        plt.xlim([0, mcl.map_x])
        plt.pause(0.05)

