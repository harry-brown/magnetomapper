# mcl_1d_map_actual.py
# author: Tom Molnar
# date: 8/6/17
#
# performs the localization using the captured map data with the measurement
# sequences saved from the experiments
#


import matplotlib
#matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import csv
import numpy as np
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
        self.map_length = env_map.shape[0]

        # generate initial random distribution of particles
        self.x_t = np.zeros((self.npart, 2))
        self.w = np.ones((self.npart, 1))

        # generate random starting location
        self.x_t[:, 0] = np.random.rand(self.npart) * (self.map_length-1)
        # generate random heading
        self.x_t[:, 1] = (np.random.rand(self.npart) * 2) - 1

    # updates the state of each particle based on it's state, essentialy moves
    # the particle position by it's velocity value
    def motion_update(self, speed):

        self.x_t[:, 0] = self.x_t[:, 0] + self.x_t[:, 1]

    # updates the weights of each particle, weight is inversely proportional to 
    # the difference between the measurement and the map value at the particle's
    # current position
    def sensor_update(self, measurement):

        # get all invalid indices
        ind1 = np.where(self.x_t[:, 0] < 0) 
        ind2 = np.where(self.x_t[:, 0] > self.map_length-1)   
        invalid = np.union1d(ind1[0],ind2[0])    

        # discard by setting weight to 0
        self.w[invalid] = 0

        # get all valid particles
        valid = np.setdiff1d(range(self.npart),invalid)

        x = np.rint(self.x_t[valid,0].ravel())
        map_vals = self.map[x.astype(int),:]

##        # calculate weights
##        weights_mag = 1/(0.01 + abs(measurement[3] - map_vals[:,3]))
##        weights_angle = 1/(0.01 + np.arccos((measurement[0]*map_vals[:,0] + measurement[1]*map_vals[:,1] + measurement[2]*map_vals[:,2])/(measurement[3] * map_vals[:,3])))
##        # update weights
##        self.w[valid,0] = weights_mag + weights_angle

        # calculate weights
        weights_x = 1/(5 + abs(measurement[0] - map_vals[:,0])**3)
        weights_y = 1/(5 + abs(measurement[1] - map_vals[:,1])**3)
        weights_z = 1/(5 + abs(measurement[2] - map_vals[:,2])**3)

        # update weights
        self.w[valid,0] = (weights_x + weights_y + weights_z)/3


    # Performs the resampling of the new particle set from the old particle set
    def resample_particles(self):

        # initialise the new sample array
        new_x_t = np.zeros((self.npart, 2))

        # normalise weights
        w_total = sum(self.w)
        self.w = self.w / w_total

        # get all non-zero weights and matching indices
        indices = np.where(self.w[:,0] != 0)
        indices = indices[0]
        w_pick = self.w[indices,0]

        # randomly resample from old particles based on probability
        index = np.random.choice(indices, size=int(self.npart*0.9), p=w_pick)
        new_x_t[range(int(self.npart*0.9))] = self.x_t[index]

        # add some noise to the newly sampled particles
        new_x_t[range(int(self.npart*0.9)), 0] = new_x_t[range(int(self.npart*0.9)), 0] + (np.random.rand(int(self.npart*0.9))) - 0.5
        new_x_t[range(int(self.npart*0.9)), 1] = new_x_t[range(int(self.npart*0.9)), 1] + (np.random.rand(int(self.npart*0.9))) - 0.5

        #always regenerate some particles entirely randomly to prevent particle deprivation

        # generate random starting location
        new_x_t[int(self.npart*0.9):self.npart-1, 0] = np.random.rand(int(self.npart*0.1)-1) * (self.map_length-1)
        # generate random heading
        new_x_t[int(self.npart*0.9):self.npart-1, 1] = (np.random.rand(int(self.npart*0.1)-1) * 2) - 1

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
    hall_map = np.genfromtxt ('1d_map_updated.csv', delimiter=",")
    
    print('Map')
    print(hall_map)
    print(' ')

    # sequence of measurements
    #sensor_seq = hall_map[10:70,:]
    sensor_seq = np.genfromtxt ('hall_run_4.csv', delimiter=",")

    #hall_map = np.round(hall_map/1) * 1;
    #sensor_seq = np.round(sensor_seq/1) * 1;
    
    # add some noise to the measurements
    #sensor_seq = sensor_seq + (np.random.rand(sensor_seq.shape[0],sensor_seq.shape[1])*10) - 5
    
    #list of locations
    seq_length = sensor_seq.shape[0]
    start_pos = 0
    num_particles = 10000
    speed = hall_map.shape[0]/sensor_seq.shape[0]

    # particle filter initialise
    mcl = monte_carlo_localizer(num_particles, hall_map)

    plt.scatter(mcl.x_t[:,0], range(mcl.npart), s=1, c='k', marker='o', label="particles")
    plt.scatter(start_pos, -10, s=100, c='g', marker='o', edgecolors='g', label="true location")
    plt.xlabel('hallway_position [x0.5 m]')
    plt.ylabel('particles')
    plt.ylim([-10, num_particles])
    plt.xlim([-1, mcl.map_length])
    plt.pause(1)
        

    # simulation
    for i in range(0,seq_length):

        plt.clf()

        # get reading
        measurement = sensor_seq[i,:]

        # update
        mcl.update(measurement, speed)

        # get the location using kde, rather than simple mean

        # this create the kernel, given an array it will estimate the probability over that values
        kde = gaussian_kde( np.transpose(mcl.x_t[:, 0]) )
        # these are the values over wich your kernel will be evaluated
        pdf = kde(range(mcl.map_length))
        inds = np.argmax(pdf)
        est = np.mean(inds)

        print('Estimate')
        print(est)
 
        true_location = start_pos + i*speed

        # plot the results
        plt.scatter(mcl.x_t[:,0], range(mcl.npart), s=1, c='k', marker='o', label="particles")
        plt.scatter(est, 0, s=100, c='red', marker='o', label="estimated location")
        plt.scatter(true_location, -10, s=100, c='g', marker='o', edgecolors='g', label="true location")
        #plt.plot(range(mcl.map_length),pdf*mcl.npart*10, label='kde')
##        plt.plot(range(mcl.map_length),np.abs(hall_map[:,0]*mcl.npart/200), label='data_x', color='m')
##        plt.plot(range(mcl.map_length),np.abs(hall_map[:,1]*mcl.npart/200), label='data_y', color='r')
##        plt.plot(range(mcl.map_length),np.abs(hall_map[:,2]*mcl.npart/200), label='data_z', color='y')
##        plt.scatter(true_location, np.abs(measurement[0]*mcl.npart/200), s=100, c='m', marker='o', edgecolors='m')
##        plt.scatter(true_location, np.abs(measurement[1]*mcl.npart/200), s=100, c='r', marker='o', edgecolors='r')
##        plt.scatter(true_location, np.abs(measurement[2]*mcl.npart/200), s=100, c='y', marker='o', edgecolors='y')
        plt.xlabel('hallway_position [x0.5 m]')
        plt.ylabel('particles')
        plt.ylim(-10, mcl.npart)
        plt.xlim([-1, mcl.map_length])
        plt.pause(0.1)
        

