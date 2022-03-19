import numpy as np
import math 

import matplotlib.pyplot as plt

class VehicleParams:
    def __init__(self,**kwargs):
        self.width = kwargs['width'] if 'width' in kwargs else 0
        self.length = kwargs['length'] if 'length' in kwargs else 0

class CircularArcTrajParams:
    def __init__(self,**kwargs):
        self.forward_speed = kwargs['forward_speed'] if 'forward_speed' in kwargs else 0
        self.angular_rate = kwargs['angular_rate'] if 'angular_rate' in kwargs else 0

class FreeSpaceTemplateParams:
    def __init__(self,**kwargs):
        self.min_obstacle_size = kwargs['min_obstacle_size'] if 'min_obstacle_size' in kwargs else 1
        self.time_horizon = kwargs['time_horizon'] if 'time_horizon' in kwargs else 1
        self.traj_params = kwargs['traj_params']
        self.vehicle_params = kwargs['vehicle_params']

class CircularArcSamplerParams:
    def __init__(self,**kwargs):
        self.xp = kwargs['xp'] if 'xp' in kwargs else 0
        self.yp = kwargs['yp'] if 'yp' in kwargs else 0
        self.min_obstacle_size = kwargs['min_obstacle_size'] if 'min_obstacle_size' in kwargs else 1
        self.time_horizon = kwargs['time_horizon'] if 'time_horizon' in kwargs else 1
        self.forward_speed = kwargs['forward_speed'] if 'forward_speed' in kwargs else 0
        self.angular_rate = kwargs['angular_rate'] if 'angular_rate' in kwargs else 0

def sample_free_space(ax,params):
    kwargs = {
        'forward_speed': params.traj_params.forward_speed,
        'angular_rate': params.traj_params.angular_rate,
        'time_horizon': params.time_horizon,
        'min_obstacle_size': params.min_obstacle_size
    }

    circular_arc_sampler_params = CircularArcSamplerParams(**kwargs)

    circular_arc_sampler_params.xp = params.vehicle_params.length/2.
    circular_arc_sampler_params.yp = -params.vehicle_params.width/2.
    samples = sample_circular_arc(circular_arc_sampler_params)
    ax.plot(samples[0,:], samples[1,:], 'ob')
    print(samples.shape)

    circular_arc_sampler_params.xp = params.vehicle_params.length/2
    circular_arc_sampler_params.yp = params.vehicle_params.width/2
    samples = sample_circular_arc(circular_arc_sampler_params)
    ax.plot(samples[0,:], samples[1,:], 'ob')
    print(samples.shape)

def sample_circular_arc(params):
    v = params.forward_speed
    w = params.angular_rate
    time_horizon = params.time_horizon
    do = params.min_obstacle_size
    xp = params.xp
    yp = params.yp


    # Suppoting variables
    d1 = (v-yp*w)/w
    d2 = xp
    
    sampling_time = (1/w)*math.acos(1 - (do**2)/(2*(d1**2 + d2**2)))

    nb_samples = math.floor(time_horizon*1./sampling_time) 
    samples = np.zeros([2,nb_samples])
    #print(1./sampling_time)
    for i in range(0, nb_samples):
        s, c = math.sin(w*sampling_time*(i+1)), math.cos(w*sampling_time*(i+1))
        x = (v/w - yp)*s + xp*(c-1) + xp
        y = (v/w - yp)*(1-c) + xp*s + yp
        samples[:,i] = np.array([x,y])
    
    diff = samples[:,1:] - samples[:,0:-1]  
    print(samples)
    #print(np.sqrt(np.diag(diff.T@diff)))
    return samples

def plot_vehicle(ax, width=.3, length=.4):
    x = vehicle_params.length/2.0
    y = vehicle_params.width/2.0
    contour = np.array([[x,y], [-x,y], [-x,-y], [x,-y], [x,y]])

    ax.plot(contour[:,0],contour[:,1])
    
"""
params is of type FreeSpaceTemplateParams
"""
def create_free_space_template(params):
    pass

if __name__ =='__main__':
    ax = plt.subplot()
    # Vehicle
    kwargs = {'width': .3,  'length': 0}
    vehicle_params = VehicleParams(**kwargs)
    plot_vehicle(ax, vehicle_params)

    # Trajectory
    kwargs = {'forward_speed': .5, 'angular_rate': 2.051051}
    circular_arc_traj_params = CircularArcTrajParams(**kwargs)

    # Free space
    kwargs = {'min_obstacle_size': .05,  'time_horizon': 1,
        'traj_params': circular_arc_traj_params,
        'vehicle_params': vehicle_params}
    free_space_params = FreeSpaceTemplateParams(**kwargs)
    sample_free_space(ax, free_space_params)
    #print(free_space_params.forward_speed)

    # plot
    ax.axis('equal')
    ax.grid('on')
    plt.show()