import numpy as np
import math 

import matplotlib.pyplot as plt

class LiDARParams:
    def __init__(self,**kwargs):
        self.x = kwargs['x'] if 'x' in kwargs else 0
        self.y = kwargs['y'] if 'y' in kwargs else 0

class VehicleParams:
    def __init__(self,**kwargs):
        self.width = kwargs['width'] if 'width' in kwargs else 0
        self.length = kwargs['length'] if 'length' in kwargs else 0
        self.rear_axle_offset = kwargs['rear_axle_offset'] if 'rear_axle_offset' in kwargs else self.length/2

        self.P_R = np.array([-self.length/2+self.rear_axle_offset, 0 ])
        self.P_FR = np.array([self.length/2, -self.width/2 ]) - self.P_R
        self.P_FL = np.array([self.length/2, self.width/2 ]) - self.P_R
        self.P_RR = np.array([-self.length/2, -self.width/2 ]) - self.P_R
        self.P_RL = np.array([-self.length/2, self.width/2 ]) - self.P_R
        self.P_rl = np.array([-self.length/2+self.rear_axle_offset, self.width/2 ]) - self.P_R
        self.P_rr = np.array([-self.length/2+self.rear_axle_offset, -self.width/2 ]) - self.P_R

class CircularArcParams:
    def __init__(self,**kwargs):
        self.x0 = kwargs['x0'] if 'x0' in kwargs else 0
        self.y0 = kwargs['y0'] if 'y0' in kwargs else 0
        self.xi, self.yi  = kwargs['xi'],  kwargs['yi'] 
        self.xf, self.yf  = kwargs['xf'],  kwargs['yf'] 
        self.radius = kwargs['radius']
        self.sampling_rate = kwargs['sampling_rate']

class LineSegmentParams:
    def __init__(self,**kwargs):
        self.xi, self.yi  = kwargs['xi'],  kwargs['yi'] 
        self.xf, self.yf  = kwargs['xf'],  kwargs['yf'] 
        self.sampling_rate = kwargs['sampling_rate'] 

class CircularFreeSpaceTemplateParams:
    def __init__(self,**kwargs):
        self.min_obstacle_size = kwargs['min_obstacle_size'] if 'min_obstacle_size' in kwargs else 1
        self.time_horizon = kwargs['time_horizon'] if 'time_horizon' in kwargs else 1
        self.radius = kwargs['radius']
        self.nominal_forward_speed = kwargs['nominal_forward_speed']


def sample_free_space(ax, traj_params, vehicle_params):
    T = traj_params.time_horizon
    v = traj_params.nominal_forward_speed
    r = traj_params.radius
    w = v/r
    min_obstacle_size = traj_params.min_obstacle_size
    s, c = math.sin(w*T), math.cos(w*T)
    x0, y0 = 0, traj_params.radius
    total_nb_samples = 0
    # ARC
    xi, yi = vehicle_params.P_FR[0], vehicle_params.P_FR[1]      
    xf, yf = r*s + xi*c - yi*s, xi*s + yi*c + r*(1-c) 
    rp = np.sqrt((r-yi)**2 + xi**2)
    sampling_rate = min_obstacle_size/(rp)

    kwargs = {"x0": x0, "y0": y0, "xi": xi, "yi": yi, "xf": xf, "yf": yf, "radius": rp, "sampling_rate": sampling_rate}
    circular_arc_params = CircularArcParams(**kwargs)
    samples = sample_circular_arc(circular_arc_params)
    ax.plot(samples[0,:], samples[1,:], '-r')
    total_nb_samples += samples.shape[1]
    
    # Line segment
    xi = r*s + vehicle_params.P_FR[0]*c - vehicle_params.P_FR[1]*s
    yi = vehicle_params.P_FR[0]*s + vehicle_params.P_FR[1]*c + r*(1-c)
    xf = r*s + vehicle_params.P_FL[0]*c - vehicle_params.P_FL[1]*s
    yf = vehicle_params.P_FL[0]*s + vehicle_params.P_FL[1]*c + r*(1-c) 
    sampling_rate = min_obstacle_size

    kwargs = {"xi": xi, "yi": yi, "xf": xf, "yf": yf, "sampling_rate": sampling_rate}
    circular_arc_params = LineSegmentParams(**kwargs)
    samples = sample_line_segment(circular_arc_params)
    ax.plot(samples[0,:], samples[1,:], '-g')
    total_nb_samples += samples.shape[1]

    # Line segment
    xi = r*s + vehicle_params.P_FL[0]*c - vehicle_params.P_FL[1]*s
    yi = vehicle_params.P_FL[0]*s + vehicle_params.P_FL[1]*c + r*(1-c)
    xf = r*s + vehicle_params.P_rl[0]*c - vehicle_params.P_rl[1]*s
    yf = vehicle_params.P_rl[0]*s + vehicle_params.P_rl[1]*c + r*(1-c) 
    sampling_rate = min_obstacle_size

    kwargs = {"xi": xi, "yi": yi, "xf": xf, "yf": yf, "sampling_rate": sampling_rate}
    circular_arc_params = LineSegmentParams(**kwargs)
    samples = sample_line_segment(circular_arc_params)
    ax.plot(samples[0,:], samples[1,:], '-b')
    total_nb_samples += samples.shape[1]
    
    # ARC
    xf, yf = vehicle_params.P_rl[0], vehicle_params.P_rl[1]      
    xi, yi = r*s + xf*c - yf*s, xf*s + yf*c + r*(1-c) 
    rp = np.sqrt((r-yi)**2 + xi**2)
    sampling_rate = min_obstacle_size/rp

    kwargs = {"x0": x0, "y0": y0, "xi": xi, "yi": yi, "xf": xf, "yf": yf, "radius": rp, "sampling_rate": sampling_rate}
    circular_arc_params = CircularArcParams(**kwargs)
    samples = sample_circular_arc(circular_arc_params)
    ax.plot(samples[0,:], samples[1,:], '-k')
    total_nb_samples += samples.shape[1]
    
    print('nb samples:', total_nb_samples)
    
def sample_circular_arc(params):
    radius = params.radius
    sampling_rate = params.sampling_rate
    x0, y0 = params.x0, params.y0
    xi, yi = params.xi, params.yi
    xf, yf = params.xf, params.yf

    # Suppoting variable
    theta_i = math.atan2(yi-y0, xi-x0)
    theta_f = math.atan2(yf-y0, xf-x0)
    delta_theta = theta_f - theta_i

    if delta_theta < 0:
        #delta_theta = -delta_theta
        sampling_rate *= -1
    nb_samples = math.ceil( delta_theta/sampling_rate) 
    samples = np.zeros([2,nb_samples])

    for i in range(0, nb_samples):
        angle = sampling_rate*i + theta_i
        s, c = math.sin(angle), math.cos(angle)
        x =  radius*c + x0
        y = radius*s + y0 
        samples[:,i] = np.array([x,y])
    
    return samples

def sample_line_segment(params):
    sampling_rate = params.sampling_rate
    xi, yi = params.xi, params.yi
    xf, yf = params.xf, params.yf

    # Suppoting variable
    length = math.sqrt((xf-xi)**2 + (yf-yi)**2 )
    xn, yn = (xf-xi)/length, (yf-yi)/length 
    nb_samples = math.ceil(length/sampling_rate) 

    samples = np.zeros([2,nb_samples])

    for i in range(0, nb_samples):
        x =  xi + i*sampling_rate*xn
        y = yi + i*sampling_rate*yn
        samples[:,i] = np.array([x,y])
    
    return samples

def plot_vehicle(ax, params):
    contour = np.array([params.P_FR, 
                        params.P_FL, 
                        params.P_RL, 
                        params.P_RR, 
                        params.P_FR])

    ax.plot(contour[:,0],contour[:,1])
    
"""
params is of type FreeSpaceTemplateParams
"""
def create_free_space_template(params):
    pass

if __name__ =='__main__':
    ax = plt.subplot()
    # Vehicle
    kwargs = {'width': .3,  'length': .4, 'rear_axle_offset': .1}
    vehicle_params = VehicleParams(**kwargs)
    plot_vehicle(ax, vehicle_params)

    # Free space
    kwargs = {'min_obstacle_size': .05,  'time_horizon': 3, 'radius': 10000000000000, 'nominal_forward_speed': .5}
    traj_params = CircularFreeSpaceTemplateParams(**kwargs)
    sample_free_space(ax, traj_params, vehicle_params)
    #print(free_space_params.forward_speed)

    # plot
    ax.axis('equal')
    ax.grid('on')
    plt.show()
