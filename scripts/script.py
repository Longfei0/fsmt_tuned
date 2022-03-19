from cmath import nan
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

        self.P_0 = np.array([-self.length/2+self.rear_axle_offset, 0 ])
        self.P_FR = np.array([self.length/2, -self.width/2 ]) - self.P_0
        self.P_FL = np.array([self.length/2, self.width/2 ]) - self.P_0
        self.P_RR = np.array([-self.length/2, -self.width/2 ]) - self.P_0
        self.P_RL = np.array([-self.length/2, self.width/2 ]) - self.P_0
        self.P_rl = np.array([-self.length/2+self.rear_axle_offset, self.width/2 ]) - self.P_0
        self.P_rr = np.array([-self.length/2+self.rear_axle_offset, -self.width/2 ]) - self.P_0

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

def create_circular_arc_template(ax, traj_params, vehicle_params):
    T = traj_params.time_horizon
    v = traj_params.nominal_forward_speed
    r = traj_params.radius
    w = v/r
    min_obstacle_size = traj_params.min_obstacle_size
    s, c = math.sin(w*T), math.cos(w*T)
    x0, y0 = 0, traj_params.radius

    # ARC
    xi, yi = vehicle_params.P_FR[0], vehicle_params.P_FR[1]      
    xf, yf = r*s + xi*c - yi*s, xi*s + yi*c + r*(1-c) 
    rp = np.sqrt((r-yi)**2 + xi**2)
    sampling_rate = min_obstacle_size/(rp)

    kwargs = {"x0": x0, "y0": y0, "xi": xi, "yi": yi, "xf": xf, "yf": yf, "radius": rp, "sampling_rate": sampling_rate}
    circular_arc_params = CircularArcParams(**kwargs)
    samples = sample_circular_arc(circular_arc_params)

    # Line segment
    xi = r*s + vehicle_params.P_FR[0]*c - vehicle_params.P_FR[1]*s
    yi = vehicle_params.P_FR[0]*s + vehicle_params.P_FR[1]*c + r*(1-c)
    xf = r*s + vehicle_params.P_FL[0]*c - vehicle_params.P_FL[1]*s
    yf = vehicle_params.P_FL[0]*s + vehicle_params.P_FL[1]*c + r*(1-c) 
    sampling_rate = min_obstacle_size

    kwargs = {"xi": xi, "yi": yi, "xf": xf, "yf": yf, "sampling_rate": sampling_rate}
    circular_arc_params = LineSegmentParams(**kwargs)
    new_samples = sample_line_segment(circular_arc_params)
    samples = np.hstack([samples, new_samples])

    # Line segment
    xi = r*s + vehicle_params.P_FL[0]*c - vehicle_params.P_FL[1]*s
    yi = vehicle_params.P_FL[0]*s + vehicle_params.P_FL[1]*c + r*(1-c)
    xf = r*s + vehicle_params.P_rl[0]*c - vehicle_params.P_rl[1]*s
    yf = vehicle_params.P_rl[0]*s + vehicle_params.P_rl[1]*c + r*(1-c) 
    sampling_rate = min_obstacle_size

    kwargs = {"xi": xi, "yi": yi, "xf": xf, "yf": yf, "sampling_rate": sampling_rate}
    circular_arc_params = LineSegmentParams(**kwargs)
    new_samples = sample_line_segment(circular_arc_params)
    samples = np.hstack([samples, new_samples])
    
    # ARC
    xf, yf = vehicle_params.P_rl[0], vehicle_params.P_rl[1]      
    xi, yi = r*s + xf*c - yf*s, xf*s + yf*c + r*(1-c) 
    rp = np.sqrt((r-yi)**2 + xi**2)
    sampling_rate = min_obstacle_size/rp

    kwargs = {"x0": x0, "y0": y0, "xi": xi, "yi": yi, "xf": xf, "yf": yf, "radius": rp, "sampling_rate": sampling_rate}
    circular_arc_params = CircularArcParams(**kwargs)
    new_samples = sample_circular_arc(circular_arc_params)
    samples = np.hstack([samples, new_samples])
    ax.plot(samples[0,:], samples[1,:], 'o', markerfacecolor='w', markeredgecolor='k', markersize=8)    

    return samples

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

def sample_template_lidar(ax, samples, traj_params, vehicle_params, lidar_params):
    T = traj_params.time_horizon
    v = traj_params.nominal_forward_speed
    r = traj_params.radius
    w = v/r
    s, c = math.sin(w*T), math.cos(w*T)

    xl, yl = lidar_params.x + vehicle_params.P_RR[0], lidar_params.y + vehicle_params.P_RR[1]
    x0, y0 = 0 - xl, traj_params.radius - yl
    xr, yr = vehicle_params.P_rl[0], vehicle_params.P_rl[1]      
    r = traj_params.radius

    rp = np.sqrt((r-yr)**2 + xr**2)
    rl = np.sqrt((r-yl)**2 + xl**2)

    c = np.sqrt(rl**2-rp**2)
    a = y0*y0 + x0*x0
    b = -2*x0*c
    c = c**2 - y0**2
    delta = b**2 - 4*a*c
    angle = nan
    if delta >= 0:
        s11 = (-b + np.sqrt(delta) )/ (2*a)
        s21 = (-b - np.sqrt(delta) )/ (2*a)

        s12 = np.sqrt(1  - s11**2) 
        s22 = np.sqrt(1  - s21**2)

        angle_s1 = math.atan2(s12,s11)
        angle_s2 = math.atan2(s22,s21)
        if angle_s1 < angle_s2:
            angle = angle_s1
            ax.plot([xl, xl+s11*2], [yl,yl+s12*2], '-r')
        else:
            angle = angle_s2
            ax.plot([xl, xl+s21*2], [yl,yl+s22*2], '-r')

    s, c = math.sin(w*T), math.cos(w*T)
    xf = r*s + vehicle_params.P_FL[0]*c - vehicle_params.P_FL[1]*s
    yf = vehicle_params.P_FL[0]*s + vehicle_params.P_FL[1]*c + r*(1-c) 
    angle_2 = math.atan2(yf - yl,
                         xf - xl) 
    
    # Choosing samples
    nb_samples = samples.shape[1]
    index = -1*np.ones(nb_samples, dtype='int')
    distance = np.ones([3, nb_samples])
    state = 0
    angle_i_ant = -np.pi/2
    for i in range(nb_samples):
        sample = samples[:,i]
        angle_i = math.atan2(sample[1] - yl,
                             sample[0] - xl) 
        print(angle_i*180/np.pi)

        distance[0,i] = np.sqrt((sample[0] - xl)**2 + (sample[1] - yl)**2)

        if (state==0 and angle_i > angle):
            state = 1
        elif (state==1 and angle_i < angle_i_ant): 
            state = 2
        elif (state== 2 and angle_i > angle_2):
            state = 3
        index[i] = state 
        angle_i_ant = angle_i


    lidar_samples_0 = samples[:, index==0]
    distance_0 = distance[0,index==0]
    lidar_samples_1 = samples[:, index==1]
    lidar_samples_2 = samples[:, index==2]
    lidar_samples_3 = samples[:, index==3]

    print(np.sum(index==0) + np.sum(index==2) + np.sum(index==3))
    print(index.shape[0])
    ax.plot(lidar_samples_0[0,:], lidar_samples_0[1,:], 'or')
    ax.plot(lidar_samples_1[0,:], lidar_samples_1[1,:], 'og')
    ax.plot(lidar_samples_3[0,:], lidar_samples_3[1,:], 'or')
        
    
"""
params is of type FreeSpaceTemplateParams
"""
def create_free_space_template(params):
    pass

if __name__ =='__main__':
    ax = plt.subplot()
    # Vehicle
    kwargs = {'width': .3,  'length': .6, 'rear_axle_offset': .0}
    vehicle_params = VehicleParams(**kwargs)
    plot_vehicle(ax, vehicle_params)

    # Free spac
    kwargs = {'min_obstacle_size': .1,  'time_horizon': 4, 'radius': 1, 'nominal_forward_speed': .5}
    traj_params = CircularFreeSpaceTemplateParams(**kwargs)
    samples = create_circular_arc_template(ax, traj_params, vehicle_params)

    # LiDAR
    kwargs = {'x': 0.5,  'y': 0.2}
    lidar_params = LiDARParams(**kwargs)
    sample_template_lidar(ax, samples, traj_params, vehicle_params, lidar_params)

    # plot
    ax.axis('equal')
    ax.grid('on')
    plt.show()