import rospy
import sys
import math
import numpy as np
from math import atan2
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.msg import ModelStates
from uav_nmpc_tracking_control.msg import output
from uav_nmpc_tracking_control.msg import Trajectory3D
from tf.transformations import euler_from_quaternion
from casadi import *
from message_filters import ApproximateTimeSynchronizer, Subscriber
#import keyboard
import cv2

master = sys.argv[1]
uav_num = master.split("_")[1]

# The time of the entire predict horizon is recommended to be greater than 1 second.
# In other words, N >= loop_rate
loop_rate = 40
T = 1./loop_rate # Sampling Time
N = 50 # Predict horizon
optimal_ibvs = False
ang_des_change = False
image_width = 640;
image_height = 480;
cx = 320
cy = 240
fx = 381.36
fy = 381.36
desired_distance = 7.0
desired_angle = pi/2
#Desired state value
sd_ = DM([(cx - cx)/fx, ((cy + 0.15*image_height) - cy)/fy, 1./desired_distance, desired_angle])   
# Callback function
drone_idx = 0
host_mocap = PoseStamped()
host_mocap_vel = TwistStamped()

def get_rotation(msg):
    orientation_list = [msg.x, msg.y, msg.z, msg.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    return roll, pitch, yaw

def callback(msg1,msg2,msg3):
    global tar_vel, X_state_, optimal_ibvs, tar_pos
    tar_vel = np.array([msg1.target_vel.x, msg1.target_vel.y, msg1.target_vel.z])
    tar_pos = np.array([msg1.target_pose.x, msg1.target_pose.y, msg1.target_pose.z])
    X_state_ = DM([msg1.feature_1.data, msg1.feature_2.data, msg1.feature_3.data, msg1.feature_4.data])
    optimal_ibvs = msg1.cmode.data

    global drone_idx, host_mocap, drone_pos, drone_vel    
    while msg2.name[drone_idx] != 'iris_'+uav_num:        
        if drone_idx<(len(msg2.name)-1):
            drone_idx += 1
    host_mocap.pose = msg2.pose[drone_idx]
    host_mocap_vel = msg2.twist[drone_idx]
    drone_pos = np.array([host_mocap.pose.position.x,host_mocap.pose.position.y,host_mocap.pose.position.z])
    drone_vel = np.array([host_mocap_vel.linear.x,host_mocap_vel.linear.y,host_mocap_vel.linear.z])
    
    global tar_acc
    tar_acc = np.array([msg3.acc.x, msg3.acc.y, msg3.acc.z])


# Declare model variables
x1 = SX.sym('x1')
x2 = SX.sym('x2')
x3 = SX.sym('x3')
x4 = SX.sym('x4')
x = vertcat(x1, x2, x3, x4)

u1 = SX.sym('u1')
u2 = SX.sym('u2')
u3 = SX.sym('u3')
u4 = SX.sym('u4')
u = vertcat(u1, u2, u3, u4)

vq1 = SX.sym('vq1')
vq2 = SX.sym('vq2')
vq3 = SX.sym('vq3')
vq4 = SX.sym('vq4')
vq = vertcat(vq1, vq2, vq3, vq4)

sd1 = SX.sym('sd1')
sd2 = SX.sym('sd2')
sd3 = SX.sym('sd3')
sd4 = SX.sym('sd4')
sd = vertcat(sd1, sd2, sd3, sd4)

# Predict Model equations
x1dot = -x3*(u1-vq1) + x1*x3*(u3-vq3) - (1 + x1**2)*u4
x2dot = -x3*(u2-vq2) + x2*x3*(u3-vq3) - x1*x2*u4
x3dot = x3**2*(u3-vq3) - x1*x3*u4
x4dot = (u1-vq1)*x3/sqrt(1+x1**2+x2**2)
xdot = vertcat(x1dot, x2dot, x3dot, x4dot)

# Multiple Function
A = SX.sym('A',3,3)
B = SX.sym('B',3)
mul = Function('mul',[A,B],[mtimes(A,B)])

# Objective term
Q = DM.eye(4)
W = DM.eye(4)
Q[0,0] = 1
Q[1,1] = 1
Q[2,2] = 100
Q[3,3] = 1
W[0,0] = 0.02
W[1,1] = 0.03
W[2,2] = 0.01
W[3,3] = 0.3
L = mtimes(mtimes((x-sd).T, Q), (x-sd)) + mtimes(mtimes((u-vq).T, W), (u-vq))

# Formulate discrete time dynamics
if True:
    f = Function('f', [x, sd, u, vq], [xdot, L])
    X0 = SX.sym('X0', 4)
    Sd = SX.sym('Sd', 4)
    U = SX.sym('U', 4)
    Vq = SX.sym('Vq', 4)
    
    X = X0
    Q = 0
    k1, k1_q = f(X, Sd, U, Vq)
    X = X + k1*T
    Q = Q + k1_q
    F = Function('F', [X0, Sd, U, Vq], [X, Q],['x0','xd','p','tv'],['xf','qf'])
else:
    # Fixed step Runge-Kutta 4 integrator
    M = 4 # RK4 steps per interval
    DT = T/M
    f = Function('f', [x, sd, u, vq], [xdot, L])
    X0 = SX.sym('X0', 4)
    Sd = SX.sym('Sd', 4)
    U = SX.sym('U', 4)
    Vq = SX.sym('Vq', 4)
    X = X0
    Q = 0
    for j in range(M):
       k1, k1_q = f(X, Sd, U, Vq)
       k2, k2_q = f(X + DT/2 * k1, Sd, U, Vq)
       k3, k3_q = f(X + DT/2 * k2, Sd, U, Vq)
       k4, k4_q = f(X + DT * k3, Sd, U, Vq)
       X=X+DT/6*(k1 +2*k2 +2*k3 +k4)
       Q = Q + DT/6*(k1_q + 2*k2_q + 2*k3_q + k4_q)
    F = Function('F', [X0, Sd, U, Vq], [X, Q],['x0','xd','p','tv'],['xf','qf'])

rospy.init_node('mpc_ibvs', anonymous=True)
tss = ApproximateTimeSynchronizer([Subscriber("/estimate_data"+uav_num, output),
                       Subscriber("/gazebo/model_states", ModelStates), Subscriber("/target_qp"+uav_num, Trajectory3D)],1, 0.1, allow_headerless=True)
tss.registerCallback(callback)

pub = rospy.Publisher('/uav'+uav_num+'/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=1)
rate = rospy.Rate(loop_rate)

for i in range(0,100):
    print('spin')
    rate.sleep()    
    

# Formulate the NLP
Xm = X_state_
cmd_vel = TwistStamped()
ang_des = TwistStamped()

# Test the initial point
Fk = F(x0=X_state_,xd=sd_,p=[0, 0, 0, 0],tv=[1,1,1,0])
print(Fk['xf'])
print(Fk['qf'])

while not rospy.is_shutdown():
    # Start with an empty NLP
    w0=[]
    w=[]
    lbw = []
    ubw = []
    g=[]
    lbg = []
    ubg = []
    J = 0
    # Calculate the rotation matrix between Camera Frame and Global Frame   

    drone_roll, drone_pitch, drone_yaw = get_rotation(host_mocap.pose.orientation)

    rotation_x = np.mat([[1,               0,                0],
                         [0, cos(drone_roll),  sin(drone_roll)],
                         [0,-sin(drone_roll),  cos(drone_roll)]])
                         
    rotation_y = np.mat([[cos(drone_pitch), 0, -sin(drone_pitch)],
                         [               0, 1,                 0],
                         [sin(drone_pitch), 0,  cos(drone_pitch)]])
                         
    rotation_z = np.mat([[ cos(drone_yaw), sin(drone_yaw), 0],
                         [-sin(drone_yaw), cos(drone_yaw), 0],
                         [0              ,              0, 1]])

    rotationB2C_y = np.mat([[cos(pi/2), 0, -sin(pi/2)],
                            [0,         1,          0],
                            [sin(pi/2), 0,  cos(pi/2)]])

    rotationB2C_z = np.mat([[ cos(-pi/2), sin(-pi/2), 0],
                            [-sin(-pi/2), cos(-pi/2), 0],
                            [0          ,          0, 1]])

    rot_g2c = rotationB2C_z*rotationB2C_y*rotation_z
    rot_c2g = rot_g2c.T
    

    # Calculate the target velocity in Camera Frame
    tar_vel_cam = mul(rot_g2c,tar_vel)
    tar_vel_cam_ = tar_vel_cam[:]
    tar_acc_cam = mul(rot_g2c,tar_acc)
    
    # "Lift" initial conditions
    X_state = X_state_[:]                              # Ensure that lbw, ubw and w0 are the same
    Xk = SX.sym('X0', 4)
    w += [Xk]
    lbw += list(X_state.full().flatten())
    ubw += list(X_state.full().flatten())
    w0 += list(X_state.full().flatten())
    
    epi = X_state - Xm
    Xd = sd_ - epi
    

    for k in range(N):
        Uk = SX.sym('U_' + str(k), u.shape[0])
        w += [Uk]
        lbw += [-10, -10, -10, -0.6]
        ubw += [10, 10, 10, 0.6]
        w0 += [0, 0, 0, 0]

        # Integrate till the end of the interval
        Fk = F(x0=Xk, xd=Xd, p=Uk, tv=DM(vertcat(tar_vel_cam_,0)))
        Xk_end = Fk['xf']
        J=J+Fk['qf']
        
        # New NLP variable for state at end of interval
        Xk = SX.sym('X_' + str(k+1), 4)
        w   += [Xk]
        lbw += [(0 - cx)/fx, (0 - cy)/fy, 1./15, -pi]
        ubw += [(image_width - cx)/fx, (image_height - cy)/fy, 1, pi]
        w0  += [0, 0, 0, 0]
        
        tar_vel_cam_ = tar_vel_cam_ + tar_acc_cam*T
    
        # Add equality constraint
        g   += [Xk_end-Xk]
        lbg += [0, 0, 0, 0]
        ubg += [0, 0, 0, 0]

    # Create an NLP solver
    prob = {'f': J, 'x': vertcat(*w), 'g': vertcat(*g)}
    opts={}
    
    opts["verbose_init"] = False
    opts["verbose"] = False
    opts["print_time"] = False
    opts["ipopt.print_level"] = 0
    
    solver = nlpsol('solver', 'ipopt', prob, opts);
    
    
    # Solve the NLP
    sol = solver(x0=w0, lbx=lbw, ubx=ubw, lbg=lbg, ubg=ubg)

    w_opt = sol['x']
    # Predict the next state by model
    Fn = F(x0=X_state, xd=sd_, p=w_opt[4:8], tv=DM(vertcat(tar_vel_cam,0)))
    Xm = Fn['xf']
    
    # Applies the first control element of u, and update the states    
    # Calculate the command velocity in Global Frame
    uv_global = mul(rot_c2g, w_opt[4:7])
    uw_global = mul(rot_c2g, DM([0,w_opt[7],0]))
    
    
    cmd_vel.twist.linear.x = uv_global[0]
    cmd_vel.twist.linear.y = uv_global[1]
    cmd_vel.twist.linear.z = uv_global[2]
    cmd_vel.twist.angular.z = uw_global[2]

    print(cmd_vel)
    if optimal_ibvs:
        print(optimal_ibvs)
        pub.publish(cmd_vel)
        
    rate.sleep()
cv2.destroyAllWindows()    
