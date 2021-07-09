/* 
Operates the UAV in offboard mode. 
Runs an MPC algorithm, sending attitude setpoint to the FCU.
*/

#include <ros/ros.h>
#include <ros/package.h>

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <dlfcn.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>

struct eulerAngles {
    double roll,pitch,yaw;
};
struct state_vector {
    geometry_msgs::Point    position;
    geometry_msgs::Vector3  linearVelocity;
    eulerAngles             orientation;
};

eulerAngles euler_from_quat(geometry_msgs::Quaternion q);
state_vector stateVector_from_subs(geometry_msgs::PoseStamped p, geometry_msgs::TwistStamped v);
geometry_msgs::Quaternion quat_from_euler(eulerAngles ea);
double thrust_from_verticalVelocityCommand(double vz_dot_cmd, state_vector x);

mavros_msgs::State          current_state;
geometry_msgs::PoseStamped  current_pose;
geometry_msgs::TwistStamped current_velocity;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_pose = *msg;
}
void velocity_cb(const geometry_msgs::TwistStamped::ConstPtr& msg){
    current_velocity = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("px4_quad/mavros/state", 10, state_cb);
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("px4_quad/mavros/set_mode");
    ros::Subscriber pose = nh.subscribe<geometry_msgs::PoseStamped>
            ("/qualisys/px4_quad/pose", 100, pose_cb);
    ros::Subscriber velocity = nh.subscribe<geometry_msgs::TwistStamped>
            ("/qualisys/px4_quad/velocity", 100, velocity_cb); 
    ros::Publisher setpoint_pub = nh.advertise<mavros_msgs::AttitudeTarget>
            ("px4_quad/mavros/setpoint_raw/attitude", 100);

    // NOTE: the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(40.0);

    // Wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
    std::cout<< "FCU connected." << std::endl;

    // Initialize things for the MPC algorithm
    std::string const package_path = ros::package::getPath("PX4Vision_AutonomousLanding");
    std::string const library_path = package_path + "/src/SharedLibs/MPC.so";
    /* Handle to the dll */
    void* handle;
    /* Load the dll */
    handle = dlopen(library_path.c_str(), RTLD_LAZY);
    if (!handle) {
        std::cerr << "Cannot open library: " << dlerror() << '\n';
        return 1;
    }
                    
    typedef long long int casadi_int;

    /* Typedefs */
    typedef void (*signal_t)(void);
    typedef casadi_int (*getint_t)(void);
    typedef int (*work_t)(casadi_int* sz_arg, casadi_int* sz_res, casadi_int* sz_iw, casadi_int* sz_w);
    typedef const casadi_int* (*sparsity_t)(casadi_int ind);
    typedef int (*eval_t)(const double** arg, double** res, casadi_int* iw, double* w, int mem);
    typedef int (*casadi_checkout_t)(void);
    typedef void (*casadi_release_t)(int);

    /* Memory management -- increase reference counter */
    signal_t incref = (signal_t)dlsym(handle, "MPC_incref");
    if(dlerror()) dlerror(); // No such function, reset error flags
    /* Memory management -- decrease reference counter */
    signal_t decref = (signal_t)dlsym(handle, "MPC_decref");
    if(dlerror()) dlerror(); // No such function, reset error flags
    /* Thread-local memory management -- checkout memory */
    casadi_checkout_t checkout = (casadi_checkout_t)dlsym(handle, "MPC_checkout");
    if(dlerror()) dlerror(); // No such function, reset error flags
    /* Thread-local memory management -- release memory */
    casadi_release_t release = (casadi_release_t)dlsym(handle, "MPC_release");
    if(dlerror()) dlerror(); // No such function, reset error flags
    /* Number of inputs */
    getint_t n_in_fcn = (getint_t)dlsym(handle, "MPC_n_in");
    if (dlerror()) return 1;
    casadi_int n_in = n_in_fcn();
    /* Number of outputs */
    getint_t n_out_fcn = (getint_t)dlsym(handle, "MPC_n_out");
    if (dlerror()) return 1;
    casadi_int n_out = n_out_fcn();

    /* Get sizes of the required work vectors */
    casadi_int sz_arg=n_in, sz_res=n_out, sz_iw=0, sz_w=0;
    work_t work = (work_t)dlsym(handle, "MPC_work");
    if(dlerror()) dlerror(); // No such function, reset error flags
    if (work && work(&sz_arg, &sz_res, &sz_iw, &sz_w)) return 1;

    /* Input sparsities */
    sparsity_t sparsity_in = (sparsity_t)dlsym(handle, "MPC_sparsity_in");
    if (dlerror()) return 1;
    /* Output sparsities */
    sparsity_t sparsity_out = (sparsity_t)dlsym(handle, "MPC_sparsity_out");
    if (dlerror()) return 1;

    /* Function for numerical evaluation */
    eval_t eval = (eval_t)dlsym(handle, "MPC");

    /* Allocate input/output buffers and work vectors*/
    const double *arg[sz_arg];
    double *res[sz_res];
    casadi_int iw[sz_iw];
    double w[sz_w];

    // First instance of the MPC
    /* Function input and output */
    state_vector current_state_vector = stateVector_from_subs(current_pose, current_velocity);
    const double x0[] = {current_state_vector.position.x, current_state_vector.position.y,
                                current_state_vector.position.z, current_state_vector.linearVelocity.x, 
                                current_state_vector.linearVelocity.y, current_state_vector.linearVelocity.z,
                                current_state_vector.orientation.roll, current_state_vector.orientation.pitch, 
                                current_state_vector.orientation.yaw};
    const double DesiredFinalPosition[] = {0.9, -1.2, 0.14};
    double ControlAction[4];

    // Allocate memory (thread-safe)
    incref();

    /* Evaluate the function */
    arg[0] = x0;
    arg[1] = DesiredFinalPosition;
    res[0] = ControlAction;

    // Checkout thread-local memory (not thread-safe)
    // Note MAX_NUM_THREADS
    int mem = checkout();

    // Evaluation is thread-safe
    if (eval(arg, res, iw, w, mem)) return 1;

    // Release thread-local (not thread-safe)
    release(mem);

    eulerAngles ea_cmd;
    ea_cmd.roll  = ControlAction[1];
    ea_cmd.pitch = ControlAction[2];
    ea_cmd.yaw   = ControlAction[3];
    geometry_msgs::Quaternion q_cmd;
    q_cmd = quat_from_euler(ea_cmd);
    mavros_msgs::AttitudeTarget setpoint;
    // setpoint.header.frame_id = "map";
    setpoint.type_mask       = 7; // ignore body_rates
    setpoint.orientation     = q_cmd;
    setpoint.thrust          = thrust_from_verticalVelocityCommand(ControlAction[0],current_state_vector);

    /* Free memory (thread-safe) */
    decref();

    // Start streaming the setpoints before starting
    static unsigned int sequenceNumber = 1;
    setpoint.header.stamp = ros::Time::now();
    setpoint.header.seq = sequenceNumber;
    setpoint_pub.publish(setpoint);

    // Switch to offboard node and arm the UAV
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    ros::Time last_request = ros::Time::now();
    bool stop = false;

    while(ros::ok() && current_state.armed){
        if( current_state.mode != "OFFBOARD" && sequenceNumber > 20 && !stop &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        }

        // Build the current state vector
        current_state_vector = stateVector_from_subs(current_pose, current_velocity);
        const double x0[] = {current_state_vector.position.x, current_state_vector.position.y,
                            current_state_vector.position.z, current_state_vector.linearVelocity.x, 
                            current_state_vector.linearVelocity.y, current_state_vector.linearVelocity.z,
                            current_state_vector.orientation.roll, current_state_vector.orientation.pitch, 
                            current_state_vector.orientation.yaw};

        // Allocate memory (thread-safe)
        incref();

        /* Evaluate the function */
        arg[0] = x0;
        arg[1] = DesiredFinalPosition;
        res[0] = ControlAction;

        // Checkout thread-local memory (not thread-safe)
        // Note MAX_NUM_THREADS
        int mem = checkout();

        // Evaluation is thread-safe
        if (eval(arg, res, iw, w, mem)) return 1;

        // Release thread-local (not thread-safe)
        release(mem);

        ea_cmd.roll  = ControlAction[1];
        ea_cmd.pitch = ControlAction[2];
        ea_cmd.yaw   = ControlAction[3];
        q_cmd = quat_from_euler(ea_cmd);
        setpoint.orientation = q_cmd;
        setpoint.thrust      = thrust_from_verticalVelocityCommand(ControlAction[0],current_state_vector);

        /* Free memory (thread-safe) */
        decref();

        // LANDING //
        // Stopping condition
        if( !stop && abs(current_pose.pose.position.z - DesiredFinalPosition[2]) < 0.1 ){
            if(abs(current_pose.pose.position.x - DesiredFinalPosition[0]) < 0.05 && abs(current_pose.pose.position.y - DesiredFinalPosition[1]) < 0.05){
                stop = true;
            }
        }
        
        // If stopping condition is satisfied command no thrust
        if(stop){
            setpoint.thrust = 0.1;
        }
        
        sequenceNumber++;
        setpoint.header.stamp = ros::Time::now();
        setpoint.header.seq = sequenceNumber;
        setpoint_pub.publish(setpoint);

        ros::spinOnce();
        rate.sleep();
    }

    /* Free the handle */
    dlclose(handle);

    return 0;
}

eulerAngles euler_from_quat(geometry_msgs::Quaternion q){
    eulerAngles ea; // [rad]
    
    // roll
    double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z;
    ea.roll          = std::atan2(sinr_cosp, cosr_cosp);
    // pitch 
    double sinp  = 2 * (q.w * q.y - q.z * q.x);
    ea.pitch     = std::asin(sinp);
    // yaw
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z;
    ea.yaw           = std::atan2(siny_cosp, cosy_cosp);

    return ea;
}

state_vector stateVector_from_subs(geometry_msgs::PoseStamped p, geometry_msgs::TwistStamped v){
    state_vector x;
    geometry_msgs::Quaternion q;
    eulerAngles ea;

    q       = p.pose.orientation;
    ea      = euler_from_quat(q);

    x.position = p.pose.position;
    x.linearVelocity = v.twist.linear;
    x.orientation = ea;

    return x;
};

geometry_msgs::Quaternion quat_from_euler(eulerAngles ea){
    geometry_msgs::Quaternion q;

    double cr = cos(ea.roll * 0.5);
    double sr = sin(ea.roll * 0.5);
    double cp = cos(ea.pitch * 0.5);
    double sp = sin(ea.pitch * 0.5);
    double cy = cos(ea.yaw * 0.5);
    double sy = sin(ea.yaw * 0.5);

    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;

    double length = sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
    q.w = q.w/length;
    q.x = q.x/length;
    q.y = q.y/length;
    q.z = q.z/length;

    return q;
}

double thrust_from_verticalVelocityCommand(double vz_dot_cmd, state_vector x){
    double T;
    double g = 9.81; // acceleration of gravity [m/s^2]
    double hoovering_T = 0.363; // hoovering nondimensional thrust [-]
    //double hoovering_T = 0.332; // hoovering nondimensional thrust [-]
    double TMax  = g/hoovering_T;   // maximum "thrust" (acceleration at maximum thrust) [m/s^2]
    double cr = cos(x.orientation.roll);
    double cp = cos(x.orientation.pitch);

    T = ((g + vz_dot_cmd)/(cr*cp))/TMax;

    return T;
}