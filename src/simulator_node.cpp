/*
Simulates the UAV and UGV. 
Receives current inputs and publishes the current agents' pose and velocity.
*/

#include <ros/ros.h>
#include <ros/package.h>

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <random>
#include <dlfcn.h>

#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

struct eulerAngles {
    double roll,pitch,yaw;
};

geometry_msgs::Quaternion quat_from_euler(eulerAngles ea);
eulerAngles euler_from_quat(geometry_msgs::Quaternion q);
double verticalAcceleration_from_thrust(double thrust, std::vector<double> x);
void printVector(std::vector<double> x0);

mavros_msgs::AttitudeTarget current_UAVsetpoint;
void UAVsetpoint_cb(const mavros_msgs::AttitudeTarget::ConstPtr& msg){
    current_UAVsetpoint = *msg;
}

geometry_msgs::Twist current_UGVsetpoint;
void UGVsetpoint_cb(const geometry_msgs::Twist::ConstPtr& msg){
    current_UGVsetpoint = *msg;
}

int main(int argc, char **argv){

    // Initialize ROS node
    ros::init(argc, argv, "simulator_node");
    ros::NodeHandle nh;
    
    ros::Subscriber UAVsetpoint_sub = nh.subscribe<mavros_msgs::AttitudeTarget>
            ("/px4_quad/mavros/setpoint_raw/attitude", 10, UAVsetpoint_cb);
    ros::Subscriber UGVsetpoint_sub = nh.subscribe<geometry_msgs::Twist>
            ("/nexus1/cmd_vel", 10, UGVsetpoint_cb);
    ros::Publisher UAVstate_pub = nh.advertise<mavros_msgs::State>
            ("/px4_quad/mavros/state", 100);
    ros::Publisher UAVpose_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("/qualisys/px4_quad/pose", 100);
    ros::Publisher UAVvelocity_pub = nh.advertise<geometry_msgs::TwistStamped>
            ("/qualisys/px4_quad/velocity", 100);
    ros::Publisher initial_UAVpose_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("/px4_quad/initial_pose", 100);
    ros::Publisher UGVpose_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("/qualisys/nexus1/pose", 100);                 
    ros::Publisher UGVvelocity_pub = nh.advertise<geometry_msgs::TwistStamped>
            ("/qualisys/nexus1/velocity", 100);             
    ros::Publisher initial_UGVpose_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("/nexus1/initial_pose", 100);

    // Publication rate
    ros::Rate rate(100.0);

    // Set initial state vectors x0
    std::vector<double> x0_UAV = {1, -1.3, 1, 0, 0, 0, .1, .1, .3};
    std::vector<double> x0_UGV = {-1., 0.9, 0., 0.};
    eulerAngles current_ea;
    current_ea.roll  = x0_UAV[6];
    current_ea.pitch = x0_UAV[7];
    current_ea.yaw   = x0_UAV[8];

    // Define random generator with Gaussian distribution
    const double mean = 0.0;
    const double stddev = 0.01;
    std::default_random_engine generator;
    std::normal_distribution<double> dist(mean, stddev);

    // Initialize published messages
    /* UGV pose, velocity and initial pose */
    geometry_msgs::PoseStamped current_UGVpose;
    current_UGVpose.header.frame_id  = "map";
    current_UGVpose.pose.position.x  = x0_UGV[0];
    current_UGVpose.pose.position.y  = x0_UGV[1];
    current_UGVpose.pose.position.z  = 0.;         // (TODO: update with the height of the platform)
    current_UGVpose.pose.orientation.w = 1;
    geometry_msgs::TwistStamped current_UGVvelocity;   
    current_UGVvelocity.header.frame_id = "map";
    current_UGVvelocity.twist.linear.x  = x0_UGV[2];
    current_UGVvelocity.twist.linear.y  = x0_UGV[3];

    geometry_msgs::PoseStamped initial_UGVpose;
    initial_UGVpose.header.frame_id  = "map";
    initial_UGVpose.header.stamp     = ros::Time::now();
    initial_UGVpose.pose.position.x  = x0_UGV[0];
    initial_UGVpose.pose.position.y  = x0_UGV[1];
    initial_UGVpose.pose.position.z  = 0.;         // (TODO: update with the height of the platform)
    initial_UGVpose.pose.orientation.w = 1;

    /* UAV state, pose, velocity and initial pose */
    mavros_msgs::State current_UAVstate;
    current_UAVstate.connected = 1;
    current_UAVstate.armed     = 1;
    current_UAVstate.mode      = "OFFBOARD";
    geometry_msgs::PoseStamped current_UAVpose;
    current_UAVpose.header.frame_id  = "map";
    current_UAVpose.pose.position.x  = x0_UAV[0];
    current_UAVpose.pose.position.y  = x0_UAV[1];
    current_UAVpose.pose.position.z  = x0_UAV[2];
    current_UAVpose.pose.orientation = quat_from_euler(current_ea);
    geometry_msgs::TwistStamped current_UAVvelocity;
    current_UAVvelocity.header.frame_id = "map";
    current_UAVvelocity.twist.linear.x  = x0_UAV[3];
    current_UAVvelocity.twist.linear.y  = x0_UAV[4];
    current_UAVvelocity.twist.linear.z  = x0_UAV[5];
    current_UAVsetpoint.type_mask = 1;

    geometry_msgs::PoseStamped initial_UAVpose;
    initial_UAVpose.header.frame_id  = "map";
    initial_UAVpose.pose.position.x = x0_UAV[0];
    initial_UAVpose.pose.position.y  = x0_UAV[1];
    initial_UAVpose.pose.position.z  = x0_UAV[2];
    initial_UAVpose.pose.orientation = quat_from_euler(current_ea);

    // Initialize things for the integrator libraries
    std::string const package_path = ros::package::getPath("Rendezvous");
    std::string const UAVlibrary_path = package_path + "/src/SharedLibs/rk4.so";
    std::string const UGVlibrary_path = package_path + "/src/SharedLibs/rk4_UGV.so";
    // Handle to the dlls
    void* UAVhandle;
    void* UGVhandle;
    // Load the dlls
    UAVhandle = dlopen(UAVlibrary_path.c_str(), RTLD_LAZY);
    if (!UAVhandle) {
        std::cerr << "Cannot open library: " << dlerror() << '\n';
        return 1;
    }
    UGVhandle = dlopen(UGVlibrary_path.c_str(), RTLD_LAZY);
    if (!UGVhandle) {
        std::cerr << "Cannot open library: " << dlerror() << '\n';
        return 1;
    }

    typedef long long int casadi_int;

    // Typedefs
    typedef void (*signal_t)(void);
    typedef casadi_int (*getint_t)(void);
    typedef int (*work_t)(casadi_int* sz_arg, casadi_int* sz_res, casadi_int* sz_iw, casadi_int* sz_w);
    typedef const casadi_int* (*sparsity_t)(casadi_int ind);
    typedef int (*eval_t)(const double** arg, double** res, casadi_int* iw, double* w, int mem);
    typedef int (*casadi_checkout_t)(void);
    typedef void (*casadi_release_t)(int);

    // Memory management -- increase reference counter
    signal_t UGVincref = (signal_t)dlsym(UGVhandle, "rk4_UGV_incref");
    signal_t UAVincref = (signal_t)dlsym(UAVhandle, "rk4_incref");
    if(dlerror()) dlerror(); // No such function, reset error flags
    // Memory management -- decrease reference counter
    signal_t UGVdecref = (signal_t)dlsym(UGVhandle, "rk4_UGV_decref");
    signal_t UAVdecref = (signal_t)dlsym(UAVhandle, "rk4_decref");
    if(dlerror()) dlerror(); // No such function, reset error flags
    // Thread-local memory management -- checkout memory
    casadi_checkout_t UGVcheckout = (casadi_checkout_t)dlsym(UGVhandle, "rk4_UGV_checkout");
    casadi_checkout_t UAVcheckout = (casadi_checkout_t)dlsym(UAVhandle, "rk4_checkout");
    if(dlerror()) dlerror(); // No such function, reset error flags
    // Thread-local memory management -- release memory
    casadi_release_t UGVrelease = (casadi_release_t)dlsym(UGVhandle, "rk4_UGV_release");
    casadi_release_t UAVrelease = (casadi_release_t)dlsym(UAVhandle, "rk4_release");
    if(dlerror()) dlerror(); // No such function, reset error flags
    // Number of inputs
    getint_t UGVn_in_fcn = (getint_t)dlsym(UGVhandle, "rk4_UGV_n_in");
    if (dlerror()) return 1;
    casadi_int UGVn_in = UGVn_in_fcn();
    getint_t UAVn_in_fcn = (getint_t)dlsym(UAVhandle, "rk4_n_in");
    if (dlerror()) return 1;
    casadi_int UAVn_in = UAVn_in_fcn();
    // Number of outputs
    getint_t UGVn_out_fcn = (getint_t)dlsym(UGVhandle, "rk4_UGV_n_out");
    if (dlerror()) return 1;
    casadi_int UGVn_out = UGVn_out_fcn();
    getint_t UAVn_out_fcn = (getint_t)dlsym(UAVhandle, "rk4_n_out");
    if (dlerror()) return 1;
    casadi_int UAVn_out = UAVn_out_fcn();

    // Get sizes of the required work vectors
    casadi_int UGVsz_arg=UGVn_in, UGVsz_res=UGVn_out, UGVsz_iw=0, UGVsz_w=0;
    work_t UGVwork = (work_t)dlsym(UGVhandle, "rk4_UGV_work");
    if(dlerror()) dlerror(); // No such function, reset error flags
    if (UGVwork && UGVwork(&UGVsz_arg, &UGVsz_res, &UGVsz_iw, &UGVsz_w)) return 1;

    casadi_int UAVsz_arg=UAVn_in, UAVsz_res=UAVn_out, UAVsz_iw=0, UAVsz_w=0;
    work_t UAVwork = (work_t)dlsym(UAVhandle, "rk4_work");
    if(dlerror()) dlerror(); // No such function, reset error flags
    if (UAVwork && UAVwork(&UAVsz_arg, &UAVsz_res, &UAVsz_iw, &UAVsz_w)) return 1;

    // Input sparsities
    sparsity_t UGVsparsity_in = (sparsity_t)dlsym(UGVhandle, "rk4_UGV_sparsity_in");
    sparsity_t UAVsparsity_in = (sparsity_t)dlsym(UAVhandle, "rk4_sparsity_in");
    if (dlerror()) return 1;
    // Output sparsities
    sparsity_t UGVsparsity_out = (sparsity_t)dlsym(UGVhandle, "rk4_UGV_sparsity_out");
    sparsity_t UAVsparsity_out = (sparsity_t)dlsym(UAVhandle, "rk4_sparsity_out");
    if (dlerror()) return 1;

    // Function for numerical evaluation
    eval_t UGVeval = (eval_t)dlsym(UGVhandle, "rk4_UGV");
    eval_t UAVeval = (eval_t)dlsym(UAVhandle, "rk4");

    // Allocate input/output buffers and work vectors
    const double *UGVarg[UGVsz_arg];
    double *UGVres[UGVsz_res];
    casadi_int UGViw[UGVsz_iw];
    double UGVw[UGVsz_w];
    const double *UAVarg[UAVsz_arg];
    double *UAVres[UAVsz_res];
    casadi_int UAViw[UAVsz_iw];
    double UAVw[UAVsz_w];

    bool firstTime = true;
    ros::Time beginning;

    while(ros::ok() && current_UAVstate.armed){
        if(current_UAVsetpoint.type_mask != 7){
            current_UAVstate.header.stamp    = ros::Time::now();
            current_UAVpose.header.stamp     = ros::Time::now();
            current_UAVvelocity.header.stamp = ros::Time::now();

            UAVstate_pub.publish(current_UAVstate);
            UAVpose_pub.publish(current_UAVpose);
            UAVvelocity_pub.publish(current_UAVvelocity);

            current_UGVpose.header.stamp     = ros::Time::now();
            current_UGVvelocity.header.stamp = ros::Time::now();

            UGVpose_pub.publish(current_UGVpose);
            UGVvelocity_pub.publish(current_UGVvelocity);
        }else if(current_UAVsetpoint.type_mask == 7 && current_UAVsetpoint.header.seq < 10){
            current_UAVstate.header.stamp    = ros::Time::now();
            current_UAVpose.header.stamp     = ros::Time::now();
            current_UAVvelocity.header.stamp = ros::Time::now();

            UAVstate_pub.publish(current_UAVstate);
            UAVpose_pub.publish(current_UAVpose);
            UAVvelocity_pub.publish(current_UAVvelocity);

            current_UGVpose.header.stamp     = ros::Time::now();
            current_UGVvelocity.header.stamp = ros::Time::now();

            UGVpose_pub.publish(current_UGVpose);
            UGVvelocity_pub.publish(current_UGVvelocity);
        }else{
            if (firstTime){
                beginning = ros::Time::now();
                firstTime = false;
            }
            // Integrators
            // Function input and output
            const double UGVxi[] = {x0_UGV[0], x0_UGV[1], x0_UGV[2], x0_UGV[3]};                  // UGV input 1 (current state)
            const double UGVu[]  = {current_UGVsetpoint.linear.x, current_UGVsetpoint.linear.y};  // UGV input 2 (current setpoint)
            double UGVxf[4];              // UGV MPC output (next state)

            double az_cmd = verticalAcceleration_from_thrust(current_UAVsetpoint.thrust, x0_UAV);
            eulerAngles ea_cmd = euler_from_quat(current_UAVsetpoint.orientation);

            const double UAVxi[] = {x0_UAV[0], x0_UAV[1], x0_UAV[2], x0_UAV[3], x0_UAV[4], x0_UAV[5], x0_UAV[6], x0_UAV[7], x0_UAV[8]}; // UAV input 1 (current state)
            const double UAVu[] = {az_cmd, ea_cmd.roll, ea_cmd.pitch, ea_cmd.yaw};  // UAV input 2 (current setpoint)
            double UAVxf[9];   // UAV MPC output (next state)

            const double DT[] = {1./100}; // MPCs input (timestep), needs to match 1/PublicationRate

            // Allocate memory (thread-safe)
            UAVincref();
            UGVincref();

            // Evaluate the functions
            UAVarg[0] = UAVxi;
            UAVarg[1] = UAVu;
            UAVarg[2] = DT; 
            UAVres[0] = UAVxf;

            UGVarg[0] = UGVxi;
            UGVarg[1] = UGVu;
            UGVarg[2] = DT; 
            UGVres[0] = UGVxf;

            // Checkout thread-local memory (not thread-safe)
            // Note MAX_NUM_THREADS
            int UAVmem = UAVcheckout();
            int UGVmem = UGVcheckout();

            // Evaluation is thread-safe
            if (UAVeval(UAVarg, UAVres, UAViw, UAVw, UAVmem)) return 1;
            if (UGVeval(UGVarg, UGVres, UGViw, UGVw, UGVmem)) return 1;

            // Release thread-local (not thread-safe)
            UAVrelease(UAVmem);
            UGVrelease(UGVmem);

            // New initial value for integrators with added Gaussian noise
            // mean = 0; std dev = 0.01 (0.005 for angles).
            x0_UGV[0] = UGVxf[0]; // + dist(generator);
            x0_UGV[1] = UGVxf[1]; // + dist(generator);
            x0_UGV[2] = UGVxf[2]; // + dist(generator);
            x0_UGV[3] = UGVxf[3]; // + dist(generator);

            x0_UAV[0] = UAVxf[0]; // + dist(generator);
            x0_UAV[1] = UAVxf[1]; // + dist(generator);
            x0_UAV[2] = UAVxf[2]; // + dist(generator);
            x0_UAV[3] = UAVxf[3]; // + dist(generator);
            x0_UAV[4] = UAVxf[4]; // + dist(generator);
            x0_UAV[5] = UAVxf[5]; // + dist(generator);
            x0_UAV[6] = UAVxf[6]; // + dist(generator)/2;
            x0_UAV[7] = UAVxf[7]; // + dist(generator)/2;
            x0_UAV[8] = UAVxf[8]; // + dist(generator)/2;

            if(x0_UAV[2] < 0){
                x0_UAV[2] = 0;
            }

            // Fill the messages to publish
            current_UAVpose.pose.position.x  = x0_UAV[0];
            current_UAVpose.pose.position.y  = x0_UAV[1];
            current_UAVpose.pose.position.z  = x0_UAV[2];
            current_ea.roll  = x0_UAV[6];
            current_ea.pitch = x0_UAV[7];
            current_ea.yaw   = x0_UAV[8];
            current_UAVpose.pose.orientation = quat_from_euler(current_ea);
            current_UAVvelocity.twist.linear.x  = x0_UAV[3];
            current_UAVvelocity.twist.linear.y  = x0_UAV[4];
            current_UAVvelocity.twist.linear.z  = x0_UAV[5];

            current_UAVstate.header.stamp    = ros::Time::now();
            current_UAVpose.header.stamp     = ros::Time::now();
            current_UAVvelocity.header.stamp = ros::Time::now();

            current_UGVpose.pose.position.x  = x0_UGV[0];
            current_UGVpose.pose.position.y  = x0_UGV[1];
            current_UGVvelocity.twist.linear.x  = x0_UGV[2];
            current_UGVvelocity.twist.linear.y  = x0_UGV[3];

            current_UGVpose.header.stamp     = ros::Time::now();
            current_UGVvelocity.header.stamp = ros::Time::now();

            // Publish the messages
            UAVstate_pub.publish(current_UAVstate);
            UAVpose_pub.publish(current_UAVpose);
            UAVvelocity_pub.publish(current_UAVvelocity);

            UGVpose_pub.publish(current_UGVpose);
            UGVvelocity_pub.publish(current_UGVvelocity);

            // Free memory (thread-safe)
            UAVdecref();
            UGVdecref();

            std::cout<< "Time from the start: " << ros::Time::now() - beginning << " s." << std::endl;
            // Print next state vectors
            std::cout << "current UAV state vector: ";
            printVector(x0_UAV);
            std::cout << "current UGV state vector: ";
            printVector(x0_UGV);
        }

        initial_UAVpose_pub.publish(initial_UAVpose);
        initial_UGVpose_pub.publish(initial_UGVpose);
        ros::spinOnce();
        rate.sleep();
   
    }

    ros::Time ending = ros::Time::now();
    std::cout << "Time for the maneuver: " << ending - beginning << " s" << std::endl;

    // Free the handles
    dlclose(UAVhandle);
    dlclose(UGVhandle);

    return 0;

}

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

double verticalAcceleration_from_thrust(double T, std::vector<double> x){
    double az_cmd;

    double g = 9.81; // acceleration of gravity [m/s^2]
    double hoovering_T = 0.363; // hoovering nondimensional thrust [-]
    //double hoovering_T = 0.332; // hoovering nondimensional thrust [-]
    double TMax  = g/hoovering_T;   // maximum "thrust" (acceleration at maximum thrust) [m/s^2]

    az_cmd = T*TMax*cos(x[6])*cos(x[7]) - g;

    return az_cmd;
}

void printVector(std::vector<double> x0){
    for(int j=0; j<x0.size(); ++j)
        std::cout << x0[j] << " ";
    std::cout << std::endl;
}