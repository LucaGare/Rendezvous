/* 
Operates the UGV. 
Runs an MPC algorithm and sends velocity setpoint to the FCU.
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
#include <geometry_msgs/Twist.h>

struct state_vector {
    double x,y,vx,vy;
};

state_vector stateVector_from_subs(geometry_msgs::PoseStamped p, geometry_msgs::TwistStamped v);

geometry_msgs::PoseStamped  current_pose;
geometry_msgs::TwistStamped current_velocity;

void UGVpose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_pose = *msg;
}
void UGVvelocity_cb(const geometry_msgs::TwistStamped::ConstPtr& msg){
    current_velocity = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber UGVpose = nh.subscribe<geometry_msgs::PoseStamped>
            ("/qualisys/nexus1/pose", 100, UGVpose_cb);
    ros::Subscriber UGVvelocity = nh.subscribe<geometry_msgs::TwistStamped>
            ("/qualisys/nexus1/velocity", 100, UGVvelocity_cb); 
    ros::Publisher UGVsetpoint_pub = nh.advertise<geometry_msgs::Twist>
            ("nexus1/cmd_vel", 100);

    // setpoint publishing rate
    ros::Rate rate(40.0);

    // Initialize things for the MPC algorithm
    std::string const package_path = ros::package::getPath("PX4Vision_AutonomousLanding");
    std::string const library_path = package_path + "/src/SharedLibs/MPC_UGV.so";
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
    signal_t incref = (signal_t)dlsym(handle, "MPC_UGV_incref");
    if(dlerror()) dlerror(); // No such function, reset error flags
    /* Memory management -- decrease reference counter */
    signal_t decref = (signal_t)dlsym(handle, "MPC_UGV_decref");
    if(dlerror()) dlerror(); // No such function, reset error flags
    /* Thread-local memory management -- checkout memory */
    casadi_checkout_t checkout = (casadi_checkout_t)dlsym(handle, "MPC_UGV_checkout");
    if(dlerror()) dlerror(); // No such function, reset error flags
    /* Thread-local memory management -- release memory */
    casadi_release_t release = (casadi_release_t)dlsym(handle, "MPC_UGV_release");
    if(dlerror()) dlerror(); // No such function, reset error flags
    /* Number of inputs */
    getint_t n_in_fcn = (getint_t)dlsym(handle, "MPC_UGV_n_in");
    if (dlerror()) return 1;
    casadi_int n_in = n_in_fcn();
    /* Number of outputs */
    getint_t n_out_fcn = (getint_t)dlsym(handle, "MPC_UGV_n_out");
    if (dlerror()) return 1;
    casadi_int n_out = n_out_fcn();

    /* Get sizes of the required work vectors */
    casadi_int sz_arg=n_in, sz_res=n_out, sz_iw=0, sz_w=0;
    work_t work = (work_t)dlsym(handle, "MPC_UGV_work");
    if(dlerror()) dlerror(); // No such function, reset error flags
    if (work && work(&sz_arg, &sz_res, &sz_iw, &sz_w)) return 1;

    /* Input sparsities */
    sparsity_t sparsity_in = (sparsity_t)dlsym(handle, "MPC_UGV_sparsity_in");
    if (dlerror()) return 1;
    /* Output sparsities */
    sparsity_t sparsity_out = (sparsity_t)dlsym(handle, "MPC_UGV_sparsity_out");
    if (dlerror()) return 1;

    /* Function for numerical evaluation */
    eval_t eval = (eval_t)dlsym(handle, "MPC_UGV");

    /* Allocate input/output buffers and work vectors*/
    const double *arg[sz_arg];
    double *res[sz_res];
    casadi_int iw[sz_iw];
    double w[sz_w];

    // First instance of the MPC
    /* Function input and output */
    state_vector current_state_vector = stateVector_from_subs(current_pose, current_velocity);
    const double x0[] = {current_state_vector.x, current_state_vector.y,
                        current_state_vector.vx, current_state_vector.vy};
    const double DesiredFinalPosition[] = {0., 0.};
    double ControlAction[2];

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

    geometry_msgs::Twist setpoint;
    setpoint.linear.x = ControlAction[0];
    setpoint.linear.y = ControlAction[1];

    /* Free memory (thread-safe) */
    decref(); 

    UGVsetpoint_pub.publish(setpoint);

    while(ros::ok()){

        // Build the current state vector
        current_state_vector = stateVector_from_subs(current_pose, current_velocity);
        const double x0[] = {current_state_vector.x, current_state_vector.y,
                            current_state_vector.vx, current_state_vector.vy};

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

        setpoint.linear.x = ControlAction[0];
        setpoint.linear.y = ControlAction[1];

        /* Free memory (thread-safe) */
        decref();

        UGVsetpoint_pub.publish(setpoint);

        ros::spinOnce();
        rate.sleep();
    }

    /* Free the handle */
    dlclose(handle);

    return 0;
}

state_vector stateVector_from_subs(geometry_msgs::PoseStamped p, geometry_msgs::TwistStamped v){
    state_vector state;

    state.x = p.pose.position.x;
    state.y = p.pose.position.y;
    state.vx = v.twist.linear.x;
    state.vy = v.twist.linear.y;

    return state;
};