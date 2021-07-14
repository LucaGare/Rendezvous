/* 
Computes the rendezvous point based on UAV and UGV's predicted trajectory.
*/

#include <ros/ros.h>
#include <ros/package.h>

#include <cmath>
#include <vector>
#include <iostream>

#include <mavros_msgs/State.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <Rendezvous/Trajectory.h>


int N = 20; // control horizon

mavros_msgs::State          current_state;
geometry_msgs::PoseStamped  initial_UAVpose, initial_UGVpose;
Rendezvous::Trajectory      UAV_predicted;
Rendezvous::Trajectory      UGV_predicted;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
void UAVpose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    initial_UAVpose = *msg;
}
void UGVpose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    initial_UGVpose = *msg;
}
void UAVtrajectory_cb(const Rendezvous::Trajectory::ConstPtr& msg){
    UAV_predicted = *msg;
}
void UGVtrajectory_cb(const Rendezvous::Trajectory::ConstPtr& msg){
    UGV_predicted = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    // subscriber to UAV state
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("/px4_quad/mavros/state", 10, state_cb);
    // subscribers to UAV and UGV pose to initialize the rendezvous setpoint
    ros::Subscriber UAVpose = nh.subscribe<geometry_msgs::PoseStamped>
            ("/qualisys/px4_quad/pose", 100, UAVpose_cb);
    ros::Subscriber UGVpose = nh.subscribe<geometry_msgs::PoseStamped>
            ("/qualisys/nexus5/pose", 100, UGVpose_cb);
    // subscribers to UAV and UGV's predicted trajectories
    ros::Subscriber UAVtrajectory = nh.subscribe<Rendezvous::Trajectory>
            ("/px4_quad/trajectory", 100, UAVtrajectory_cb);
    ros::Subscriber UGVtrajectory = nh.subscribe<Rendezvous::Trajectory>
            ("/nexus5/trajectory", 100, UGVtrajectory_cb);
    // publisher of the rendezvous point
    ros::Publisher rendezvous_point_pub = nh.advertise<geometry_msgs::Point>
            ("/rendezvous_point", 100);

    // rendezvous point publishing rate
    ros::Rate rate(40.0);

    // initialize useful variables
    int j = 0;
    bool initialized = false;
    geometry_msgs::Point rendezvous_point;
    rendezvous_point.z = 0.2; // z position of the platform (constant) --> to be updated with the height of the platform
    bool UAV_arrived, UGV_arrived;
    int UAV_arrival_time, UGV_arrival_time;

    while(ros::ok()){
        if( !initialized ){
            // initialize rendezvous point
            rendezvous_point.x = (initial_UAVpose.pose.position.x + initial_UGVpose.pose.position.x)/2;
            rendezvous_point.y = (initial_UAVpose.pose.position.y + initial_UGVpose.pose.position.y)/2;

            if (current_state.mode == "OFFBOARD"){
                if(j<5){
                    ros::spinOnce();
                    rate.sleep();

                    j++;
                }else{
                    initialized = true;
                    std::cout<< "Initialized! " << std::endl;
                }
            }
        }else{
            UAV_arrived = false;
            UGV_arrived = false;
            for (int i=0; i<N+1; ++i){
                // check when the UAV predicts to reach the setpoint
                if( !UAV_arrived && (UAV_predicted.data[i].z - rendezvous_point.z) < 0.15){
                    if(abs(UAV_predicted.data[i].x - rendezvous_point.x) < 0.05 && abs(UAV_predicted.data[i].y - rendezvous_point.y) < 0.05){
                       UAV_arrival_time = i;
                       UAV_arrived = true;
                       std::cout<< "UAV arrival time: " << UAV_arrival_time << std::endl;
                    }
                }
                // check when the UGV predicts to reach the setpoint
                if( !UGV_arrived ){
                    if(abs(UGV_predicted.data[i].x - rendezvous_point.x) < 0.05 && abs(UGV_predicted.data[i].y - rendezvous_point.y) < 0.05){
                        UGV_arrival_time = i;
                        UGV_arrived = true;
                        std::cout<< "UGV arrival time: " << UGV_arrival_time << std::endl;
                    }
                }
                // exit the loop if both UAV and UGV has reached the setpoint at time i
                if( UAV_arrived && UGV_arrived){
                    i = 2*N;
                }
            }
            /* rendezvous point update */
            if ( !UAV_arrived && !UGV_arrived){
                // CASE 1: neither the UAV nor the UGV reached the setpoint within the control horizon
                rendezvous_point.x = (UAV_predicted.data[N].x + UGV_predicted.data[N].x)/2;
                rendezvous_point.y = (UAV_predicted.data[N].y + UGV_predicted.data[N].y)/2;
            }else if( UAV_arrived && UAV_arrival_time < (UGV_arrival_time - 3)){
                // CASE 2: UAV reaches setpoint within control horizon and before UGV by at least 3 control steps (0.6s)
                rendezvous_point.x = (UAV_predicted.data[UAV_arrival_time].x + UGV_predicted.data[UAV_arrival_time].x)/2;
                rendezvous_point.y = (UAV_predicted.data[UAV_arrival_time].y + UGV_predicted.data[UAV_arrival_time].y)/2;
            }else if( UGV_arrived && UGV_arrival_time < (UAV_arrival_time - 3)){
                // CASE 3: UGV reaches setpoint within control horizon and before UAV by at least 3 control steps (0.6s)
                rendezvous_point.x = (UAV_predicted.data[UGV_arrival_time].x + UGV_predicted.data[UGV_arrival_time].x)/2;
                rendezvous_point.y = (UAV_predicted.data[UGV_arrival_time].y + UGV_predicted.data[UGV_arrival_time].y)/2;
            }
            // CASE 4: both UAV and UGV reached setpoint, the second reached the setpoint within 3 control steps (0.6s) 
            // after the first: the rendezvous point does not need to be updated.
        }

        rendezvous_point_pub.publish(rendezvous_point);

        ros::spinOnce();
        rate.sleep();
    }
    
    return 0;
}