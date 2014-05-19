//// ros headers
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <moveit_msgs/GetMotionPlan.h>

// local headers
#include <pacman/PaCMan/ROS.h>

// system headers
#include <time.h>
#include <pthread.h>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <semaphore.h>

//// generated headers
#include "definitions/MoveHead.h"
#include "definitions/TrajectoryPlanning.h"
#include "definitions/TrajectoryExecution.h"

#define T_STEP_HEAD_MICROSEC 9000000

namespace head_planner {

void* moveHeadThread(void* head);

// template for the full trajectory planner node
class HeadPlanner
{
  private:

    // the node handle
    ros::NodeHandle nh_;
    
    // Node handle in the private namespace
    ros::NodeHandle priv_nh_;

    // services
    ros::ServiceServer srv_head_planning_;
    ros::ServiceClient srv_head_execution_;
    definitions::TrajectoryExecution trajectory_srv_;

    // joint state topic
    std::string joint_state_topic_;

    // thread for think motion
    pthread_t thread_;
    boost::posix_time::ptime t_;
    boost::posix_time::time_duration thread_dt_;
    sem_t semaphore_;
    bool think_flag_;

  public:

  	// the service callback 
    bool moveHead(definitions::MoveHead::Request &request, definitions::MoveHead::Response &response);

    // constructor
    HeadPlanner(ros::NodeHandle nh) : nh_(nh), priv_nh_("~")
    {
       // advertise the main service
       srv_head_planning_ = nh_.advertiseService(nh_.resolveName("/head_planning_srv"),&HeadPlanner::moveHead, this);

       std::string trajectory_execution_service_name = nh_.resolveName("/trajectory_execution_srv");
       srv_head_execution_ = nh_.serviceClient<definitions::TrajectoryExecution>(trajectory_execution_service_name);

       joint_state_topic_ = nh.resolveName("/joint_states");

       trajectory_srv_.request.nonblocking = true;

       // set interval time for head thinking thread
       thread_dt_ = boost::posix_time::microseconds(T_STEP_HEAD_MICROSEC);
       // initialize semaphore (blocking)
       sem_init(&semaphore_,0,0);
       // create thread
       pthread_attr_t attr;
       pthread_attr_init(&attr);
       pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
       think_flag_ = false;

       std::cout << "HeadPlanner node: creating \"thinking\" thread" << std::endl;
       int rc = pthread_create(&thread_, &attr, moveHeadThread, this);
       if (rc) {
           ROS_ERROR("ERROR; return code from pthread_create() is %d\n", rc);
           exit(-1);
       }
    }

    //! Empty stub
    ~HeadPlanner() {}

    friend void* moveHeadThread(void* head);

};


bool HeadPlanner::moveHead(definitions::MoveHead::Request &request, definitions::MoveHead::Response &response)
{
    // wait for a joint state to read the current state
    boost::shared_ptr<const sensor_msgs::JointState> current_state_ptr = ros::topic::waitForMessage<sensor_msgs::JointState>(joint_state_topic_, ros::Duration(3.0));
    if (!current_state_ptr)
    {
        ROS_ERROR("No real start state available, since no joint state recevied in topic: %s", joint_state_topic_.c_str());
        ROS_ERROR("Did you forget to start a controller?");
        //response.result = response.OTHER_ERROR;
        return false;
        // ROS_ERROR("Planning will be done from home position, however this trajectory might not be good for execution!");
    }
    sensor_msgs::JointState startJointState = *current_state_ptr;

    definitions::Trajectory head_trajectory;

    if (request.type == request.THINK && !think_flag_)
    {
        head_trajectory.eddie_path.clear();
        head_trajectory.time_from_previous.clear();
        head_trajectory.eddie_path.resize(4);
        head_trajectory.time_from_previous.resize(4);

        for (int i = 0; i < head_trajectory.eddie_path.size(); i++)
        {
            pacman::convertJointStateToEddie( startJointState, head_trajectory.eddie_path.at(i) );
            head_trajectory.eddie_path.at(i).head.joints.at(0) = 0;
            head_trajectory.eddie_path.at(i).head.joints.at(1) = 0;
            head_trajectory.eddie_path.at(i).head.joints.at(2) = 0;
            head_trajectory.eddie_path.at(i).head.joints.at(3) = 0;
            head_trajectory.eddie_path.at(i).head.joints.at(4) = 0;
            head_trajectory.eddie_path.at(i).head.jointsLEye = 0;
            head_trajectory.eddie_path.at(i).head.jointsREye = 0;
        }

        // pose #1 only time, the pose is the current pose
        head_trajectory.time_from_previous.at(0) = ros::Duration().fromSec(0.);

        // pose #2
        head_trajectory.eddie_path.at(1).head.joints.at(1) = 0.4;
        head_trajectory.time_from_previous.at(1) = ros::Duration().fromSec(2.0);

        // pose #2
        head_trajectory.eddie_path.at(2).head.joints.at(1) = -0.4;
        head_trajectory.time_from_previous.at(2) = ros::Duration().fromSec(4.0);

        // pose #3 only time, the pose is the initial from the current pose
        head_trajectory.time_from_previous.at(3) = ros::Duration().fromSec(4.0);

        trajectory_srv_.request.trajectory = head_trajectory;

        //thread variables set
        t_ = boost::posix_time::microsec_clock::universal_time();
        think_flag_ = true;
        sem_post( &semaphore_ );

    }
    else if(request.type == request.STOP_THINK)
    {
        //detach thread
        think_flag_ = false;
    }
    else if(request.type == request.LOOK_AT)
    {
        //detach thread
        think_flag_ = false;

        head_trajectory.eddie_path.clear();
        head_trajectory.time_from_previous.clear();
        head_trajectory.eddie_path.resize(1);
        head_trajectory.time_from_previous.resize(1);

        for (int i = 0; i < head_trajectory.eddie_path.size(); i++)
        {
            pacman::convertJointStateToEddie( startJointState, head_trajectory.eddie_path.at(i) );
        }

        // // // // to implement
        // choose timing
        // choose RPY of the head based on request.look_at (probably R = 0)

        // test pose
        head_trajectory.eddie_path.at(0).head.joints.at(0) = -1.0;
        head_trajectory.time_from_previous.at(0) = ros::Duration().fromSec(0.5);

        // service set-up and call
        trajectory_srv_.request.trajectory = head_trajectory;
        srv_head_execution_.call( trajectory_srv_ );
    }

	return true;
}

// function associated to thread
void* moveHeadThread(void* head)
{
    boost::posix_time::time_duration td;

    head_planner::HeadPlanner* h = (head_planner::HeadPlanner*)head;

    while( true )
    {
        // sleep or wait on the semaphore to be awaken
        if ( h->think_flag_ )
        {
            // do what you have to
            h->srv_head_execution_.call(h->trajectory_srv_);

            // sleep
            h->t_ = h->t_ + h->thread_dt_;
            td = h->t_ - boost::posix_time::microsec_clock::universal_time();

            if( !(td.seconds()*1000000 + td.fractional_seconds()<0) )
            {
                usleep(td.seconds()*1000000 + td.fractional_seconds());
            }
            else
            {
                std::cout << "Negative wait interval (NEG-HEAD): " << td << std::endl;
            }
        }
        else
        {
            sem_wait( &(h->semaphore_) );
        }
    }
}

} // namespace head_planner

int main(int argc, char **argv)
{
    ros::init(argc, argv, "head_planner_node");
    ros::NodeHandle nh;

    head_planner::HeadPlanner node(nh);

    ROS_INFO("This node is ready to do head planning!");

    while(ros::ok())
    {
    	ros::spinOnce();
    }

    return 0;
}
