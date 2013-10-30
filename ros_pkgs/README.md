ros_pkgs
========

Meta-package containing PaCMan ROS packages.


Guidelines to develop ROS wrappers
----------------------------------

1. Create your package using catkin_create_pkg tool. Configure the dependencies later.

2. The folder structure should be something like:

`launch`
`msg`
`src`
`srv`
`CMakeLists.txt`
`package.xml`

Other folders might be added as well, such as include, urdf, models, geometry, etc.

3. Nodes (which implement class ROS wrappers) go into `src`. The name of the file should be as: `my_class_node.cpp`  The `_node` part is mandatory, so we can know that this is ROS dependant code, even when we are inside the ROS workspace.

        // use <> for system headers
        #include <STANDARD_HEADERS>
        #include <ROS_HEADERS.h>
        #include <EXTERNAL_LIB_HEADERS.h>
    
        // use "" for local headers
        #include "MY_CLASSES_HEADERS.h"
    
        // ROS generated headers
        // typically these ones are generated at compilation time, 
        // for instance, for a service called ClassParameterSetting.srv 
        // within package_name, we need to include this 
        // #include "other_package_name/ClassParameterSetting.h" 
    
    
        namespace package_name {
    
        // use a name for the node and a verb it is suppose to do, Publisher, Server, etc...
        class NamePerformer
        {
          private:
            
        	// suggested/quasi_mandatory members, note the member_name_ naming convention
    
            // the node handle
            ros::NodeHandle nh_;
            
            // Node handle in the private namespace
            ros::NodeHandle priv_nh_;
    
            // your class object
            MY_CLASS1* class1_object_;
            MY_CLASS2* class2_object_;
    
            // subscribers
            ros::Subscriber sub_some_node_messages_;
    
            // publishers
            ros::Publisher pub_class_postprocessing_info_;
            
            // services
            ros::ServiceServer srv_set_parameters_;
    
            // it is very useful to have a listener and broadcaster to know where all frames are
            tf::TransformListener tf_listener_;
            tf::TransformBroadcaster tf_broadcaster_;
            
          public:
    
            // callback functions
            void doClassComputations(const X_msgs::Wrench & msg);
            bool setClassParameters(ClassParameterSetting::Request &request, 
                                      ClassParameterSetting::Response &response);
            void doSomething();
    
            // constructor
            NamePerformer(ros::NodeHandle nh) : nh_(nh), priv_nh_("~")
            {
                // create objects of your classes
                class1_object_ = new MY_CLASS1();
    
                // subscribe to topics
                sub_some_node_messages_ = nh_.subscribe(nh_.resolveName("topic_name_you_want_to_subscribe_to"), 
                                                          10, &NamePerformer::doClassComputations, this);
    
                // advertise topics
                pub_class_postprocessing_info_ =
                  nh_.advertise<type_msgs::MessageType>(nh_.resolveName("topic_name_you_want_to_pusblih"), 10);
                
                // advertise service
                srv_set_parameters_ = 
                  nh_.advertiseService(nh_.resolveName("service_name_you_want_to_advertise"), 
                                                                &NamePerformer::setClassParameterss, this);
            }
    
            //! Empty stub
            ~NamePerformer() {}
    
        };
    
        // this function is called when a new message is received at the topic_name_you_want_to_subscribe_to
        void NamePerformer::doClassComputations(const X_msgs::Wrench & msg)
        {
            // 1. convert the message into a useful data type for instance, used by your class
    
            // 2. call your class function with the correct data types
            class1_object_.doFunction();
            class2_object_.doStuff();
    
            // 3. convert the data type return from your class to a ROS type_msgs::MessageType
    
            // 4. and publish the processed information
            pub_class_postprocessing_info_.publish(type_msgs::MessageType);
        }
    
        // this function is called when service_name_you_want_to_advertise is advertise
        bool NamePerformer::setClassParameterss(ClassParameterSetting::Request &request, 
                                                  ClassParameterSetting::Response &response)
        {
            // 1. convert the message into a useful data type for instance, used by your class
    
            // 2. call your class function with the correct data types
            class1_object_.setParameters();
            class2_object_.setParameters();
        }
    
        // this function is called as fast as ROS can from the main loop directly
        void NamePerformer::doSomething()
        {
            // 1. do something
        }
    
        } // namespace package_name
    
        int main(int argc, char **argv)
        {
            ros::init(argc, argv, "name_performer_node");
            ros::NodeHandle nh;
    
            package_name::NamePerformer node(nh);
    
            while(ros::ok())
            {
            node.doSomething();
            ros::spinOnce();
            }
    
            return 0;
        }


4. Launch files go into the `launch` folder. There should be a launch file for each node, which remaps, set parameters, and any configuration for this node to work well. Launch files can be nested in other launch files, and configuration can be done at higher levels, but this is not advised, only if it is strictly necessary. There might be a general launch file for the package which brings up the all nodes available for a proper functionality.

6. Message files go into the `msg` folder and define ROS data types. The naming should be `Object.msg` or `Grasp.msg`, in this way we define what an object or a grasp is for us.

7. Service files go into the `srv` folder. The naming should `ClassParameterSetting.srv`, which must be related to the node name and the function is doing. Note that, the verb is written as a noun.

8. Configuration of the `CMakeLists.txt` to link against an external library. You should define `include_directories()` and `link_directories()` to point where your class library was compiled. And specify at the `target_link_library()` the name of the library you want to link against.

9. Configuration of the `package.xml` depends on which ROS components you are using.
