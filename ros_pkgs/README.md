ros_pkgs
========

PaCMan ROS packages.


Folder strucure
---------------

The subfolders are organized according to the topic is inside. Each folder, including this one, is a ROS metapackage, that is, a collection of packages. This is just a matter of file/package organization, however, each package can be used independently of where it is located. The main folders we have are

* `controllers` self-explains, contains code related to controllers we develop for the available hardware.
* `definitions` contains the messages we should be using to communicate between the ROS packages, initially thought to be similar to the PaCMan defs in c++. However, it might be the case that these messages are redundant with respect to the ones already available in ROS.
* `demos` contains code related to the demos we perform. Typically, all logic/state/information flow management should be here.
* `hardware` contains the geometric and dynnamic description of the available hardware. Any change in the geometry of the robots should be updated here, and not in the c++ code, such as adding connectors, mounting different hands, and so on.
* `perception` self-explains, contains code related to perception algorithms we develop for the available hardware.
* `planners` self-explains, contais code related to the planning algorithms we develop for the available hardware.
* `ros_pkgs` is the folder defining that this is a meta-package.

You can find more details on how to build/use the (meta-)packages in their spcific README files.

Build
-----

It is strongly advised that all packages be built using the main CMakeLists.txt.

However, you can build this package following [this tutorial](http://wiki.ros.org/ROS/Tutorials/BuildingPackages) as a ROS package, but you might need to change the CMakeLists.txt to correct paths.

ToDo: Port the [pose_estimation_uibk](https://github.com/pacman-project/pacman/tree/master/ros_pkgs/perception/pose_estimation_uibk) package to use the pacman interface so there will be no need to adjust the internal CMakeLists.txt file.

IMPORTANT: If the compilation breaks, please, see the README of the package that is breaking the compilation before going crazy because this project is under development. 

Use
---

From the user point of view, the main folder you need to see is the `demos` folder, which contains the available demos. Please, go there, and take a look at the README file for more details. 


Guidelines to write a ROS wrapper of your lib/class
---------------------------------------------------

1. Create your package using catkin_create_pkg tool. you can configure the dependencies later.

2. The package folder structure should be:

`include`
`launch`
`msg`
`src`
`srv`
`CMakeLists.txt`
`package.xml`

Other folders might be added as well, such as urdf, meshes, xacro, etc.

3. Launch files go into the `launch` folder. There should be a launch file for each node, which remaps, set parameters, and any configuration for this node to work well. Launch files can be nested in other launch files, and configuration can be done at higher levels, but this is not advised, only if it is strictly necessary. There might be a general launch file for the package which brings up the all nodes available for a proper functionality. Try not to include launch files from other packages to make your package as independent as posisble. The high level app, such as a demo node, shall include and assemble all the required launch files.

4. Message files go into the `msg` folder and define ROS data types. The naming should use capital letters at the beggining, such as `Object.msg` or `Grasp.msg`, which define what an object or a grasp is within the PaCMan project.

5. Service files go into the `srv` folder. The naming should `ClassParameterSetting.srv`, which must be related to the node name and the function is doing. Note that, the verb is written as a noun.

6. Configuration of the `CMakeLists.txt` to link against an external library. You should define `include_directories()` and `link_directories()` to point where your class library was compiled. And specify at the `target_link_library()` the name of the library you want to link against. We still need to see how a var can be passed during the main building procedure.

7. Configuration of the `package.xml` depends on which ROS components you are using, and it is mainly used to tell ROS the existence of your package and create the dependency tree for your package.

8. Nodes (which implement class ROS wrappers) go into `src`. The name of the file should be as: `my_class_wrapper_node.cpp`  The `_node` part is advised, so we can know that this code is ROS dependant, even when we are inside the ROS workspace. you can start from the template shown below to write your wrapper:

```

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
        // suggested members, note the member_name_ naming convention
        
        // the node handle
        ros::NodeHandle nh_;
        
        // Node handle in the private namespace
        ros::NodeHandle priv_nh_;
        
        // your class object
        MY_CLASS1* class1_object_;
        
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
        bool setClassParameters(ClassParameterSetting::Request &request, ClassParameterSetting::Response &response);
        void doSomething();
        
        // constructor
        NamePerformer(ros::NodeHandle nh) : nh_(nh), priv_nh_("~")
        {
            // create objects of your classes
            class1_object_ = new MY_CLASS1();

            // subscribe to topics
            sub_some_node_messages_ = nh_.subscribe(nh_.resolveName("topic_name_you_want_to_subscribe_to"), 10, &NamePerformer::doClassComputations, this);

            // advertise topics
            pub_class_postprocessing_info_ = nh_.advertise<type_msgs::MessageType>(nh_.resolveName("topic_name_you_want_to_pusblih"), 10);
            
            // advertise service
            srv_set_parameters_ = nh_.advertiseService(nh_.resolveName("service_name_you_want_to_advertise"), &NamePerformer::setClassParameterss, this);
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
bool NamePerformer::setClassParameterss(ClassParameterSetting::Request &request, ClassParameterSetting::Response &response)
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

```

9. In addition to the node that wraps your controller, you are asked to have a test for its functionality. The test can be implemented as a message/service/action advertising that uses the ROS shell interface, or as ping_YOUR_NODE.cpp that uses the ROS c++ interface.

10. Last but not least, please, the minimum documentation required is to comment your code, and provide a README file explaining what your package do, installation specifics, or anyother special things you would like users to be noticed.