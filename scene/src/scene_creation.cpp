#include <ros/ros.h>
#include <ros/package.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <yaml-cpp/yaml.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>


void addObjects(const YAML::Node &node)
{
    static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener transf_listener(buffer);
    moveit::planning_interface::PlanningSceneInterface psi;
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    moveit_msgs::CollisionObject object;
    object.header.stamp = ros::Time::now();
    object.meshes.resize(1);
    object.mesh_poses.resize(1);
    object.operation = moveit_msgs::CollisionObject::ADD;

    geometry_msgs::TransformStamped object_static_transform;
    for (int i = 0; i < node.size(); i++)
    {
        /* Static TF creation */
        while (ros::ok() && !buffer._frameExists(node[i]["frame_id"].as<std::string>()))
        {
            ROS_INFO_STREAM_THROTTLE_NAMED(1.0, "setup_node", "Waiting for " << node[i]["frame_id"].as<std::string>() << " frame..");
            ros::Duration(0.5).sleep();
        }
        object_static_transform.header.stamp = ros::Time::now();
        object_static_transform.header.frame_id = node[i]["frame_id"].as<std::string>();
        object_static_transform.child_frame_id = node[i]["child_frame_id"].as<std::string>();
        object_static_transform.transform.translation.x = node[i]["x"].as<double>();
        object_static_transform.transform.translation.y = node[i]["y"].as<double>();
        object_static_transform.transform.translation.z = node[i]["z"].as<double>();
        tf2::Quaternion quat;
        if (object_static_transform.child_frame_id == "breadboard")
        {
            quat.setRPY(0.0, 0.0, node[i]["r_z"].as<double>() * M_PI / 180.0);
        }
        else
        {
            quat.setRPY(node[i]["r_x"].as<double>() * M_PI / 180.0,
                        node[i]["r_y"].as<double>() * M_PI / 180.0,
                        node[i]["r_z"].as<double>() * M_PI / 180.0);
        }        
        object_static_transform.transform.rotation.x = quat.x();
        object_static_transform.transform.rotation.y = quat.y();
        object_static_transform.transform.rotation.z = quat.z();
        object_static_transform.transform.rotation.w = quat.w();
        static_broadcaster.sendTransform(object_static_transform);

        if (node[i]["x_tf"].IsDefined())
        {
            object_static_transform.header.frame_id = node[i]["child_frame_id"].as<std::string>();
            object_static_transform.child_frame_id = node[i]["child_frame_id"].as<std::string>() + "_tf";
            object_static_transform.transform.translation.x = node[i]["x_tf"].as<double>();
            object_static_transform.transform.translation.y = node[i]["y_tf"].as<double>();
            object_static_transform.transform.translation.z = 0.0;
            object_static_transform.transform.rotation.x = 0.0;
            object_static_transform.transform.rotation.y = 0.0;
            object_static_transform.transform.rotation.z = 0.0;
            object_static_transform.transform.rotation.w = 1.0;
            static_broadcaster.sendTransform(object_static_transform);
        }

        if (node[i]["mesh"].IsDefined())
        {
            /* Collision object creation */
            object.header.frame_id = node[i]["frame_id"].as<std::string>();
            shapes::Mesh *c_mesh = shapes::createMeshFromResource(node[i]["mesh"].as<std::string>(), Eigen::Vector3d(0.001, 0.001, 0.001));
            shapes::ShapeMsg mesh_msg;
            shapes::constructMsgFromShape(c_mesh, mesh_msg);
            shape_msgs::Mesh mesh_custom = boost::get<shape_msgs::Mesh>(mesh_msg);

            object.id = node[i]["child_frame_id"].as<std::string>() + "_obj";
            object.meshes[0] = mesh_custom;
            object.mesh_poses[0].position.x = node[i]["x"].as<double>();
            object.mesh_poses[0].position.y = node[i]["y"].as<double>();
            object.mesh_poses[0].position.z = 0.000001;
            quat.setRPY(0.0, 0.0, node[i]["r_z"].as<double>() * M_PI / 180.0);
            object.mesh_poses[0].orientation.w = quat.w();
            object.mesh_poses[0].orientation.x = quat.x();
            object.mesh_poses[0].orientation.y = quat.y();
            object.mesh_poses[0].orientation.z = quat.z();
            collision_objects.push_back(object);
        }
    }

    psi.applyCollisionObjects(collision_objects);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "scene_creation_node");
    ros::NodeHandle nh;

    std::string pkg_path = ros::package::getPath("scene_creation");
    YAML::Node node = YAML::LoadFile(pkg_path + "/cfg/setup.yaml");
    addObjects(node["scene"]);
    ros::spin();

    return 0;
}