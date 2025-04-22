#include<graph_core/solvers/birrt.h>
#include<jsk_rviz_plugins/OverlayText.h>
#include<replanners_lib/replanner_managers/replanner_manager_DRRT.h>
#include<replanners_lib/replanner_managers/replanner_manager_MARS.h>
#include<replanners_lib/replanner_managers/replanner_manager_MPRRT.h>
#include<replanners_lib/replanner_managers/replanner_manager_DRRTStar.h>
#include<replanners_lib/replanner_managers/replanner_manager_anytimeDRRT.h>

#include <yaml-cpp/yaml.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "replanners_benchmark");
  ros::AsyncSpinner spinner(4);
  spinner.start();
  ros::Duration(5).sleep();

  ros::NodeHandle nh;
  ros::ServiceClient ps_client = nh.serviceClient<moveit_msgs::GetPlanningScene>("/get_planning_scene");
  ros::ServiceClient add_obj = nh.serviceClient<object_loader_msgs::AddObjects>("/add_object_to_scene");
  ros::ServiceClient move_obj = nh.serviceClient<object_loader_msgs::MoveObjects>("/move_object_in_scene");
  ros::ServiceClient remove_obj = nh.serviceClient<object_loader_msgs::RemoveObjects>("/remove_object_from_scene");
  ros::Publisher text_overlay_pub = nh.advertise<jsk_rviz_plugins::OverlayText>("/rviz_text_overlay", 1);

  object_loader_msgs::AddObjects srv_add_object;
  object_loader_msgs::MoveObjects srv_move_objects;
  object_loader_msgs::RemoveObjects srv_remove_object;
  std::vector<object_loader_msgs::Object> spawned_objects;
  std::vector<std::string> obj_ids;

  if (!ps_client.waitForExistence(ros::Duration(10)))
  {
    ROS_ERROR("unable to connect to /get_planning_scene");
    return 1;
  }

  if(!add_obj.waitForExistence(ros::Duration(10)))
  {
    ROS_ERROR("unable to connect to /add_object_to_scene");
    return 1;
  }

  if(!remove_obj.waitForExistence(ros::Duration(10)))
  {
    ROS_ERROR("unable to connect to /remove_object_to_scene");
    return 1;
  }
  
  //  ////////////////////////////////////////// GETTING ROS PARAM ///////////////////////////////////////////////
  int n_query = 1;
  nh.getParam("n_query",n_query);

  int n_iter_per_query = 1;
  nh.getParam("n_iter_per_query",n_iter_per_query);

  std::vector<std::string> replanner_type_vector;
  nh.getParam("replanner_type_vector",replanner_type_vector);

  if(replanner_type_vector.empty())
  {
    ROS_INFO("replanner_type_vector not set");
    return 0;
  }

  std::string bench_name;
  if (!nh.getParam("bench_name",bench_name))
  {
    ROS_INFO("bench_name not set");
    return 0;
  }

  std::string group_name;
  if (!nh.getParam("group_name",group_name))
  {
    ROS_ERROR("group_name not set, exit");
    return 0;
  }

  double max_distance;
  if(!nh.getParam("max_distance",max_distance))
  {
    ROS_ERROR("max_distance not set, set 0.5");
    max_distance = 0.5;
  }

  bool display;
  if (!nh.getParam("display",display))
  {
    display = false;
  }

  double max_solver_time;
  if (!nh.getParam("max_solver_time",max_solver_time))
  {
    max_solver_time = 10;
  }
  
  std::string obs_pose_topic;
  if(!nh.getParam("virtual_obj/obs_pose_topic", obs_pose_topic))
    obs_pose_topic = "/poses";
  ros::Publisher obj_pose_pub = nh.advertise<geometry_msgs::PoseArray>(obs_pose_topic, 10);

  std_msgs::ColorRGBA fg_color, bg_color;
  fg_color.r = 0;
  fg_color.g = 0;
  fg_color.b = 1;
  fg_color.a = 0.8;

  bg_color.r = 0;
  bg_color.g = 0;
  bg_color.b = 0;
  bg_color.a = 0;

  jsk_rviz_plugins::OverlayText overlayed_text;
  overlayed_text.font = "FreeSans";
  overlayed_text.bg_color = bg_color;
  overlayed_text.fg_color = fg_color;
  overlayed_text.height = 70;
  overlayed_text.left = 10;
  overlayed_text.top = 10;
  overlayed_text.width = 1000;
  overlayed_text.line_width = 2;
  overlayed_text.text_size = 20;

  //  ///////////////////////////////////UPLOADING THE ROBOT ARM/////////////////////////////////////////////////////////////
  moveit::planning_interface::MoveGroupInterface move_group(group_name);
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  planning_scene::PlanningScenePtr planning_scene = std::make_shared<planning_scene::PlanningScene>(kinematic_model);

  const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(group_name);
  std::vector<std::string> joint_names = joint_model_group->getActiveJointModelNames();

  unsigned int dof = joint_names.size();
  Eigen::VectorXd lb(dof);
  Eigen::VectorXd ub(dof);

  for (unsigned int idx = 0; idx < dof; idx++)
  {
    const robot_model::VariableBounds& bounds = kinematic_model->getVariableBounds(joint_names.at(idx));
    if (bounds.position_bounded_)
    {
      lb(idx) = bounds.min_position_;
      ub(idx) = bounds.max_position_;
    }
  }

  std::vector<int> num_obstacles;
  nh.getParam("num_obstacles", num_obstacles);

  std::string project_path(__FILE__);
  for (size_t i = 0; i < 3; i++) {	// This depends on how deep is this file located
    project_path = project_path.substr(0, project_path.find_last_of("/\\"));
	}
  YAML::Node node { YAML::LoadFile(project_path + "/replanners_benchmark/config/random_scenarios.yaml") };
	std::cout << "Random scenarios file path: " << project_path + "/replanners_benchmark/config/random_scenarios.yaml" << "\n";

  for(const std::string replanner_type : replanner_type_vector)
  {
    for (size_t num_obs : num_obstacles)
    {
      ROS_INFO("Number of random obstacles: %d", num_obs);
      std::vector<double> spawn_instants(num_obs, 0);
      nh.setParam("virtual_obj/spawn_instants", spawn_instants);
      nh.setParam("replanner_type",replanner_type);

      int n_query_start;
      nh.getParam("n_query_start", n_query_start);
      if (num_obs != num_obstacles.front())
        n_query_start = 0;

      // ------------------------------------------------------------------------------- //

      geometry_msgs::Quaternion q;
      q.x = 0.0; q.y = 0.0; q.z = 0.0; q.w = 1.0;

      object_loader_msgs::Object new_obj;
      new_obj.object_type = "random_box";
      new_obj.pose.header.frame_id = "world";
      new_obj.pose.pose.orientation = q;

      srv_add_object.request.objects.clear();
      srv_remove_object.request.obj_ids.clear();
      obj_ids.clear();
      spawned_objects.clear();
      
      for (size_t ii = 0; ii < num_obs; ii++)
      {
        new_obj.pose.pose.position.x = 0;
        new_obj.pose.pose.position.y = 0;
        new_obj.pose.pose.position.z = 0;
        srv_add_object.request.objects.push_back(new_obj);
        spawned_objects.push_back(new_obj);
      }

      if(not srv_add_object.request.objects.empty())
      {
        if(not add_obj.call(srv_add_object))
        {
          ROS_ERROR("call to add obj srv not ok");
          return 1;
        }

        if(not srv_add_object.response.success)
          ROS_ERROR("add obj srv error");
        else
        {
          ROS_BOLDMAGENTA_STREAM("Initial obstacles spawned!");
          for (const std::string &str : srv_add_object.response.ids)
          {
            obj_ids.push_back(str);
            srv_remove_object.request.obj_ids.push_back(str);
          }
        }
      }
      
      geometry_msgs::PoseArray pose_array;
      pose_array.header.frame_id = "world";
      pose_array.header.stamp = ros::Time::now();

      geometry_msgs::Pose pose;
      pose.orientation = q;

      for (size_t ii = 0; ii < num_obs; ii++)
      {
        pose.position.x = srv_add_object.request.objects[ii].pose.pose.position.x;
        pose.position.y = srv_add_object.request.objects[ii].pose.pose.position.y;
        pose.position.z = srv_add_object.request.objects[ii].pose.pose.position.z;
        pose_array.poses.push_back(pose);
      }

      obj_pose_pub.publish(pose_array); //publish poses for SSM node

      // ------------------------------------------------------------------------------- //

      for(int i = n_query_start; i < n_query; i++)
      {
        // Nermin added reading from a yaml file:
        Eigen::VectorXd start_conf { Eigen::VectorXd::Zero(dof) };
        Eigen::VectorXd goal_conf { Eigen::VectorXd::Zero(dof) };
        for (size_t ii = 0; ii < dof; ii++)
        {
          start_conf(ii) = node["scenario_" + std::to_string(num_obs)]["run_" + std::to_string(i)]["start"][ii].as<float>();
          goal_conf(ii) = node["scenario_" + std::to_string(num_obs)]["run_" + std::to_string(i)]["goal"][ii].as<float>();
        }
        std::cout << "start_conf: " << start_conf.transpose() << "\n";
        std::cout << "goal_conf:  " << goal_conf.transpose() << "\n";

        srv_move_objects.request.poses.clear();
        srv_move_objects.request.obj_ids.clear();
        pathplan::ReplannerManagerBase::InitObstacles init_obstacles;
        Eigen::Vector3d pos {}, vel {};
        for (size_t j = 0; j < num_obs; j++)
        {
          for (size_t ii = 0; ii < 3; ii++)
          {
            pos(ii) = node["scenario_" + std::to_string(num_obs)]["run_" + std::to_string(i)]
                ["object_" + std::to_string(j)]["pos"][ii].as<float>();
            vel(ii) = node["scenario_" + std::to_string(num_obs)]["run_" + std::to_string(i)]
                ["object_" + std::to_string(j)]["vel"][ii].as<float>();
          }
          init_obstacles.positions.emplace_back(pos);
          init_obstacles.velocities.emplace_back(vel);
          
          spawned_objects.at(j).pose.pose.position.x = pos.x();
          spawned_objects.at(j).pose.pose.position.y = pos.y();
          spawned_objects.at(j).pose.pose.position.z = pos.z();
          spawned_objects.at(j).pose.pose.orientation = q;

          srv_move_objects.request.obj_ids.push_back(obj_ids.at(j));
          srv_move_objects.request.poses.push_back(spawned_objects.at(j).pose);
        }
        
        if(not srv_move_objects.request.poses.empty())
        {
          if(not move_obj.call(srv_move_objects))
            ROS_ERROR("call to move obj srv not ok");

          if(not srv_move_objects.response.success)
            ROS_ERROR("move obj srv error");
        }

        //  /////////////////////////////////////UPDATING THE PLANNING STATIC SCENE////////////////////////////////////
        moveit_msgs::GetPlanningScene ps_srv;
        if (!ps_client.call(ps_srv))
        {
          ROS_ERROR("call to srv not ok");
          return 1;
        }

        if (!planning_scene->setPlanningSceneMsg(ps_srv.response.scene))
        {
          ROS_ERROR("unable to update planning scene");
          return 1;
        }

        // /////////////////////////////////////////////////////////////////////////////////////////////////////////
        std::string last_link = planning_scene->getRobotModel()->getJointModelGroup(group_name)->getLinkModelNames().back();
        pathplan::DisplayPtr disp = std::make_shared<pathplan::Display>(planning_scene,group_name,last_link);
        ros::Duration(0.1).sleep();
        pathplan::MetricsPtr metrics;
        pathplan::CollisionCheckerPtr checker;
        pathplan::SamplerPtr sampler;
        pathplan::RRTPtr solver;
        pathplan::PathPtr current_path, new_path;
        std::vector<pathplan::PathPtr> other_paths;
        pathplan::ReplannerManagerBasePtr replanner_manager;
        pathplan::TrajectoryPtr trajectory = std::make_shared<pathplan::Trajectory>(nh,planning_scene,group_name);
        pathplan::MoveitUtils moveit_utils(planning_scene, group_name);

        int id_start, id_goal;
        disp->changeNodeSize();
        id_start = disp->displayNode(std::make_shared<pathplan::Node>(start_conf),"pathplan",{1.0,0.0,0.0,1.0});
        id_goal = disp->displayNode(std::make_shared<pathplan::Node>(goal_conf),"pathplan",{1.0,0.0,0.0,1.0});
        disp->defaultNodeSize();

        disp->clearMarker(id_start);
        disp->clearMarker(id_goal);

        disp->changeNodeSize();
        id_start = disp->displayNode(std::make_shared<pathplan::Node>(start_conf),"pathplan",{1.0,0.0,0.0,1.0});
        id_goal = disp->displayNode(std::make_shared<pathplan::Node>(goal_conf),"pathplan",{1.0,0.0,0.0,1.0});
        disp->defaultNodeSize();

        double distance = (goal_conf-start_conf).norm();

        for(int j = 0; j < n_iter_per_query; j++)
        {
          ROS_INFO("---------------------------------------------------------------------------------------------------------");
          ROS_INFO_STREAM(replanner_type<<": query: "<<std::to_string(i)<<" Iter: "<<std::to_string(j)<<" start: "<<start_conf.transpose()<< " goal: "<<goal_conf.transpose()<< " distance: "<<distance);
          // std::string test_name = "test_q_"+std::to_string(i)+"_i_"+std::to_string(j);

          // Nermin added:
          std::string replanner_type = "replanner";
          nh.getParam("replanner_type",replanner_type);

          std::string test_name = "test_" + std::to_string(num_obs);

          std::string bench_name = "bench";
          nh.getParam("bench_name",bench_name);

          std::string path = "./replanners_benchmark";
          std::string file_name = path+"/"+bench_name+"/"+replanner_type+"/"+test_name+".log";

          std::ofstream file;
          file.open(file_name, std::ofstream::app);
          file << "Test num:\n" << i << "\n";

          double init_duration_offset;
          ros::WallTime tic_init_path;
          tic_init_path = ros::WallTime::now();

          nh.setParam("test_name",test_name); //to save test results

          overlayed_text.text = "Replanner: "+replanner_type+"\nQuery: "+std::to_string(i)+"/"+std::to_string(n_query-1)+", iter: "+std::to_string(j)+"/"+std::to_string(n_iter_per_query-1);
          text_overlay_pub.publish(overlayed_text);

          if(display)
            disp->nextButton();

          if (!ps_client.call(ps_srv))
          {
            ROS_ERROR("Call to srv not ok");
            file << "success:\n" << "0" << "\n";
            file << "Reason: Call to srv not ok\n";
            file << "----------------------------------------------------\n";
            file.close();
            return 1;
          }

          if (!planning_scene->setPlanningSceneMsg(ps_srv.response.scene))
          {
            ROS_ERROR("Unable to update planning scene");
            file << "success:\n" << "0" << "\n";
            file << "Reason: Unable to update planning scene\n";
            file << "----------------------------------------------------\n";
            file.close();
            return 1;
          }

          metrics = std::make_shared<pathplan::Metrics>();
          checker = std::make_shared<pathplan::ParallelMoveitCollisionChecker>(planning_scene, group_name);
          sampler = std::make_shared<pathplan::InformedSampler>(start_conf,goal_conf,lb,ub);
          solver = std::make_shared<pathplan::BiRRT>(metrics,checker,sampler);
          solver->setMaxDistance(max_distance);

          std::srand(std::time(NULL));
          current_path = trajectory->computePath(start_conf,goal_conf,solver,true,max_solver_time);

          if(not current_path)
          {
            file << "success:\n" << "0" << "\n";
            file << "Reason: Could not compute initial path!\n";
            file << "----------------------------------------------------\n";
            file.close();
            continue;
          }
          else
            ROS_INFO_STREAM("current path cost "<<current_path->cost());

          // //////////////////////////////////////////DEFINING THE REPLANNER//////////////////////////////////////////////
          replanner_manager.reset();
          if(replanner_type == "MPRRT")
          {
            replanner_manager.reset(new pathplan::ReplannerManagerMPRRT(current_path,solver,nh));
          }
          else if(replanner_type ==  "DRRTStar")
          {
            replanner_manager.reset(new pathplan::ReplannerManagerDRRTStar(current_path,solver,nh));
          }
          else if(replanner_type == "DRRT")
          {
            replanner_manager.reset(new pathplan::ReplannerManagerDRRT(current_path,solver,nh));
          }
          else if(replanner_type == "anytimeDRRT")
          {
            replanner_manager.reset(new pathplan::ReplannerManagerAnytimeDRRT(current_path,solver,nh));
          }
          else if(replanner_type == "MARS")
          {
            int n_other_paths;
            if (!nh.getParam("/MARS/n_other_paths",n_other_paths))
            {
              ROS_ERROR("n_other_paths not set, set 1");
              n_other_paths = 1;
            }

            for(unsigned int i = 0; i < n_other_paths; i++)
            {
              // Nermin added: Find other paths if there is enough remaining time
              if ((ros::WallTime::now() - tic_init_path).toSec() > max_solver_time)
                break;

              std::srand(std::time(NULL));
              solver = std::make_shared<pathplan::BiRRT>(metrics,checker,sampler);
              new_path = trajectory->computePath(start_conf,goal_conf,solver,true,max_solver_time);

              other_paths.clear();
              if(new_path)
              {
                other_paths.push_back(new_path);
                ROS_INFO_STREAM("other path cost "<<new_path->cost());
                assert(new_path->getTree());
              }
              else
                ROS_INFO("other path not found");
            }

            std::srand(std::time(NULL));
            solver = std::make_shared<pathplan::BiRRT>(metrics,checker,sampler);
            solver->config(nh);
            replanner_manager = std::make_shared<pathplan::ReplannerManagerMARS>(current_path,solver,nh,other_paths);

          }
          else
          {
            ROS_ERROR("Replanner manager %s does not exist",replanner_type.c_str());
            return 0;
          }

          // Update planning scene again
          if (!ps_client.call(ps_srv))
          {
            ROS_ERROR("call to srv not ok");
            return 1;
          }

          if (!planning_scene->setPlanningSceneMsg(ps_srv.response.scene))
          {
            ROS_ERROR("unable to update planning scene");
            return 1;
          }

          init_duration_offset = (ros::WallTime::now() - tic_init_path).toSec();
          ROS_INFO("Elapsed time for planning initial path(s): %f [s]", init_duration_offset);
          if (init_duration_offset > max_solver_time)
          {
            ROS_ERROR("Maximal time for planning initial path(s) exceeded!");
            file << "success:\n" << "0" << "\n";
            file << "Reason: Maximal time for planning initial path(s) exceeded!\n";
            file << "----------------------------------------------------\n";
            file.close();
            continue;
          }
          file.close();

          // //////////////////////////////REPLANNING///////////////////////////////////////////////////
          replanner_manager->setInitObstacles(init_obstacles);
          replanner_manager->setInitDurationOffset(init_duration_offset);
          replanner_manager->setObjIds(obj_ids);
          replanner_manager->setSpawnedObjects(spawned_objects);
          replanner_manager->start();

          //std::system("clear"); //clear terminal
        }
      }

      if (not remove_obj.call(srv_remove_object))
        ROS_ERROR("call to remove obj srv not ok");
      if(not srv_remove_object.response.success)
        ROS_ERROR("remove obj srv error");

    }
  }

  return 0;
}
