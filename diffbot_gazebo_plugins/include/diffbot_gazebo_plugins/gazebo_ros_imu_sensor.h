/* Copyright [2015] [Alessandro Settimi]
 *
 * email: ale.settimi@gmail.com
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.*/

/* Modified by the diffbot project: a copy of gazebo_plugins' gazebo_ros_imu_sensor.h from
 * gazebo_ros_pkgs 2.9.3 (https://github.com/ros-simulation/gazebo_ros_pkgs/tree/2.9.3).
 * Changes in this file:
 * - The class is in the namespace diffbot_gazebo_plugins instead of gazebo. Gazebo loads plugin
 *   libraries with RTLD_GLOBAL. With the same class name as gazebo_plugins' class, a Gazebo
 *   process that loads both libraries could run gazebo_plugins' code for this plugin too.
 * - The Gazebo names are qualified with gazebo::, the lines are at most 99 characters, and the
 *   file includes the header of every name it uses.
 * gazebo_ros_imu_sensor.cpp lists the changes to the plugin's behavior. */

#ifndef DIFFBOT_GAZEBO_PLUGINS_GAZEBO_ROS_IMU_SENSOR_H
#define DIFFBOT_GAZEBO_PLUGINS_GAZEBO_ROS_IMU_SENSOR_H

#include <gazebo/common/CommonTypes.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/UpdateInfo.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <sdf/Element.hh>
#include <sensor_msgs/Imu.h>
#include <string>

namespace gazebo
{
  namespace sensors
  {
    class ImuSensor;
  }
}

namespace diffbot_gazebo_plugins
{
  /**
  @anchor GazeboRosImuSensor
  \ref GazeboRosImuSensor is a plugin to simulate an Inertial Motion Unit sensor, the main
  differences from gazebo_plugins' GazeboRosIMU are:
  - inheritance from SensorPlugin instead of ModelPlugin,
  - measurements are given by gazebo ImuSensor instead of being computed by the ros plugin,
  - gravity is included in inertial measurements.
  */
  /** @brief Gazebo Ros imu sensor plugin. */
  class GazeboRosImuSensor : public gazebo::SensorPlugin
  {
  public:
    /// \brief Constructor.
    GazeboRosImuSensor();
    /// \brief Destructor.
    virtual ~GazeboRosImuSensor();
    /// \brief Load the sensor.
    /// \param sensor_ pointer to the sensor.
    /// \param sdf_ pointer to the sdf config file.
    virtual void Load(gazebo::sensors::SensorPtr sensor_, sdf::ElementPtr sdf_);

  protected:
    /// \brief Update the sensor.
    virtual void UpdateChild(const gazebo::common::UpdateInfo &/*_info*/);

  private:
    /// \brief Load the parameters from the sdf file.
    bool LoadParameters();
    /// \brief Gaussian noise generator.
    /// \param mu offset value.
    /// \param sigma scaling value.
    double GuassianKernel(double mu, double sigma);

    /// \brief Ros NodeHandle pointer.
    ros::NodeHandle* node;
    /// \brief Ros Publisher for imu data.
    ros::Publisher imu_data_publisher;
    /// \brief Ros IMU message.
    sensor_msgs::Imu imu_msg;

    /// \brief last time on which the data was published.
    gazebo::common::Time last_time;
    /// \brief Pointer to the update event connection.
    gazebo::event::ConnectionPtr connection;
    /// \brief Pointer to the sensor.
    gazebo::sensors::ImuSensor* sensor;
    /// \brief Pointer to the sdf config file.
    sdf::ElementPtr sdf;
    /// \brief Orientation data from the sensor.
    ignition::math::Quaterniond orientation;
    /// \brief Linear acceleration data from the sensor.
    ignition::math::Vector3d accelerometer_data;
    /// \brief Angular velocity data from the sensor.
    ignition::math::Vector3d gyroscope_data;

    //loaded parameters
    /// \brief The data is published on the topic named: /robot_namespace/topic_name.
    std::string robot_namespace;
    /// \brief The data is published on the topic named: /robot_namespace/topic_name.
    std::string topic_name;
    /// \brief Name of the link of the IMU.
    std::string body_name;
    /// \brief Sensor update rate.
    double update_rate;
    /// \brief Gaussian noise.
    double gaussian_noise;
    /// \brief Offset parameter, position part is unused.
    ignition::math::Pose3d offset;
  };
}

#endif //DIFFBOT_GAZEBO_PLUGINS_GAZEBO_ROS_IMU_SENSOR_H
