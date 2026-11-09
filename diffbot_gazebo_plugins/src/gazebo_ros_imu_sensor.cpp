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

/* Modified by the diffbot project: a copy of gazebo_plugins' gazebo_ros_imu_sensor.cpp from
 * gazebo_ros_pkgs 2.9.3 (https://github.com/ros-simulation/gazebo_ros_pkgs/tree/2.9.3).
 * Changes in this file:
 * - UpdateChild() doesn't call ros::spinOnce() after it publishes. UpdateChild() runs in
 *   Gazebo's physics thread, and ros::spinOnce() runs the callbacks waiting in ROS's global
 *   callback queue in the calling thread. Among them are the controller manager's services that
 *   load and switch controllers. Those services wait until the controller manager's update,
 *   which also runs in the physics thread, has taken the change. So the physics thread waited
 *   for itself, and Gazebo's clock stopped for good
 *   (https://github.com/ros-mobile-robots/diffbot/issues/185). This plugin only publishes, so it
 *   has no callbacks of its own to run. gazebo_ros_api_plugin's spinner threads serve the global
 *   queue.
 * - UpdateChild() starts its update rate check anew when the sensor's time is earlier than the
 *   time of the last update. A reset of the simulation (/gazebo/reset_simulation) sets Gazebo's
 *   clock back to zero. gazebo_plugins' plugin keeps the time of its last update from before the
 *   reset, and publishes nothing until the clock has passed that time. gazebo_plugins'
 *   gazebo_ros_p3d.cpp treats a clock that went back the same way.
 * - The class is in the namespace diffbot_gazebo_plugins (see gazebo_ros_imu_sensor.h).
 * - Without the optional profiler calls (ENABLE_PROFILER), which this package doesn't enable.
 * - The lines are at most 99 characters, and the file includes the header of every name it
 *   uses.
 * The parameters, the published message and the computations are unchanged. */

#include <diffbot_gazebo_plugins/gazebo_ros_imu_sensor.h>

#include <boost/bind/bind.hpp>
#include <boost/bind/placeholders.hpp>
#include <cstddef>
#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/UpdateInfo.hh>
#include <gazebo/sensors/ImuSensor.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Rand.hh>
#include <ignition/math/Vector3.hh>
#include <math.h>
#include <ros/console.h>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <sdf/Element.hh>
#include <sensor_msgs/Imu.h>
#include <string>

GZ_REGISTER_SENSOR_PLUGIN(diffbot_gazebo_plugins::GazeboRosImuSensor)

namespace diffbot_gazebo_plugins
{

GazeboRosImuSensor::GazeboRosImuSensor(): SensorPlugin()
{
  accelerometer_data = ignition::math::Vector3d(0, 0, 0);
  gyroscope_data = ignition::math::Vector3d(0, 0, 0);
  orientation = ignition::math::Quaterniond(1,0,0,0);
  sensor=NULL;
}

void GazeboRosImuSensor::Load(gazebo::sensors::SensorPtr sensor_, sdf::ElementPtr sdf_)
{
  sdf=sdf_;
  sensor=dynamic_cast<gazebo::sensors::ImuSensor*>(sensor_.get());

  if(sensor==NULL)
  {
    ROS_FATAL("Error: Sensor pointer is NULL!");
    return;
  }

  bool initial_orientation_as_reference = false;
  if (!sdf->HasElement("initialOrientationAsReference"))
  {
    ROS_INFO("<initialOrientationAsReference> is unset, using default value of false "
             "to comply with REP 145 (world as orientation reference)");
  }
  else
  {
    initial_orientation_as_reference = sdf->Get<bool>("initialOrientationAsReference");
  }

  if (initial_orientation_as_reference)
  {
    ROS_WARN("<initialOrientationAsReference> set to true, this behavior is deprecated "
             "as it does not comply with REP 145.");
  }
  else
  {
    // This complies with REP 145
    sensor->SetWorldToReferenceOrientation(ignition::math::Quaterniond::Identity);
  }

  sensor->SetActive(true);

  if(!LoadParameters())
  {
    ROS_FATAL("Error Loading Parameters!");
    return;
  }

  if (!ros::isInitialized()) //check if ros is initialized properly
  {
    ROS_FATAL("ROS has not been initialized!");
    return;
  }

  node = new ros::NodeHandle(this->robot_namespace);

  imu_data_publisher = node->advertise<sensor_msgs::Imu>(topic_name,1);

  connection = gazebo::event::Events::ConnectWorldUpdateBegin(
    boost::bind(&GazeboRosImuSensor::UpdateChild, this, boost::placeholders::_1));

  last_time = sensor->LastUpdateTime();
}

void GazeboRosImuSensor::UpdateChild(const gazebo::common::UpdateInfo &/*_info*/)
{
  gazebo::common::Time current_time = sensor->LastUpdateTime();

  if(current_time < last_time) //the clock went back (a reset): start the update rate check anew
    last_time = current_time;

  if(update_rate>0 && (current_time-last_time).Double() < 1.0/update_rate) //update rate check
    return;

  if(imu_data_publisher.getNumSubscribers() > 0)
  {
    //applying offsets to the orientation measurement
    orientation = offset.Rot()*sensor->Orientation();
    accelerometer_data = sensor->LinearAcceleration();
    gyroscope_data = sensor->AngularVelocity();

    //Guassian noise is applied to all measurements
    imu_msg.orientation.x = orientation.X() + GuassianKernel(0,gaussian_noise);
    imu_msg.orientation.y = orientation.Y() + GuassianKernel(0,gaussian_noise);
    imu_msg.orientation.z = orientation.Z() + GuassianKernel(0,gaussian_noise);
    imu_msg.orientation.w = orientation.W() + GuassianKernel(0,gaussian_noise);

    imu_msg.linear_acceleration.x = accelerometer_data.X() + GuassianKernel(0,gaussian_noise);
    imu_msg.linear_acceleration.y = accelerometer_data.Y() + GuassianKernel(0,gaussian_noise);
    imu_msg.linear_acceleration.z = accelerometer_data.Z() + GuassianKernel(0,gaussian_noise);

    imu_msg.angular_velocity.x = gyroscope_data.X() + GuassianKernel(0,gaussian_noise);
    imu_msg.angular_velocity.y = gyroscope_data.Y() + GuassianKernel(0,gaussian_noise);
    imu_msg.angular_velocity.z = gyroscope_data.Z() + GuassianKernel(0,gaussian_noise);

    //covariance is related to the Gaussian noise
    double gn2 = gaussian_noise*gaussian_noise;
    imu_msg.orientation_covariance[0] = gn2;
    imu_msg.orientation_covariance[4] = gn2;
    imu_msg.orientation_covariance[8] = gn2;
    imu_msg.angular_velocity_covariance[0] = gn2;
    imu_msg.angular_velocity_covariance[4] = gn2;
    imu_msg.angular_velocity_covariance[8] = gn2;
    imu_msg.linear_acceleration_covariance[0] = gn2;
    imu_msg.linear_acceleration_covariance[4] = gn2;
    imu_msg.linear_acceleration_covariance[8] = gn2;

    //preparing message header
    imu_msg.header.frame_id = body_name;
    imu_msg.header.stamp.sec = current_time.sec;
    imu_msg.header.stamp.nsec = current_time.nsec;

    imu_data_publisher.publish(imu_msg);
    // No ros::spinOnce() here: this function runs in Gazebo's physics thread, which must not
    // run ROS callbacks. A controller manager service that it ran would wait for this thread
    // (see the notice at the top of this file).
  }

  last_time = current_time;
}

double GazeboRosImuSensor::GuassianKernel(double mu, double sigma)
{
  // generation of two normalized uniform random variables
  double U1 = ignition::math::Rand::DblUniform();
  double U2 = ignition::math::Rand::DblUniform();

  // using Box-Muller transform to obtain a varaible with a standard normal distribution
  double Z0 = sqrt(-2.0 * ::log(U1)) * cos(2.0*M_PI * U2);

  // scaling
  Z0 = sigma * Z0 + mu;
  return Z0;
}

bool GazeboRosImuSensor::LoadParameters()
{
  //loading parameters from the sdf file

  //NAMESPACE
  if (sdf->HasElement("robotNamespace"))
  {
    robot_namespace =  sdf->Get<std::string>("robotNamespace") +"/";
    ROS_INFO_STREAM("<robotNamespace> set to: "<<robot_namespace);
  }
  else
  {
    std::string scoped_name = sensor->ParentName();
    std::size_t it = scoped_name.find("::");

    robot_namespace = "/" +scoped_name.substr(0,it)+"/";
    ROS_WARN_STREAM("missing <robotNamespace>, set to default: " << robot_namespace);
  }

  //TOPIC
  if (sdf->HasElement("topicName"))
  {
    topic_name =  sdf->Get<std::string>("topicName");
    ROS_INFO_STREAM("<topicName> set to: "<<topic_name);
  }
  else
  {
    topic_name = "imu_data";
    ROS_WARN_STREAM("missing <topicName>, set to /namespace/default: " << topic_name);
  }

  //BODY NAME
  if (sdf->HasElement("frameName"))
  {
    body_name =  sdf->Get<std::string>("frameName");
    ROS_INFO_STREAM("<frameName> set to: "<<body_name);
  }
  else
  {
    ROS_FATAL("missing <frameName>, cannot proceed");
    return false;
  }

  //UPDATE RATE
  if (sdf->HasElement("updateRateHZ"))
  {
    update_rate =  sdf->Get<double>("updateRateHZ");
    ROS_INFO_STREAM("<updateRateHZ> set to: " << update_rate);
  }
  else
  {
    update_rate = 1.0;
    ROS_WARN_STREAM("missing <updateRateHZ>, set to default: " << update_rate);
  }

  //NOISE
  if (sdf->HasElement("gaussianNoise"))
  {
    gaussian_noise =  sdf->Get<double>("gaussianNoise");
    ROS_INFO_STREAM("<gaussianNoise> set to: " << gaussian_noise);
  }
  else
  {
    gaussian_noise = 0.0;
    ROS_WARN_STREAM("missing <gaussianNoise>, set to default: " << gaussian_noise);
  }

  //POSITION OFFSET, UNUSED
  if (sdf->HasElement("xyzOffset"))
  {
    offset.Pos() =  sdf->Get<ignition::math::Vector3d>("xyzOffset");
    ROS_INFO_STREAM("<xyzOffset> set to: " << offset.Pos()[0] << ' ' << offset.Pos()[1] << ' '
                    << offset.Pos()[2]);
  }
  else
  {
    offset.Pos() = ignition::math::Vector3d(0, 0, 0);
    ROS_WARN_STREAM("missing <xyzOffset>, set to default: " << offset.Pos()[0] << ' '
                    << offset.Pos()[1] << ' ' << offset.Pos()[2]);
  }

  //ORIENTATION OFFSET
  if (sdf->HasElement("rpyOffset"))
  {
    offset.Rot() = ignition::math::Quaterniond(sdf->Get<ignition::math::Vector3d>("rpyOffset"));
    ROS_INFO_STREAM("<rpyOffset> set to: " << offset.Rot().Roll() << ' ' << offset.Rot().Pitch()
                    << ' ' << offset.Rot().Yaw());
  }
  else
  {
    offset.Rot() = ignition::math::Quaterniond::Identity;
    ROS_WARN_STREAM("missing <rpyOffset>, set to default: " << offset.Rot().Roll() << ' '
                    << offset.Rot().Pitch() << ' ' << offset.Rot().Yaw());
  }

  return true;
}

GazeboRosImuSensor::~GazeboRosImuSensor()
{
  if (connection.get())
  {
    connection.reset();
    connection = gazebo::event::ConnectionPtr();
  }

  node->shutdown();
}

}  // namespace diffbot_gazebo_plugins
