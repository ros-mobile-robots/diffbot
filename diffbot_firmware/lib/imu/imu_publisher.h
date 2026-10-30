#ifndef DIFFBOT_IMU_PUBLISHER_H
#define DIFFBOT_IMU_PUBLISHER_H

#include <stdint.h>

#include <ros/node_handle.h>
#include <sensor_msgs/Imu.h>

#include "bno055.h"
#include "imu_config.h"

namespace diffbot
{
    /**
     * @brief Drives the IMU's start-up and publishes it on imu/data_raw, with the settings of an
     * \ref ImuPublisherConfig.
     *
     * The start-up doesn't wait: each \ref update does at most a few short transfers, so the
     * control loop keeps its rate while the sensor resets. A missing or failed IMU is tried again
     * after retry_ms. A sample is published only after a successful read that took at most
     * read_budget_us, and only while the clock is synchronized with the host, so a stalled or
     * unplugged IMU stops the topic instead of repeating old values with new time stamps. With
     * enabled false, it doesn't advertise the topic and doesn't touch the I2C bus.
     *
     * The sampling period counts on the Teensy's own clock (Clock::millis), not on ROS time: ROS
     * time jumps when the host's clock is corrected or when the Teensy connects to a host whose
     * clock is earlier, and the rate must not depend on that. ROS time only stamps the samples.
     *
     * The stamp is the ROS time when the read begins. It estimates the sample's time, but it isn't
     * the time when the sensor measured: the read returns the registers' latest values, and the
     * BNO055 doesn't tell when it wrote them.
     *
     * Header-only and a template, so that the native unit tests run it with a fake serial port and
     * clock (test/test_imu_publisher).
     *
     * @tparam NodeHandle \ref SyncedNodeHandleT
     * @tparam Clock A type with static millis() and micros(), as the Arduino functions
     */
    template <class NodeHandle, class Clock>
    class ImuPublisher
    {
    public:
        static const int RESTART_AFTER_FAILED_READS = 10;

        ImuPublisher(NodeHandle& nh, Bno055& imu, const ImuPublisherConfig& config)
            : nh_(nh), imu_(imu), config_(config), publisher_("imu/data_raw", &message_)
        {
        }

        /// Call before the node handle connects: rosserial announces the topics then
        void advertise()
        {
            if (config_.enabled)
                nh_.advertise(publisher_);
        }

        /// Call in every loop
        void update();

        const sensor_msgs::Imu& message() const { return message_; }

    private:
        void publish(const ros::Time& stamp, const ImuSample& sample);

        NodeHandle& nh_;
        Bno055& imu_;
        const ImuPublisherConfig config_;
        sensor_msgs::Imu message_;
        ros::Publisher publisher_;
        Bno055::State reported_state_ = Bno055::STARTING;
        uint32_t retry_ms_ = 0;
        uint32_t sampled_ms_ = 0;
        int failed_reads_ = 0;
    };

    template <class NodeHandle, class Clock>
    void ImuPublisher<NodeHandle, Clock>::update()
    {
        if (!config_.enabled)
            return;
        uint32_t now_ms = Clock::millis();
        Bno055::State state = imu_.update(now_ms);
        if (state != reported_state_)
        {
            if (state == Bno055::READY)
                nh_.loginfo("IMU: BNO055 started, publishing imu/data_raw");
            else if (state == Bno055::ABSENT)
                nh_.logwarn("IMU: no BNO055 found, trying again later. The robot drives without it.");
            else if (state == Bno055::FAILED)
                nh_.logwarn("IMU: the BNO055 didn't start, trying again later");
            if (state == Bno055::READY || state == Bno055::ABSENT || state == Bno055::FAILED)
                reported_state_ = state;
            retry_ms_ = now_ms;
        }
        if ((state == Bno055::ABSENT || state == Bno055::FAILED)
            && now_ms - retry_ms_ >= config_.retry_ms)
        {
            imu_.restart();
            retry_ms_ = now_ms;
        }

        // Unsigned, so it stays right when millis() wraps
        if (now_ms - sampled_ms_ < config_.period_ms)
            return;
        sampled_ms_ = now_ms;
        if (state != Bno055::READY || !nh_.timeSynced())
            return;

        // The time when the read begins (see the class's description)
        ros::Time stamp = nh_.now();
        uint32_t start_us = Clock::micros();
        ImuSample sample;
        bool ok = imu_.read(sample);
        uint32_t duration_us = Clock::micros() - start_us;
        if (ok && duration_us <= config_.read_budget_us)
        {
            failed_reads_ = 0;
            publish(stamp, sample);
        }
        else if (!ok && imu_.state() == Bno055::READY && ++failed_reads_ >= RESTART_AFTER_FAILED_READS)
        {
            failed_reads_ = 0;
            imu_.restart();
            nh_.logwarn("IMU: reading the BNO055 failed 10 times in a row, starting it again");
        }
    }

    /**
     * Only the angular velocity and the acceleration are published. The orientation is marked as
     * unavailable (orientation_covariance[0] = -1, REP 145). The covariances come from the
     * configuration.
     */
    template <class NodeHandle, class Clock>
    void ImuPublisher<NodeHandle, Clock>::publish(const ros::Time& stamp, const ImuSample& sample)
    {
        message_.header.stamp = stamp;
        message_.header.frame_id = config_.frame_id;
        message_.orientation_covariance[0] = -1.0;
        message_.angular_velocity.x = sample.angular_velocity[0];
        message_.angular_velocity.y = sample.angular_velocity[1];
        message_.angular_velocity.z = sample.angular_velocity[2];
        message_.linear_acceleration.x = sample.linear_acceleration[0];
        message_.linear_acceleration.y = sample.linear_acceleration[1];
        message_.linear_acceleration.z = sample.linear_acceleration[2];
        for (int i = 0; i < 9; i += 4)
        {
            message_.angular_velocity_covariance[i] = config_.angular_velocity_variance;
            message_.linear_acceleration_covariance[i] = config_.linear_acceleration_variance;
        }
        publisher_.publish(&message_);
    }
}

#endif // DIFFBOT_IMU_PUBLISHER_H
