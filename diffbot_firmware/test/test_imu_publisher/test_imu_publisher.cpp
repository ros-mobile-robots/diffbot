// Unit tests of ImuPublisher (lib/imu/imu_publisher.h) on the PC: the real driver and rosserial's
// real parser, with a fake BNO055, serial port and clock. Run them with:
//   pio test -e native
#include <unity.h>

#include "../fakes/fake_bno055.h"
#include "../fakes/fake_rosserial.h"
#include "imu_publisher.h"
#include "synced_node_handle.h"

using namespace diffbot;

uint64_t FakeClock::us = 0;

typedef SyncedNodeHandleT<ros::NodeHandle_<FakeSerial>> NodeHandle;
typedef ImuPublisher<NodeHandle, FakeClock> Publisher;

const uint32_t HOST_SEC = 1700000000;

// How long each data read takes, in µs of the fake clock
uint32_t read_duration_us = 0;
void advanceDuringRead() { FakeClock::us += read_duration_us; }

struct Robot
{
    NodeHandle nh;
    FakeBno055Bus bus;
    Bno055 imu;
    Publisher publisher;

    Robot() : imu(bus), publisher(nh, imu)
    {
        bus.on_data_read = advanceDuringRead;
        nh.initNode();
        publisher.advertise();
    }

    // The host's clock: host_sec at connected_ms
    uint32_t host_sec = HOST_SEC;
    uint32_t connected_ms = 0;
    int time_requests = 0;
    bool answer_time_requests = true;

    void connect(uint32_t ms, uint32_t sec = HOST_SEC)
    {
        host_sec = sec;
        connected_ms = ms;
        FakeClock::setMs(ms);
        nh.getHardware()->negotiation();
        nh.spinOnce();
        time_requests = nh.getHardware()->count(rosserial_msgs::TopicInfo::ID_TIME);
        nh.getHardware()->timeReply(host_sec);
        nh.spinOnce();
    }

    /// Runs the loop once per millisecond until the given time; returns the IMU messages sent. A
    /// slow read moves the clock on, as on the Teensy. The host answers rosserial's periodic time
    /// requests in the next millisecond, as rosserial's node would; without answers, rosserial
    /// drops the connection after 11 s
    int runUntil(uint32_t end_ms)
    {
        int before = nh.getHardware()->countTopics();
        while (FakeClock::millis() < end_ms)
        {
            FakeClock::setMs(FakeClock::millis() + 1);
            nh.spinOnce();
            publisher.update();
            int requests = nh.getHardware()->count(rosserial_msgs::TopicInfo::ID_TIME);
            if (requests > time_requests && answer_time_requests)
            {
                time_requests = requests;
                uint32_t since_ms = FakeClock::millis() - connected_ms;
                nh.getHardware()->timeReply(host_sec + since_ms / 1000, (since_ms % 1000) * 1000000UL);
            }
        }
        return nh.getHardware()->countTopics() - before;
    }
};

// A robot whose IMU has started and whose clock is synchronized, at 2000 ms
Robot* startedRobot()
{
    static Robot* robot = nullptr;
    delete robot;
    robot = new Robot();
    robot->connect(10);
    robot->runUntil(2000);
    TEST_ASSERT_EQUAL(Bno055::READY, robot->imu.state());
    return robot;
}

void setUp()
{
    FakeClock::setMs(0);
    read_duration_us = 0;
}
void tearDown() {}

void test_publishes_at_the_rate()
{
    Robot& robot = *startedRobot();
    TEST_ASSERT_EQUAL(1000 / Publisher::PERIOD_MS, robot.runUntil(3000));
    const sensor_msgs::Imu& message = robot.publisher.message();
    TEST_ASSERT_EQUAL_STRING(IMU_FRAME_ID, message.header.frame_id);
    TEST_ASSERT_EQUAL_FLOAT(-1.0f, message.orientation_covariance[0]);
    TEST_ASSERT_EQUAL_FLOAT(IMU_ANGULAR_VELOCITY_VARIANCE, message.angular_velocity_covariance[8]);
    TEST_ASSERT_EQUAL_FLOAT(IMU_LINEAR_ACCELERATION_VARIANCE, message.linear_acceleration_covariance[8]);
    TEST_ASSERT_EQUAL_UINT32(HOST_SEC + 2, message.header.stamp.sec);
}

void test_nothing_before_the_time_sync()
{
    // A host that answers the time request late: runUntil() doesn't answer the negotiation's
    Robot robot;
    FakeClock::setMs(10);
    robot.connected_ms = 10;
    robot.nh.getHardware()->negotiation();
    robot.nh.spinOnce();
    robot.time_requests = robot.nh.getHardware()->count(rosserial_msgs::TopicInfo::ID_TIME);
    TEST_ASSERT_EQUAL(0, robot.runUntil(2000));
    TEST_ASSERT_EQUAL(Bno055::READY, robot.imu.state());
    robot.nh.getHardware()->timeReply(HOST_SEC);
    TEST_ASSERT_EQUAL(1000 / Publisher::PERIOD_MS, robot.runUntil(3000));
}

void test_the_rate_doesnt_depend_on_ros_time()
{
    // The Teensy reconnects to a host whose clock is 100 s earlier: ROS time jumps back, the
    // samples go on at their rate, with the new time in their stamps
    Robot& robot = *startedRobot();
    robot.runUntil(3000);
    robot.connect(3001, HOST_SEC - 100);
    TEST_ASSERT_TRUE(robot.nh.timeSynced());
    TEST_ASSERT_EQUAL(1000 / Publisher::PERIOD_MS, robot.runUntil(4001));
    TEST_ASSERT_EQUAL_UINT32(HOST_SEC - 100, robot.publisher.message().header.stamp.sec);
}

void test_nothing_between_a_new_negotiation_and_its_reply()
{
    // The host's rosserial node restarts right after a periodic time request, in the same
    // millisecond: nothing is published until the new connection's reply
    Robot& robot = *startedRobot();
    int requests = robot.nh.getHardware()->count(rosserial_msgs::TopicInfo::ID_TIME);
    robot.answer_time_requests = false;
    while (robot.nh.getHardware()->count(rosserial_msgs::TopicInfo::ID_TIME) == requests)
        robot.runUntil(FakeClock::millis() + 1);
    uint32_t ms = FakeClock::millis();
    robot.nh.getHardware()->negotiation();
    robot.nh.spinOnce();
    TEST_ASSERT_EQUAL(0, robot.runUntil(ms + 200));
    robot.host_sec = HOST_SEC - 100;
    robot.connected_ms = ms;
    robot.nh.getHardware()->timeReply(robot.host_sec);
    TEST_ASSERT_EQUAL(1000 / Publisher::PERIOD_MS, robot.runUntil(ms + 1200));
    TEST_ASSERT_EQUAL_UINT32(HOST_SEC - 100 + 1, robot.publisher.message().header.stamp.sec);
}

void test_stamp_before_the_read_and_read_budget()
{
    Robot& robot = *startedRobot();
    read_duration_us = IMU_READ_BUDGET_US;
    TEST_ASSERT_EQUAL(1, robot.runUntil(2000 + Publisher::PERIOD_MS));
    // Stamped when the read began, not when it ended: ROS time is the host's time at the reply
    // (at 10 ms) plus the time since then
    ros::Time stamp = robot.publisher.message().header.stamp;
    uint32_t since_reply_ms = 2000 + Publisher::PERIOD_MS - 10;
    TEST_ASSERT_EQUAL_UINT32(HOST_SEC + since_reply_ms / 1000, stamp.sec);
    TEST_ASSERT_EQUAL_UINT32((since_reply_ms % 1000) * 1000000UL, stamp.nsec);
    read_duration_us = IMU_READ_BUDGET_US + 1;
    TEST_ASSERT_EQUAL(0, robot.runUntil(2100));
}

void test_failed_reads_restart_the_sensor()
{
    Robot& robot = *startedRobot();
    robot.bus.fail_reads = true;
    const uint32_t tenth_read_ms = 2000 + Publisher::RESTART_AFTER_FAILED_READS * Publisher::PERIOD_MS;
    robot.runUntil(tenth_read_ms - 1);
    TEST_ASSERT_EQUAL(Bno055::READY, robot.imu.state());
    robot.runUntil(tenth_read_ms);
    TEST_ASSERT_EQUAL(Bno055::STARTING, robot.imu.state());
    // It doesn't answer: tried again after IMU_RETRY_PERIOD, and found once it's back
    robot.runUntil(tenth_read_ms + 1);
    TEST_ASSERT_EQUAL(Bno055::ABSENT, robot.imu.state());
    robot.bus.fail_reads = false;
    const uint32_t retry_ms = tenth_read_ms + 1 + IMU_RETRY_PERIOD * 1000UL;
    robot.runUntil(retry_ms - 1);
    TEST_ASSERT_EQUAL(Bno055::ABSENT, robot.imu.state());
    robot.runUntil(retry_ms + 1000);
    TEST_ASSERT_EQUAL(Bno055::READY, robot.imu.state());
    TEST_ASSERT_TRUE(robot.runUntil(retry_ms + 2000) > 0);
}

void test_nothing_while_the_fusion_doesnt_run()
{
    Robot& robot = *startedRobot();
    robot.bus.registers[Bno055::REG_SYS_STATUS] = 0x03; // initializing
    TEST_ASSERT_EQUAL(0, robot.runUntil(2100));
}

void test_disconnect_stops_the_topic()
{
    Robot& robot = *startedRobot();
    robot.nh.getHardware()->receive(rosserial_msgs::TopicInfo::ID_TX_STOP);
    TEST_ASSERT_EQUAL(0, robot.runUntil(3000));
}

int main(int argc, char** argv)
{
    UNITY_BEGIN();
    RUN_TEST(test_publishes_at_the_rate);
    RUN_TEST(test_nothing_before_the_time_sync);
    RUN_TEST(test_the_rate_doesnt_depend_on_ros_time);
    RUN_TEST(test_nothing_between_a_new_negotiation_and_its_reply);
    RUN_TEST(test_stamp_before_the_read_and_read_budget);
    RUN_TEST(test_failed_reads_restart_the_sensor);
    RUN_TEST(test_nothing_while_the_fusion_doesnt_run);
    RUN_TEST(test_disconnect_stops_the_topic);
    return UNITY_END();
}
