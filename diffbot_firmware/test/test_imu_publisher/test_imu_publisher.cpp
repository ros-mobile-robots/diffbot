// Unit tests of ImuPublisher (lib/imu/imu_publisher.h) on the PC: the real driver and rosserial's
// real parser, with a fake BNO055, serial port and clock. Run them with:
//   pio test -e native
#include <unity.h>

#include "../fakes/fake_bno055.h"
#include "../fakes/fake_rosserial.h"
#include "imu_config.h"
#include "imu_publisher.h"
#include "synced_node_handle.h"

using namespace diffbot;

uint64_t FakeClock::us = 0;

typedef SyncedNodeHandleT<ros::NodeHandle_<FakeSerial>> NodeHandle;
typedef ImuPublisher<NodeHandle, FakeClock> Publisher;

const uint32_t HOST_SEC = 1700000000;

// The publisher's settings in these tests, different from the robot's (diffbot_base_config.h), so
// that a value the publisher takes from elsewhere fails a test
const ImuPublisherConfig TEST_CONFIG = {true, 25, 7000, 12000, "test_imu", 1.0e-3f, 2.0e-2f};
const int SAMPLES_PER_SECOND = 1000 / TEST_CONFIG.period_ms;

// How long each data read takes, in µs of the fake clock. Shared, as the fake clock is, so setUp()
// resets it
uint32_t read_duration_us = 0;
void advanceDuringRead() { FakeClock::us += read_duration_us; }

// One test's robot: the node handle, the fake sensor, the driver and the publisher. Each test
// makes its own, so nothing carries over from one test to the next
struct Robot
{
    NodeHandle nh;
    FakeBno055Bus bus;
    Bno055 imu;
    Publisher publisher;

    explicit Robot(const ImuPublisherConfig& config = TEST_CONFIG)
        : imu(bus), publisher(nh, imu, config)
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

    /// Connects at 10 ms and runs until 2000 ms: then the IMU has started and the clock is
    /// synchronized
    void startReadyAndSynced()
    {
        connect(10);
        runUntil(2000);
        TEST_ASSERT_EQUAL(Bno055::READY, imu.state());
        TEST_ASSERT_TRUE(nh.timeSynced());
    }
};

void setUp()
{
    FakeClock::setMs(0);
    read_duration_us = 0;
}
void tearDown() {}

void test_imu_publisher_publishes_at_the_configured_rate()
{
    Robot robot;
    robot.startReadyAndSynced();
    TEST_ASSERT_EQUAL(SAMPLES_PER_SECOND, robot.runUntil(3000));
    const sensor_msgs::Imu& message = robot.publisher.message();
    TEST_ASSERT_EQUAL_STRING(TEST_CONFIG.frame_id, message.header.frame_id);
    TEST_ASSERT_EQUAL_FLOAT(-1.0f, message.orientation_covariance[0]);
    TEST_ASSERT_EQUAL_FLOAT(TEST_CONFIG.angular_velocity_variance,
                            message.angular_velocity_covariance[8]);
    TEST_ASSERT_EQUAL_FLOAT(TEST_CONFIG.linear_acceleration_variance,
                            message.linear_acceleration_covariance[8]);
    TEST_ASSERT_EQUAL_UINT32(HOST_SEC + 2, message.header.stamp.sec);
}

void test_imu_publisher_publishes_nothing_before_the_time_sync()
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
    TEST_ASSERT_EQUAL(SAMPLES_PER_SECOND, robot.runUntil(3000));
}

void test_imu_publisher_keeps_its_rate_when_ros_time_jumps()
{
    // The Teensy reconnects to a host whose clock is 100 s earlier: ROS time jumps back, the
    // samples go on at their rate, with the new time in their stamps
    Robot robot;
    robot.startReadyAndSynced();
    robot.runUntil(3000);
    robot.connect(3001, HOST_SEC - 100);
    TEST_ASSERT_TRUE(robot.nh.timeSynced());
    TEST_ASSERT_EQUAL(SAMPLES_PER_SECOND, robot.runUntil(4001));
    TEST_ASSERT_EQUAL_UINT32(HOST_SEC - 100, robot.publisher.message().header.stamp.sec);
}

void test_imu_publisher_publishes_nothing_between_a_new_negotiation_and_its_reply()
{
    // The host's rosserial node restarts right after a periodic time request, in the same
    // millisecond: nothing is published until the new connection's reply
    Robot robot;
    robot.startReadyAndSynced();
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
    TEST_ASSERT_EQUAL(SAMPLES_PER_SECOND, robot.runUntil(ms + 1200));
    TEST_ASSERT_EQUAL_UINT32(HOST_SEC - 100 + 1, robot.publisher.message().header.stamp.sec);
}

void test_imu_publisher_stamps_the_sample_when_the_read_begins()
{
    Robot robot;
    robot.startReadyAndSynced();
    read_duration_us = TEST_CONFIG.read_budget_us;
    TEST_ASSERT_EQUAL(1, robot.runUntil(2000 + TEST_CONFIG.period_ms));
    // Stamped when the read began, not when it ended: ROS time is the host's time at the reply
    // (at 10 ms) plus the time since then
    ros::Time stamp = robot.publisher.message().header.stamp;
    uint32_t since_reply_ms = 2000 + TEST_CONFIG.period_ms - 10;
    TEST_ASSERT_EQUAL_UINT32(HOST_SEC + since_reply_ms / 1000, stamp.sec);
    TEST_ASSERT_EQUAL_UINT32((since_reply_ms % 1000) * 1000000UL, stamp.nsec);
}

void test_imu_publisher_drops_a_sample_whose_read_exceeds_the_budget()
{
    Robot robot;
    robot.startReadyAndSynced();
    read_duration_us = TEST_CONFIG.read_budget_us + 1;
    TEST_ASSERT_EQUAL(0, robot.runUntil(2100));
    read_duration_us = TEST_CONFIG.read_budget_us;
    TEST_ASSERT_TRUE(robot.runUntil(2200) > 0);
}

void test_imu_publisher_restarts_the_sensor_after_ten_failed_reads()
{
    Robot robot;
    robot.startReadyAndSynced();
    robot.bus.fail_reads = true;
    const uint32_t tenth_read_ms =
        2000 + Publisher::RESTART_AFTER_FAILED_READS * TEST_CONFIG.period_ms;
    robot.runUntil(tenth_read_ms - 1);
    TEST_ASSERT_EQUAL(Bno055::READY, robot.imu.state());
    robot.runUntil(tenth_read_ms);
    TEST_ASSERT_EQUAL(Bno055::STARTING, robot.imu.state());
    // It doesn't answer: tried again after retry_ms, and found once it's back
    robot.runUntil(tenth_read_ms + 1);
    TEST_ASSERT_EQUAL(Bno055::ABSENT, robot.imu.state());
    robot.bus.fail_reads = false;
    const uint32_t retry_ms = tenth_read_ms + 1 + TEST_CONFIG.retry_ms;
    robot.runUntil(retry_ms - 1);
    TEST_ASSERT_EQUAL(Bno055::ABSENT, robot.imu.state());
    robot.runUntil(retry_ms + 1000);
    TEST_ASSERT_EQUAL(Bno055::READY, robot.imu.state());
    TEST_ASSERT_TRUE(robot.runUntil(retry_ms + 2000) > 0);
}

void test_imu_publisher_publishes_nothing_while_the_fusion_doesnt_run()
{
    Robot robot;
    robot.startReadyAndSynced();
    robot.bus.registers[Bno055::REG_SYS_STATUS] = 0x03; // initializing
    TEST_ASSERT_EQUAL(0, robot.runUntil(2100));
}

void test_imu_publisher_publishes_nothing_after_a_disconnect()
{
    Robot robot;
    robot.startReadyAndSynced();
    robot.nh.getHardware()->receive(rosserial_msgs::TopicInfo::ID_TX_STOP);
    TEST_ASSERT_EQUAL(0, robot.runUntil(3000));
}

void test_imu_publisher_disabled_advertises_nothing_and_leaves_the_sensor_alone()
{
    ImuPublisherConfig disabled = TEST_CONFIG;
    disabled.enabled = false;
    Robot robot(disabled);
    robot.connect(10);
    TEST_ASSERT_EQUAL(0, robot.nh.getHardware()->count(rosserial_msgs::TopicInfo::ID_PUBLISHER));
    TEST_ASSERT_EQUAL(0, robot.runUntil(3000));
    TEST_ASSERT_EQUAL(Bno055::STARTING, robot.imu.state());
    TEST_ASSERT_EQUAL(0, robot.bus.transfers);
}

int main(int argc, char** argv)
{
    UNITY_BEGIN();
    RUN_TEST(test_imu_publisher_publishes_at_the_configured_rate);
    RUN_TEST(test_imu_publisher_publishes_nothing_before_the_time_sync);
    RUN_TEST(test_imu_publisher_keeps_its_rate_when_ros_time_jumps);
    RUN_TEST(test_imu_publisher_publishes_nothing_between_a_new_negotiation_and_its_reply);
    RUN_TEST(test_imu_publisher_stamps_the_sample_when_the_read_begins);
    RUN_TEST(test_imu_publisher_drops_a_sample_whose_read_exceeds_the_budget);
    RUN_TEST(test_imu_publisher_restarts_the_sensor_after_ten_failed_reads);
    RUN_TEST(test_imu_publisher_publishes_nothing_while_the_fusion_doesnt_run);
    RUN_TEST(test_imu_publisher_publishes_nothing_after_a_disconnect);
    RUN_TEST(test_imu_publisher_disabled_advertises_nothing_and_leaves_the_sensor_alone);
    return UNITY_END();
}
