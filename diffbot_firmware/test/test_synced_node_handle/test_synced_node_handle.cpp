// Unit tests of SyncedNodeHandleT (lib/imu/synced_node_handle.h) with rosserial's real parser, on
// the PC: pio test -e native
#include <unity.h>

#include "../fakes/fake_rosserial.h"
#include "synced_node_handle.h"

uint64_t FakeClock::us = 0;

typedef diffbot::SyncedNodeHandleT<ros::NodeHandle_<FakeSerial>> NodeHandle;

const uint32_t HOST_SEC = 1700000000;

// A node handle that the host has just connected to at the given time, before the time reply
void connect(NodeHandle& nh, uint32_t ms)
{
    FakeClock::setMs(ms);
    nh.getHardware()->negotiation();
    nh.spinOnce();
}

void reply(NodeHandle& nh, uint32_t ms, uint32_t sec = HOST_SEC)
{
    FakeClock::setMs(ms);
    nh.getHardware()->timeReply(sec);
    nh.spinOnce();
}

void setUp() { FakeClock::setMs(0); }
void tearDown() {}

void test_synced_node_handle_is_not_synced_before_the_time_reply()
{
    NodeHandle nh;
    nh.initNode();
    TEST_ASSERT_FALSE(nh.timeSynced());
    connect(nh, 1000);
    TEST_ASSERT_TRUE(nh.connected());
    TEST_ASSERT_FALSE(nh.timeSynced());
    reply(nh, 1005);
    TEST_ASSERT_TRUE(nh.timeSynced());
    TEST_ASSERT_EQUAL_UINT32(HOST_SEC, nh.now().sec);
}

void test_synced_node_handle_counts_a_reply_in_the_negotiations_millisecond()
{
    // rosserial counts in milliseconds; a reply in the negotiation's millisecond leaves its
    // counters equal, but the clock is set
    NodeHandle nh;
    nh.initNode();
    connect(nh, 1000);
    reply(nh, 1000);
    TEST_ASSERT_TRUE(nh.timeSynced());
}

void test_synced_node_handle_stays_synced_while_periodic_replies_arrive()
{
    NodeHandle nh;
    nh.initNode();
    connect(nh, 1000);
    reply(nh, 1001);
    // rosserial asks for the time every 2.5 s. Between a request and its reply, the clock still
    // runs on the previous reply and stays synchronized; each reply here comes in its request's
    // millisecond
    int requests = nh.getHardware()->count(rosserial_msgs::TopicInfo::ID_TIME);
    int replies = 0;
    for (uint32_t ms = 1002; ms <= 20000; ++ms)
    {
        FakeClock::setMs(ms);
        nh.spinOnce();
        TEST_ASSERT_TRUE(nh.timeSynced());
        if (nh.getHardware()->count(rosserial_msgs::TopicInfo::ID_TIME) > requests)
        {
            requests = nh.getHardware()->count(rosserial_msgs::TopicInfo::ID_TIME);
            reply(nh, ms, HOST_SEC + ms / 1000);
            TEST_ASSERT_TRUE(nh.timeSynced());
            ++replies;
        }
    }
    TEST_ASSERT_EQUAL(7, replies);
}

void test_synced_node_handle_is_not_synced_after_a_new_negotiation_until_its_reply()
{
    NodeHandle nh;
    nh.initNode();
    connect(nh, 1000);
    reply(nh, 1001);
    connect(nh, 3000); // the host's rosserial node restarted
    TEST_ASSERT_FALSE(nh.timeSynced());
    FakeClock::setMs(3001);
    nh.spinOnce();
    TEST_ASSERT_FALSE(nh.timeSynced());
    reply(nh, 3002, HOST_SEC - 100); // a host whose clock is earlier
    TEST_ASSERT_TRUE(nh.timeSynced());
    TEST_ASSERT_EQUAL_UINT32(HOST_SEC - 100, nh.now().sec);
}

void test_synced_node_handle_ignores_a_reply_that_arrives_before_a_negotiation()
{
    // Both frames arrive in one spinOnce(): the reply belongs to the old connection
    NodeHandle nh;
    nh.initNode();
    connect(nh, 1000);
    reply(nh, 1001);
    FakeClock::setMs(2000);
    nh.getHardware()->timeReply(HOST_SEC + 1);
    nh.getHardware()->negotiation();
    nh.spinOnce();
    TEST_ASSERT_FALSE(nh.timeSynced());
}

void test_synced_node_handle_is_not_synced_after_a_negotiation_in_a_requests_millisecond()
{
    // A periodic request and then a new negotiation in the same millisecond, in two calls: the
    // negotiation still ends the sync
    NodeHandle nh;
    nh.initNode();
    connect(nh, 1000);
    reply(nh, 1001);
    int requests = nh.getHardware()->count(rosserial_msgs::TopicInfo::ID_TIME);
    uint32_t ms = 1001;
    while (nh.getHardware()->count(rosserial_msgs::TopicInfo::ID_TIME) == requests)
    {
        FakeClock::setMs(++ms);
        nh.spinOnce();
    }
    TEST_ASSERT_TRUE(nh.timeSynced());
    nh.getHardware()->negotiation();
    nh.spinOnce();
    TEST_ASSERT_TRUE(nh.connected());
    TEST_ASSERT_FALSE(nh.timeSynced());
    reply(nh, ms + 1);
    TEST_ASSERT_TRUE(nh.timeSynced());
}

void test_synced_node_handle_counts_a_reply_that_leaves_the_clock_unchanged()
{
    // After a reconnect, the host's reply can leave the clock offsets exactly as they were
    NodeHandle nh;
    nh.initNode();
    connect(nh, 1000);
    reply(nh, 1001); // the host's clock: HOST_SEC at 1000 ms
    connect(nh, 2000);
    TEST_ASSERT_FALSE(nh.timeSynced());
    reply(nh, 2001, HOST_SEC + 1); // the same clock: HOST_SEC + 1 s at 2000 ms
    TEST_ASSERT_TRUE(nh.timeSynced());
    TEST_ASSERT_EQUAL_UINT32(HOST_SEC + 1, nh.now().sec);
}

void test_synced_node_handle_is_not_synced_after_a_disconnect()
{
    NodeHandle nh;
    nh.initNode();
    connect(nh, 1000);
    reply(nh, 1001);
    nh.getHardware()->receive(rosserial_msgs::TopicInfo::ID_TX_STOP);
    nh.spinOnce();
    TEST_ASSERT_FALSE(nh.connected());
    TEST_ASSERT_FALSE(nh.timeSynced());
}

void test_synced_node_handle_is_not_synced_after_11_s_without_a_reply()
{
    // rosserial drops the connection 11 s after the last reply
    NodeHandle nh;
    nh.initNode();
    connect(nh, 1000);
    reply(nh, 1001);
    FakeClock::setMs(1001 + 11000);
    nh.spinOnce();
    TEST_ASSERT_TRUE(nh.timeSynced());
    FakeClock::setMs(1001 + 11001);
    nh.spinOnce();
    TEST_ASSERT_FALSE(nh.connected());
    TEST_ASSERT_FALSE(nh.timeSynced());
}

void test_synced_node_handle_counts_a_reply_across_the_millis_wrap()
{
    NodeHandle nh;
    nh.initNode();
    connect(nh, 0xFFFFFFFEu);
    reply(nh, 1);
    TEST_ASSERT_TRUE(nh.timeSynced());
}

int main(int argc, char** argv)
{
    UNITY_BEGIN();
    RUN_TEST(test_synced_node_handle_is_not_synced_before_the_time_reply);
    RUN_TEST(test_synced_node_handle_counts_a_reply_in_the_negotiations_millisecond);
    RUN_TEST(test_synced_node_handle_stays_synced_while_periodic_replies_arrive);
    RUN_TEST(test_synced_node_handle_is_not_synced_after_a_new_negotiation_until_its_reply);
    RUN_TEST(test_synced_node_handle_ignores_a_reply_that_arrives_before_a_negotiation);
    RUN_TEST(test_synced_node_handle_is_not_synced_after_a_negotiation_in_a_requests_millisecond);
    RUN_TEST(test_synced_node_handle_counts_a_reply_that_leaves_the_clock_unchanged);
    RUN_TEST(test_synced_node_handle_is_not_synced_after_a_disconnect);
    RUN_TEST(test_synced_node_handle_is_not_synced_after_11_s_without_a_reply);
    RUN_TEST(test_synced_node_handle_counts_a_reply_across_the_millis_wrap);
    return UNITY_END();
}
