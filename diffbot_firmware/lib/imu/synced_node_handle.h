#ifndef DIFFBOT_SYNCED_NODE_HANDLE_H
#define DIFFBOT_SYNCED_NODE_HANDLE_H

#include <stdint.h>

#include <ros/node_handle.h>

namespace diffbot
{
    /**
     * @brief rosserial's node handle, plus whether its clock is synchronized with the host for the
     * current connection.
     *
     * connected() turns true when the host has negotiated the topics, before it answers the time
     * request that comes with the negotiation; until that answer, now() is based on the Teensy's
     * uptime, not on ROS time. A message stamped then would carry a wrong time.
     *
     * rosserial (0.9.1) has no events for a negotiation or a time reply, so this class checks
     * after each spinOnce() what they leave behind:
     * - A negotiation makes spinOnce() return SPIN_ERR, its only use of that value, right after
     *   the negotiation. Not synchronized, until the next reply; a reply processed earlier in the
     *   same call belongs to the old connection.
     * - A time reply sets last_sync_receive_time to the time it arrived and the clock offsets
     *   (sec_offset, nsec_offset) from the host's time. Either one changing means a reply: the
     *   receive time misses a reply in the same millisecond as the last one, the offsets miss a
     *   reply that keeps the clock where it was.
     * - Not connected (the host stopped, or no reply for 11 s): not synchronized.
     *
     * @tparam NodeHandle rosserial's NodeHandle_ (ros::NodeHandle on the Teensy; the unit tests use
     * one with a fake serial port)
     */
    template <class NodeHandle>
    class SyncedNodeHandleT : public NodeHandle
    {
    public:
        int spinOnce() override
        {
            int result = NodeHandle::spinOnce();
            bool replied = this->last_sync_receive_time != seen_receive_time_
                || this->sec_offset != seen_sec_offset_ || this->nsec_offset != seen_nsec_offset_;
            seen_receive_time_ = this->last_sync_receive_time;
            seen_sec_offset_ = this->sec_offset;
            seen_nsec_offset_ = this->nsec_offset;
            if (!this->connected() || result == ros::SPIN_ERR)
                time_synced_ = false;
            else if (replied)
                time_synced_ = true;
            return result;
        }

        bool timeSynced() const { return time_synced_; }

    private:
        bool time_synced_ = false;
        uint32_t seen_receive_time_ = 0;
        uint32_t seen_sec_offset_ = 0;
        uint32_t seen_nsec_offset_ = 0;
    };
}

#endif // DIFFBOT_SYNCED_NODE_HANDLE_H
