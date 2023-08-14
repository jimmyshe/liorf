//
// Created by jimmy on 23-8-11.
//

#pragma once

#include <boost/container/flat_map.hpp>
#include <boost/noncopyable.hpp>
#include <boost/signals2.hpp>
#include <cinttypes>
#include <mutex>
#include <optional>
#include <queue>
#include <vector>
template<class Message>
class MsgBuffer : boost::noncopyable {

public:
    using MessageConstPtr = typename std::shared_ptr<const Message>;
    using MessagePtrVector = std::vector<MessageConstPtr>;
    using MessagePtrOption = std::optional<MessageConstPtr>;

    using MessagePtrQueueAsFlatMap = boost::container::flat_map<int64_t, MessageConstPtr>;
    const uint32_t queue_size_ = 1E4;
    explicit MsgBuffer(const uint32_t queueSize) : queue_size_(queueSize) {}

    void put_in(int64_t time, const Message &msg) {
        auto msg_ptr = std::make_shared<Message>(msg);
        put_in(time, msg_ptr);
    }

    void put_in(int64_t time, const MessageConstPtr &msg_ptr) {
        message_queue_.emplace(time, msg_ptr);

        // make sure the queue size is not too large
        while (message_queue_.size() > queue_size_) {
            message_queue_.erase(message_queue_.begin());
        }
        //todo:  if new msg is newer than the last msg, clear the queue
    }

    MessagePtrVector get_between(int64_t start_time, int64_t end_time) {
        MessagePtrVector msgs;
        if (message_queue_.empty()) {
            return msgs;
        }
        auto start_itr = message_queue_.lower_bound(start_time);
        auto end_itr = message_queue_.upper_bound(end_time);
        for (auto itr = start_itr; itr != end_itr; ++itr) {
            msgs.push_back(itr->second);
        }
        return msgs;
    }


    MessagePtrOption get_msg_cloest(int64_t time, std::chrono::duration<double> tolerance) const {
        if (message_queue_.empty()) {
            return {};
        }
        auto lower_itr = message_queue_.lower_bound(time);
        if (lower_itr != message_queue_.begin()) {
            auto tolerance_in_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(tolerance).count();
            auto before_itr = std::prev(lower_itr);
            auto before_diff = std::abs(time - before_itr->first);
            auto after_diff = std::abs(time - lower_itr->first);

            if (before_diff < after_diff) {
                if (before_diff < tolerance_in_ns) {
                    return before_itr->second;
                }
            } else {
                if (after_diff < tolerance_in_ns) {
                    return lower_itr->second;
                }
            }
        }
        return {};
    }


    MessagePtrOption get_msg_before(int64_t time, std::chrono::duration<double> tolerance) const {
        if (message_queue_.empty()) {
            return {};
        }
        auto lower_itr = message_queue_.lower_bound(time);
        if (lower_itr != message_queue_.begin()) {
            auto tolerance_in_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(tolerance).count();
            auto before_itr = std::prev(lower_itr);
            auto before_diff = std::abs(time - before_itr->first);
            if (before_diff < tolerance_in_ns) {
                return before_itr->second;
            }
        }
        return {};
    }

    MessagePtrOption get_msg_after(int64_t time, std::chrono::duration<double> tolerance) const {
        if (message_queue_.empty()) {
            return {};
        }
        auto lower_itr = message_queue_.lower_bound(time);
        if (lower_itr != message_queue_.end()) {
            auto tolerance_in_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(tolerance).count();
            auto after_diff = std::abs(time - lower_itr->first);
            if (after_diff < tolerance_in_ns) {
                return lower_itr->second;
            }
        }
        return {};
    }


    size_t drop_message_after(uint64_t time) {
        auto it = message_queue_.upper_bound(time);
        auto count = std::distance(it, message_queue_.end());
        message_queue_.erase(it, message_queue_.end());
        return count;
    }

    size_t drop_message_before(uint64_t time) {
        auto it = message_queue_.lower_bound(time);
        auto count = std::distance(message_queue_.begin(), it);
        message_queue_.erase(message_queue_.begin(), it);
        return count;
    }

private:
    MessagePtrQueueAsFlatMap message_queue_;
};
