#pragma once

// field_access.hpp — runtime introspection of ROS 2 messages by dotted path
//
// wraps `rosidl_typesupport_introspection_cpp` so the gateway can deserialize
// a `rclcpp::SerializedMessage` into a typed buffer at runtime, without
// knowing the message type at compile time, and then read fields by their
// dotted YAML names (e.g. "header.stamp")
//
// this is the mechanism that makes the gateway sensor-agnostic.

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/serialized_message.hpp"
#include "rcpputils/shared_library.hpp"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"

namespace semantic_gateway {

class MessageView
{
public:
    // type_id like "sensor_msgs/msg/LaserScan"
    explicit MessageView(const std::string & type_id);
    ~MessageView();

    MessageView(const MessageView &) = delete;
    MessageView & operator=(const MessageView &) = delete;

    void deserialize(const rclcpp::SerializedMessage & serialized);

    // scalar reads
    double      read_double (const std::string & dotted_path) const;
    std::string read_string (const std::string & dotted_path) const;
    // sec*1e6 + nanosec/1000 (µs since UNIX epoch)
    uint64_t    read_time_us(const std::string & dotted_path) const;

    // array reads
    std::size_t array_size       (const std::string & dotted_path) const;
    double      read_array_double(const std::string & dotted_path,
                                   std::size_t index) const;

private:
    struct Resolved {
        const rosidl_typesupport_introspection_cpp::MessageMember * member;
        const void * obj_ptr;  // pointer to the field's storage
    };

    Resolved resolve(const std::string & dotted_path) const;

    std::string type_id_;
    std::shared_ptr<rcpputils::SharedLibrary> ts_library_;
    std::shared_ptr<rcpputils::SharedLibrary> intro_library_;
    const rosidl_message_type_support_t * ts_{nullptr};
    const rosidl_typesupport_introspection_cpp::MessageMembers * members_{nullptr};
    std::vector<uint8_t> buf_;
    bool initialized_{false};
};

}  // namespace semantic_gateway
