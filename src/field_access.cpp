#include "field_access.hpp"

#include "rclcpp/serialization.hpp"
#include "rclcpp/typesupport_helpers.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"

#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace semantic_gateway {

using IntrospectionMembers = rosidl_typesupport_introspection_cpp::MessageMembers;
using IntrospectionMember  = rosidl_typesupport_introspection_cpp::MessageMember;
namespace ti = rosidl_typesupport_introspection_cpp;

namespace {

std::vector<std::string> split_path(const std::string & p)
{
    std::vector<std::string> out;
    std::stringstream ss(p);
    std::string tok;
    while (std::getline(ss, tok, '.')) out.push_back(tok);
    return out;
}

const IntrospectionMember * find_member(
    const IntrospectionMembers * members, const std::string & name)
{
    for (uint32_t i = 0; i < members->member_count_; ++i) {
        if (name == members->members_[i].name_) {
            return &members->members_[i];
        }
    }
    return nullptr;
}

double scalar_to_double(const void * p, uint8_t type_id)
{
    using namespace rosidl_typesupport_introspection_cpp;
    switch (type_id) {
    case ROS_TYPE_FLOAT:    return *static_cast<const float    *>(p);
    case ROS_TYPE_DOUBLE:   return *static_cast<const double   *>(p);
    case ROS_TYPE_INT8:     return *static_cast<const int8_t   *>(p);
    case ROS_TYPE_UINT8:    return *static_cast<const uint8_t  *>(p);
    case ROS_TYPE_OCTET:    return *static_cast<const uint8_t  *>(p);
    case ROS_TYPE_INT16:    return *static_cast<const int16_t  *>(p);
    case ROS_TYPE_UINT16:   return *static_cast<const uint16_t *>(p);
    case ROS_TYPE_INT32:    return *static_cast<const int32_t  *>(p);
    case ROS_TYPE_UINT32:   return *static_cast<const uint32_t *>(p);
    case ROS_TYPE_INT64:    return static_cast<double>(*static_cast<const int64_t *>(p));
    case ROS_TYPE_UINT64:   return static_cast<double>(*static_cast<const uint64_t*>(p));
    case ROS_TYPE_BOOLEAN:  return *static_cast<const bool     *>(p) ? 1.0 : 0.0;
    default:
        throw std::runtime_error("scalar_to_double: unsupported type_id "
                                 + std::to_string(type_id));
    }
}

}  // namespace

MessageView::MessageView(const std::string & type_id) : type_id_(type_id)
{
    ts_library_ = rclcpp::get_typesupport_library(
        type_id_, "rosidl_typesupport_cpp");
    ts_ = rclcpp::get_typesupport_handle(
        type_id_, "rosidl_typesupport_cpp", *ts_library_);
    if (!ts_) {
        throw std::runtime_error("typesupport not found for: " + type_id_);
    }

    intro_library_ = rclcpp::get_typesupport_library(
        type_id_, "rosidl_typesupport_introspection_cpp");
    const auto intro_ts = rclcpp::get_typesupport_handle(
        type_id_, "rosidl_typesupport_introspection_cpp", *intro_library_);
    if (!intro_ts) {
        throw std::runtime_error("introspection typesupport not found for: " + type_id_);
    }
    members_ = static_cast<const IntrospectionMembers *>(intro_ts->data);

    buf_.resize(members_->size_of_);
    members_->init_function(buf_.data(), rosidl_runtime_cpp::MessageInitialization::ALL);
    initialized_ = true;
}

MessageView::~MessageView()
{
    if (initialized_ && members_) {
        members_->fini_function(buf_.data());
    }
}

void MessageView::deserialize(const rclcpp::SerializedMessage & serialized)
{
    rclcpp::SerializationBase ser(ts_);
    ser.deserialize_message(&serialized, buf_.data());
}

MessageView::Resolved MessageView::resolve(const std::string & dotted_path) const
{
    const auto parts = split_path(dotted_path);
    if (parts.empty()) {
        throw std::runtime_error("empty dotted path");
    }

    const IntrospectionMembers * members = members_;
    const void * ptr = buf_.data();

    for (std::size_t i = 0; i < parts.size(); ++i) {
        const auto * m = find_member(members, parts[i]);
        if (!m) {
            throw std::runtime_error(
                "field '" + parts[i] + "' not found in '" + type_id_
                + "' (path: " + dotted_path + ")");
        }
        const void * field_ptr =
            static_cast<const uint8_t *>(ptr) + m->offset_;

        if (i + 1 == parts.size()) {
            return Resolved{m, field_ptr};
        }

        if (m->type_id_ != ti::ROS_TYPE_MESSAGE) {
            throw std::runtime_error(
                "field '" + parts[i] + "' is not a message — cannot descend "
                "(path: " + dotted_path + ")");
        }
        members = static_cast<const IntrospectionMembers *>(m->members_->data);
        ptr     = field_ptr;
    }

    throw std::runtime_error("unreachable in resolve()");
}

double MessageView::read_double(const std::string & dotted_path) const
{
    const Resolved r = resolve(dotted_path);
    if (r.member->is_array_) {
        throw std::runtime_error(
            "read_double: '" + dotted_path + "' is an array; use read_array_double");
    }
    return scalar_to_double(r.obj_ptr, r.member->type_id_);
}

std::string MessageView::read_string(const std::string & dotted_path) const
{
    const Resolved r = resolve(dotted_path);
    if (r.member->type_id_ != ti::ROS_TYPE_STRING) {
        throw std::runtime_error(
            "read_string: '" + dotted_path + "' is not a string");
    }
    return *static_cast<const std::string *>(r.obj_ptr);
}

uint64_t MessageView::read_time_us(const std::string & dotted_path) const
{
    const Resolved r = resolve(dotted_path);
    if (r.member->type_id_ != ti::ROS_TYPE_MESSAGE || !r.member->members_) {
        throw std::runtime_error(
            "read_time_us: '" + dotted_path + "' is not a message (Time)");
    }
    const auto * sub_members =
        static_cast<const IntrospectionMembers *>(r.member->members_->data);
    const auto * sec_m     = find_member(sub_members, "sec");
    const auto * nanosec_m = find_member(sub_members, "nanosec");
    if (!sec_m || !nanosec_m) {
        throw std::runtime_error(
            "read_time_us: '" + dotted_path
            + "' submessage has no sec/nanosec fields (expected Time)");
    }
    const auto sec_ptr =
        static_cast<const uint8_t *>(r.obj_ptr) + sec_m->offset_;
    const auto nanosec_ptr =
        static_cast<const uint8_t *>(r.obj_ptr) + nanosec_m->offset_;
    const int32_t  sec     = *reinterpret_cast<const int32_t  *>(sec_ptr);
    const uint32_t nanosec = *reinterpret_cast<const uint32_t *>(nanosec_ptr);
    return static_cast<uint64_t>(sec) * 1'000'000ULL
         + static_cast<uint64_t>(nanosec) / 1'000ULL;
}

std::size_t MessageView::array_size(const std::string & dotted_path) const
{
    const Resolved r = resolve(dotted_path);
    if (!r.member->is_array_) {
        throw std::runtime_error(
            "array_size: '" + dotted_path + "' is not an array");
    }
    if (r.member->array_size_ != 0 && !r.member->is_upper_bound_) {
        return r.member->array_size_;          // fixed-size C array
    }
    if (!r.member->size_function) {
        throw std::runtime_error(
            "array_size: '" + dotted_path
            + "' lacks size_function (cannot inspect dynamic length)");
    }
    return r.member->size_function(r.obj_ptr);
}

double MessageView::read_array_double(
    const std::string & dotted_path, std::size_t index) const
{
    const Resolved r = resolve(dotted_path);
    if (!r.member->is_array_) {
        throw std::runtime_error(
            "read_array_double: '" + dotted_path + "' is not an array");
    }
    const void * elem = nullptr;
    if (r.member->array_size_ != 0 && !r.member->is_upper_bound_) {
        // Fixed-size C array — compute pointer manually.
        // The element size depends on type_id_; for the primitive types we
        // support, indexing as (raw + i*sizeof(T)) works because the array
        // is contiguous in the same struct.
        const auto element_size = [&]() -> std::size_t {
            using namespace rosidl_typesupport_introspection_cpp;
            switch (r.member->type_id_) {
            case ROS_TYPE_FLOAT:   return sizeof(float);
            case ROS_TYPE_DOUBLE:  return sizeof(double);
            case ROS_TYPE_INT8:
            case ROS_TYPE_UINT8:
            case ROS_TYPE_OCTET:
            case ROS_TYPE_BOOLEAN: return 1;
            case ROS_TYPE_INT16:
            case ROS_TYPE_UINT16:  return 2;
            case ROS_TYPE_INT32:
            case ROS_TYPE_UINT32:  return 4;
            case ROS_TYPE_INT64:
            case ROS_TYPE_UINT64:  return 8;
            default:
                throw std::runtime_error(
                    "read_array_double: fixed-array of unsupported type_id");
            }
        }();
        elem = static_cast<const uint8_t *>(r.obj_ptr) + index * element_size;
    } else {
        if (!r.member->get_const_function) {
            throw std::runtime_error(
                "read_array_double: '" + dotted_path
                + "' lacks get_const_function");
        }
        elem = r.member->get_const_function(r.obj_ptr, index);
    }
    return scalar_to_double(elem, r.member->type_id_);
}

}  // namespace semantic_gateway
