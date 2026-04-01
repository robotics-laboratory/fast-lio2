#include "utils.h"
#include <algorithm>
#include <limits>
#include "include/livox_pc2_layout.h"

namespace
{
bool MatchLivoxPc2Layout(const sensor_msgs::msg::PointCloud2 &cloud)
{
    if (cloud.fields.size() != livox_ros::kLivoxPc2FieldCount)
        return false;
    for (size_t i = 0; i < livox_ros::kLivoxPc2FieldCount; ++i)
    {
        const auto &f = cloud.fields[i];
        const auto &exp = livox_ros::kLivoxPc2Fields[i];
        if (f.name != exp.name || f.offset != exp.offset || f.datatype != exp.datatype || f.count != exp.count)
            return false;
    }
    return true;
}
} // namespace

pcl::PointCloud<pcl::PointXYZINormal>::Ptr Utils::livoxPc22PCL(const sensor_msgs::msg::PointCloud2::SharedPtr msg, int filter_num, double min_range, double max_range)
{
    if (!msg || msg->width == 0 || !MatchLivoxPc2Layout(*msg))
        return nullptr;
    if (msg->point_step < static_cast<uint32_t>(sizeof(livox_ros::LivoxPC2Point)))
        return nullptr;

    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
    const uint32_t height = std::max<uint32_t>(1U, msg->height);
    const uint64_t total_u64 = static_cast<uint64_t>(height) * static_cast<uint64_t>(msg->width);
    if (total_u64 > static_cast<uint64_t>(std::numeric_limits<int>::max()))
        return nullptr;
    const int point_num = static_cast<int>(total_u64);
    cloud->reserve(point_num / filter_num + 1);

    for (int i = 0; i < point_num; i += filter_num)
    {
        const size_t row = static_cast<size_t>(i) / static_cast<size_t>(msg->width);
        const uint32_t col = static_cast<uint32_t>(i % static_cast<int>(msg->width));
        const size_t base = row * static_cast<size_t>(msg->row_step) + static_cast<size_t>(col) * static_cast<size_t>(msg->point_step);
        if (base + sizeof(livox_ros::LivoxPC2Point) > msg->data.size())
            break;

        const auto *p = reinterpret_cast<const livox_ros::LivoxPC2Point *>(msg->data.data() + base);
        if ((p->line < 4) && (((p->tag & 0x30) == 0x10) || ((p->tag & 0x30) == 0x00)))
        {
            float x = p->x;
            float y = p->y;
            float z = p->z;
            if (x * x + y * y + z * z < min_range * min_range || x * x + y * y + z * z > max_range * max_range)
                continue;
            pcl::PointXYZINormal pt;
            pt.x = x;
            pt.y = y;
            pt.z = z;
            pt.intensity = static_cast<float>(p->intensity);
            pt.curvature = static_cast<float>(p->time) / 1000000.0f;
            cloud->push_back(pt);
        }
    }
    return cloud;
}

double Utils::getSec(const std_msgs::msg::Header &header)
{
    return static_cast<double>(header.stamp.sec) + static_cast<double>(header.stamp.nanosec) * 1e-9;
}
builtin_interfaces::msg::Time Utils::getTime(const double &sec)
{
    builtin_interfaces::msg::Time time_msg;
    time_msg.sec = static_cast<int32_t>(sec);
    time_msg.nanosec = static_cast<uint32_t>((sec - time_msg.sec) * 1e9);
    return time_msg;
}
