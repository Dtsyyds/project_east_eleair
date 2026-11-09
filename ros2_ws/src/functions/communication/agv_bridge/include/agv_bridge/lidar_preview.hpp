#pragma once
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <json/json.h>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <stdexcept>

namespace agv_bridge {
inline bool validLivoxCustom(const livox_ros_driver2::msg::CustomMsg& m) {
    return m.point_num>0 && m.point_num==m.points.size() && m.timebase>0 &&
        m.header.stamp.sec>=0 && m.header.stamp.nanosec<1000000000U &&
        (m.header.stamp.sec>0 || m.header.stamp.nanosec>0);
}

// Sampling affects the browser preview only; the received ROS message is immutable.
inline Json::Value lidarPreview(const livox_ros_driver2::msg::CustomMsg& m, size_t limit=2000) {
    if(!limit || !validLivoxCustom(m))throw std::runtime_error("invalid_livox_custom_msg");
    const size_t stride=(m.points.size()+limit-1)/limit;
    Json::Value out,points(Json::arrayValue);
    for(size_t i=0;i<m.points.size();i+=stride) {
        const auto& p=m.points[i];
        if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z))continue;
        Json::Value xyz(Json::arrayValue);xyz.append(p.x);xyz.append(p.y);xyz.append(p.z);
        points.append(std::move(xyz));
    }
    out["points"]=std::move(points);out["source_points"]=m.point_num;
    out["preview_points"]=out["points"].size();out["sample_stride"]=Json::UInt64(stride);
    out["visualization_only"]=true;out["position_unit"]="m";
    out["timebase"]=std::to_string(m.timebase);
    return out;
}

// Read ROS fields explicitly: never reinterpret an unaligned vendor point struct.
inline Json::Value lidarPreview(const sensor_msgs::msg::PointCloud2& m, size_t limit=2000) {
    const uint64_t count=uint64_t(m.width)*m.height;
    if(!limit || !count || !m.point_step || m.row_step<uint64_t(m.width)*m.point_step ||
       m.data.size()!=uint64_t(m.row_step)*m.height || m.data.size()>64*1024*1024)
        throw std::runtime_error("invalid PointCloud2 layout");
    sensor_msgs::msg::PointField fields[3];
    size_t axis=0;
    for(const char* name:{"x","y","z"}) {
        auto it=std::find_if(m.fields.begin(),m.fields.end(),[&](const auto& f){return f.name==name;});
        if(it==m.fields.end() || it->count!=1 || (it->datatype!=7 && it->datatype!=8) ||
           uint64_t(it->offset)+(it->datatype==7?4:8)>m.point_step)
            throw std::runtime_error("missing or invalid XYZ fields");
        fields[axis++]=*it;
    }
    const uint16_t one=1;
    const bool swap=m.is_bigendian==(*reinterpret_cast<const uint8_t*>(&one)==1);
    const size_t stride=(count+limit-1)/limit;
    Json::Value out,points(Json::arrayValue);
    for(size_t i=0;i<count;i+=stride) {
        const size_t base=(i/m.width)*m.row_step+(i%m.width)*m.point_step;
        double xyz[3];bool finite=true;
        for(size_t j=0;j<3;j++) {
            uint8_t bytes[8]{};const auto& f=fields[j];const size_t size=f.datatype==7?4:8;
            std::memcpy(bytes,m.data.data()+base+f.offset,size);
            if(swap)std::reverse(bytes,bytes+size);
            if(size==4){float value;std::memcpy(&value,bytes,4);xyz[j]=value;}
            else std::memcpy(&xyz[j],bytes,8);
            finite=finite&&std::isfinite(xyz[j]);
        }
        if(!finite)continue;
        Json::Value p(Json::arrayValue);for(double v:xyz)p.append(v);points.append(p);
    }
    out["points"]=std::move(points);out["source_points"]=Json::UInt64(count);
    out["preview_points"]=out["points"].size();out["sample_stride"]=Json::UInt64(stride);
    out["visualization_only"]=true;out["position_unit"]="m";
    return out;
}
}
