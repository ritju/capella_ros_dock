#ifndef CAPELLA_ROS_DOCK_DOUBLE_MARKER_CORRECTION_HPP_
#define CAPELLA_ROS_DOCK_DOUBLE_MARKER_CORRECTION_HPP_

#include "boost/optional.hpp"
#include <string>

namespace capella_ros_dock
{

/* this class input two marker's coords, only output the correct one base on one of the two marker's coordinate*/
template <typename PoseMsgType>
class DoubleMarkerCorrection
{
public:

using optional_output_t = boost::optional<PoseMsgType>;

DoubleMarkerCorrection(int marker1_id, int marker2_id, std::string marker1_frame, std::string marker2_frame float threshold);
{
        this->marker1_id = marker1_id;
        this->marker2_id = marker2_id;
        thi->marker1_frame = marker2_frame;
        this->threshold = threshold;
}

~DoubleMarkerCorrection()
{};

// compare timestamps of pose1 and pose2, when timestamp is equal, return true,else false;
bool paired_check()
{
        if (pose1_valid && pose2_valid && (pose1.timestamp == pose2.timestamp))
        {
                pose1_valid = false;
                pose2_valid = false;
                return true;
        }
        else
        {

        }
}

// 
bool valid_check(double w_fixed, double x_fixed, double y_fixed, double z_fixed, double w_current, double x_current, double y_current, double z_current)
{
        double result = w_fixed * w_current + x_fixed * x_current + y_fixed * y_current + z_fixed * z_current;
        if (result > this->threshold)
        {
                return true;
        }
        else
        {
                return false;
        }
}

// set pose1 and pose2 according pose_id 
void set_pose(PoseMsgType pose)
{
        switch(pose.marker_id)
        {
                case marker_id1:
                {
                        pose_input1 = pose;
                        break;
                }
                case marker_id2:
                {
                        pose_input2 = pose;
                        break;
                }
                default:
                {

                }
        }
}

// get the correct pose for output
optional_output_t get_correct_pose(PoseMsgType pose)
{
        optional_output_t pose_output;
        set_pose(pose);
        if (paired_check())
        {
                return pose_input1;
        }
        else
        {
                return pose_output;
        }
}


int marker1_id, marker2_id;
std::string marker1_frame, marker_frame;
PoseMsgType pose_input1, pose_input2;
optional_output_t pose_output;
float threshold;
bool pose1_ready = false;
bool pose2_ready = false;


};

} // end of namespace

#endif



