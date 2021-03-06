#ifndef _ROS_play_motion_builder_msgs_BuildMotionAction_h
#define _ROS_play_motion_builder_msgs_BuildMotionAction_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "play_motion_builder_msgs/BuildMotionActionGoal.h"
#include "play_motion_builder_msgs/BuildMotionActionResult.h"
#include "play_motion_builder_msgs/BuildMotionActionFeedback.h"

namespace play_motion_builder_msgs
{

  class BuildMotionAction : public ros::Msg
  {
    public:
      typedef play_motion_builder_msgs::BuildMotionActionGoal _action_goal_type;
      _action_goal_type action_goal;
      typedef play_motion_builder_msgs::BuildMotionActionResult _action_result_type;
      _action_result_type action_result;
      typedef play_motion_builder_msgs::BuildMotionActionFeedback _action_feedback_type;
      _action_feedback_type action_feedback;

    BuildMotionAction():
      action_goal(),
      action_result(),
      action_feedback()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->action_goal.serialize(outbuffer + offset);
      offset += this->action_result.serialize(outbuffer + offset);
      offset += this->action_feedback.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->action_goal.deserialize(inbuffer + offset);
      offset += this->action_result.deserialize(inbuffer + offset);
      offset += this->action_feedback.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "play_motion_builder_msgs/BuildMotionAction"; };
    virtual const char * getMD5() override { return "455cd360788835a49638c295be24e0cf"; };

  };

}
#endif
