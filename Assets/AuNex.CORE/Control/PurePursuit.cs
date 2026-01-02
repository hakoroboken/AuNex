using System;
using System.Collections;
using System.Collections.Generic;
using AuNex.Common;
using UnityEngine;

namespace AuNex
{
    namespace Control
    {
        public class PurePursuit
        {
            private float lookAheadDistance = 0.0f;

            public PurePursuit(float look_ahead_distance)
            {
                lookAheadDistance = look_ahead_distance;
            }

            public geometry_msgs.msg.Twist Compute(geometry_msgs.msg.PoseStamped current_pose, nav_msgs.msg.Path path)
            {
                var target_pose = new geometry_msgs.msg.PoseStamped();
                bool found = false;
                foreach(var p in path.Poses)
                {
                    double _dx = p.Pose.Position.X - current_pose.Pose.Position.X;
                    double _dy = p.Pose.Position.Y - current_pose.Pose.Position.Y;

                    if((_dx*_dx+_dy*_dy) > lookAheadDistance*lookAheadDistance)
                    {
                        target_pose = p;
                        found = true;
                        break;
                    }
                }

                if(!found)target_pose = path.Poses[path.Poses.Length-1];

                double dx = target_pose.Pose.Position.X - current_pose.Pose.Position.X;
                double dy = target_pose.Pose.Position.Y - current_pose.Pose.Position.Y;
                double distance = Math.Sqrt(dx*dx + dy*dy);

                double alpha = Math.Atan2(dy, dx) - TransformUtils.QuatToYaw(current_pose.Pose.Orientation) - 90.0 * (double)Mathf.Deg2Rad;

                var cmd_vel = new geometry_msgs.msg.Twist();
                cmd_vel.Linear.Y = distance;
                cmd_vel.Angular.Z = 2.0 * distance * Math.Sin(alpha) / lookAheadDistance;

                return cmd_vel;
            }
        }
    }
}