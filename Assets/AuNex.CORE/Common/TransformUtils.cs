using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using ROS2;

namespace AuNex
{
    namespace Common
    {
        public static class TransformUtils
        {
            /// <summary>
            /// ROS2座標をUnity座標に変換
            /// </summary>
            /// <param name="vector3"></param>
            /// <returns></returns>
            public static Vector3 Ros2Unity(this Vector3 vector3)
            {
                return new Vector3(-vector3.y, vector3.z, vector3.x);
            }

            /// <summary>
            /// Unity座標をROS2座標に変換
            /// </summary>
            /// <param name="vector3"></param>
            /// <returns></returns>
            public static Vector3 Unity2Ros(this Vector3 vector3)
            {
                return new Vector3(vector3.z, -vector3.x, vector3.y);
            }

            /// <summary>
            /// ROS2姿勢をUnity姿勢に変換
            /// </summary>
            /// <param name="quaternion"></param>
            /// <returns></returns>
            public static Quaternion Ros2Unity(this Quaternion quaternion)
            {
                return new Quaternion(quaternion.y, -quaternion.z, -quaternion.x, quaternion.w);
            }

            /// <summary>
            /// Unity姿勢をROS2姿勢に変換
            /// </summary>
            /// <param name="quaternion"></param>
            /// <returns></returns>
            public static Quaternion Unity2Ros(this Quaternion quaternion)
            {
                return new Quaternion(-quaternion.z, quaternion.x, -quaternion.y, quaternion.w);
            }

            public static tf2_msgs.msg.TFMessage CreateTFMessage(geometry_msgs.msg.TransformStamped transform)
            {
                var tf = new tf2_msgs.msg.TFMessage();

                tf.Transforms = new[]{transform};

                return tf;
            }

            public static float QuatToYaw(geometry_msgs.msg.Quaternion q)
            {
                var unity_q = new Quaternion((float)q.Y, (float)(-1.0*q.Z), (float)(-1.0*q.X), (float)(q.W));

                return unity_q.eulerAngles.y * Mathf.Deg2Rad;
            }          
        }

        public class TFBroadCaster
        {
            private IPublisher<tf2_msgs.msg.TFMessage> publisher;

            public TFBroadCaster(ROS2Node node)
            {
                publisher = node.CreatePublisher<tf2_msgs.msg.TFMessage>("/tf");
            }

            public void SendTransform(geometry_msgs.msg.TransformStamped t)
            {
                var tf = TransformUtils.CreateTFMessage(t);

                publisher.Publish(tf);
            }
        }
    }// end of namespace Common
}// end of namespace AuNex


