using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using AuNex.Algorithm;
using AuNex.Common;

namespace AuNex
{
    namespace Planning
    {
        public class LinearPath
        {
            /// <summary>
            /// 現在位置と目的位置から差動２輪用の経路を計画する
            /// </summary>
            /// <param name="current"></param>
            /// <param name="target"></param>
            /// <returns></returns>
            public static nav_msgs.msg.Path CreatePath(
                geometry_msgs.msg.TransformStamped current,
                geometry_msgs.msg.TransformStamped target
            )
            {
                var path = new nav_msgs.msg.Path();

                // 現在のロボットの向きと目的のロボットの向きを取得
                float current_yaw = TransformUtils.QuatToYaw(current.Transform.Rotation);
                float target_yaw = TransformUtils.QuatToYaw(target.Transform.Rotation);
                // 現在位置と目的位置を２次元にする
                Vector2 current2d = new((float)current.Transform.Translation.X, (float)current.Transform.Translation.Y);
                Vector2 target2d = new((float)target.Transform.Translation.X, (float)target.Transform.Translation.Y);
                // 距離ベクトルを計算する
                Vector2 delta2d = target2d - current2d;
                // 距離ベクトルの傾き
                float delta_yaw = Mathf.Atan2(delta2d.y, delta2d.x);
                // 距離の絶対値を計算
                float distance = Mathf.Sqrt(delta2d.x*delta2d.x + delta2d.y*delta2d.y);

                // 120（適当な数字）分割前提で１ステップあたりの長さを計算する
                float step_size = distance / 120.0f;

                for(int i = 0; i < 120; i++)
                {
                    
                }

                return path;
            }
        }
    }
}