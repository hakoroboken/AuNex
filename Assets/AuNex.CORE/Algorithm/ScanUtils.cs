using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace AuNex
{
    namespace Algorithm
    {
        class ScanUtils
        {
            /// <summary>
            /// LaserScanメッセージを2Dポイントクラウドに変換する
            /// </summary>
            /// <param name="scan">LaserScanメッセージ</param>
            /// <returns>2Dポイントクラウド</returns>
            public static List<Vector2> LaserScanToPointCloud(sensor_msgs.msg.LaserScan scan)
            {
                List<Vector2> pointCloud = new List<Vector2>();
                float angle = scan.Angle_min;
                for (int i = 0; i < scan.Ranges.Length; i++)
                {
                    float range = scan.Ranges[i];
                    if (range >= scan.Range_min && range <= scan.Range_max)
                    {
                        float x = range * Mathf.Cos(angle);
                        float y = range * Mathf.Sin(angle);
                        pointCloud.Add(new Vector2(x, y));
                    }
                    angle += scan.Angle_increment;
                }
                return pointCloud;
            }
        }
    }
}