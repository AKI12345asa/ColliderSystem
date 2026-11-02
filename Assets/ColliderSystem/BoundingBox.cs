using Unity.Mathematics;
using UnityEngine;
using Ray = ColliderMath.Ray;
/// <summary>
/// AABB包围盒
/// </summary>
public struct BoundingBox
{
    public float3 Center;
    public float3 Extent;

    public float3 Size => Extent * 2;
    public float3 Min => Center - Extent;
    public float3 Max => Center + Extent;
    
    public BoundingBox(float3 center, float3 extent)
    {
        Center =center;
        Extent = extent;
    }

    public BoundingBox Encapsulate(float3 point)
    {
        var max = Max;
        var min = Min;
        max.x = math.max(max.x, point.x);
        max.y = math.max(max.y, point.y);
        max.z = math.max(max.z, point.z);
        min.x = math.min(min.x, point.x);
        min.y = math.min(min.y, point.y);
        min.z = math.min(min.z, point.z);
        return new BoundingBox((min + max) / 2, (max - min) / 2);
    }

    public bool BoundsIntersectWithRay(Ray ray, out Interval interval)
    {
        interval = new Interval(float.MinValue,float.MaxValue);
        if (ray.direction.Equals(float3.zero))
        {
            return false;
        }
        
        float tEnter = float.NegativeInfinity;
        float tExit = float.PositiveInfinity;

        // 每个坐标轴对应一个 slab：射线必须同时落在 X、Y、Z 三个轴向区间内。
        for (int axis = 0; axis < 3; axis++)
        {
            float origin = ray.origin[axis];
            float direction = ray.direction[axis];

            // 射线与该轴平行；若起点在此轴范围外，就不可能穿过盒子。
            if (direction == 0.0f)
            {
                if (origin < Min[axis] || origin > Max[axis])
                {
                    return false;
                }

                continue;
            }

            // 计算与该轴两侧平面的相交参数，并排序为进入/离开参数。
            float t0 = (Min[axis] - origin) / direction;
            float t1 = (Max[axis] - origin) / direction;
            if (t0 > t1) //确保t0为较小值,t1为当前较大值
            {
                (t0, t1) = (t1, t0);
            }

            tEnter = Mathf.Max(tEnter, t0);
            tExit = Mathf.Min(tExit, t1);

            // 三个轴向 slab 的参数区间没有公共交集。
            if (tEnter > tExit)
            {
                return false;
            }
        }

        // 交点全在射线起点之后的反方向时，不算射线命中。
        if (tExit < 0.0f)
        {
            return false;
        }

        // 裁掉射线起点之前的部分；起点在盒内时，前向区间从 t=0 开始。
        interval = new Interval(Mathf.Max(tEnter, 0.0f), tExit);
        return true;
    }

    public bool IntersectBounds(BoundingBox bounds)
    {
        var xAxis = bounds.Min.x > Max.x || bounds.Max.x < Min.x;
        var yAxis = bounds.Min.y > Max.y || bounds.Max.y < Min.y;
        var zAxis = bounds.Min.z > Max.z || bounds.Max.z < Min.z;
        return !(xAxis || yAxis || zAxis);
    }
}
