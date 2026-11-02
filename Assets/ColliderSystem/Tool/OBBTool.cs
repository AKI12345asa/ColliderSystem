using ColliderMath;
using Unity.Mathematics;

public static class OBBIntersectTool
{
    /// <summary>
    /// 依旧是会把capsule的轴线放到Obb空间下来简化运算
    /// </summary>
    /// <param name="box"></param>
    /// <param name="capsule"></param>
    /// <param name="intersectSegment"></param>
    /// <returns></returns>
    public static unsafe bool IntersectsSAT(BoxColliderEntity box, CapsuleColliderEntity capsule, out float3 intersectSegment)
    {
        float3* axes = stackalloc float3[14];
        float3* points = stackalloc float3[2];
        var obb = new OrientedBoundingBox(box.Center, box.Size * 0.5f, box.Rotation);
        quaternion inverse = math.inverse(obb.Orientation);
        float3 localCenter = math.mul(inverse, capsule.Center - box.Center);
        float3 worldCapsuleAxis = math.mul(capsule.Rotation, new float3(0f, 1f, 0f));
        float3 localCapsuleAxis = math.normalizesafe(math.mul(inverse, worldCapsuleAxis));
        float halfSegment = math.max(0f, capsule.Height * 0.5f - capsule.Radius);
        points[0] = localCenter - localCapsuleAxis * halfSegment;
        points[1] = localCenter + localCapsuleAxis * halfSegment;

        float3 axisX = new float3(1f, 0f, 0f);
        float3 axisY = new float3(0f, 1f, 0f);
        float3 axisZ = new float3(0f, 0f, 1f);
        axes[0] = axisX;
        axes[1] = axisY;
        axes[2] = axisZ;
        axes[3] = math.cross(localCapsuleAxis, axisX);
        axes[4] = math.cross(localCapsuleAxis, axisY);
        axes[5] = math.cross(localCapsuleAxis, axisZ);

        float3 extents = obb.Extents;
        for (int corner = 0; corner < 8; corner++)
        {
            float3 vertex = new float3(
                (corner & 1) == 0 ? -extents.x : extents.x,
                (corner & 2) == 0 ? -extents.y : extents.y,
                (corner & 4) == 0 ? -extents.z : extents.z);
            float3 closestOnSegment = ClosestPointOnSegment(vertex, points[0], points[1]);
            axes[6 + corner] = vertex - closestOnSegment;
        }

        bool intersects = GeometryMathTool.SatForAxes(axes, 14, points, 2, extents, capsule.Radius, out float3 localIntersectSegment);
        intersectSegment = -math.mul(obb.Orientation, localIntersectSegment);
        return intersects;
    }

    private static float3 ClosestPointOnSegment(float3 point, float3 start, float3 end)
    {
        float3 segment = end - start;
        float lengthSquared = math.lengthsq(segment);
        if (lengthSquared <= 1e-12f)
        {
            return start;
        }

        float t = math.saturate(math.dot(point - start, segment) / lengthSquared);
        return start + segment * t;
    }

    public static bool Intersects(
        BoxColliderEntity box,
        CapsuleColliderEntity capsule,
        out float3 intersectSegment)
    {
        return IntersectsSAT(box, capsule, out intersectSegment);
    }



    /// <summary>
    /// 这里依旧使用将sphere转化到box的空间坐标系下以简化运算
    /// </summary>
    /// <param name="box"></param>
    /// <param name="sphere"></param>
    /// <param name="intersectSegment"></param>
    /// <returns></returns>
    public static bool Intersects(BoxColliderEntity box, SphereColliderEntity sphere,out float3 intersectSegment)
    {
        var obb = new OrientedBoundingBox(box.Center, box.Size * 0.5f, box.Rotation);
        quaternion inverse = math.inverse(obb.Orientation);
        float3 localCenter = math.mul(inverse, sphere.Center - box.Center);
        intersectSegment = float3.zero;
        float3 closestPoint = math.clamp(localCenter, -obb.Extents, obb.Extents);//一个点到某个AABB上的最近点的某一个轴上的数值可以这样判断:如果轴上数值在区间则可以视为在该轴上垂直
        float3 centerToBox = localCenter - closestPoint;
        float distanceSq = math.lengthsq(centerToBox);
        if (distanceSq > 1e-12f) //球心在盒外
        {
            if (distanceSq > sphere.Radius * sphere.Radius)
            {
                return false;
            }

            float distance = math.sqrt(distanceSq);
            intersectSegment = -math.mul(obb.Orientation, centerToBox * ((sphere.Radius - distance) / distance));
            return true;
        }
        //球心在盒内，计算球心到盒子各面的距离，选择最近的面作为碰撞法线方向
        float3 faceDistances = obb.Extents - math.abs(localCenter);
        int nearestAxis = faceDistances.x <= faceDistances.y && faceDistances.x <= faceDistances.z ? 0 : faceDistances.y <= faceDistances.z ? 1 : 2;
        float3 outwardAxis = float3.zero;
        outwardAxis[nearestAxis] = localCenter[nearestAxis] < 0f ? -1f : 1f;
        intersectSegment = -math.mul(obb.Orientation, outwardAxis * (faceDistances[nearestAxis] + sphere.Radius));
        return true;
    }

    public static unsafe bool Intersects(BoxColliderEntity boxA, BoxColliderEntity boxB,out float3 intersectSegment)
    {
        float3* axes = stackalloc float3[15];
        float3* points = stackalloc float3[8];
        var obbA = new OrientedBoundingBox(boxA.Center, boxA.Size * 0.5f, boxA.Rotation);
        
        //这里为了计算方便直接将盒体B转换到盒体A的空间下做计算
        var obbB = new OrientedBoundingBox(boxB.Center, boxB.Size * 0.5f, boxB.Rotation).ToLocalSpace(obbA);

        for (int axisIndex = 0; axisIndex < 15; axisIndex++)
        {
            if (axisIndex < 3)
            {
                axes[axisIndex] = axisIndex == 0 ? new float3(1f, 0f, 0f) : axisIndex == 1 ? new float3(0f, 1f, 0f) : new float3(0f, 0f, 1f);
            }
            else if (axisIndex < 6)
            {
                float3 axisB = axisIndex == 3 ? obbB.AxisX : axisIndex == 4 ? obbB.AxisY : obbB.AxisZ;
                axes[axisIndex] = axisB;
            }
            else
            {
                int crossIndex = axisIndex - 6;
                float3 axisA = crossIndex / 3 == 0 ? new float3(1f, 0f, 0f) : crossIndex / 3 == 1 ? new float3(0f, 1f, 0f) : new float3(0f, 0f, 1f);
                float3 axisB = crossIndex % 3 == 0 ? obbB.AxisX : crossIndex % 3 == 1 ? obbB.AxisY : obbB.AxisZ;
                axes[axisIndex] = math.cross(axisA, axisB);
            }
        }

        for (int cornerIndex = 0; cornerIndex < 8; cornerIndex++)
        {
            points[cornerIndex] = obbB.Center +
                obbB.AxisX * ((cornerIndex & 1) == 0 ? -obbB.Extents.x : obbB.Extents.x) +
                obbB.AxisY * ((cornerIndex & 2) == 0 ? -obbB.Extents.y : obbB.Extents.y) +
                obbB.AxisZ * ((cornerIndex & 4) == 0 ? -obbB.Extents.z : obbB.Extents.z);
        }

        int validAxisCount = 0;//去除无效轴
        for (int axisIndex = 0; axisIndex < 15; axisIndex++)
        {
            float axisLengthSq = math.lengthsq(axes[axisIndex]);
            if (!math.isfinite(axisLengthSq) || axisLengthSq <= 1e-12f)
            {
                continue;
            }

            axes[validAxisCount++] = axes[axisIndex] * math.rsqrt(axisLengthSq);
        }
        if (validAxisCount == 0)
        {
            intersectSegment = float3.zero;
            return false;
        }
        bool result = GeometryMathTool.SatForAxes(axes, validAxisCount, points, 8, obbA.LocalBounds.Extent, out intersectSegment);
        intersectSegment = math.mul(boxA.Rotation, intersectSegment);
        if (math.dot(intersectSegment, boxA.Center - boxB.Center) < 0f)
        {
            intersectSegment = -intersectSegment;
        }

        return result;
    }
    
}
