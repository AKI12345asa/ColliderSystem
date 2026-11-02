#region 常规版本

using System.Drawing;
using Unity.Collections;
using Unity.Mathematics;
namespace ColliderMath
{
    public struct MathInterval
    {
        public float min;
        public float max;

        public MathInterval(float min, float max)
        {
            this.min = min;
            this.max = max;
        }

        public bool Contains(float value)
        {
            return value >= min && value <= max;
        }
    }

    public static partial class GeometryMathTool
    {
        public static float CalculatePathLength(NativeArray<float3> path)
        {
            // 路径长度是相邻路径点之间线段长度之和。
            if (path.Length <= 1) return 0.0f;
            float sum = 0.0f;
            for (int i = 1; i < path.Length; i++)
            {
                sum += math.distance(path[i], path[i - 1]);
            }

            return sum;
        }
        
        public static bool ContainTriangle(BoundingBox bounds, ColliderTriangle triangle)
        {
            return ContainsPoint(bounds.Min, bounds.Max, triangle[0]) &&
                   ContainsPoint(bounds.Min, bounds.Max, triangle[1]) &&
                   ContainsPoint(bounds.Min, bounds.Max, triangle[2]);
        }

        public static bool OverlapTriangle(BoundingBox bounds, ColliderTriangle triangle, NativeArray<float3> axes0, NativeArray<float3> axes1, NativeArray<float3> axes2)
        {
            // 若三角形已完全在盒内可直接命中；否则完整测试 13 个候选分离轴。
            if (ContainTriangle(bounds, triangle))
            {
                return true;
            }

            float3 extends = bounds.Extent;
            float3 v0 = triangle[0] - bounds.Center;
            float3 v1 = triangle[1] - bounds.Center;
            float3 v2 = triangle[2] - bounds.Center;

            float3 f0 = v1 - v0;
            float3 f1 = v2 - v1;
            float3 f2 = v0 - v2;

            axes0[0] = new float3(0, -f0.z, f0.y);
            axes0[1] = new float3(0, -f1.z, f1.y);
            axes0[2] = new float3(0, -f2.z, f2.y);
            axes0[3] = new float3(f0.z, 0, -f0.x);
            axes0[4] = new float3(f1.z, 0, -f1.x);
            axes0[5] = new float3(f2.z, 0, -f2.x);
            axes0[6] = new float3(-f0.y, f0.x, 0);
            axes0[7] = new float3(-f1.y, f1.x, 0);
            axes0[8] = new float3(-f2.y, f2.x, 0);

            if (!SatForAxes(axes0, v0, v1, v2, extends))
            {
                return false;
            }

            axes1[0] = new float3(1, 0, 0);
            axes1[1] = new float3(0, 1, 0);
            axes1[2] = new float3(0, 0, 1);
            if (!SatForAxes(axes1, v0, v1, v2, extends))
            {
                return false;
            }

            // finally testing the face normal of the triangle
            // use already existing triangle edge vectors here
            float3 triangleNormal = math.cross(f0, f1);
            axes2[0] = triangleNormal;
            return SatForAxes(axes2, v0, v1, v2, extends);
        }

        public static bool IntersectsRayInside(
            BoundingBox bounds, Ray ray, out float nearestDistance)
        {
            // 该方法要求射线起点在盒内，返回射线离开盒体时的参数距离。
            nearestDistance = 0;
            if (!ContainsPoint(bounds.Min, bounds.Max, ray.origin))
            {
                return false;
            }

            float3 min = bounds.Min;
            float3 max = bounds.Max;
            float3 origin = ray.origin;
            float3 direction = ray.direction;

            float tExit = float.MaxValue;

            for (int i = 0; i < 3; i++)
            {
                float dir = direction[i];
                float ori = origin[i];

                if (math.abs(dir) < 1e-6f)
                {
                    continue;
                }

                float t1 = (min[i] - ori) / dir;
                float t2 = (max[i] - ori) / dir;
                float tFar = math.max(t1, t2);
                tExit = math.min(tExit, tFar);
            }

            nearestDistance = tExit;
            return true;
        }

        public static bool OverlapTriangle(float2 boxMin, float2 boxMax, ColliderTriangle2D triangle, NativeArray<float2> satAxies, NativeArray<float2> triPoints, NativeArray<float2> boxPoints)
        {
            // 2D SAT 的候选轴为盒子的 X/Y 轴和三角形三条边的法线。
            for (int i = 0; i < 3; i++)
            {
                if (ContainsPoint(boxMin, boxMax, triangle[i]))
                {
                    return true;
                }
            }

            satAxies[0] = new float2(1, 0);
            satAxies[1] = new float2(0, 1);

            float2 e0 = math.normalizesafe(triangle[1] - triangle[0]);
            float2 e1 = math.normalizesafe(triangle[2] - triangle[0]);
            float2 e2 = math.normalizesafe(triangle[2] - triangle[1]);

            satAxies[2] = new float2(-e0.y, e0.x);
            satAxies[3] = new float2(-e1.y, e1.x);
            satAxies[4] = new float2(-e2.y, e2.x);

            triPoints[0] = triangle[0];
            triPoints[1] = triangle[1];
            triPoints[2] = triangle[2];

            float2 size = boxMax - boxMin;
            boxPoints[0] = boxMin;
            boxPoints[1] = boxMax;
            boxPoints[2] = boxMin + new float2(1, 0) * size.x;
            boxPoints[3] = boxMin + new float2(0, 1) * size.y;

            for (int i = 0; i < satAxies.Length; i++)
            {
                float2 satAxis = satAxies[i];

                ProjectToAxis(satAxis, triPoints, out float triMin, out float triMax);

                ProjectToAxis(satAxis, boxPoints, out float boxMinOnAxis, out float boxMaxOnAxis);

                if (triMax < boxMinOnAxis || boxMaxOnAxis < triMin)
                {
                    return false;
                }
            }

            return true;
        }

        private static void ProjectToAxis(float2 axis, NativeArray<float2> points, out float min, out float max)
        {
            // 将点集投影到给定轴上，得到一维区间以供 SAT 比较。
            min = 99999999.0f;
            max = -min;

            for (int i = 0; i < points.Length; i++)
            {
                float value = math.dot(axis, points[i]);
                min = math.min(min, value);
                max = math.max(max, value);
            }
        }

        public static bool OverlapCircle(float2 boxMin, float2 boxMax, float2 circleCenter, float circleRadius)
        {
            // 求圆心到盒子的最近点距离平方，再与半径平方比较。
            float2 centerBox = (boxMin + boxMax) * 0.5f;
            float2 centerCircle = circleCenter;
            float2 v = centerCircle - centerBox;
            v.x = math.abs(v.x);
            v.y = math.abs(v.y);

            float2 h = boxMax - centerBox;

            float2 u = v - h;

            u.x = math.max(u.x, 0.0f);
            u.y = math.max(u.y, 0.0f);

            return math.lengthsq(u) <= circleRadius * circleRadius;
        }

        private static bool SatForAxes(NativeArray<float3> axes, float3 v0, float3 v1, float3 v2, float3 extents)
        {
            // 三角形顶点已相对盒心平移；盒在任意轴上的投影半径可由 extents 加权求得。
            for (int i = 0, j = axes.Length - 1; i <= j; i++)
            {
                float3 testAxis = axes[i];
                float r = extents.x * math.abs(testAxis.x) + extents.y * math.abs(testAxis.y) +
                          extents.z * math.abs(testAxis.z);
                float p0 = math.dot(v0, testAxis);
                float p1 = math.dot(v1, testAxis);
                float p2 = math.dot(v2, testAxis);
                if (math.max(-math.max(p0, math.max(p1, p2)), math.min(p0, math.min(p1, p2))) > r)
                {
                    return false;
                }
            }

            return true;
        }

        public static bool SatForAxes(NativeArray<float3> axes, NativeArray<float3> points, float3 extents)
        {
            for (int i = 0; i < axes.Length; i++)
            {
                float3 testAxis = axes[i];
                float r = extents.x * math.abs(testAxis.x) + extents.y * math.abs(testAxis.y) +
                          extents.z * math.abs(testAxis.z);
                var max = float.MinValue;
                var min = float.MaxValue;
                for (int j = 0; j < points.Length; j++)
                {
                    max = math.max(max, math.dot(points[j], testAxis));
                    min = math.min(min, math.dot(points[j], testAxis));
                }

                if (min > r || max < -r) return false;
            }

            return true;
        }

        public static bool BoundsIntersect(BoundingBox boundsLeft, BoundingBox boundsRight)
        {
            // 先处理完全包含，再检测三条坐标轴方向是否有间隙。
            if (ContainsBounds(boundsLeft, boundsRight))
            {
                return true;
            }

            return OverlapBounds(boundsLeft, boundsRight);
        }

        private static bool OverlapBounds(BoundingBox boundsLeft, BoundingBox boundsRight)
        {
            if (!(boundsRight.Max.x <= boundsLeft.Min.x || boundsRight.Min.x >= boundsLeft.Max.x ||
                  boundsRight.Max.y <= boundsLeft.Min.y || boundsRight.Min.y >= boundsLeft.Max.y ||
                  boundsRight.Max.z <= boundsLeft.Min.z || boundsRight.Min.z >= boundsLeft.Max.z))
            {
                return true;
            }

            return false;
        }

        private static bool ContainsBounds(BoundingBox bounds, BoundingBox objBounds)
        {
            return bounds.Min.x <= objBounds.Min.x && objBounds.Max.x <= bounds.Max.x &&
                   bounds.Min.y <= objBounds.Min.y && objBounds.Max.y <= bounds.Max.y &&
                   bounds.Min.z <= objBounds.Min.z && objBounds.Max.z <= bounds.Max.z;
        }

        public static bool ContainsPoint(float3 min, float3 max, float3 point)
        {
            return min.x <= point.x && point.x <= max.x &&
                   min.y <= point.y && point.y <= max.y &&
                   min.z <= point.z && point.z <= max.z;
        }

        public static bool ContainsPoint(float2 min, float2 max, float2 point)
        {
            return min.x <= point.x && point.x <= max.x &&
                   min.y <= point.y && point.y <= max.y;
        }

        public static BoundingBox GetTriangleAABB(NativeArray<float3> triangles)
        {
            // 逐点扩展最小/最大坐标，最终构造包围所有输入顶点的轴对齐盒。
            float3 min = new float3(9999999, 999999, 99999);
            float3 max = -min;
            for (int i = 0; i < triangles.Length; i++)
            {
                min = math.min(min, triangles[i]);
                max = math.max(max, triangles[i]);
            }

            float3 center = (min + max) * 0.5f;
            float3 size = max - min;
            return new BoundingBox(center, size * 0.5f);
        }

        public static BoundingBox GetTriangleAABB(NativeArray<ColliderTriangle> triangles)
        {
            float3 min = new float3(9999999, 999999, 99999);
            float3 max = -min;
            for (int i = 0; i < triangles.Length; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    min = math.min(min, triangles[i][j]);
                    max = math.max(max, triangles[i][j]);
                }
            }

            float3 center = (min + max) * 0.5f;
            float3 size = max - min;
            return new BoundingBox(center, size * 0.5f);
        }

        public static BoundingBox GetTerrainAABB(NativeArray<BoundingBox> boundsList)
        {
            // 合并多个已有包围盒的 min/max，得到覆盖全部地形块的总包围盒。
            float3 min = new float3(float.MaxValue, float.MaxValue, float.MaxValue);
            float3 max = -min;
            for (int i = 0; i < boundsList.Length; i++)
            {
                BoundingBox bounds = boundsList[i];
                min = math.min(min, bounds.Min);
                max = math.max(max, bounds.Max);
            }

            float3 center = (max + min) * 0.5f;
            float3 size = max - min;
            return new BoundingBox(center, size * 0.5f);
        }

        public static BoundingBox MakeBoundsFromMinMax(float3 min, float3 max)
        {
            float3 center = (min + max) * 0.5f;
            float3 size = max - min;
            return new BoundingBox(center, size * 0.5f);
        }

        public static bool Overlap(MathInterval slap0, MathInterval slap1)
        {
            // 两区间不相交的条件是一个区间完全位于另一个区间左侧或右侧。
            return !(slap0.max <= slap1.min || slap0.min >= slap1.max);
        }

        public static MathInterval Intersect(MathInterval slap0, MathInterval slap1)
        {
            return new MathInterval(math.max(slap0.min, slap1.min), math.min(slap0.max, slap1.max));
        }

        public static bool IntersectWithRay(float2 boxMin, float2 boxMax, float2 rayOrigin, float2 rayDirection,
            out MathInterval interval)
        {
            // 分别计算射线在 X/Y 轴进入与离开盒子的参数区间，再求两个区间交集。
            MathInterval slapX = new MathInterval(boxMin.x, boxMax.x);
            MathInterval slapY = new MathInterval(boxMin.y, boxMax.y);

            interval = new MathInterval(float.PositiveInfinity, float.PositiveInfinity);

            if (math.abs(rayDirection.x) < 1e-6f)
            {
                if (!slapX.Contains(rayOrigin.x))
                {
                    return false;
                }

                interval = IntersectWithSplitRay(slapY, rayOrigin.y, rayDirection.y);
            }
            else if (math.abs(rayDirection.y) < 1e-6f)
            {
                if (!slapY.Contains(rayOrigin.y))
                {
                    return false;
                }

                interval = IntersectWithSplitRay(slapX, rayOrigin.x, rayDirection.x);
            }
            else
            {
                MathInterval intersectX = IntersectWithSplitRay(slapX, rayOrigin.x, rayDirection.x);
                MathInterval intersectY = IntersectWithSplitRay(slapY, rayOrigin.y, rayDirection.y);
                if (!Overlap(intersectX, intersectY))
                {
                    return false;
                }

                interval = Intersect(intersectX, intersectY);
            }

            if (interval.max < 0)
                return false; //此时说明要反向延长才能与盒子相交，射线不算命中。
            else if (interval.min < 0)
                // 射线起点在盒内时，最近的正向交点是离开盒子的参数。
                interval.min = interval.max;
            return true;
        }

        /// <summary>
        /// 使用三维 slab 区间法判断射线是否与 AABB 相交。
        /// 返回的 interval 是射线参数 t 的前向相交区间；起点在盒内时 min 为 0。
        /// </summary>
        public static bool BoundsIntersectWithRay(
            BoundingBox bounds, Ray ray, out MathInterval interval)
        {
            interval = new MathInterval(float.PositiveInfinity, float.PositiveInfinity);
            if (math.all(ray.direction == float3.zero))
            {
                return false;
            }

            float3 min = bounds.Min;
            float3 max = bounds.Max;
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
                    if (origin < min[axis] || origin > max[axis])
                    {
                        return false;
                    }

                    continue;
                }

                // 计算与该轴两侧平面的相交参数，并排序为进入/离开参数。
                float t0 = (min[axis] - origin) / direction;
                float t1 = (max[axis] - origin) / direction;
                if (t0 > t1) //确保t0为较小值,t1为当前较大值
                {
                    (t0, t1) = (t1, t0);
                }

                tEnter = math.max(tEnter, t0);
                tExit = math.min(tExit, t1);

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
            interval = new MathInterval(math.max(tEnter, 0.0f), tExit);
            return true;
        }

        public static MathInterval IntersectWithSplitRay(MathInterval slap, float origin, float dir)
        {
            // 将区间两个端点换算为射线参数，并按大小排列以兼容负方向。
            float t0 = (slap.min - origin) / dir;
            float t1 = (slap.max - origin) / dir;
            return new MathInterval(math.min(t0, t1), math.max(t0, t1));
        }

        /// <summary>
        /// 根据盒子的中心和尺寸，计算8个顶点的位置
        /// 顶点布局与BoundsToMesh一致：
        /// 0: min(-x,-y,-z)  1: (-x,-y,+z)  2: (+x,-y,+z)  3: (+x,-y,-z)
        /// 4: (-x,+y,-z)     5: (-x,+y,+z)  6: (+x,+y,+z)  7: (+x,+y,-z)
        /// </summary>
        public static void GetBoxCorners(float3 center, float3 size, NativeArray<float3> corners)
        {
            float3 extents = size * 0.5f;
            corners[0] = center + new float3(-extents.x, -extents.y, -extents.z);
            corners[1] = center + new float3(-extents.x, -extents.y, extents.z);
            corners[2] = center + new float3(extents.x, -extents.y, extents.z);
            corners[3] = center + new float3(extents.x, -extents.y, -extents.z);
            corners[4] = center + new float3(-extents.x, extents.y, -extents.z);
            corners[5] = center + new float3(-extents.x, extents.y, extents.z);
            corners[6] = center + new float3(extents.x, extents.y, extents.z);
            corners[7] = center + new float3(extents.x, extents.y, -extents.z);
        }

        public static BoundingBox GenerateBoundsWithPoints(NativeArray<float3> points)
        {
            float3 min = new float3(float.MaxValue, float.MaxValue, float.MaxValue);
            float3 max = new float3(float.MinValue, float.MinValue, float.MinValue);
            for (int i = 0; i < points.Length; i++)
            {
                float3 point = points[i];
                min.x = math.min(min.x, point.x);
                min.y = math.min(min.y, point.y);
                min.z = math.min(min.z, point.z);
                max.x = math.max(max.x, point.x);
                max.y = math.max(max.y, point.y);
                max.z = math.max(max.z, point.z);
            }

            return new BoundingBox((min + max) / 2, (max - min) / 2);
        }

        public static bool ObbIntersectsWithRay(OrientedBoundingBox obb, Ray ray, out float distance)
        {
            quaternion inverse = math.inverse(obb.Orientation);
            Ray localRay = new Ray
            {
                origin = math.mul(inverse, (ray.origin - obb.Center)),
                direction = math.mul(inverse, ray.direction)
            };
            BoundingBox localBounds = new BoundingBox(float3.zero, obb.Extents);
            if (BoundsIntersectWithRay(localBounds, localRay, out MathInterval interval))
            {
                distance = interval.min;
                return true;
            }

            distance = 0f;
            return false;
        }

        public static float PointToAabbDistanceSquared(float3 point, float3 extents)
        {
            float3 separation = math.max(float3.zero, math.abs(point) - extents);
            return math.lengthsq(separation);
        }

        /// <summary>
        /// 将8个顶点表示的包围盒表面转换为12个三角形（6个面，每面2个三角形）
        /// 顶点布局需与GetBoxCorners一致
        /// </summary>
        public static void BoxCornersToTriangles(NativeArray<float3> corners, NativeArray<ColliderTriangle> triangles)
        {
            // 底面
            triangles[0] = new ColliderTriangle(corners[0], corners[1], corners[2]);
            triangles[1] = new ColliderTriangle(corners[0], corners[2], corners[3]);
            // 前面 (z-)
            triangles[2] = new ColliderTriangle(corners[0], corners[1], corners[5]);
            triangles[3] = new ColliderTriangle(corners[0], corners[5], corners[4]);
            // 左面 (x-)
            triangles[4] = new ColliderTriangle(corners[0], corners[4], corners[7]);
            triangles[5] = new ColliderTriangle(corners[0], corners[7], corners[3]);
            // 右面 (x+)
            triangles[6] = new ColliderTriangle(corners[1], corners[5], corners[6]);
            triangles[7] = new ColliderTriangle(corners[1], corners[6], corners[2]);
            // 后面 (z+)
            triangles[8] = new ColliderTriangle(corners[2], corners[6], corners[7]);
            triangles[9] = new ColliderTriangle(corners[2], corners[7], corners[3]);
            // 顶面
            triangles[10] = new ColliderTriangle(corners[4], corners[5], corners[6]);
            triangles[11] = new ColliderTriangle(corners[4], corners[6], corners[7]);
        }

        /// <summary>
        /// 求线段之间的最短距离
        /// </summary>
        /// <param name="center1"></param>
        /// <param name="rotation1"></param>
        /// <param name="height1"></param>
        /// <param name="r1"></param>
        /// <param name="center2"></param>
        /// <param name="rotation2"></param>
        /// <param name="height2"></param>
        /// <param name="r2"></param>
        /// <param name="intersectSegment"></param>
        /// <returns></returns>
        public static bool CheckTwoCapsuleIntersect(float3 center1, quaternion rotation1, float height1, float r1, float3 center2, quaternion rotation2, float height2, float r2,out float3 intersectSegment)
        {
            intersectSegment = default;
            if (height1 < 0 || height2 < 0) return false;
            var check1 = CheckCapsuleSegment(center1, rotation1, height1, r1, out var p1, out var p2);
            var check2 = CheckCapsuleSegment(center2, rotation2, height2, r2, out var q1, out var q2);
            if (!check1 || !check2)//防止胶囊体退化为球体
            {
                if (!check1 && !check2) return CheckTwoSphereIntersect(p1, r1, q1, r2,out intersectSegment);
                if (!check2) return CheckCapsuleIntersectWithSphere(center1, rotation1, height1, r1, center2, r2,out intersectSegment);
                return CheckCapsuleIntersectWithSphere(center2, rotation2, height2, r2, center1, r1,out intersectSegment);
            }
            // 计算两条线段之间的最短距离，并与半径和进行比较。
            float distSq = SquaredDistanceBetweenSegments(p1, p2 - p1, q1, q2 - q1,out intersectSegment);
            float radiusSum = r1 + r2;
            float3 fallbackDirection = math.normalizesafe(math.cross(p2 - p1, q2 - q1));
            if (math.lengthsq(fallbackDirection) <= math.EPSILON)
            {
                fallbackDirection = GetPerpendicularRightDirection(p2 - p1);
            }
            intersectSegment = math.normalizesafe(intersectSegment, fallbackDirection) * (radiusSum - math.sqrt(distSq));
            return distSq <= radiusSum * radiusSum;
        }

        
        /// <summary>
        /// 就是求点到线的最短距离
        /// </summary>
        /// <param name="center1"></param>
        /// <param name="rotation"></param>
        /// <param name="height"></param>
        /// <param name="r1"></param>
        /// <param name="center2"></param>
        /// <param name="r2"></param>
        /// <param name="intersectSegment"></param>
        /// <returns></returns>
        public static bool CheckCapsuleIntersectWithSphere(float3 center1, quaternion rotation, float height, float r1, float3 center2, float r2,out float3 intersectSegment)
        {
            intersectSegment = default;
            if (height < 0) return false;
            var check = CheckCapsuleSegment(center1, rotation, height, r1, out var start, out var end);
            if (!check)
            {
                return CheckTwoSphereIntersect(center1, r1, center2, r2,out intersectSegment);
            }

            var distance = PointToSegmentSqrDistance(center2, start, end, out var closestPoint);
            var direction = math.normalizesafe(closestPoint - center2, GetPerpendicularRightDirection(end - start));
            var radiusSum = r1 + r2;
            intersectSegment = direction * (radiusSum - math.sqrt(distance));
            return distance <= radiusSum * radiusSum;
        }

        public static bool CheckTwoSphereIntersect(float3 center1, float r1, float3 center2, float r2,out float3 intersectSegment)
        {
            var distance = math.lengthsq(center1 - center2);
            var direction = math.normalizesafe(center1 - center2, new float3(1f, 0f, 0f));
            var radiusSum = r1 + r2;
            intersectSegment = direction * (radiusSum - math.sqrt(distance));
            return distance <= radiusSum * radiusSum;
        }

        private static float3 GetPerpendicularRightDirection(float3 axis)
        {
            axis = math.normalizesafe(axis);
            if (math.lengthsq(axis) <= math.EPSILON)
            {
                return new float3(1f, 0f, 0f);
            }

            float3 direction = math.cross(axis, new float3(1f, 0f, 0f));
            if (math.lengthsq(direction) <= math.EPSILON)
            {
                direction = math.cross(axis, new float3(0f, 1f, 0f));
            }

            return math.normalizesafe(direction, new float3(1f, 0f, 0f));
        }

        /// <summary>
        /// 一个校验方法检测胶囊体是否会退化为球体，如果退化了就返回false，并且start和end会是胶囊体的中心点
        /// </summary>
        /// <param name="center"></param>
        /// <param name="rotation"></param>
        /// <param name="height"></param>
        /// <param name="radius"></param>
        /// <param name="start"></param>
        /// <param name="end"></param>
        /// <returns></returns>
        public static bool CheckCapsuleSegment(float3 center, quaternion rotation, float height, float radius, out float3 start, out float3 end)
        {
            var halfHeight = math.max(0, height / 2 - radius);
            var direction = math.mul(rotation, new float3(0, 1, 0));
            start = center - halfHeight * direction;
            end = center + halfHeight * direction;
            return math.abs(halfHeight) > math.EPSILON;
        }

        private static float SquaredDistanceBetweenSegments(float3 start1, float3 line1, float3 start2, float3 line2,out float3 intersectSegment)
        {
            //根据两线间最短距离的公式直接写出
            var a = math.dot(line1, line1);
            var b = math.dot(line1, line2);
            var c = math.dot(line2, line2);
            var d = math.dot(line1, start1 - start2);
            var e = math.dot(line2, start1 - start2);
            var denom = a * c - b * b;
            float s, t;
            if (math.abs(denom) > math.EPSILON)
            {
                s = (b * e - c * d) / denom;
                t = (a * e - b * d) / denom;
            }
            else
            {
                s = 0;
                t = 0;
            }

            if (s >= 0 && s <= 1 && t >= 0 && t <= 1)
            {
                var closestPoint1 = start1 + line1 * s;
                var closestPoint2 = start2 + line2 * t;
                intersectSegment = closestPoint1 - closestPoint2;
                return math.lengthsq(closestPoint1 - closestPoint2);
            }
            else//如果发现两线上能取到的最短线段无法取得则轮流计算四个端点到对方线段的最短距离
            {
                var end1 = start1 + line1;
                var end2 = start2 + line2;
                var min = PointToSegmentSqrDistance(start1, start2, end2, out var closestPoint);
                intersectSegment = start1 - closestPoint;
                var distance2 = PointToSegmentSqrDistance(end1, start2, end2, out closestPoint);
                if (min > distance2)
                {
                    min = distance2;
                    intersectSegment = end1 - closestPoint;
                }

                var distance3 = PointToSegmentSqrDistance(start2, start1, end1, out closestPoint);
                if (min > distance3)
                {
                    min = distance3;
                    intersectSegment = closestPoint - start2;
                }
                
                var distance4 = PointToSegmentSqrDistance(end2,start1,end1,out closestPoint);

                if (min > distance4)
                {
                    min = distance4;
                    intersectSegment = closestPoint - end2;
                }

                return min;
            }
        }

    

        private static float GetPointSqrDistanceToTriangle(float3 point, ColliderTriangle triangle)
        {
            float3 a = triangle[0];
            float3 b = triangle[1];
            float3 c = triangle[2];
            float3 ab = b - a;
            float3 ac = c - a;
            float abSqr = math.lengthsq(ab);
            float bcSqr = math.lengthsq(c - b);
            float caSqr = math.lengthsq(ac);
            float longestEdgeSqr = math.max(abSqr, math.max(bcSqr, caSqr));
            float3 normal = math.cross(ab, ac);

            // 面积相对最长边过小时，将三角形退化为最长边；避免对共线或重合顶点做平面投影。
            if (longestEdgeSqr <= math.EPSILON ||
                math.lengthsq(normal) <= 1e-12f * longestEdgeSqr * longestEdgeSqr)
            {
                if (abSqr >= bcSqr && abSqr >= caSqr)
                {
                    return PointToSegmentSqrDistance(point, a, b, out _);
                }

                if (bcSqr >= caSqr)
                {
                    return PointToSegmentSqrDistance(point, b, c, out _);
                }

                return PointToSegmentSqrDistance(point, c, a, out _);
            }

            float3 pointProject = ProjectPointOntoPlane(point, math.normalize(normal), a, out float distance);
            if (CheckPointInTrangle(pointProject, triangle))
            {
                return distance * distance;
            }

            float minSqrDistanceOnPlane = math.min(
                math.min(
                    PointToSegmentSqrDistance(pointProject, a, b, out _),
                    PointToSegmentSqrDistance(pointProject, b, c, out _)),
                PointToSegmentSqrDistance(pointProject, c, a, out _));
            return minSqrDistanceOnPlane + distance * distance;
        }

        public static bool CheckSegmentIntersectWithPlane(float3 start, float3 segment, float3 normal,
            float3 pointOnPlane, out float3 intersectionPoint)
        {
            intersectionPoint = float3.zero;
            var d = math.dot(segment, normal);
            var n = math.dot(pointOnPlane - start, normal);
            if (math.abs(d) <= math.EPSILON) //说明与平面平行
            {
                return false;
            }

            var t = n / d;
            if (t >= 0f && t <= 1f)
            {
                intersectionPoint = start + t * segment;
                return true;
            }

            return false;
        }

        public static bool CheckPointInTrangle(float3 point, ColliderTriangle triangle)
        {
            var v1 = triangle[0] - triangle[1];
            var v2 = triangle[1] - triangle[2];
            var v3 = triangle[2] - triangle[0];
            var p1 = point - triangle[0];
            var p2 = point - triangle[1];
            var p3 = point - triangle[2];
            var triangleNormal = math.normalize(math.cross(v1, v2));
            if (math.length(math.cross(point - triangle[0], triangleNormal)) > math.EPSILON) return false;
            var cross1 = math.cross(v1, p1);
            var cross2 = math.cross(v2, p2);
            var cross3 = math.cross(v3, p3);
            return (math.dot(cross1, cross2) >= 0 && math.dot(cross2, cross3) >= 0 && math.dot(cross3, cross1) >= 0) ||
                   (math.dot(cross1, cross2) <= 0 && math.dot(cross2, cross3) <= 0 && math.dot(cross3, cross1) <= 0);
        }


        private static float3 ProjectPointOntoPlane(float3 point, float3 planeNormal, float3 planePoint,
            out float distance)
        {
            float3 toPoint = point - planePoint;
            var projectLine = (math.dot(toPoint, planeNormal) / math.lengthsq(planeNormal)) * planeNormal;
            distance = math.length(projectLine);
            return point - projectLine;
        }


        private static float PointToSegmentSqrDistance(float3 point, float3 start, float3 end, out float3 closestPoint)
        {
            float3 segment = end - start;
            float segmentSqrMagnitude = math.lengthsq(segment);
            if (segmentSqrMagnitude <= math.EPSILON)
            {
                closestPoint = start;
                return math.lengthsq(point - closestPoint);
            }

            // 将点投影到线段方向，并钳制到 [0, 1]，因此投影在线段外时会取最近端点。
            float t = math.saturate(math.dot(point - start, segment) / segmentSqrMagnitude);
            closestPoint = start + t * segment;
            return math.lengthsq(point - closestPoint);
        }
    }

    public struct ColliderTriangle2D
    {
        public float2 P0;
        public float2 P1;
        public float2 P2;

        public ColliderTriangle2D(float2 p0, float2 p1, float2 p2)
        {
            P0 = p0;
            P1 = p1;
            P2 = p2;
        }

        public float2 this[int index]
        {
            get
            {
                if (index <= 0) return P0;
                if (index == 1) return P1;
                return P2;
            }
        }
    }

}

#endregion

#region unSafe

namespace ColliderMath
{
    public static unsafe partial class GeometryMathTool
    {
        public static bool OverlapTriangle(
            BoundingBox bounds,
            ColliderTriangle triangle,
            float3* axes0,
            float3* axes1,
            float3* axes2)
        {
            if (ContainTriangle(bounds, triangle))
            {
                return true;
            }
            float3 extents = bounds.Extent;
            float3 v0 = triangle.P0 - bounds.Center;
            float3 v1 = triangle.P1 - bounds.Center;
            float3 v2 = triangle.P2 - bounds.Center;
            float3 f0 = v1 - v0;
            float3 f1 = v2 - v1;
            float3 f2 = v0 - v2;

            axes0[0] = new float3(0f, -f0.z, f0.y);
            axes0[1] = new float3(0f, -f1.z, f1.y);
            axes0[2] = new float3(0f, -f2.z, f2.y);
            axes0[3] = new float3(f0.z, 0f, -f0.x);
            axes0[4] = new float3(f1.z, 0f, -f1.x);
            axes0[5] = new float3(f2.z, 0f, -f2.x);
            axes0[6] = new float3(-f0.y, f0.x, 0f);
            axes0[7] = new float3(-f1.y, f1.x, 0f);
            axes0[8] = new float3(-f2.y, f2.x, 0f);

            if (!SatForAxes(axes0, 9, v0, v1, v2, extents))
            {
                return false;
            }

            axes1[0] = new float3(1f, 0f, 0f);
            axes1[1] = new float3(0f, 1f, 0f);
            axes1[2] = new float3(0f, 0f, 1f);
            if (!SatForAxes(axes1, 3, v0, v1, v2, extents))
            {
                return false;
            }

            axes2[0] = math.cross(f0, f1);
            return SatForAxes(axes2, 1, v0, v1, v2, extents);
        }

        /// <summary>
        /// 对于AABB包特化的SAT可以简化计算
        /// </summary>
        /// <param name="axes"></param>
        /// <param name="axisCount"></param>
        /// <param name="points"></param>
        /// <param name="pointCount"></param>
        /// <param name="extents"></param>
        /// <param name="interactSegment"></param>
        /// <returns></returns>
        public static bool SatForAxes(float3* axes, int axisCount, float3* points, int pointCount, float3 extents,out float3 interactSegment)
        {
            interactSegment = default;
            if (axes == null || axisCount <= 0 || points == null || pointCount <= 0)
            {
                return false;
            }

            float minOverlap = float.MaxValue;
            float3 minAxis = float3.zero;
            bool hasValidAxis = false;
            for (int i = 0; i < axisCount; i++)
            {
                float axisLengthSq = math.lengthsq(axes[i]);
                if (!math.isfinite(axisLengthSq) || axisLengthSq <= 1e-12f)
                {
                    continue;
                }

                float3 axis = axes[i] * math.rsqrt(axisLengthSq);
                float radius = extents.x * math.abs(axis.x) + extents.y * math.abs(axis.y) + extents.z * math.abs(axis.z); //这里就可以根据AABB半轴长直接得出轴上投影区间
                float min = float.MaxValue;
                float max = float.MinValue;
                for (int j = 0; j < pointCount; j++)
                {
                    float projection = math.dot(points[j], axis);
                    min = math.min(min, projection);
                    max = math.max(max, projection);
                }
                if (min > radius || max < -radius)
                {
                    return false;
                }

                //以下是在计算最小分离距离也就是使当前点集区间和AABB区间之间能够分离的最小距离
                float moveAlongNegativeAxis = radius - min;
                float moveAlongPositiveAxis = radius + max;
                float overlap = math.min(moveAlongNegativeAxis, moveAlongPositiveAxis);
                if (overlap < minOverlap)
                {
                    minOverlap = overlap;
                    minAxis = axis;
                }

                hasValidAxis = true;
            }

            if (!hasValidAxis)
            {
                return false;
            }

            interactSegment = minAxis * minOverlap;
            return true;
        }

        /// <summary>
        /// 对AABB和Capsule特化的SAT
        /// </summary>
        /// <param name="axes"></param>
        /// <param name="axisCount"></param>
        /// <param name="points"></param>
        /// <param name="pointCount"></param>
        /// <param name="extents"></param>
        /// <param name="pointRadius"></param>
        /// <param name="interactSegment"></param>
        /// <returns></returns>
        public static bool SatForAxes(
           float3* axes,
           int axisCount,
           float3* points,
           int pointCount,
           float3 extents,
           float pointRadius,
           out float3 interactSegment)
        {
           interactSegment = default;
           if (axes == null || axisCount <= 0 ||
               points == null || pointCount <= 0 ||
               !math.isfinite(pointRadius) || pointRadius < 0f)
           {
               return false;
           }

           float minOverlap = float.MaxValue;
           float3 minAxis = float3.zero;
           bool hasValidAxis = false;
           for (int i = 0; i < axisCount; i++)
           {
               float3 axis = axes[i];
               float axisLengthSq = math.lengthsq(axis);
               if (!math.isfinite(axisLengthSq) || axisLengthSq <= 1e-12f)
               {
                   continue;
               }

               float axisLength = math.sqrt(axisLengthSq);
               float boxRadius = extents.x * math.abs(axis.x) +
                                 extents.y * math.abs(axis.y) +
                                 extents.z * math.abs(axis.z);
               float segmentMin = float.MaxValue;
               float segmentMax = float.MinValue;
               for (int j = 0; j < pointCount; j++)
               {
                   float projection = math.dot(points[j], axis);
                   segmentMin = math.min(segmentMin, projection);
                   segmentMax = math.max(segmentMax, projection);
               }

               float capsuleRadius = pointRadius * axisLength;
               float capsuleMin = segmentMin - capsuleRadius;
               float capsuleMax = segmentMax + capsuleRadius;//因为胶囊体的特性是为一条线段在每一条轴上扩展了r因为其边界也需要朝该轴的政府方向上分别扩展r的距离
               if (capsuleMin > boxRadius || capsuleMax < -boxRadius)
               {
                   return false;
               }
               float overlap = math.min(boxRadius, capsuleMax) - math.max(-boxRadius, capsuleMin);//重合范围即为区间相交距离
               if (overlap < minOverlap)
               {
                   minOverlap = overlap;
                   minAxis = axis / axisLength;
               }

               hasValidAxis = true;
           }

           if (!hasValidAxis)
           {
               return false;
           }

           float segmentCenterProjection = 0f;
           for (int i = 0; i < pointCount; i++)
           {
               segmentCenterProjection += math.dot(points[i], minAxis);
           }

           segmentCenterProjection /= pointCount;
           if (segmentCenterProjection < 0f)
           {
               minAxis = -minAxis;
           }

           interactSegment = minAxis * minOverlap;
           return true;
        }
 
        private static bool SatForAxes(float3* axes, int axisCount, float3 v0, float3 v1, float3 v2, float3 extents)
        {
            for (int i = 0; i < axisCount; i++)
            {
                float3 axis = axes[i];
                float radius = extents.x * math.abs(axis.x) +
                               extents.y * math.abs(axis.y) +
                               extents.z * math.abs(axis.z);
                float p0 = math.dot(v0, axis);
                float p1 = math.dot(v1, axis);
                float p2 = math.dot(v2, axis);
                if (math.max(-math.max(p0, math.max(p1, p2)), math.min(p0, math.min(p1, p2))) > radius)
                {
                    return false;
                }
            }

            return true;
        }
    }
}

#endregion