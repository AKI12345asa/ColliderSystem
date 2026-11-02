using ColliderMath;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Mathematics;
public static unsafe class GJKTool
    {
        private enum GJKType
        {
            MeshSphere,
            MeshBox,
            MeshCapsule
        }

        /// <summary>使用 GJK 判断两个非空点集所构成的凸包是否相交。</summary>
        public static bool GJK(NativeArray<float3> pointSet1, NativeArray<float3> pointSet2,float3* simplex, float tolerance = 1e-6f)
        {
            if (!pointSet1.IsCreated || !pointSet2.IsCreated ||
                pointSet1.Length == 0 || pointSet2.Length == 0 ||
                !math.isfinite(tolerance) || tolerance < 0f)
            {
                return false;
            }

            return GJK((float3*)pointSet1.GetUnsafeReadOnlyPtr(), pointSet1.Length, (float3*)pointSet2.GetUnsafeReadOnlyPtr(), pointSet2.Length,simplex,tolerance);
        }
        

        /// <summary>使用 GJK 判断两个三角形凸包是否相交，不分配托管或 Native 容器。</summary>
        public static bool GJK(ColliderTriangle triangle1, ColliderTriangle triangle2,float3* simplex,float tolerance = 1e-6f)
        {
            if (!math.isfinite(tolerance) || tolerance < 0f)
            {
                return false;
            }
            float3* points1 = stackalloc float3[3];
            float3* points2 = stackalloc float3[3];
            points1[0] = triangle1.P0;
            points1[1] = triangle1.P1;
            points1[2] = triangle1.P2;
            points2[0] = triangle2.P0;
            points2[1] = triangle2.P1;
            points2[2] = triangle2.P2;
            return GJK(points1, 3, points2, 3,simplex,  tolerance);
        }

        public static bool GJK(ColliderTriangle triangle, BoxColliderEntity box, float3* simplex, float tolerance = 1e-6f)
        {
            float3* trianglePoints = stackalloc float3[3];
            trianglePoints[0] = triangle.P0;
            trianglePoints[1] = triangle.P1;
            trianglePoints[2] = triangle.P2;
            return GJK(trianglePoints, 3, box, simplex, tolerance);
        }

        public static bool GJK(ColliderTriangle triangle, SphereColliderEntity sphere, float3* simplex, float tolerance = 1e-6f)
        {
            float3* trianglePoints = stackalloc float3[3];
            trianglePoints[0] = triangle.P0;
            trianglePoints[1] = triangle.P1;
            trianglePoints[2] = triangle.P2;
            return GJK(trianglePoints, 3, sphere, simplex, tolerance);
        }

        public static bool GJK(ColliderTriangle triangle, CapsuleColliderEntity capsule, float3* simplex, float tolerance = 1e-6f)
        {
            float3* trianglePoints = stackalloc float3[3];
            trianglePoints[0] = triangle.P0;
            trianglePoints[1] = triangle.P1;
            trianglePoints[2] = triangle.P2;
            return GJK(trianglePoints, 3, capsule, simplex, tolerance);
        }
        

        public static bool GJK(float3* pointSet, int pointSetLength, BoxColliderEntity box, float3* simplex, float tolerance = 1e-6f)
        {
            if (!IsValidPointSet(pointSet, pointSetLength, simplex, tolerance) ||
                !IsFinite(box.Center) || !IsFinite(box.Size) ||
                !IsFinite(box.Rotation.value))
            {
                return false;
            }

            OrientedBoundingBox orientedBox = new OrientedBoundingBox(box.Center, math.abs(box.Size) * 0.5f, math.normalizesafe(box.Rotation));
            return GJKAgainstShape(pointSet, pointSetLength, orientedBox.Center, GJKType.MeshBox, 0f, float3.zero, float3.zero, orientedBox, simplex, tolerance);
        }

        public static bool GJK(float3* pointSet, int pointSetLength, SphereColliderEntity sphere, float3* simplex, float tolerance = 1e-6f)
        {
            if (!IsValidPointSet(pointSet, pointSetLength, simplex, tolerance) ||
                !IsFinite(sphere.Center) || !math.isfinite(sphere.Radius) || sphere.Radius < 0f)
            {
                return false;
            }

            return GJKAgainstShape(pointSet, pointSetLength, sphere.Center, GJKType.MeshSphere, sphere.Radius, float3.zero, float3.zero, default, simplex, tolerance);
        }

        public static bool GJK(float3* pointSet, int pointSetLength, CapsuleColliderEntity capsule, float3* simplex, float tolerance = 1e-6f)
        {
            if (!IsValidPointSet(pointSet, pointSetLength, simplex, tolerance) ||
                !IsFinite(capsule.Center) ||
                !math.isfinite(capsule.Radius) || capsule.Radius < 0f ||
                !math.isfinite(capsule.Height) || capsule.Height < 0f ||
                !IsFinite(capsule.Rotation.value) ||
                capsule.Direction < 0 || capsule.Direction > 2)
            {
                return false;
            }

            if (!GeometryMathTool.CheckCapsuleSegment(capsule.Center, capsule.Rotation, capsule.Height, capsule.Radius, out var start, out var end))
            {
                return GJKAgainstShape(pointSet, pointSetLength, capsule.Center, GJKType.MeshSphere, capsule.Radius, float3.zero, float3.zero, default, simplex, tolerance);
            }
            return GJKAgainstShape(pointSet, pointSetLength, capsule.Center, GJKType.MeshCapsule, capsule.Radius, start, end, default, simplex, tolerance);
        }

        private static bool GJKAgainstShape(float3* pointSet, int pointSetLength, float3 shapeCenter, GJKType shapeType, float radius, float3 segmentStart, float3 segmentEnd, OrientedBoundingBox box, float3* simplex, float tolerance)
        {
            float3 pointSetCenter = GetBoundsCenter(pointSet, pointSetLength);
            float3 direction = math.normalizesafe(shapeCenter - pointSetCenter, new float3(1f, 0f, 0f));
            int simplexCount = 1;
            simplex[0] = SupportAgainstShape(pointSet, pointSetLength, shapeType, shapeCenter, radius, segmentStart, segmentEnd, box, direction);
            direction = -simplex[0];

            float toleranceSquared = tolerance * tolerance;
            for (int iteration = 0; iteration < 64; iteration++)
            {
                if (math.lengthsq(direction) <= toleranceSquared)
                {
                    return true;
                }

                float3 point = SupportAgainstShape(pointSet, pointSetLength, shapeType, shapeCenter, radius, segmentStart, segmentEnd, box, direction);
                float supportProjection = math.dot(point, direction);
                if (supportProjection < -tolerance * math.length(direction))
                {
                    return false;
                }

                for (int i = 0; i < simplexCount; i++)
                {
                    if (math.lengthsq(simplex[i] - point) <= toleranceSquared)
                    {
                        return supportProjection <= tolerance * math.length(direction);
                    }
                }

                simplex[simplexCount++] = point;
                if (HandleSimplex(simplex, ref simplexCount, ref direction, tolerance))
                {
                    return true;
                }
            }

            return false;
        }

        /// <summary>
        /// 通过对应Type获取该法线方向上的支撑点用于迭代
        /// </summary>
        /// <param name="pointSet"></param>
        /// <param name="pointSetLength"></param>
        /// <param name="shapeType"></param>
        /// <param name="shapeCenter"></param>
        /// <param name="radius"></param>
        /// <param name="segmentStart"></param>
        /// <param name="segmentEnd"></param>
        /// <param name="box"></param>
        /// <param name="direction"></param>
        /// <returns></returns>
        private static float3 SupportAgainstShape(float3* pointSet, int pointSetLength, GJKType shapeType, float3 shapeCenter, float radius, float3 segmentStart, float3 segmentEnd, OrientedBoundingBox box, float3 direction)
        {
            switch (shapeType)
            {
                case GJKType.MeshSphere:
                    return Support(pointSet, pointSetLength, shapeCenter, radius, direction);
                case GJKType.MeshCapsule:
                    return Support(pointSet, pointSetLength, segmentStart, segmentEnd, radius, direction);
                case GJKType.MeshBox:
                    return Support(pointSet, pointSetLength, box, direction);
                default:
                    return float3.zero;
            }
        }

        private static bool IsValidPointSet(float3* pointSet, int pointSetLength, float3* simplex, float tolerance)
        {
            return pointSet != null && pointSetLength > 0 && simplex != null &&
                   math.isfinite(tolerance) && tolerance >= 0f;
        }

        private static bool IsFinite(float3 value)
        {
            return math.all(math.isfinite(value));
        }

        private static bool IsFinite(float4 value)
        {
            return math.all(math.isfinite(value));
        }

        private static bool GJK(float3* pointSet1, int pointSet1Length, float3* pointSet2, int pointSet2Length,float3* simplex,float tolerance)
        {
            float3 center1 = GetBoundsCenter(pointSet1, pointSet1Length);
            float3 center2 = GetBoundsCenter(pointSet2, pointSet2Length);
            float3 direction = math.normalizesafe(center2 - center1, new float3(1f, 0f, 0f));
            
            int simplexCount = 1;
            simplex[0] = Support(pointSet1, pointSet1Length, pointSet2, pointSet2Length, direction);
            direction = -simplex[0];

            const int maxIterations = 64;
            float toleranceSquared = tolerance * tolerance;
            for (int iteration = 0; iteration < maxIterations; iteration++)
            {
                if (math.lengthsq(direction) <= toleranceSquared)
                {
                    return true;
                }

                float3 point = Support(pointSet1, pointSet1Length, pointSet2, pointSet2Length, direction);
                float supportProjection = math.dot(point, direction);
                if (supportProjection < -tolerance * math.length(direction))
                {
                    return false;
                }

                for (int i = 0; i < simplexCount; i++)
                {
                    if (math.lengthsq(simplex[i] - point) <= toleranceSquared)
                    {
                        return supportProjection <= tolerance * math.length(direction);
                    }
                }

                simplex[simplexCount++] = point;
                if (HandleSimplex(simplex, ref simplexCount, ref direction, tolerance))
                {
                    return true;
                }
            }

            return false;
        }

        private static float3 GetBoundsCenter(float3* points, int length)
        {
            float3 min = points[0];
            float3 max = points[0];
            for (int i = 1; i < length; i++)
            {
                min = math.min(min, points[i]);
                max = math.max(max, points[i]);
            }

            return (min + max) * 0.5f;
        }

        public static float3 Support(float3* pointSet1, int pointSet1Length, float3* pointSet2, int pointSet2Length, float3 direction)
        {
            float3 p1 = FurthestPointInDirection(pointSet1, pointSet1Length, direction);
            float3 p2 = FurthestPointInDirection(pointSet2, pointSet2Length, -direction);
            return p1 - p2;
        }

        /// <summary>
        /// sphere上取用沿着法线负方向上最大的那个点就是圆面上那个点
        /// </summary>
        /// <param name="pointSet"></param>
        /// <param name="pointSetLength"></param>
        /// <param name="center"></param>
        /// <param name="radius"></param>
        /// <param name="direction"></param>
        /// <returns></returns>
        public static float3 Support(float3* pointSet,int pointSetLength,float3 center,float radius,float3 direction)
        {
            float3 p1 = FurthestPointInDirection(pointSet, pointSetLength, direction);
            float3 p2 = center + radius * math.normalizesafe(-direction);
            return p1 - p2;
        }

        /// <summary>
        /// 胶囊体上的取点沿着法线最远的点就是首尾两点上点积更大的那个点再沿着半径方向扩展得到的
        /// </summary>
        /// <param name="pointSet"></param>
        /// <param name="pointSetLength"></param>
        /// <param name="start"></param>
        /// <param name="end"></param>
        /// <param name="radius"></param>
        /// <param name="direction"></param>
        /// <returns></returns>
        public static float3 Support(float3* pointSet, int pointSetLength, float3 start, float3 end, float radius,float3 direction)
        {
            float3 p1 = FurthestPointInDirection(pointSet, pointSetLength, direction);
            float3 p2 = start;
            float dot = math.dot(start, -direction);
            if (dot < math.dot(end, -direction)) p2 = end;
            p2 -= radius * math.normalizesafe(direction);
            return p1 - p2;
        }
        
        /// <summary>
        /// obb取点则是沿着每一条轴上可以扩大这个点积的方向上去取得那个端点
        /// </summary>
        /// <param name="pointSet"></param>
        /// <param name="pointSetLength"></param>
        /// <param name="box"></param>
        /// <param name="direction"></param>
        /// <returns></returns>
        public static float3 Support(float3* pointSet, int pointSetLength, OrientedBoundingBox box, float3 direction)
        {
            int Signed(float3 axis)
            {
                return math.dot(axis, -direction) > 0 ? 1 : -1;
            }
            float3 p1 = FurthestPointInDirection(pointSet, pointSetLength, direction);
            float3 p2 = box.Center + Signed(box.AxisX) * box.AxisX * box.Extents.x + Signed(box.AxisY) * box.AxisY * box.Extents.y + box.Extents.z * Signed(box.AxisZ) * box.AxisZ;
            return p1 - p2;
        }

        /// <summary>
        /// 遍历得到点积最大点
        /// </summary>
        /// <param name="pointSet"></param>
        /// <param name="length"></param>
        /// <param name="direction"></param>
        /// <returns></returns>
        private static float3 FurthestPointInDirection(float3* pointSet, int length, float3 direction)
        {
            float3 maxPoint = pointSet[0];
            float maxDot = math.dot(maxPoint, direction);
            for (int i = 1; i < length; i++)
            {
                float dot = math.dot(pointSet[i], direction);
                if (dot > maxDot)
                {
                    maxDot = dot;
                    maxPoint = pointSet[i];
                }
            }

            return maxPoint;
        }

        private static bool HandleSimplex(float3* simplex, ref int simplexCount, ref float3 direction, float tolerance)
        {
            if (simplexCount == 2)
            {
                return HandleLineSimplex(simplex, ref simplexCount, ref direction, tolerance);
            }

            if (simplexCount == 3)
            {
                return HandleTriangle(simplex, ref simplexCount, ref direction, tolerance);
            }

            return HandleTetrahedron(simplex, ref simplexCount, ref direction, tolerance);
        }

        
        /// <summary>
        /// 处理四面体:
        /// </summary>
        /// <param name="simplex"></param>
        /// <param name="simplexCount"></param>
        /// <param name="direction"></param>
        /// <param name="tolerance"></param>
        /// <returns></returns>
        private static bool HandleTetrahedron(float3* simplex, ref int simplexCount, ref float3 direction, float tolerance)
        {
            float3 a = simplex[0];
            float3 b = simplex[1];
            float3 c = simplex[2];
            float3 d = simplex[3];
            float3 abcNormal = math.cross(b - a, c - a);
            float baseNormalMagnitude = math.length(abcNormal);

            if (baseNormalMagnitude <= float.Epsilon || math.abs(math.dot(abcNormal, d - a)) <= tolerance * baseNormalMagnitude)
            {
                simplexCount = 3;
                return HandleTriangle(simplex, ref simplexCount, ref direction, tolerance);
            }

            if (IsOriginOutsideFace(a, b, d, c, tolerance, out direction))
            {
                //这里的取舍原则也是一样的因为d一定是更靠近原点的那个可能顶点因此不考虑舍去d的情况
                //而原点如果在某个四面体面外部那么一定离这个面更近因此舍去不是这个面上的点就可以了
                SetSimplexToFace(simplex, ref simplexCount, a, b, d);
                return false;
            }

            if (IsOriginOutsideFace(b, c, d, a, tolerance, out direction))
            {
                SetSimplexToFace(simplex, ref simplexCount, b, c, d);
                return false;
            }

            if (IsOriginOutsideFace(c, a, d, b, tolerance, out direction))
            {
                SetSimplexToFace(simplex, ref simplexCount, c, a, d);
                return false;
            }

            return true;
        }

        private static bool IsOriginOutsideFace(float3 a, float3 b, float3 c, float3 opposite, float tolerance, out float3 outwardNormal)
        {
            outwardNormal = math.cross(b - a, c - a);
            float normalMagnitude = math.length(outwardNormal);
            if (normalMagnitude <= float.Epsilon)
            {
                return false;
            }

            if (math.dot(outwardNormal, opposite - a) > 0f)//保证当前用于判断的法线是指向单纯性外部的
            {
                outwardNormal = -outwardNormal;
            }

            return math.dot(outwardNormal, -a) > tolerance * normalMagnitude;
        }
        
        private static void SetSimplexToFace(float3* simplex, ref int simplexCount, float3 a, float3 b, float3 c)
        {
            simplex[0] = a;
            simplex[1] = b;
            simplex[2] = c;
            simplexCount = 3;
        }

        /// <summary>
        /// 处理线段:根据原点投影是否在线段范围内判断是否有必要沿着当前支撑点取继续迭代
        /// </summary>
        /// <param name="simplex"></param>
        /// <param name="simplexCount"></param>
        /// <param name="direction"></param>
        /// <param name="tolerance"></param>
        /// <returns></returns>
        private static bool HandleLineSimplex(float3* simplex, ref int simplexCount, ref float3 direction, float tolerance)
        {
            float3 a = simplex[0];
            float3 b = simplex[1];
            float3 ab = b - a;
            float3 ao = -a;

            float distanceSquared = PointToSegmentDistanceSquared(float3.zero, a, b, out _);
            if (distanceSquared <= tolerance * tolerance)
            {
                return true;
            }

            float abSquared = math.lengthsq(ab);
            if (abSquared <= tolerance * tolerance)
            {
                simplexCount = 1;
                direction = ao;
                return false;
            }

            float projection = math.dot(ao, ab) / abSquared;
            if (projection <= 0f)
            {
                simplexCount = 1;
                direction = ao;
            }
            else if (projection >= 1f)
            {
                simplex[0] = b;
                simplexCount = 1;
                direction = -b;
            }
            else
            {
                direction = math.cross(math.cross(ab, ao), ab);
            }

            return false;
        }
        
        /// <summary>
        /// 处理三角形:根据当前原点投影是否在落在当前三角面上来判断因为如果没有落在面上可以从两种角度考虑:
        /// 1.当前根本就迭代不出包含原点的单纯形那么这种回退就是无所谓的反正最后也会排除
        /// 2.当前可以迭代出包含原点的单纯性,那么一个包含原点的单纯形(四面体)那么肯定含有一个原点投影在其上是三角形内部的面,而当前就是那个不在的面因此我们需要舍去当前面朝着更有可能的那个面去迭代
        /// </summary>
        /// <param name="simplex"></param>
        /// <param name="simplexCount"></param>
        /// <param name="direction"></param>
        /// <param name="tolerance"></param>
        /// <returns></returns>
        private static bool HandleTriangle(float3* simplex, ref int simplexCount, ref float3 direction, float tolerance)
        {
            float3 a = simplex[0];
            float3 b = simplex[1];
            float3 c = simplex[2];
            float3 ab = b - a;
            float3 ca = a - c;
            float3 co = -c;
            float3 cb = b - c;
            float3 abcNormal = math.cross(ab, ca);
            float longestEdgeSquared = math.max(math.lengthsq(ab),
                math.max(math.lengthsq(ca), math.lengthsq(cb)));
            float longestEdgeMagnitude = math.sqrt(longestEdgeSquared);

            //三角形过小则退化
            if (math.length(abcNormal) <= tolerance * longestEdgeMagnitude)
            {
                return ReduceToNearestAdjacentEdge(
                    simplex, ref simplexCount, ref direction, c, a, b, float3.zero, tolerance);
            }

            if (math.dot(co, abcNormal) < 0f)
            {
                abcNormal = -abcNormal;
            }

            float3 coInAbcProject = co - abcNormal * (math.dot(co, abcNormal) / math.lengthsq(abcNormal));
            float3 oInAbcProject = c + coInAbcProject;
            float3 caNormal = math.cross(ca, abcNormal);
            float3 cbNormal = math.cross(cb, abcNormal);

            if (math.dot(caNormal, cb) > 0f)
            {
                caNormal = -caNormal;
            }

            if (math.dot(cbNormal, ca) > 0f)
            {
                cbNormal = -cbNormal;
            }

           
            if (math.dot(caNormal, coInAbcProject) <= tolerance * math.length(caNormal) && math.dot(cbNormal, coInAbcProject) <= tolerance * math.length(cbNormal))
            {
                if (math.lengthsq(oInAbcProject) <= tolerance * tolerance)
                {
                    return true;
                }

                direction = abcNormal;
            }
            else
            {
                return ReduceToNearestAdjacentEdge(simplex, ref simplexCount, ref direction, c, a, b, oInAbcProject, tolerance);
            }

            return false;
        }

        /// <summary>
        /// 这里不考虑ab是因为我们的c点是朝着原点上去查找到在GJK的定义上这个点是更接近原点的点因此不考虑舍去c的情况
        /// </summary>
        /// <param name="simplex"></param>
        /// <param name="simplexCount"></param>
        /// <param name="direction"></param>
        /// <param name="newest"></param>
        /// <param name="oldA"></param>
        /// <param name="oldB"></param>
        /// <param name="point"></param>
        /// <param name="tolerance"></param>
        /// <returns></returns>
        private static bool ReduceToNearestAdjacentEdge(float3* simplex, ref int simplexCount, ref float3 direction, float3 newest, float3 oldA, float3 oldB, float3 point, float tolerance)
        {
            float caDistanceSquared = PointToSegmentDistanceSquared(point, newest, oldA, out float3 closestOnCA);
            float cbDistanceSquared = PointToSegmentDistanceSquared(point, newest, oldB, out float3 closestOnCB);
            float3 closestPoint;
            if (caDistanceSquared <= cbDistanceSquared)
            {
                closestPoint = closestOnCA;
                simplex[0] = oldA;
                simplex[1] = newest;
            }
            else
            {
                closestPoint = closestOnCB;
                simplex[0] = oldB;
                simplex[1] = newest;
            }

            simplexCount = 2;
            if (math.lengthsq(closestPoint) <= tolerance * tolerance)
            {
                return true;
            }

            direction = -closestPoint;
            return false;
        }

        private static float PointToSegmentDistanceSquared(float3 point, float3 start, float3 end, out float3 closestPoint)
        {
            float3 segment = end - start;
            float segmentSquared = math.lengthsq(segment);
            if (segmentSquared <= float.Epsilon)
            {
                closestPoint = start;
                return math.lengthsq(point - start);
            }

            float t = math.saturate(math.dot(point - start, segment) / segmentSquared);
            closestPoint = start + t * segment;
            return math.lengthsq(point - closestPoint);
        }
    }
