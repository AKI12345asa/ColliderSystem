using System;
using ColliderMath;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Mathematics;

/// <summary>
/// EPA通过GJK中获取到的单纯形来计算两个凸形状之间的最小距离和接触点。它使用了一个固定容量的最小堆来存储当前单纯形的面，并在每次迭代中找到距离原点最近的面，然后计算该面的支持点。如果支持点与面之间的距离小于给定的容差，则认为找到了最小距离，否则将支持点添加到单纯形中并继续迭代。
/// </summary>
public static unsafe class EPATool
{
    private enum SupportMode
    {
        PointSets,
        Box,
        Sphere,
        Capsule
    }

    public static bool EPA(float3* points1,int length1,float3* points2,int length2,float3* tetrahedron,int capacity, out float3 disLine,float tolerance = 1e-6f)
    {
        return EPAInternal(
            points1, length1, points2, length2, tetrahedron, capacity,
            SupportMode.PointSets, float3.zero, 0f, float3.zero, float3.zero, default,
            out disLine, tolerance);
    }

    public static bool EPA(float3* pointSet, int pointSetLength, BoxColliderEntity box, float3* tetrahedron, int capacity, out float3 disLine, float tolerance = 1e-6f)
    {
        if (!math.all(math.isfinite(box.Center)) ||
            !math.all(math.isfinite(box.Size)) ||
            !math.all(math.isfinite(box.Rotation.value)))
        {
            disLine = float3.zero;
            return false;
        }

        var orientedBox = new OrientedBoundingBox(box.Center, math.abs(box.Size) * 0.5f, math.normalizesafe(box.Rotation));
        return EPAInternal(
            pointSet, pointSetLength, null, 0, tetrahedron, capacity,
            SupportMode.Box, float3.zero, 0f, float3.zero, float3.zero, orientedBox,
            out disLine, tolerance);
    }

    public static bool EPA(float3* pointSet, int pointSetLength, SphereColliderEntity sphere, float3* tetrahedron, int capacity, out float3 disLine, float tolerance = 1e-6f)
    {
        if (!math.all(math.isfinite(sphere.Center)) || !math.isfinite(sphere.Radius) || sphere.Radius < 0f)
        {
            disLine = float3.zero;
            return false;
        }

        return EPAInternal(
            pointSet, pointSetLength, null, 0, tetrahedron, capacity,
            SupportMode.Sphere, sphere.Center, sphere.Radius, float3.zero, float3.zero, default,
            out disLine, tolerance);
    }

    public static bool EPA(float3* pointSet, int pointSetLength, CapsuleColliderEntity capsule, float3* tetrahedron, int capacity, out float3 disLine, float tolerance = 1e-6f)
    {
        if (!math.all(math.isfinite(capsule.Center)) ||
            !math.isfinite(capsule.Radius) || capsule.Radius < 0f ||
            !math.isfinite(capsule.Height) || capsule.Height < 0f ||
            !math.all(math.isfinite(capsule.Rotation.value)) ||
            capsule.Direction < 0 || capsule.Direction > 2)
        {
            disLine = float3.zero;
            return false;
        }

        if (!GeometryMathTool.CheckCapsuleSegment(capsule.Center, capsule.Rotation, capsule.Height, capsule.Radius, out var start, out var end))
        {
            return EPAInternal(
                pointSet, pointSetLength, null, 0, tetrahedron, capacity,
                SupportMode.Sphere, capsule.Center, capsule.Radius, float3.zero, float3.zero, default,
                out disLine, tolerance);
        }

        return EPAInternal(pointSet, pointSetLength, null, 0, tetrahedron, capacity, SupportMode.Capsule, float3.zero, capsule.Radius, start, end, default, out disLine, tolerance);
    }

    private static bool EPAInternal(float3* points1, int length1, float3* points2, int length2, float3* tetrahedron, int capacity, SupportMode supportMode,float3 shapeCenter, float shapeRadius, float3 capsuleStart, float3 capsuleEnd, OrientedBoundingBox box, out float3 disLine, float tolerance)
    {
        disLine = float3.zero;
        if (points1 == null || length1 <= 0 ||
            (supportMode == SupportMode.PointSets && (points2 == null || length2 <= 0)) ||
            tetrahedron == null ||
            capacity < 4 ||
            !math.isfinite(tolerance) || tolerance < 0f)
        {
            return false;
        }

        //创建一个会弹出距离原点最近的面的最小堆
        var byteCount = (long)UnsafeUtility.SizeOf<MinHeapEntry>() * capacity;
        var heapBuffer = (MinHeapEntry*)UnsafeUtility.Malloc(byteCount, UnsafeUtility.AlignOf<MinHeapEntry>(), Allocator.Temp);
        var minHeap = new MinHeap(heapBuffer, capacity);
        
        
        try
        {
            var planes = stackalloc ColliderTriangle[4];
            planes[0] = new ColliderTriangle(tetrahedron[0], tetrahedron[1], tetrahedron[2]);
            planes[1] = new ColliderTriangle(tetrahedron[0], tetrahedron[1], tetrahedron[3]);
            planes[2] = new ColliderTriangle(tetrahedron[0], tetrahedron[2], tetrahedron[3]);
            planes[3] = new ColliderTriangle(tetrahedron[1], tetrahedron[2], tetrahedron[3]);
            if (!minHeap.Add(planes, 4)) return false;
            const int maxIterations = 64;//迭代上限
            for (int iteration = 0; iteration < maxIterations; iteration++)
            {
                var entry = minHeap.GetTop();
                var minPlane = entry.Triangle;
                var planeNormal = entry.Normal;
                float3 support;//计算当前面在法线方向上的支持点
                switch (supportMode)
                {
                    case SupportMode.PointSets:
                        support = GJKTool.Support(points1, length1, points2, length2, planeNormal);
                        break;
                    case SupportMode.Box:
                        support = GJKTool.Support(points1, length1, box, planeNormal);
                        break;
                    case SupportMode.Sphere:
                        support = GJKTool.Support(points1, length1, shapeCenter, shapeRadius, planeNormal);
                        break;
                    case SupportMode.Capsule:
                        support = GJKTool.Support(points1, length1, capsuleStart, capsuleEnd, shapeRadius, planeNormal);
                        break;
                    default:
                        return false;
                }
                float normalLength = math.length(planeNormal);
                if (!math.isfinite(normalLength) || normalLength <= 1e-12f)//如果法线长度为0或者不是有限数,说明当前面是退化的,无法计算距离,直接返回false
                {
                    return false;
                }

                float signedGap = math.dot(planeNormal, support - minPlane.P0) / normalLength;
                if (!math.isfinite(signedGap) || signedGap < -tolerance)
                {
                    return false;
                }

                if (signedGap <= tolerance)//小于容差就说明当前求出的点已经是弹出的最小面已经是最接近闵可夫斯基差上原点的面了,所以直接返回距离和法线方向
                {
                    disLine = entry.Distance * (planeNormal / normalLength);
                    return  true;
                }
                if (!minHeap.AdjustWithSupport(support)) return false;
            }
            return false;
        }
        finally
        {
            UnsafeUtility.Free(heapBuffer, Allocator.Temp);
        }
    }
    
    private static float GetPointToPlaneDistance(float3 point,float3 planeNormal,float3 pointOnPlane)
    {
        return math.abs(math.dot(planeNormal, new float3(point - pointOnPlane))) * math.rsqrt(math.lengthsq(planeNormal));//这里用了下倒数乘运算来避免开方运算,提高性能
    }
/// <summary>
/// 一个规定了最大容量的最小堆,用于存储三角形和它们到原点的距离,以便在EPA算法中快速找到距离原点最近的三角形面。
/// </summary>
private struct MinHeap
{
    private MinHeapEntry* Heap;
    private int TotalSize;//最小堆规定了的最大容量超出直接判定为校验失败(主要是为了防止无限制的内存分配)
    private int HeapSize;

    public MinHeap(MinHeapEntry* heap, int totalSize)
    {
        if (totalSize < 0 || (totalSize > 0 && heap == null))
        {
            throw new ArgumentOutOfRangeException(nameof(totalSize));
        }

        Heap = heap;
        TotalSize = totalSize;
        HeapSize = 0;
    }

    /// <summary>
    /// 添加一组三角形到最小堆中，如果添加后堆的大小超过了总容量，则返回false。否则返回true。
    /// </summary>
    /// <param name="triangles"></param>
    /// <param name="length"></param>
    /// <returns></returns>
    public bool Add(ColliderTriangle* triangles, int length)
    {
        if (length < 0 || (length > 0 && triangles == null) || HeapSize + length > TotalSize)
        {
            return false;
        }

        if (length == 0)
        {
            return true;
        }

        bool buildHeap = HeapSize == 0;
        for (int i = 0; i < length; i++)
        {
            int index = HeapSize++;
            Heap[index] = new MinHeapEntry(triangles[i], GetPointToPlaneDistance(float3.zero,triangles[i].GetNormal(true),triangles[i].P0));
            if (!buildHeap)
            {
                SiftUp(index);
            }
        }

        if (buildHeap)
        {
            BuildHeap();
        }

        return true;
    }
    
    private bool Add(ColliderTriangle triangle)
    {
        if (HeapSize == TotalSize) return false;
        bool buildHeap = HeapSize == 0;
        int index = HeapSize++;
        Heap[index] = new MinHeapEntry(triangle, GetPointToPlaneDistance(float3.zero,triangle.GetNormal(true),triangle.P0));
        if (!buildHeap) 
        {
                SiftUp(index);
        }
        if (buildHeap)
        {
            BuildHeap();
        }

        return true;
    }

    public MinHeapEntry GetTop()
    {
        if (HeapSize == 0)
        {
            return default;
        }
        return Heap[0];
    }

    private void BuildHeap()
    {
        for (int i = HeapSize / 2; i > 0; i--)
        {
            SiftDown(i - 1);
        }
    }

    /// <summary>
    /// 向最小堆维护某个节点的下沉操作其实也就是维护这个点的子树的最小堆性质
    /// </summary>
    /// <param name="index"></param>
    private void SiftDown(int index)
    {
        while (true)
        {
            int left = index * 2 + 1;
            if (left >= HeapSize)
            {
                return;
            }

            int right = left + 1;
            int smallest = left;
            if (right < HeapSize && Heap[right].Distance < Heap[left].Distance)
            {
                smallest = right;
            }

            if (Heap[index].Distance <= Heap[smallest].Distance)
            {
                return;
            }

            Swap(index, smallest);
            index = smallest;
        }
    }

    
    
    public bool AdjustWithSupport(float3 support,float visibilityTolerance = 1e-6f)
    {
        var originalSize = HeapSize;
        int entryIndex = 0;
        while (entryIndex < HeapSize)
        {
            var entry = Heap[entryIndex];
            float normalLength = math.length(entry.Normal);
            if (normalLength <= 1e-12f)
            {
                entryIndex++;
                continue;
            }

            //校验当前点与三角形的法线点乘,如果距离大于容差则说明该点在三角形的法线方向上,不在三角形的可见范围内,属于会被当前三角面遮挡的面
            float signedDistance = math.dot(entry.Normal, support - entry.Triangle.P0) / normalLength;
            if (signedDistance > visibilityTolerance)
            {
                //将不要的值与堆的最后一个值交换,然后将堆的大小减一,这样就可以在不破坏堆结构的情况下移除该值
                (Heap[HeapSize - 1], Heap[entryIndex]) = (Heap[entryIndex], Heap[HeapSize - 1]);
                HeapSize--;
            }
            else
            {
                entryIndex++;
            }
        }
        BuildHeap();//所有值都被移除后,堆的结构可能被破坏,所以需要重新构建堆
        var edgeStack = stackalloc Edge[(originalSize - HeapSize) * 3];//每个三角形有3条边,所以需要3倍的空间来存储所有的边
        var edgeCount = 0;
        for (var i = HeapSize; i < originalSize; i++) //这里是因为去除是将不要的面放在了堆的后面,所以需要从堆的大小开始遍历到原始大小,这样就可以获取到所有被移除的三角形
        {
            var triangle1 = Heap[i].Triangle;
            for (var j = 0; j < HeapSize; j++)
            {
                var triangle2= Heap[j].Triangle;
                if (!triangle1.TryGetCommonEdge(triangle2, out var commonEdge)) continue;//获取不可见面和可见面的公共线用于构建新的三角形,如果没有公共线则说明两个三角形不相交,所以直接跳过
                edgeStack[edgeCount++] = commonEdge;
            }
        }

        QuickSort(edgeStack,edgeCount);//快速排序根据这些线使得相同的线排在一起,这样就可以在后续的循环中去除重复的线,避免重复构建三角形
        for (int i = 0; i < edgeCount;)
        {
            int next = i + 1;
            while (next < edgeCount && edgeStack[i].Equals(edgeStack[next]))
            {
                next++;
            }
            if (!Add(new ColliderTriangle(support, edgeStack[i].P0, edgeStack[i].P1))) return false;
            i = next;
        }
        return true;
    }

    private static void QuickSort(Edge* edges, int length)
    {
        if (edges == null || length < 2)
        {
            return;
        }

        int* rangeStack = stackalloc int[64];//栈深度用于存储每次分割的范围,避免递归调用,提高性能
        int stackCount = 0;
        int low = 0;
        int high = length - 1;

        while (true)
        {
            while (low < high)
            {
                int left = low;
                int right = high;
                Edge pivot = edges[low + (high - low) / 2];

                while (left <= right)
                {
                    while (edges[left].CompareTo(pivot) < 0)left++;
                    while (edges[right].CompareTo(pivot) > 0)  right--;

                    if (left <= right)
                    {
                        (edges[left], edges[right]) = (edges[right], edges[left]);
                        left++;
                        right--;
                    }
                }

                int leftLength = right - low + 1;
                int rightLength = high - left + 1;

                
                //一种快速排序的取舍逻辑也就是当右侧长度比左侧长度更大时则优先处理,优先处理更长的那一侧之后在栈内存储另一侧的范围,这样可以减少栈的深度,提高性能
                if (leftLength < rightLength)
                {
                    if (left < high)
                    {
                        rangeStack[stackCount++] = left;
                        rangeStack[stackCount++] = high;
                    }

                    high = right;
                }
                else
                {
                    if (low < right)
                    {
                        rangeStack[stackCount++] = low;
                        rangeStack[stackCount++] = right;
                    }

                    low = left;
                }
            }

            if (stackCount == 0)
            {
                return;
            }

            high = rangeStack[--stackCount];
            low = rangeStack[--stackCount];
        }
    }

    
    /// <summary>
    /// 向上校验用于将某个点插入到最小堆中后，确保堆的性质仍然成立。它通过比较当前节点与其父节点的距离，如果当前节点的距离小于父节点的距离，则交换它们的位置，并继续向上检查，直到达到堆顶或找到一个不需要交换的父节点为止。
    /// </summary>
    /// <param name="index"></param>
    private void SiftUp(int index)
    {
        while (index > 0)
        {
            int parent = (index - 1) / 2;
            if (Heap[parent].Distance <= Heap[index].Distance)
            {
                return;
            }

            Swap(parent, index);
            index = parent;
        }
    }

    private void Swap(int indexA, int indexB)
    {
        (Heap[indexA], Heap[indexB]) = (Heap[indexB], Heap[indexA]);
    }
}


}
