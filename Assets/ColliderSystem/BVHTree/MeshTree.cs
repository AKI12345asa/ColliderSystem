using System;
using System.Collections.Generic;
using ColliderMath;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using Unity.Mathematics;
/// <summary>
/// 这里会让多个Mesh公用一颗MeshTree以节约存储空间
/// </summary>
public unsafe class MeshTree : IDisposable
{
    private readonly int MaxCoordinateCount;
    private readonly int MaxRank;
    private readonly float3 CellSize;
    private NativeArray<BVTMortonData> MortonData;
    private NativeArray<BVTNode> Nodes;
    private NativeArray<BVTBuildTask> BuildTasks;
    private NativeArray<int> RootIndies;
    private NativeArray<MeshTriangleEntity> MeshTriangleEntities;
    private NativeArray<float3> MeshOrigins;
    private NativeArray<int> TriangleStarts;
    private NativeArray<int> NodeStarts;
    private NativeArray<int> TaskStarts;
    private NativeArray<int> SortLeft;
    private NativeArray<int> SortRight;
    private JobHandle BuildHandle;
    private bool HasScheduledBuild;

    #region Intersect

    /// <summary>
    /// 现根据本地MeshTree去粗筛可能碰撞的三角面如果判断相交则再进行具体的GJK运算后续所有的Mesh相关的碰撞运算都是基于这个逻辑
    /// 筛选的逻辑就是把other碰撞体的AABB转化为本地坐标系下的可包围原先AABB的包围盒之后进行粗筛再带入GJK
    /// </summary>
    /// <param name="mesh"></param>
    /// <param name="box"></param>
    /// <param name="meshLocalToWorld"></param>
    /// <param name="intersectSegment"></param>
    /// <returns></returns>
    public static bool IntersectingWithMesh(MeshColliderEntity mesh, BoxColliderEntity box, float4x4 meshLocalToWorld, out float3 intersectSegment)
    {
        var bvtData = mesh.BVTData;
        int rootIndex = bvtData.RootIndex;
        intersectSegment = default;
        if (bvtData.Nodes == null || bvtData.Entities == null || rootIndex < 0)
        {
            return false;
        }

        float4x4 worldToMeshLocal = math.inverse(meshLocalToWorld);
        BoundingBox localQueryBounds = TransformBounds(box.Bounds, worldToMeshLocal);
        var nodeStack = new FixedList4096Bytes<int>();
        nodeStack.Add(rootIndex);
        var simplex = stackalloc float3[4];
        while (nodeStack.Length > 0)
        {
            int stackTop = nodeStack.Length - 1;
            int currentIndex = nodeStack[stackTop];
            nodeStack.RemoveAt(stackTop);

            BVTNode currentNode = bvtData.Nodes[currentIndex];
            if (!currentNode.Bounds.IntersectBounds(localQueryBounds))
            {
                continue;
            }

            if (currentNode.IsLeaf)
            {
                ColliderTriangle triangle = TransformTriangle(bvtData.Entities[currentNode.TriangleIndex].Triangle, meshLocalToWorld);
                if (GJKTool.GJK(triangle,box,simplex))
                {
                    if(mesh.IsTrigger == 1 || box.IsTrigger == 1) return true;
                    if (mesh.VerticesInWorldSpace != null)
                    {
                        EPATool.EPA(mesh.VerticesInWorldSpace, mesh.VertexLength, box, simplex, 64, out intersectSegment);
                    }
                    return true;
                }
                continue;
            }

            if (currentNode.Left >= 0)
            {
                nodeStack.Add(currentNode.Left);
            }

            if (currentNode.Right >= 0)
            {
                nodeStack.Add(currentNode.Right);
            }
        }
        return false;
    }

    public static bool IntersectingWithMesh(
        MeshColliderEntity mesh,
        CapsuleColliderEntity capsule,
        float4x4 meshLocalToWorld,
        out float3 intersectSegment)
    {
        return IntersectingWithMesh(mesh, capsule.Bounds, capsule, meshLocalToWorld,out intersectSegment);
    }

    public static bool IntersectingWithMesh(
        MeshColliderEntity mesh,
        SphereColliderEntity sphere,
        float4x4 meshLocalToWorld,
        out float3 intersectSegment)
    {
        return IntersectingWithMesh(mesh, sphere.Bounds, sphere, meshLocalToWorld,out intersectSegment);
    }

    /// <summary>
    /// 
    /// </summary>
    /// <param name="mesh"></param>
    /// <param name="meshLocalToWorld"></param>
    /// <param name="otherMesh"></param>
    /// <param name="otherMeshLocalToWorld"></param>
    /// <param name="intersectSegment"></param>
    /// <returns></returns>
    public static bool IntersectingWithMesh(MeshColliderEntity mesh, float4x4 meshLocalToWorld, MeshColliderEntity otherMesh, float4x4 otherMeshLocalToWorld,out float3 intersectSegment)
    {
        intersectSegment = default;
        var bvtData = mesh.BVTData;
        var otherBvtData = otherMesh.BVTData;
        int rootA = bvtData.RootIndex;
        int rootB = otherBvtData.RootIndex;
        if (bvtData.Nodes == null || bvtData.Entities == null || rootA < 0 ||
            otherBvtData.Nodes == null || otherBvtData.Entities == null || rootB < 0)
        {
            return false;
        }
        float4x4 meshLocalToOtherLocal = math.mul(math.inverse(meshLocalToWorld), otherMeshLocalToWorld);
        var nodePairs = new FixedList4096Bytes<long>();
        nodePairs.Add(((long)rootA << 32) | (uint)rootB);//让一个long能代表一组三角形的碰撞
        var simplex = stackalloc float3[4];
        while (nodePairs.Length > 0)
        {
            int top = nodePairs.Length - 1;
            long pair = nodePairs[top];
            nodePairs.RemoveAt(top);
            int nodeAIndex = (int)(pair >> 32);
            int nodeBIndex = (int)pair;
            BVTNode nodeA = bvtData.Nodes[nodeAIndex];
            BVTNode nodeB = otherBvtData.Nodes[nodeBIndex];
            BoundingBox nodeBBoundsInALocal = TransformBounds(nodeB.Bounds, meshLocalToOtherLocal);
            if (!nodeA.Bounds.IntersectBounds(nodeBBoundsInALocal))
            {
                continue;
            }

            if (nodeA.IsLeaf && nodeB.IsLeaf)
            {
                ColliderTriangle triangleA = TransformTriangle(bvtData.Entities[nodeA.TriangleIndex].Triangle, meshLocalToWorld);
                ColliderTriangle triangleB = TransformTriangle(otherBvtData.Entities[nodeB.TriangleIndex].Triangle, otherMeshLocalToWorld);
                if (GJKTool.GJK(triangleA, triangleB,simplex))
                {
                    if (mesh.IsTrigger == 1 || otherMesh.IsTrigger == 1) return true;
                    if (mesh.VerticesInWorldSpace != null && otherMesh.VerticesInWorldSpace != null)
                    {
                        EPATool.EPA(mesh.VerticesInWorldSpace, mesh.VertexLength, otherMesh.VerticesInWorldSpace, otherMesh.VertexLength, simplex, 128, out intersectSegment);
                    }
                    return true;
                }

                continue;
            }

            //记录所有可能碰撞对
            if (nodeA.IsLeaf)
            {
                if (nodeB.Left >= 0)
                {
                    nodePairs.Add(((long)nodeAIndex << 32) | (uint)nodeB.Left);
                }
                if (nodeB.Right >= 0)
                {
                    nodePairs.Add(((long)nodeAIndex << 32) | (uint)nodeB.Right);
                }
            }
            else if (nodeB.IsLeaf)
            {
                if (nodeA.Left >= 0)
                {
                    nodePairs.Add(((long)nodeA.Left << 32) | (uint)nodeBIndex);
                }
                if (nodeA.Right >= 0)
                {
                    nodePairs.Add(((long)nodeA.Right << 32) | (uint)nodeBIndex);
                }
            }
            else
            {
                if (nodeA.Left >= 0 && nodeB.Left >= 0)
                {
                    nodePairs.Add(((long)nodeA.Left << 32) | (uint)nodeB.Left);
                }
                if (nodeA.Left >= 0 && nodeB.Right >= 0)
                {
                    nodePairs.Add(((long)nodeA.Left << 32) | (uint)nodeB.Right);
                }
                if (nodeA.Right >= 0 && nodeB.Left >= 0)
                {
                    nodePairs.Add(((long)nodeA.Right << 32) | (uint)nodeB.Left);
                }
                if (nodeA.Right >= 0 && nodeB.Right >= 0)
                {
                    nodePairs.Add(((long)nodeA.Right << 32) | (uint)nodeB.Right);
                }
            }
        }

        return false;
    }

    private static bool IntersectingWithMesh(MeshColliderEntity mesh, BoundingBox queryBounds, CapsuleColliderEntity capsule, float4x4 meshLocalToWorld,out float3 intersectSegment)
    {
        var bvtData = mesh.BVTData;
        intersectSegment = default;
        int rootIndex = bvtData.RootIndex;
        if (bvtData.Nodes == null || bvtData.Entities == null || rootIndex < 0) return false;
        float4x4 worldToMeshLocal = math.inverse(meshLocalToWorld);
        BoundingBox localQueryBounds = TransformBounds(queryBounds, worldToMeshLocal);
        var nodeStack = new FixedList4096Bytes<int>();
        nodeStack.Add(rootIndex);
        var simplex = stackalloc float3[4];
        while (nodeStack.Length > 0)
        {
            int stackTop = nodeStack.Length - 1;
            int currentIndex = nodeStack[stackTop];
            nodeStack.RemoveAt(stackTop);
            BVTNode currentNode = bvtData.Nodes[currentIndex];
            if (!currentNode.Bounds.IntersectBounds(localQueryBounds)) continue;
            
            if (currentNode.IsLeaf)
            {
                ColliderTriangle triangle = TransformTriangle(
                    bvtData.Entities[currentNode.TriangleIndex].Triangle,
                    meshLocalToWorld);
                if (GJKTool.GJK(triangle,capsule,simplex))
                {
                    if (mesh.IsTrigger == 1 || capsule.IsTrigger == 1) return true;
                    if (mesh.VerticesInWorldSpace != null)
                    {
                        EPATool.EPA(mesh.VerticesInWorldSpace, mesh.VertexLength, capsule, simplex, 64, out intersectSegment);
                    }
                    return true;
                }
                continue;
            }

            if (currentNode.Left >= 0) nodeStack.Add(currentNode.Left);
            if (currentNode.Right >= 0) nodeStack.Add(currentNode.Right);
        }

        return false;
    }

    private static bool IntersectingWithMesh(MeshColliderEntity mesh, BoundingBox queryBounds, SphereColliderEntity sphere, float4x4 meshLocalToWorld,out float3 intersectSegment)
    {
        var bvtData = mesh.BVTData;
        intersectSegment = default;
        int rootIndex = bvtData.RootIndex;
        if (bvtData.Nodes == null || bvtData.Entities == null || rootIndex < 0) return false;

        float4x4 worldToMeshLocal = math.inverse(meshLocalToWorld);
        BoundingBox localQueryBounds = TransformBounds(queryBounds, worldToMeshLocal);
        var nodeStack = new FixedList4096Bytes<int>();
        nodeStack.Add(rootIndex);
        var simplex = stackalloc float3[4];
        while (nodeStack.Length > 0)
        {
            int stackTop = nodeStack.Length - 1;
            int currentIndex = nodeStack[stackTop];
            nodeStack.RemoveAt(stackTop);
            BVTNode currentNode = bvtData.Nodes[currentIndex];
            if (!currentNode.Bounds.IntersectBounds(localQueryBounds)) continue;

            if (currentNode.IsLeaf)
            {
                ColliderTriangle triangle = TransformTriangle(bvtData.Entities[currentNode.TriangleIndex].Triangle, meshLocalToWorld);
                if (GJKTool.GJK(triangle,sphere,simplex))
                {
                    if (mesh.IsTrigger == 1 || sphere.IsTrigger == 1) return true;
                    if (mesh.VerticesInWorldSpace != null)
                    {
                        EPATool.EPA(mesh.VerticesInWorldSpace, mesh.VertexLength, sphere, simplex, 64, out intersectSegment);
                    }
                    return true;
                }
                continue;
            }

            if (currentNode.Left >= 0) nodeStack.Add(currentNode.Left);
            if (currentNode.Right >= 0) nodeStack.Add(currentNode.Right);
        }

        return false;
    }

    private static BoundingBox TransformBounds(BoundingBox bounds, float4x4 transform)
    {
        float3 min = new float3(float.MaxValue);
        float3 max = new float3(float.MinValue);
        for (int corner = 0; corner < 8; corner++)
        {
            float3 point = new float3(
                (corner & 1) == 0 ? bounds.Min.x : bounds.Max.x,
                (corner & 2) == 0 ? bounds.Min.y : bounds.Max.y,
                (corner & 4) == 0 ? bounds.Min.z : bounds.Max.z);
            point = math.transform(transform, point);
            min = math.min(min, point);
            max = math.max(max, point);
        }

        return new BoundingBox((min + max) * 0.5f, (max - min) * 0.5f);
    }

    public static float3* TransformVerticesToWorld(float3* source, int vertexCount, float4x4 localToWorld)
    {
        if (source == null || vertexCount <= 0)
        {
            return null;
        }

        long bytes = (long)vertexCount * UnsafeUtility.SizeOf<float3>();
        float3* destination = (float3*)UnsafeUtility.Malloc(bytes, UnsafeUtility.AlignOf<float3>(), Allocator.Temp);
        if (destination == null)
        {
            return null;
        }

        for (int i = 0; i < vertexCount; i++)
        {
            destination[i] = math.transform(localToWorld, source[i]);
        }

        return destination;
    }

    public static void FreeTransformedVertices(float3* vertices)
    {
        if (vertices != null)
        {
            UnsafeUtility.Free(vertices, Allocator.Temp);
        }
    }

    private static ColliderTriangle TransformTriangle(ColliderTriangle triangle, float4x4 transform)
    {
        return new ColliderTriangle(
            math.transform(transform, triangle.P0),
            math.transform(transform, triangle.P1),
            math.transform(transform, triangle.P2));
    }

    #endregion

    public MeshTree(int maxCoordinateCount, float3 cellSize)
    {
        if (maxCoordinateCount < 1 || maxCoordinateCount > 1024 ||
            (maxCoordinateCount & (maxCoordinateCount - 1)) != 0)
        {
            throw new ArgumentOutOfRangeException(nameof(maxCoordinateCount),
                "坐标网格数量必须是 1 到 1024 之间的 2 的幂。");
        }

        if (!IsFinitePositive(cellSize.x) || !IsFinitePositive(cellSize.y) || !IsFinitePositive(cellSize.z))
        {
            throw new ArgumentOutOfRangeException(nameof(cellSize), "每个轴的网格尺寸都必须是有限的正数。");
        }

        MaxCoordinateCount = maxCoordinateCount;
        CellSize = cellSize;
        MaxRank = 0;
        for (int count = maxCoordinateCount; count > 1; count >>= 1)
        {
            MaxRank++;
        }
    }

    public MeshColliderBVTData GetBvtData(int meshIndex)
    {
        CompleteScheduledBuild();
        if (!RootIndies.IsCreated || meshIndex < 0 || meshIndex >= RootIndies.Length)
        {
            throw new ArgumentOutOfRangeException(nameof(meshIndex));
        }

        return new MeshColliderBVTData
        {
            Nodes = (BVTNode*)Nodes.GetUnsafePtr(),
            Entities = (MeshTriangleEntity*)MeshTriangleEntities.GetUnsafePtr(),
            RootIndies = (int*)RootIndies.GetUnsafePtr(),
            Index = meshIndex
        };
    }

    public JobHandle ScheduleMeshBuild(NativeArray<MeshColliderBuildData> meshes, NativeArray<int> meshTriangleCounts, NativeArray<MeshColliderEntity> meshColliderEntities, JobHandle dependsOn = default)
    {
        CompleteScheduledBuild();
        DisposeBuildBuffers();

        if (!meshTriangleCounts.IsCreated || meshTriangleCounts.Length != meshes.Length)
        {
            throw new ArgumentException("每个 Mesh 必须有一个对应的三角形数量槽位。", nameof(meshTriangleCounts));
        }
        if (!meshColliderEntities.IsCreated || meshColliderEntities.Length < meshes.Length)
        {
            throw new ArgumentException("Mesh 实体数组必须覆盖全部建树输入。", nameof(meshColliderEntities));
        }

        int triangleCount = 0;
        for (int i = 0; i < meshes.Length; i++)
        {
            MeshColliderBuildData mesh = meshes[i];
            if (mesh.Vertices == null || mesh.Triangles == null ||
                mesh.TriangleCount < 0 || mesh.TriangleCount % 3 != 0 ||
                mesh.VertexCount < 0 || mesh.EntityStart != triangleCount)
            {
                throw new ArgumentException("MeshColliderBuildData 的数组区间或三角形起始位置无效。", nameof(meshes));
            }

            for (int index = 0; index < mesh.TriangleCount; index++)
            {
                int vertexIndex = mesh.Triangles[index];
                if (vertexIndex < 0 || vertexIndex >= mesh.VertexCount)
                {
                    throw new ArgumentException("MeshColliderBuildData 包含超出其顶点区间的索引。", nameof(meshes));
                }
            }

            int meshTriangleCount = mesh.TriangleCount / 3;
            if (meshTriangleCount > int.MaxValue - triangleCount)
            {
                throw new ArgumentException("MeshColliderBuildData 的三角形总数超出可支持范围。", nameof(meshes));
            }

            meshTriangleCounts[i] = meshTriangleCount;
            triangleCount += meshTriangleCount;
        }

        MeshTriangleEntities = new NativeArray<MeshTriangleEntity>(triangleCount, Allocator.Persistent);
        int nodeCapacity = 0;
        int taskCapacity = 0;
        for (int i = 0; i < meshTriangleCounts.Length; i++)
        {
            int meshTriangleCount = meshTriangleCounts[i];
            if (meshTriangleCount > 0)
            {
                nodeCapacity = checked(nodeCapacity + meshTriangleCount * 2 - 1);
            }
            taskCapacity = checked(taskCapacity + meshTriangleCount * 2);
        }

        AllocateBuildBuffers(triangleCount, meshes.Length, nodeCapacity, taskCapacity);
        MeshOrigins = new NativeArray<float3>(meshes.Length, Allocator.Persistent);
        TriangleStarts = new NativeArray<int>(meshes.Length, Allocator.Persistent);
        NodeStarts = new NativeArray<int>(meshes.Length, Allocator.Persistent);
        TaskStarts = new NativeArray<int>(meshes.Length, Allocator.Persistent);
        SortLeft = new NativeArray<int>(meshes.Length, Allocator.Persistent);
        SortRight = new NativeArray<int>(meshes.Length, Allocator.Persistent);

        for (int i = 0; i < meshes.Length; i++)
        {
            MeshColliderEntity mesh = meshColliderEntities[i];
            mesh.BVTData = new MeshColliderBVTData
            {
                Entities = (MeshTriangleEntity*)MeshTriangleEntities.GetUnsafePtr(),
                Nodes = (BVTNode*)Nodes.GetUnsafePtr(),
                RootIndies = (int*)RootIndies.GetUnsafePtr(),
                Index = i,
            };
            meshColliderEntities[i] = mesh;
        }

        int triangleStart = 0;
        int nodeStart = 0;
        int taskStart = 0;
        for (int i = 0; i < meshes.Length; i++)
        {
            int meshTriangleCount = meshTriangleCounts[i];
            TriangleStarts[i] = triangleStart;
            NodeStarts[i] = nodeStart;
            TaskStarts[i] = taskStart;
            SortLeft[i] = triangleStart;
            SortRight[i] = triangleStart + meshTriangleCount - 1;

            triangleStart += meshTriangleCount;
            if (meshTriangleCount > 0)
            {
                nodeStart += meshTriangleCount * 2 - 1;
            }
            taskStart += meshTriangleCount * 2;
        }

        if (meshes.Length == 0)
        {
            BuildHandle = dependsOn;
            HasScheduledBuild = true;
            return BuildHandle;
        }

        JobHandle meshHandle = dependsOn;
        var meshJob = new BuildMeshTriangleBoundsJob
        {
            Meshes = meshes,
            MeshTriangleEntities = MeshTriangleEntities
        };
        meshHandle = meshJob.Schedule(meshes.Length, 10, dependsOn);

        var originJob = new GetMeshOriginJob
        {
            TriangleEntities = MeshTriangleEntities,
            TriangleStarts = TriangleStarts,
            TriangleCounts = meshTriangleCounts,
            Origin = MeshOrigins
        };
        meshHandle = originJob.Schedule(meshes.Length, 10, meshHandle);

        if (triangleCount > 0)
        {
            var mortonJob = new CreatMeshMortonDataJob
            {
                TriangleEntities = MeshTriangleEntities,
                MortonData = MortonData,
                Origin = MeshOrigins,
                TriangleStarts = TriangleStarts,
                TriangleCounts = meshTriangleCounts,
                MaxCoordinateCount = MaxCoordinateCount,
                MaxRank = MaxRank,
                CellSize = CellSize
            };
            meshHandle = mortonJob.Schedule(triangleCount, 64, meshHandle);
        }

        if (meshes.Length > 0)
        {
            var sortJob = new SortMeshMortonCode
            {
                MortonData = MortonData,
                BuildTasks = BuildTasks,
                TaskStartIndex = TaskStarts,
                Left = SortLeft,
                Right = SortRight
            };
            meshHandle = sortJob.Schedule(meshes.Length, 1, meshHandle);
        }

        var buildJob = new BuildMeshBVTTreeJob
        {
            TriangleStarts = TriangleStarts,
            NodeStarts = NodeStarts,
            TaskStarts = TaskStarts,
            MeshTriangleCounts = meshTriangleCounts,
            MortonData = MortonData,
            Nodes = Nodes,
            BuildTasks = BuildTasks,
            RootIndex = RootIndies
        };

        BuildHandle = buildJob.Schedule(meshes.Length, 10, meshHandle);
        HasScheduledBuild = true;
        return BuildHandle;
    }

    public JobHandle BuildBvtTree(NativeArray<MeshColliderEntity> meshColliderEntities,int count,JobHandle dependsOn = default)
    {
        if (!meshColliderEntities.IsCreated || count < 0 || count > meshColliderEntities.Length)
        {
            throw new ArgumentOutOfRangeException(nameof(count));
        }

        for (int i = 0; i < count; i++)
        {
            MeshColliderEntity mesh = meshColliderEntities[i];
            if (mesh.Vertices == null || mesh.Triangles == null)
            {
                throw new ArgumentException("MeshColliderEntity 的顶点或索引指针无效。",
                    nameof(meshColliderEntities));
            }
        }

        var meshBuildData = new NativeArray<MeshColliderBuildData>(count, Allocator.TempJob);
        var meshTriangleCounts = new NativeArray<int>(count, Allocator.TempJob);
        JobHandle buildHandle;
        try
        {
            int entityStart = 0;
            for (int i = 0; i < count; i++)
            {
                MeshColliderEntity mesh = meshColliderEntities[i];
                meshBuildData[i] = new MeshColliderBuildData
                {
                    ColliderId = mesh.Id,
                    Vertices = mesh.Vertices,
                    VertexCount = mesh.VertexCount,
                    Triangles = mesh.Triangles,
                    TriangleCount = mesh.TriangleIndexCount,
                    EntityStart = entityStart,
                };

                entityStart += mesh.TriangleIndexCount / 3;
            }

            buildHandle = ScheduleMeshBuild(meshBuildData, meshTriangleCounts, meshColliderEntities, dependsOn);
        }
        catch
        {
            CompleteScheduledBuild();
            if (meshBuildData.IsCreated) meshBuildData.Dispose();
            if (meshTriangleCounts.IsCreated) meshTriangleCounts.Dispose();
            throw;
        }

        JobHandle disposeHandle = meshBuildData.Dispose(buildHandle);
        disposeHandle = meshTriangleCounts.Dispose(disposeHandle);
        return disposeHandle;
    }
    
    private void AllocateBuildBuffers(int triangleCount, int meshCount, int nodeCapacity, int taskCapacity)
    {
        MortonData = new NativeArray<BVTMortonData>(triangleCount, Allocator.Persistent);
        Nodes = new NativeArray<BVTNode>(math.max(1, nodeCapacity), Allocator.Persistent);
        BuildTasks = new NativeArray<BVTBuildTask>(math.max(1, taskCapacity), Allocator.Persistent);
        RootIndies = new NativeArray<int>(meshCount, Allocator.Persistent);
    }

    private void CompleteScheduledBuild()
    {
        if (!HasScheduledBuild) return;
        BuildHandle.Complete();
        HasScheduledBuild = false;
    }

    private void DisposeBuildBuffers()
    {
        if (MortonData.IsCreated) MortonData.Dispose();
        if (Nodes.IsCreated) Nodes.Dispose();
        if (BuildTasks.IsCreated) BuildTasks.Dispose();
        if (RootIndies.IsCreated) RootIndies.Dispose();
        if (MeshTriangleEntities.IsCreated) MeshTriangleEntities.Dispose();
        if (MeshOrigins.IsCreated) MeshOrigins.Dispose();
        if (TriangleStarts.IsCreated) TriangleStarts.Dispose();
        if (NodeStarts.IsCreated) NodeStarts.Dispose();
        if (TaskStarts.IsCreated) TaskStarts.Dispose();
        if (SortLeft.IsCreated) SortLeft.Dispose();
        if (SortRight.IsCreated) SortRight.Dispose();
    }

    private static bool IsFinitePositive(float value)
    {
        return value > 0f && math.isfinite(value);
    }

    public void Dispose()
    {
        CompleteScheduledBuild();
        DisposeBuildBuffers();
    }
}
