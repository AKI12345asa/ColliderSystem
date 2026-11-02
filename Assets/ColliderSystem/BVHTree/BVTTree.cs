using System;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using Unity.Mathematics;
/// <summary>
/// 这里的Node也用于MeshTree的构建所以里面还会含有TriangleIndex
/// </summary>
public struct BVTNode
{
    public BoundingBox Bounds;
    public int ColliderID;
    public int Left;
    public int Right;
    public int EntityIndex;
    public int Layer;
    public ColliderType ColliderType;
    public int TriangleIndex;
    public bool IsLeaf => Left == -1 && Right == -1;
    public int TypeIndex;
}

public struct BVTMortonData
{
    public int GameObjectID;
    public uint MortonCode;
    public BoundingBox Bounds;
    public ColliderType ColliderType;
    public int Layer;
    public int TypeIndex;
    public int EntityIndex;
}

public struct BVTBuildTask
{
    public int Left;
    public int Right;
    public int Parent;
    public byte ChildSide;
}

public class BVTTree : IDisposable
{
    private readonly int MaxCoordinateCount;
    private readonly int MaxRank;
    private readonly float3 CellSize;
    private NativeArray<BVTMortonData> MortonData;
    private NativeArray<BVTNode> Nodes;
    private NativeArray<BVTBuildTask> BuildTasks;
    private NativeArray<int> RootIndex;
    private NativeArray<float3> Origin;
    private JobHandle BuildHandle;
    private bool HasScheduledBuild;
    public BVTTree(int maxCoordinateCount, float3 cellSize)
    {
        if (maxCoordinateCount < 1 || maxCoordinateCount > 1024 ||
            (maxCoordinateCount & (maxCoordinateCount - 1)) != 0)
        {
            throw new ArgumentOutOfRangeException(nameof(maxCoordinateCount),
                "坐标网格数量必须是 1 到 1024 之间的 2 的幂。");
        }
        
        MaxCoordinateCount = maxCoordinateCount;
        CellSize = cellSize;
        MaxRank = 0;
        for (int count = maxCoordinateCount; count > 1; count >>= 1)
        {
            MaxRank++;
        }
    }

    public JobHandle ScheduleBuild(NativeArray<ColliderBoundsData> colliderBounds, JobHandle dependsOn = default)
    {
        CompleteScheduledBuild();
        DisposeBuildBuffers();
        AllocateBuildBuffers(colliderBounds.Length);

        JobHandle buildDependency = dependsOn;
        if (colliderBounds.Length > 0)
        {
            Origin = new NativeArray<float3>(1, Allocator.Persistent);
            var originJob = new GetOriginJob
            {
                ColliderBounds = colliderBounds,
                Origin = Origin
            };
            buildDependency = originJob.Schedule(buildDependency);

            var mortonJob = new CreatMortonDataJob
            {
                ColliderBounds = colliderBounds,
                MortonData = MortonData,
                Origin = Origin,
                MaxCoordinateCount = MaxCoordinateCount,
                MaxRank = MaxRank,
                CellSize = CellSize
            };
            buildDependency = mortonJob.Schedule(colliderBounds.Length, 64, buildDependency);

            if (colliderBounds.Length > 1)
            {
                var sortJob = new SortMortonCode
                {
                    MortonData = MortonData,
                    BuildTasks = BuildTasks,
                    Left = 0,
                    Right = colliderBounds.Length - 1,
                    
                    
                };
                buildDependency = sortJob.Schedule(buildDependency);
            }
        }
        RootIndex[0] = colliderBounds.Length == 0
            ? -1
            : colliderBounds.Length == 1
                ? 0
                : colliderBounds.Length;
        var buildJob = new BuildBVTTreeJob
        {
            ColliderBounds = colliderBounds,
            MortonData = MortonData,
            Nodes = Nodes,
            BuildTasks = BuildTasks,
        };
        BuildHandle = buildJob.Schedule(buildDependency);
        HasScheduledBuild = true;
        return BuildHandle;
    }

    public void BuildBvtTree(NativeArray<ColliderBoundsData> colliderBoundsDatas)
    {
        ScheduleBuild(colliderBoundsDatas).Complete();
    }

    
    /// <summary>
    /// 请确保检查前所有物体均已写入该BVT树
    /// </summary>
    /// <param name="checkEntities"></param>
    /// <param name="colliderCoupleData"></param>
    /// <param name="typeArray"></param>
    /// <param name="dependsOn"></param>
    /// <returns>返回当前碰撞对的数量</returns>
    public JobHandle GetIntersectingObjects(
        NativeArray<ColliderBoundsData> checkBounds,
        out NativeStream colliderCoupleData,
        JobHandle dependsOn = default)
    {
        colliderCoupleData = new NativeStream(math.max(1, checkBounds.Length), Allocator.TempJob);
        return ScheduleIntersectionJob(checkBounds, colliderCoupleData, dependsOn);
    }

    private JobHandle ScheduleIntersectionJob(
        NativeArray<ColliderBoundsData> checkBounds,
        NativeStream colliderCoupleData,
        JobHandle dependsOn)
    {
        int rootIndex = RootIndex.IsCreated && RootIndex.Length > 0 ? RootIndex[0] : -1;
        var checkJob = new GetBvtCheckCoupleJob
        {
            ColliderBounds = checkBounds,
            Nodes = Nodes,
            RootIndex = rootIndex,
            Result = colliderCoupleData.AsWriter(),
        };
        return checkJob.Schedule(checkBounds.Length, 64, dependsOn);
    }
    
    
    public void GetSortedObjectIds(NativeArray<int> ids)
    {
        CompleteScheduledBuild();
        int count = math.min(ids.Length, MortonData.Length);
        for (int i = 0; i < count; i++)
        {
            ids[i] = MortonData[i].GameObjectID;
        }
    }


    private void AllocateBuildBuffers(int count,int rootCount = 1)
    {
        int nodeCapacity = count == 0 ? 1 : count * 2 - 1;
        MortonData = new NativeArray<BVTMortonData>(count, Allocator.Persistent);
        Nodes = new NativeArray<BVTNode>(nodeCapacity, Allocator.Persistent);
        BuildTasks = new NativeArray<BVTBuildTask>(count == 0 ? 1 : count * 2, Allocator.Persistent);
        RootIndex = new NativeArray<int>(rootCount, Allocator.Persistent);
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
        if (RootIndex.IsCreated) RootIndex.Dispose();
        if (Origin.IsCreated) Origin.Dispose();
    }
    
    public void Dispose()
    {
        CompleteScheduledBuild();
        DisposeBuildBuffers();
    }
}
