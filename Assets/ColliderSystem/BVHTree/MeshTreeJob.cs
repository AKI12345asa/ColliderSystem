using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using Unity.Mathematics;

[BurstCompile]
public struct GetMeshOriginJob : IJobParallelFor
{
    [ReadOnly] public NativeArray<MeshTriangleEntity> TriangleEntities;
    [ReadOnly] public NativeArray<int> TriangleStarts;
    [ReadOnly] public NativeArray<int> TriangleCounts;
    public NativeArray<float3> Origin;
    public void Execute(int index)
    {
        int start = TriangleStarts[index];
        int count = TriangleCounts[index];
        if (count == 0)
        {
            Origin[index] = float3.zero;
            return;
        }

        float3 original = TriangleEntities[start].Bounds.Min;
        for (int i = start + 1; i < start + count; i++)
        {
            original = math.min(original, TriangleEntities[i].Bounds.Min);
        }
        Origin[index] = original; //在规定的Mesh数的范围内获取到最小的坐标点作为原点
    }
}

[BurstCompile]
public struct CreatMeshMortonDataJob : IJobParallelFor
{
    [ReadOnly] public NativeArray<MeshTriangleEntity> TriangleEntities;
    [WriteOnly] public NativeArray<BVTMortonData> MortonData;
    [ReadOnly]public NativeArray<float3> Origin;
    [ReadOnly] public NativeArray<int> TriangleStarts;
    [ReadOnly] public NativeArray<int> TriangleCounts;
    public int MaxCoordinateCount;
    public int MaxRank;
    public float3 CellSize;
    public void Execute(int index)
    {
        int low = 0;
        int high = TriangleStarts.Length;
        while (low < high)
        {
            int middle = low + ((high - low) >> 1);
            if (TriangleStarts[middle] <= index)
            {
                low = middle + 1;
            }
            else
            {
                high = middle;
            }
        }//通过二分查找快速获取到origin位置

        int originIndex = low - 1;
        while (originIndex >= 0 && TriangleCounts[originIndex] == 0) originIndex--;

        var collider = TriangleEntities[index];
        MortonData[index] = new BVTMortonData
        {
            GameObjectID = collider.Id,
            MortonCode = GetMortonCode(collider.Bounds.Center, Origin[originIndex]),
            Bounds = collider.Bounds,
            ColliderType = ColliderType.Mesh,
            TypeIndex = collider.Index,
            EntityIndex = collider.Index
        };
    }
    
    private uint GetMortonCode(float3 point, float3 original)
    {
        int x = GetAxesCoordinate(point.x, original.x, CellSize.x);
        int y = GetAxesCoordinate(point.y, original.y, CellSize.y);
        int z = GetAxesCoordinate(point.z, original.z, CellSize.z);
        uint mortonCode = 0;
        for (int i = MaxRank - 1; i >= 0; i--)
        {
            mortonCode = (mortonCode << 1) | ((uint)(x >> i) & 1u);
            mortonCode = (mortonCode << 1) | ((uint)(y >> i) & 1u);
            mortonCode = (mortonCode << 1) | ((uint)(z >> i) & 1u);
        }

        return mortonCode;
    }
    
    private int GetAxesCoordinate(float position, float origin, float cellSize)
    {
        int coordinate = (int)math.floor((position - origin) / cellSize);
        return math.clamp(coordinate, 0, MaxCoordinateCount - 1);
    }
}

[BurstCompile]
public struct SortMeshMortonCode : IJobParallelFor
{
    [NativeDisableParallelForRestriction]public NativeArray<BVTMortonData> MortonData;
    [NativeDisableParallelForRestriction]public NativeArray<BVTBuildTask> BuildTasks;
    [ReadOnly]public NativeArray<int> TaskStartIndex;
    [ReadOnly]public NativeArray<int> Left;
    [ReadOnly]public NativeArray<int> Right;
    public void Execute(int index)
    {
        var left = Left[index];
        var right = Right[index];
        if (left >= right) return;
        var taskStartIndex = TaskStartIndex[index];
        int taskCount = taskStartIndex;
        BuildTasks[taskCount++] = new BVTBuildTask
        {
            Left = left,
            Right = right
        };

        while (taskCount > taskStartIndex)
        {
            BVTBuildTask task = BuildTasks[--taskCount];
            int low = task.Left;
            int high = task.Right;

            while (low < high)
            {
                BVTMortonData pivot = MortonData[low + ((high - low) >> 1)];
                int i = low;
                int j = high;
                while (i <= j)
                {
                    while (Compare(MortonData[i], pivot) < 0) i++;
                    while (Compare(MortonData[j], pivot) > 0) j--;

                    if (i <= j)
                    {
                        (MortonData[i], MortonData[j]) = (MortonData[j], MortonData[i]);
                        i++;
                        j--;
                    }
                }

                if (j - low < high - i)
                {
                    if (i < high)
                    {
                        BuildTasks[taskCount++] = new BVTBuildTask { Left = i, Right = high };
                    }
                    high = j;
                }
                else
                {
                    if (low < j)
                    {
                        BuildTasks[taskCount++] = new BVTBuildTask { Left = low, Right = j };
                    }
                    low = i;
                }
            }
        }
    }
    
    private static int Compare(BVTMortonData a, BVTMortonData b)
    {
        int codeComparison = a.MortonCode.CompareTo(b.MortonCode);
        return codeComparison != 0 ? codeComparison : a.GameObjectID.CompareTo(b.GameObjectID);
    }
}

[BurstCompile]
public unsafe struct BuildMeshTriangleBoundsJob : IJobParallelFor
{
    [ReadOnly] public NativeArray<MeshColliderBuildData> Meshes;
    [WriteOnly,NativeDisableParallelForRestriction] public NativeArray<MeshTriangleEntity> MeshTriangleEntities;

    public void Execute(int meshIndex)
    {
        MeshColliderBuildData mesh = Meshes[meshIndex];
        int triangleCount = mesh.TriangleCount / 3;
        for (int triangleIndex = 0; triangleIndex < triangleCount; triangleIndex++)
        {
            int sourceIndex = triangleIndex * 3;
            int vertexIndex0 = mesh.Triangles[sourceIndex];
            int vertexIndex1 = mesh.Triangles[sourceIndex + 1];
            int vertexIndex2 = mesh.Triangles[sourceIndex + 2];
            float3 vertex0 = mesh.Vertices[vertexIndex0];
            float3 vertex1 = mesh.Vertices[vertexIndex1];
            float3 vertex2 = mesh.Vertices[vertexIndex2];

            float3 min = math.min(vertex0, math.min(vertex1, vertex2));
            float3 max = math.max(vertex0, math.max(vertex1, vertex2));
            int outputIndex = mesh.EntityStart + triangleIndex;
            BoundingBox bounds = new BoundingBox((min + max) * 0.5f, (max - min) * 0.5f);
            MeshTriangleEntities[outputIndex] = new MeshTriangleEntity
            {
                TriangleIndex = outputIndex,
                Triangle = new ColliderTriangle(vertex0, vertex1, vertex2),
                TriangleBounds = bounds,
                Index = outputIndex
            };
        }
    }
}

public unsafe struct BuildMeshBVTTreeJob : IJobParallelFor
{
    [ReadOnly] public NativeArray<int> TriangleStarts;
    [ReadOnly] public NativeArray<int> NodeStarts;
    [ReadOnly] public NativeArray<int> TaskStarts;
    [ReadOnly] public NativeArray<int> MeshTriangleCounts;
    [ReadOnly]public NativeArray<BVTMortonData> MortonData;
    [NativeDisableParallelForRestriction]public NativeArray<BVTNode> Nodes;
    [NativeDisableParallelForRestriction]public NativeArray<BVTBuildTask> BuildTasks;
    public NativeArray<int> RootIndex;
    public void Execute(int index)
    {
        int entitiesStartIndex = TriangleStarts[index];
        int nodeStartIndex = NodeStarts[index];
        int taskStartIndex = TaskStarts[index];
        var count = MeshTriangleCounts[index];
        
        if (count == 0)
        {
            RootIndex[index] = -1;
            return;
        }
        var nodes = (BVTNode*)NativeArrayUnsafeUtility.GetUnsafePtr(Nodes);
        var entitiesNodeIndex = nodeStartIndex;
        for (int j = entitiesStartIndex; j < entitiesStartIndex + count; j++)
        {
            nodes[entitiesNodeIndex] = new BVTNode
            {
                Bounds = MortonData[j].Bounds,
                ColliderID = MortonData[j].GameObjectID,
                EntityIndex = MortonData[j].EntityIndex,
                ColliderType = MortonData[j].ColliderType,
                TriangleIndex = MortonData[j].EntityIndex,
                Left = -1,
                Right = -1
            };
            entitiesNodeIndex++;
        }

        int root = -1;
        int taskCount = taskStartIndex + 1;
        int nextInternalNode = nodeStartIndex + count;
        BuildTasks[taskStartIndex] = new BVTBuildTask
        {
            Left = nodeStartIndex,
            Right = nodeStartIndex + count - 1,
            Parent = -1,
            ChildSide = 0
        };

        while (taskCount > taskStartIndex)
        {
            BVTBuildTask task = BuildTasks[--taskCount];
            int nodeIndex;
            if (task.Left == task.Right)
            {
                nodeIndex = task.Left;
            }
            else
            {
                nodeIndex = nextInternalNode++;
                nodes[nodeIndex] = new BVTNode
                {
                    Bounds = default,
                    ColliderID = -1,
                    Left = -1,
                    Right = -1
                };

                int middle = task.Left + ((task.Right - task.Left) >> 1);
                BuildTasks[taskCount++] = new BVTBuildTask
                {
                    Left = middle + 1,
                    Right = task.Right,
                    Parent = nodeIndex,
                    ChildSide = 1
                };
                BuildTasks[taskCount++] = new BVTBuildTask
                {
                    Left = task.Left,
                    Right = middle,
                    Parent = nodeIndex,
                    ChildSide = 0
                };
            }

            if (task.Parent < 0)
            {
                root = nodeIndex;
            }
            else if (task.ChildSide == 0)
            {
                nodes[task.Parent].Left = nodeIndex;
            }
            else
            {
                nodes[task.Parent].Right = nodeIndex;
            }
        }

        for (int j = nextInternalNode - 1; j >= entitiesNodeIndex; j--)
        {
            BVTNode* node = nodes + j;
            node->Bounds = Encapsulate(nodes[node->Left].Bounds,nodes[node->Right].Bounds);
        }
        RootIndex[index] = root;
    }
    
    
    
    
    private static BoundingBox Encapsulate(BoundingBox a, BoundingBox b)
    {
        float3 min = math.min(a.Min, b.Min);
        float3 max = math.max(a.Max, b.Max);
        return new BoundingBox((min + max) * 0.5f, (max - min) * 0.5f);
    }
}