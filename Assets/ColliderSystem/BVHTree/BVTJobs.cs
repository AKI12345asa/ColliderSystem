using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using Unity.Mathematics;

[BurstCompile]
public struct GetOriginJob : IJob
{
    [ReadOnly] public NativeArray<ColliderBoundsData> ColliderBounds;
    public NativeArray<float3> Origin;
    
    public void Execute()
    {
        var original = ColliderBounds[0].Bounds.Min;
        for (int i = 1; i < ColliderBounds.Length; i++)
        {
            original = math.min(original, ColliderBounds[i].Bounds.Min);
        }
        Origin[0] = original;
    }
}

[BurstCompile]
public struct CreatMortonDataJob : IJobParallelFor
{
    [ReadOnly] public NativeArray<ColliderBoundsData> ColliderBounds;
    [WriteOnly]public NativeArray<BVTMortonData> MortonData;
    [ReadOnly] public NativeArray<float3> Origin;
    public int MaxCoordinateCount;
    public int MaxRank;
    public float3 CellSize;
    public void Execute(int index)
    {
        var collider = ColliderBounds[index];
        MortonData[index] = new BVTMortonData
        {
            GameObjectID = collider.Id,
            MortonCode = GetMortonCode(collider.Bounds.Center, Origin[0]),
            Bounds = collider.Bounds,
            ColliderType = collider.ColliderType,
            TypeIndex = collider.TypeIndex,
            EntityIndex = collider.EntityIndex,
            Layer = collider.Layer,
        };
    }
    
    
    //生成莫顿码的原理:假设一个包围盒中心为(x,y,z)从二进制的角度看待这个坐标那么每一条轴上的数字在当前位数上是否为一则能直接表达其在该轴上的大小
    //因此如果轮流沿着x当前一位y当前一位z当前一位的方式遍历排序这些0或者1排列得到的编码从小到大排列起来就可以直接视为轮流按照x,y,z三条轴的顺序轮流进行二分排列这些包围盒
    //这样产生的编码实现了BVT树对于每一个碰撞体包围盒的划分
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
public struct SortMortonCode : IJob
{
    public NativeArray<BVTMortonData> MortonData;
    public NativeArray<BVTBuildTask> BuildTasks;
    public int Left;
    public int Right;
    public void Execute()
    {
        int taskCount = 0;
        BuildTasks[taskCount++] = new BVTBuildTask
        {
            Left = Left,
            Right = Right
        };

        while (taskCount > 0)
        {
            BVTBuildTask task = BuildTasks[--taskCount];//这里是正好有BVTBuildTask所以拿来当作记录区间的临时数组(正好免掉了额外开辟空间记录区间这件事)
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
                }//快速排序的原理就是随机抽取一个数将大于这个数的数字都排在当前数组的右侧把小于这个数的数字都排在左侧，如果这件事对每一个区间都成立那么就可以认为该数组已经有序了

                //此时存储更大的那个区间然后对着小区间继续进行快排
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
public unsafe struct BuildBVTTreeJob : IJob
{
    [ReadOnly] public NativeArray<ColliderBoundsData> ColliderBounds;
    [ReadOnly] public NativeArray<BVTMortonData> MortonData;
    public NativeArray<BVTNode> Nodes;
    public NativeArray<BVTBuildTask> BuildTasks;
    
    public void Execute()
    {
        int count = ColliderBounds.Length;
        if (count == 0)
        {
            return;
        }
        
        var nodes = (BVTNode*)Nodes.GetUnsafePtr();
        for (int i = 0; i < count; i++)
        {
            nodes[i] = new BVTNode
            {
                Bounds = MortonData[i].Bounds,
                ColliderID = MortonData[i].GameObjectID,
                EntityIndex = MortonData[i].EntityIndex,
                ColliderType = MortonData[i].ColliderType,
                TypeIndex = MortonData[i].TypeIndex,
                Left = -1,
                Right = -1,
                Layer = MortonData[i].Layer,
            };
        }
        
        int taskCount = 1;
        int nextInternalNode = count;
        BuildTasks[0] = new BVTBuildTask
        {
            Left = 0,
            Right = count - 1,
            Parent = -1,
            ChildSide = 0
        };

        while (taskCount > 0)
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
                nodes[nodeIndex] = new BVTNode //0代表左枝1代表右枝
                {
                    Bounds = default,
                    ColliderID = -1,
                    Left = -1,
                    Right = -1,
                    Layer = -1
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
                continue;
            }

            if (task.ChildSide == 0)
            {
                nodes[task.Parent].Left = nodeIndex;
            }
            else
            {
                nodes[task.Parent].Right = nodeIndex;
            }
        }

        //轮流创建节点之后合并
        for (int i = nextInternalNode - 1; i >= count; i--)
        {
            BVTNode* node = nodes + i;
            node->Bounds = Encapsulate(nodes[node->Left].Bounds,nodes[node->Right].Bounds);
        }
    }
    
    private static BoundingBox Encapsulate(BoundingBox a, BoundingBox b)
    {
        float3 min = math.min(a.Min, b.Min);
        float3 max = math.max(a.Max, b.Max);
        return new BoundingBox((min + max) * 0.5f, (max - min) * 0.5f);
    }
}



[BurstCompile]
public struct GetBvtCheckCoupleJob : IJobParallelFor
{
    [ReadOnly]public NativeArray<ColliderBoundsData> ColliderBounds;
    [ReadOnly]public NativeArray<BVTNode> Nodes;
    public NativeStream.Writer Result;
    public int RootIndex;
    
    public void Execute(int index)
    {
        var writer = Result;
        writer.BeginForEachIndex(index);
        int pairCount = 0;

        if (RootIndex >= 0 && RootIndex < Nodes.Length)
        {
            var checkBound = ColliderBounds[index];
            var nodeStack = new FixedList4096Bytes<int>();
            nodeStack.Add(RootIndex);
            while (nodeStack.Length > 0)
            {
                int stackTop = nodeStack.Length - 1;
                int currentIndex = nodeStack[stackTop];
                nodeStack.RemoveAt(stackTop);
                var currentNode = Nodes[currentIndex];
                if (!currentNode.Bounds.IntersectBounds(checkBound.Bounds))
                {
                    continue;
                }
                if (currentNode.IsLeaf)
                {
                    if (currentNode.ColliderID != checkBound.Id && checkBound.EntityIndex < currentNode.EntityIndex
                        && (checkBound.Layer & currentNode.Layer) != 0) //比较序号其实就是保证碰撞对不会重复计算,因为碰撞对是对称的所以只要保证碰撞体1的序号小于碰撞体2的序号就可以了
                    {
                        var type1 = checkBound.ColliderType;
                        var type2 = currentNode.ColliderType;
                        writer.Write(ColliderSystem.GetCollideType(type1,type2));
                        //以一定的规则来保证两种碰撞体发生碰撞时其在数据流中写入的顺序是固定的这样在读取的时候就可以保证碰撞体的顺序是固定的,也就是说一个type在读取之后可以映射到后续的两个序号的碰撞体类别一定是固定的
                        if (ColliderSystem.GetColliderOrder(type1) <= ColliderSystem.GetColliderOrder(type2))
                        {
                           writer.Write(checkBound.TypeIndex);
                           writer.Write(currentNode.TypeIndex);
                        }
                        else
                        {
                            writer.Write(currentNode.TypeIndex);
                            writer.Write(checkBound.TypeIndex);
                        }
                        pairCount++;
                    }
                }
                else
                {
                    int left = currentNode.Left;
                    int right = currentNode.Right;
                    if (left != -1)
                    {
                        nodeStack.Add(left);
                    }

                    if (right != -1)
                    {
                        nodeStack.Add(right);
                    }
                }
            }
        }
        writer.EndForEachIndex();
    }
    
}
