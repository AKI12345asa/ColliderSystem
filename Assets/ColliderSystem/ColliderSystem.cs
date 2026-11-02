using System;
using System.Collections.Generic;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Jobs;

public class ColliderSystem:IDisposable
{
    private static ColliderSystem _instance = new();//单例模式
    private static ColliderSystem Instance => _instance ??= new ColliderSystem();
    
    [ReadOnly] private static Dictionary<int, MColliderBase> ColliderDic = new();//碰撞体字典使物体ID可以直接反向映射到碰撞体
    private static int BoxCount = 0;
    private const int BoxNativeArrayDefaultCount = 1024;
    private static int SphereCount = 0;
    private const int SphereNativeArrayDefaultCount = 1024;
    private static int CapsuleCount = 0;
    private const int CapsuleNativeArrayDefaultCount = 1024;
    private static int MeshCount = 0;
    private const int MeshNativeArrayDefaultCount = 1024;
    private const int ColliderBoundsArrayDefaultCount = 1024 * 4;
    
    //开辟五个NativeArray用于写入碰撞实体数据并传入Job中使用
    private NativeArray<BoxColliderEntity> BoxColliderEntities = new(BoxNativeArrayDefaultCount, Allocator.Persistent);
    private NativeArray<SphereColliderEntity> SphereColliderEntities = new(SphereNativeArrayDefaultCount, Allocator.Persistent);
    private NativeArray<CapsuleColliderEntity> CapsuleColliderEntities = new(CapsuleNativeArrayDefaultCount, Allocator.Persistent);
    private NativeArray<MeshColliderEntity> MeshColliderEntities = new(MeshNativeArrayDefaultCount, Allocator.Persistent);
    private NativeArray<ColliderBoundsData> ColliderBoundsEntities = new(ColliderBoundsArrayDefaultCount, Allocator.Persistent);
    
    //缓存所有Mesh的网格和三角形削减搬运数组产生的GC开销
    private readonly Dictionary<int, NativeArray<float3>> MeshVertexDic = new();
    private readonly Dictionary<int, NativeArray<int>> MeshTriangleDic = new();
    private readonly List<NativeArray<float3>> MeshWorldVertexBufferList = new();
    
    //用于标记已经产生的MeshTree
    private readonly HashSet<int> MeshTreeBuildBuffer = new();
    //存储关于Mesh的本地BVT树节约每次建树开销
    private readonly Dictionary<int, (MeshTree Tree, int TreeIndex)> MeshTreeDic = new();
    private readonly Dictionary<MeshTree, int> MeshTreeRefCounts = new();
    private readonly Dictionary<int, int> ColliderIndexDic = new();
    
    private readonly HashSet<ColliderCouple>[] CoupleSets;//两个哈希表用于前后对比筛选碰撞状态
    private int SetIndex;
    private bool IsHandlingCollision;
   
    private ColliderSystem()
    {
        CoupleSets = new HashSet<ColliderCouple>[2];
        CoupleSets[0] = new();
        CoupleSets[1] = new();
    }
    
    public static void UpdateCollision()
    {
       Instance.HandleCollision();
    }

    private void HandleCollision()
    {
        if (IsHandlingCollision)
        {
            throw new InvalidOperationException("Collision updates cannot be invoked recursively.");
        }

        IsHandlingCollision = true;
        try
        {
            HandleCollisionInternal();
        }
        finally
        {
            IsHandlingCollision = false;
        }
    }

    //流程简述:先初步创建所有碰撞实体数据->填充碰撞实体数据并且写入世界空间下的包围盒同时对每一个Mesh开辟本地MeshTree->等待每一个实体Bounds及其数据计算完成之后
    //开始顶层Bvt树->根据每一个碰撞实体的包围盒通过BVT树访问包围盒是否相交来粗筛碰撞对->再根据不同实体之间的碰撞算法计算实际碰撞对->等待任务完成之后
    //处理实体挤压 ->Trigger的Stay,Enter,Exit根据前后两张哈希表来判断
    private void HandleCollisionInternal()
    {
        var registeredColliders = ColliderDic.Values.GetEnumerator();
        JobHandle handle = default;
        NativeStream couples = default;
        NativeStream results = default;
        // 容量按字典实际数量给；真实数量要等遍历结束才确定
        var transformArray = new TransformAccessArray(Mathf.Max(1, ColliderDic.Count));
        NativeArray<float3> mtvArray = default;
        BVTTree bvtTree = null;
        HashSet<ColliderCouple> currentSet = null;
        bool stateCommitted = false;
        try
        {
            int boxIndex = 0;
            int sphereIndex = 0;
            int capsuleIndex = 0;
            int meshIndex = 0;
            int index = 0;
            while (registeredColliders.MoveNext())
            {
                MColliderBase collider = registeredColliders.Current;
                transformArray.Add(collider.transform);
                switch (collider)
                {
                    case MBoxCollider boxCollider:
                        var box = CreateEntity(boxCollider);
                        box.Index = boxIndex;
                        ColliderIndexDic.TryAdd(box.Id,index);
                        BoxColliderEntities[boxIndex++] = box;
                        break;
                    case MSphereCollider sphereCollider:
                        var sphere = CreateEntity(sphereCollider);
                        sphere.Index = sphereIndex;
                        ColliderIndexDic.TryAdd(sphere.Id,index);
                        SphereColliderEntities[sphereIndex++] = sphere;
                        break;
                    case MCapsuleCollider capsuleCollider:
                        var capsule = CreateEntity(capsuleCollider);
                        capsule.Index = capsuleIndex;
                        ColliderIndexDic.TryAdd(capsule.Id,index);
                        CapsuleColliderEntities[capsuleIndex++] = capsule;
                        break;
                    case MMeshCollider meshCollider:
                        var mesh = CreateEntity(meshCollider);
                        mesh.Index = meshIndex;
                        ColliderIndexDic.TryAdd(mesh.Id,index);
                        MeshColliderEntities[meshIndex++] = mesh;
                        break;
                    default:
                        throw new NotSupportedException($"不支持的碰撞体类型: {collider.GetType()}");
                }
                index++;
            }
            
            int colliderCount = boxIndex + sphereIndex + capsuleIndex + meshIndex;
            BoxCount = boxIndex;
            SphereCount = sphereIndex;
            CapsuleCount = capsuleIndex;
            MeshCount = meshIndex;
            
            var boxSlice     = BoxColliderEntities.GetSubArray(0, boxIndex);
            var sphereSlice  = SphereColliderEntities.GetSubArray(0, sphereIndex);
            var capsuleSlice = CapsuleColliderEntities.GetSubArray(0, capsuleIndex);
            var meshSlice    = MeshColliderEntities.GetSubArray(0, meshIndex);
            var boundsSlice  = ColliderBoundsEntities.GetSubArray(0, colliderCount);

            mtvArray = new NativeArray<float3>(Mathf.Max(1, colliderCount), Allocator.TempJob);

            handle = CreateMeshTree();
            if (colliderCount > 0)
            {
                var fillEntityJob = new FillEntityJob
                {
                    BoxColliderEntities = boxSlice,
                    SphereColliderEntities = sphereSlice,
                    CapsuleColliderEntities = capsuleSlice,
                    MeshColliderEntities = meshSlice,
                    ColliderBoundsEntities = boundsSlice
                };
                handle = fillEntityJob.Schedule(colliderCount, 64,handle);
            }
            bvtTree = new BVTTree(1024, new Vector3(1, 1, 1));
            handle =  bvtTree.ScheduleBuild(boundsSlice,handle);
            handle = bvtTree.GetIntersectingObjects(boundsSlice, out couples, handle);
            results = new NativeStream(Math.Max(1, colliderCount), Allocator.TempJob);
            var collideHandler = new ColliderHandlerJob
            {
                BoxColliderEntities = boxSlice,
                SphereColliderEntities = sphereSlice,
                CapsuleColliderEntities = capsuleSlice,
                MeshColliderEntities = meshSlice,
                ColliderCoupleStream = couples.AsReader(),
                Result = results.AsWriter()
            };
            handle = collideHandler.Schedule(colliderCount, 64, handle);
            handle.Complete();
            var resultReader = results.AsReader();
            currentSet = CoupleSets[SetIndex];
            var previousSet = CoupleSets[1 - SetIndex];
            
            var pendingEvents = new List<PendingCollisionEvent>();
            for(int i = 0; i < colliderCount; i++)
            {
                resultReader.BeginForEachIndex(i);
                while (resultReader.RemainingItemCount > 0)
                {
                    var couple = resultReader.Read<ColliderCouple>();
                    if (!ColliderDic.TryGetValue(couple.ColliderAId, out var collider1) || !ColliderDic.TryGetValue(couple.ColliderBId, out var collider2))
                    {
                        continue;
                    }
                    if (!currentSet.Add(couple))
                    {
                        continue;
                    }
                    if(collider1.IsTrigger == 0 && collider2.IsTrigger == 0)
                    {
                        CalculateMTV(collider1,collider2,couple.MVT,out var mtv1,out var mtv2);
                        mtvArray[ColliderIndexDic[couple.ColliderAId]] += mtv1;
                        mtvArray[ColliderIndexDic[couple.ColliderBId]] += mtv2;
                        continue;
                    }
                    TriggerEventType eventType = previousSet.Contains(couple) ? TriggerEventType.Stay : TriggerEventType.Enter;
                    pendingEvents.Add(new PendingCollisionEvent(eventType, collider1, collider2));
                }
                resultReader.EndForEachIndex();
            }
            var mtvJob = new ColliderMTVJob() { MTVArray = mtvArray };
            mtvJob.Schedule(transformArray).Complete();
            foreach (var couple in previousSet)
            {
                if (currentSet.Contains(couple)) continue;
                ColliderDic.TryGetValue(couple.ColliderAId, out var collider1);
                ColliderDic.TryGetValue(couple.ColliderBId, out var collider2);
                if (collider1 != null || collider2 != null)
                {
                    pendingEvents.Add(new PendingCollisionEvent(
                        TriggerEventType.Exit,
                        collider1,
                        collider2));
                }
            }
            SetIndex = 1 - SetIndex;
            previousSet.Clear();
            stateCommitted = true;
            foreach (PendingCollisionEvent pendingEvent in pendingEvents)
            {
                switch (pendingEvent.Type)
                {
                    case TriggerEventType.Enter:
                        pendingEvent.ColliderA?.TriggerEnter(pendingEvent.ColliderB);
                        pendingEvent.ColliderB?.TriggerEnter(pendingEvent.ColliderA);
                        break;
                    case TriggerEventType.Stay:
                        pendingEvent.ColliderA?.TriggerStay(pendingEvent.ColliderB);
                        pendingEvent.ColliderB?.TriggerStay(pendingEvent.ColliderA);
                        break;
                    case TriggerEventType.Exit:
                        pendingEvent.ColliderA?.TriggerExit(pendingEvent.ColliderB);
                        pendingEvent.ColliderB?.TriggerExit(pendingEvent.ColliderA);
                        break;
                }
            }
        }
        finally
        {
            if (!stateCommitted)
            {
                currentSet?.Clear();
            }
            try
            {
                handle.Complete();
            }
            finally
            {
                if (couples.IsCreated) couples.Dispose();
                if (results.IsCreated) results.Dispose();
                if (bvtTree != null) bvtTree.Dispose();
                if(mtvArray.IsCreated) mtvArray.Dispose();
                if(transformArray.isCreated) transformArray.Dispose();
                
                MeshWorldVertexBufferList.ForEach(vertexBuffer => vertexBuffer.Dispose());
                ColliderIndexDic.Clear();
                MeshWorldVertexBufferList.Clear();
            }
        }
    }


    public void Dispose()
    {
            if (BoxColliderEntities.IsCreated)     BoxColliderEntities.Dispose();
            if (SphereColliderEntities.IsCreated)  SphereColliderEntities.Dispose();
            if (CapsuleColliderEntities.IsCreated) CapsuleColliderEntities.Dispose();
            if (MeshColliderEntities.IsCreated)    MeshColliderEntities.Dispose();
            if (ColliderBoundsEntities.IsCreated)  ColliderBoundsEntities.Dispose();

            foreach (var kv in MeshVertexDic)   if (kv.Value.IsCreated)   kv.Value.Dispose();
            foreach (var kv in MeshTriangleDic) if (kv.Value.IsCreated)   kv.Value.Dispose();
            foreach (var buf in MeshWorldVertexBufferList) if (buf.IsCreated) buf.Dispose();
            foreach (var tree in MeshTreeRefCounts.Keys) tree.Dispose();

            MeshVertexDic.Clear();
            MeshTriangleDic.Clear();
            MeshWorldVertexBufferList.Clear();
            MeshTreeDic.Clear();
            MeshTreeRefCounts.Clear();
            ColliderIndexDic.Clear();
            _instance = null;
    }

    /// <summary>
    /// 释放系统持有的全部 Persistent 分配（单例归零，下次访问自动重建）
    /// </summary>
    public static void Shutdown()
    {
        _instance?.Dispose();
        _instance = null;
    }

#if UNITY_EDITOR
    // 编辑器里每次进入 Play 之前先清一次，避免上一次 Play 中未走 OnDestroy 的残留
    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.SubsystemRegistration)]
    private static void ResetOnPlay() => Shutdown();
#endif
    /// <summary>
    /// 计算最小分离向量
    /// </summary>
    /// <param name="collider1"></param>
    /// <param name="collider2"></param>
    /// <param name="coupleMvt"></param>
    /// <param name="mtv1"></param>
    /// <param name="mtv2"></param>
    private void CalculateMTV(MColliderBase collider1, MColliderBase collider2, float3 coupleMvt, out float3 mtv1, out float3 mtv2)
    {
        var totalInMass = collider1.InMass + collider2.InMass;
        mtv1 = coupleMvt * collider1.InMass / totalInMass;
        mtv2 = -coupleMvt * collider2.InMass / totalInMass;
    }


    /// <summary>
    /// 该方法会筛选还没有建造本地BVT树的Mesh对他们进行MeshTree的创建有则直接输入对应指针即可
    /// </summary>
    /// <param name="curHandle"></param>
    /// <param name="treeCapacity"></param>
    /// <returns></returns>
    /// <exception cref="InvalidOperationException"></exception>
    private unsafe JobHandle CreateMeshTree(JobHandle curHandle = default,int treeCapacity = 8)
    {
        var meshNativeArray = new NativeArray<MeshColliderEntity>(treeCapacity, Allocator.TempJob);
        var sourceIndices = new List<int>(treeCapacity);
        int batchCount = 0;
        try
        {
            for (int i = 0; i < MeshCount; i++)
            {
                MeshColliderEntity mesh = MeshColliderEntities[i];
                int colliderId = mesh.Id;
                if (!MeshTreeBuildBuffer.Contains(colliderId))
                {
                    if (MeshTreeDic.TryGetValue(colliderId, out var cachedTree))
                    {
                        mesh.BVTData = cachedTree.Tree.GetBvtData(cachedTree.TreeIndex);
                        MeshColliderEntities[i] = mesh;
                        continue;
                    }
                }

                if (mesh.Vertices == null || mesh.Triangles == null)
                {
                    throw new InvalidOperationException($"MeshColliderEntity {mesh.Id} 的顶点或索引数组未创建。");
                }

                meshNativeArray[batchCount] = mesh;
                sourceIndices.Add(i);
                batchCount++;
                if (batchCount == treeCapacity)
                {
                    curHandle = BuildMeshTreeBatch(meshNativeArray, sourceIndices, batchCount, curHandle);
                    batchCount = 0;
                    sourceIndices.Clear();
                }
            }
            if (batchCount > 0)
            {
                curHandle = BuildMeshTreeBatch(meshNativeArray, sourceIndices, batchCount, curHandle);
            }
            return curHandle;
        }
        finally
        {
            if (meshNativeArray.IsCreated) meshNativeArray.Dispose();
        }
    }

    private JobHandle BuildMeshTreeBatch(NativeArray<MeshColliderEntity> meshNativeArray, List<int> sourceIndices, int count, JobHandle dependsOn)
    {
        var meshTree = new MeshTree(512, Vector3.one);
        JobHandle handle = meshTree.BuildBvtTree(meshNativeArray, count, dependsOn);
        for (int i = 0; i < count; i++)
        {
            MeshColliderEntity mesh = meshNativeArray[i];
            MeshColliderEntities[sourceIndices[i]] = mesh;
            MeshTreeDic.Add(mesh.Id, (meshTree, mesh.BVTData.Index));
            MeshTreeBuildBuffer.Remove(mesh.Id);
        }
        MeshTreeRefCounts.Add(meshTree, count);
        return handle;
    }

    private BoxColliderEntity CreateEntity(MBoxCollider collider)
    {
        Transform colliderTransform = collider.transform;
        return new BoxColliderEntity
        {
            ColliderId = collider.GetInstanceID(),
            Center = collider.Center,
            Size = collider.Size,
            Rotation = colliderTransform.rotation,
            LocalTransform = colliderTransform.localToWorldMatrix,
            IsTrigger = collider.IsTrigger,
            Layer = 1 << collider.Layer
        };
    }

    private static SphereColliderEntity CreateEntity(MSphereCollider collider)
    {
        Transform colliderTransform = collider.transform;
        return new SphereColliderEntity
        {
            ColliderId = collider.GetInstanceID(),
            Center = collider.Center,
            Radius = collider.Radius,
            LocalTransform = colliderTransform.localToWorldMatrix,
            IsTrigger = collider.IsTrigger,
            Layer = 1 << collider.Layer
        };
    }

    private static CapsuleColliderEntity CreateEntity(MCapsuleCollider collider)
    {
        Transform colliderTransform = collider.transform;
        return new CapsuleColliderEntity
        {
            ColliderId = collider.GetInstanceID(),
            Center = collider.Center,
            Radius = collider.Radius,
            Height = collider.Height,
            Direction = collider.Direction,
            Rotation = colliderTransform.rotation,
            LocalTransform = colliderTransform.localToWorldMatrix,
            IsTrigger = collider.IsTrigger,
            Layer = 1 << collider.Layer
        };
    }

    private unsafe MeshColliderEntity CreateEntity(MMeshCollider collider)
    {
        Mesh mesh = collider.SharedMesh;
        NativeArray<float3> vertexBuffer = MeshVertexDic[collider.GetInstanceID()];
        NativeArray<int> triangleBuffer =  MeshTriangleDic[collider.GetInstanceID()];
        NativeArray<float3> worldVertexBuffer = new NativeArray<float3>(vertexBuffer.Length, Allocator.TempJob);
        MeshWorldVertexBufferList.Add(worldVertexBuffer);
        var entity = new MeshColliderEntity
        {
            ColliderId = collider.GetInstanceID(),
            ColliderBounds = new BoundingBox(mesh.bounds.center, mesh.bounds.extents),
            Vertices = (float3*)vertexBuffer.GetUnsafePtr(),
            VertexCount = vertexBuffer.Length,
            Triangles = (int*)triangleBuffer.GetUnsafePtr(),
            VerticesInWorldSpace = (float3*)worldVertexBuffer.GetUnsafePtr(),
            TriangleIndexCount = triangleBuffer.Length,
            LocalTransform = collider.transform.localToWorldMatrix,
            IsTrigger = collider.IsTrigger,
            VertexLength = vertexBuffer.Length,
            Layer = 1 << collider.Layer
        };

        if (MeshTreeDic.TryGetValue(entity.ColliderId, out var cachedTree))
        {
            entity.BVTData = cachedTree.Tree.GetBvtData(cachedTree.TreeIndex);
        }

        return entity;
    }

    private static NativeArray<float3> ToFloat3NativeArray(
        Vector3[] vertices,
        Allocator allocator)
    {
        var result = new NativeArray<float3>(vertices.Length, allocator);
        for (int i = 0; i < vertices.Length; i++)
        {
            result[i] = vertices[i];
        }
        return result;
    }
    
    
    /// <summary>
    /// 记录碰撞类型方便后续处理碰撞时能够查询到对应的碰撞实体
    /// </summary>
    /// <param name="a"></param>
    /// <param name="b"></param>
    /// <returns></returns>
    /// <exception cref="ArgumentOutOfRangeException"></exception>
    public static ColliderCoupleType GetCollideType(ColliderType a, ColliderType b)
    {
        if (a == ColliderType.Box)
        {
            if (b == ColliderType.Box) return ColliderCoupleType.BoxBox;
            if (b == ColliderType.Sphere) return ColliderCoupleType.BoxSphere;
            if (b == ColliderType.Capsule) return ColliderCoupleType.BoxCapsule;
            if (b == ColliderType.Mesh) return ColliderCoupleType.BoxMesh;
        }
        else if (a == ColliderType.Sphere)
        {
            if (b == ColliderType.Box) return ColliderCoupleType.BoxSphere;
            if (b == ColliderType.Sphere) return ColliderCoupleType.SphereSphere;
            if (b == ColliderType.Capsule) return ColliderCoupleType.SphereCapsule;
            if (b == ColliderType.Mesh) return ColliderCoupleType.SphereMesh;
        }
        else if (a == ColliderType.Capsule)
        {
            if (b == ColliderType.Box) return ColliderCoupleType.BoxCapsule;
            if (b == ColliderType.Sphere) return ColliderCoupleType.SphereCapsule;
            if (b == ColliderType.Capsule) return ColliderCoupleType.CapsuleCapsule;
            if (b == ColliderType.Mesh) return ColliderCoupleType.CapsuleMesh;
        }
        else if (a == ColliderType.Mesh)
        {
            if (b == ColliderType.Box) return ColliderCoupleType.BoxMesh;
            if (b == ColliderType.Sphere) return ColliderCoupleType.SphereMesh;
            if (b == ColliderType.Capsule) return ColliderCoupleType.CapsuleMesh;
            if (b == ColliderType.Mesh) return ColliderCoupleType.MeshMesh;
        }
        throw new ArgumentOutOfRangeException(nameof(a), $"不支持的碰撞类型组合: {a}, {b}。");
    }

    public static void RegisterCollider(MColliderBase collider)
    {
        if (_instance == null) return;  
        var id = collider.GetInstanceID();
        ColliderDic.Add(id, collider);
        switch (collider)
        {
            case MMeshCollider meshCollider:
                MeshCount++;
                var vertices = ToFloat3NativeArray(meshCollider.SharedMesh.vertices, Allocator.Persistent);
                var triangles = new NativeArray<int>(meshCollider.SharedMesh.triangles,Allocator.Persistent);
                Instance.MeshVertexDic.Add(id,vertices);
                Instance.MeshTriangleDic.Add(id,triangles);
                Instance.MeshTreeBuildBuffer.Add(id);
                ExpansionNativeArray(ref Instance.MeshColliderEntities, MeshCount);
                break;
            case MSphereCollider:
                SphereCount++;
                ExpansionNativeArray(ref Instance.SphereColliderEntities, SphereCount);
                break;
            case MCapsuleCollider:
                CapsuleCount++;
                ExpansionNativeArray(ref Instance.CapsuleColliderEntities, CapsuleCount);
                break;
            case MBoxCollider:
                BoxCount++;
                ExpansionNativeArray(ref Instance.BoxColliderEntities, BoxCount);
                break;
        }
        ExpansionNativeArray(ref Instance.ColliderBoundsEntities, BoxCount + SphereCount + CapsuleCount + MeshCount);
    }

    private static void ExpansionNativeArray<T>(ref NativeArray<T> array, int requiredCount) where T : struct
    {
        
        // 已经够用就直接返回；若数组已被释放（IsCreated == false）则重建
        if (array.IsCreated && array.Length >= requiredCount) return;

        int oldLength = array.IsCreated ? array.Length : 0;
        int capacity = Mathf.NextPowerOfTwo(Mathf.Max(requiredCount, Mathf.Max(oldLength * 2, 16)));

        var nextArray = new NativeArray<T>(capacity, Allocator.Persistent);
        if (array.IsCreated)
        {
            NativeArray<T>.Copy(array, nextArray, Mathf.Min(oldLength, capacity));
            array.Dispose();
        }
        array = nextArray;
    }
    
    private static void ScaleDownNativeArray<T>(ref NativeArray<T> array, int count) where T : struct
    {
        if (!array.IsCreated || array.Length <= count * 2)  return;
        var nextArray = new NativeArray<T>(array.Length / 2, Allocator.Persistent);
        if (array.IsCreated)
        {
            NativeArray<T>.Copy(array, nextArray, Math.Min(array.Length, nextArray.Length));
            array.Dispose();
        }
        array = nextArray;
    }

    public static void UnRegisterCollider(MColliderBase collider)
    {
        if (_instance == null) return;  
        var id = collider.GetInstanceID();
        ColliderDic.Remove(id);
        switch (collider)
        {
            case MMeshCollider:
                MeshCount--;
                Instance.MeshTreeBuildBuffer.Remove(id);
                Instance.MeshVertexDic.Remove(id,out var vertexBuffer);
                vertexBuffer.Dispose();
                Instance.MeshTriangleDic.Remove(id,out var triangles);
                triangles.Dispose();
                if (Instance.MeshTreeDic.TryGetValue(id, out var meshTreeData))
                {
                    Instance.MeshTreeDic.Remove(id);
                    int remainingReferences = Instance.MeshTreeRefCounts[meshTreeData.Tree] - 1;
                    if (remainingReferences == 0)
                    {
                        meshTreeData.Tree.Dispose();
                        Instance.MeshTreeRefCounts.Remove(meshTreeData.Tree);
                    }
                    else
                    {
                        Instance.MeshTreeRefCounts[meshTreeData.Tree] = remainingReferences;
                    }
                }
                ScaleDownNativeArray(ref Instance.MeshColliderEntities, MeshCount);
                break;
            case MSphereCollider:
                SphereCount--;
                ScaleDownNativeArray(ref Instance.SphereColliderEntities, SphereCount);
                break;
            case MCapsuleCollider:
                CapsuleCount--;
                ScaleDownNativeArray(ref Instance.CapsuleColliderEntities, CapsuleCount);
                break;
            case MBoxCollider:
                BoxCount--;
                ScaleDownNativeArray(ref Instance.BoxColliderEntities, BoxCount);
                break;
        }
        ScaleDownNativeArray(ref Instance.ColliderBoundsEntities, BoxCount + SphereCount + CapsuleCount + MeshCount);
    }   
    
    public static int GetColliderOrder(ColliderType colliderType)
    {
        switch (colliderType)
        {
            case ColliderType.Box: return 0;
            case ColliderType.Sphere: return 1;
            case ColliderType.Capsule: return 2;
            case ColliderType.Mesh: return 3;
            default: throw new ArgumentOutOfRangeException(nameof(colliderType), colliderType, "不支持的碰撞类型。");
        }
    }
    
    private enum TriggerEventType
    {
        Enter,
        Stay,
        Exit
    }
    private readonly struct PendingCollisionEvent
    {
        public readonly TriggerEventType Type;
        public readonly MColliderBase ColliderA;
        public readonly MColliderBase ColliderB;

        public PendingCollisionEvent(
            TriggerEventType type,
            MColliderBase colliderA,
            MColliderBase colliderB)
        {
            Type = type;
            ColliderA = colliderA;
            ColliderB = colliderB;
        }
    }
}
