# ColliderSystem

从零实现的 Unity 凸体碰撞检测系统。不依赖内置物理引擎，完整覆盖物理管线的四段：**实体收集 → 粗筛 → 精算 → 求解与事件**，全流程跑在 JobSystem + Burst 上。

- Unity **2022.3.62f3c1**
- 约 **25 个 C# 脚本 / 5,300 行**，**12 处 `[BurstCompile]`**

---

## 管线

```
① 实体收集   ColliderDic → 按类型拆到 4 个 NativeArray（Box / Sphere / Capsule / Mesh）
       │
② 粗筛       Mesh 本地 BVH（引用计数缓存）+ 顶层 Morton 码 LBVH → 候选碰撞对（NativeStream）
       │
③ 精算       GJK 判交 → EPA 求穿透深度 → 最小平移向量（MTV）
       │
④ 求解/事件   MTV 按逆质量加权 → IJobParallelForTransform 回写 Transform
             双缓冲 HashSet 对比前后两帧 → Trigger 的 Enter / Stay / Exit
```

---

## 实现要点

| 模块 | 做法 |
| --- | --- |
| **粗筛** | 世界坐标 → 格子 → **Morton 编码** → 排序 → 自底向上建树（LBVH）。比逐轴分裂的朴素 BVH 更适合并行 |
| **两层 BVH** | 网格三角形数量大且不变，单独建本地 BVH 并做**引用计数缓存**；物体级用顶层 BVH 只处理包围盒 |
| **精算** | 手写 **GJK** 判交与 **EPA** 求穿透深度，按「点集 / 三角形 / 盒 / 球 / 胶囊」的组合提供多套支撑函数重载 |
| **数据布局** | 四种碰撞体各一个 `NativeArray<T>`（SoA），job 内按类型分派，避免虚调用与缓存失效 |
| **并行收集** | `IJobParallelFor` 不能直接写共享数组，用 **`NativeStream`** 每线程一段，主线程顺序读取 |
| **事件判定** | `HashSet<ColliderCouple>[2]` 双缓冲 + 索引翻转，前后两帧对比即得 Enter / Stay / Exit，且**零分配** |
| **求解** | MTV 按逆质量加权分配，使重的物体位移更小 |
| **内存管理** | 实体数组与包围盒数组改为**容量池 + 逻辑计数**，Job 只接收 `GetSubArray(0, count)` 视图，避免把「容量」当成「物体数量」；补齐 `Dispose` 与运行时生命周期钩子 |

---

## 目录结构

```
Assets/ColliderSystem/
├── ColliderSystem.cs          # 主流程：收集 → 粗筛 → 精算 → 求解 → 事件
├── ColliderData.cs            # 实体与碰撞对数据结构
├── BoundingBox.cs / OrientedBoundingBox.cs / Interval.cs / Ray.cs
├── BVHTree/
│   ├── BVTTree.cs             # 顶层 BVH（Morton 码 LBVH）
│   ├── BVTJobs.cs             # 建树与查询 Job
│   ├── MeshTree.cs            # 网格本地 BVH（两层结构的底层）
│   └── MeshTreeJob.cs
├── ColliderJobs/
│   ├── ColliderHandler.cs     # 精算调度
│   ├── FillEntityJob.cs       # 并行填充世界变换与包围盒
│   └── ColliderMTVJob.cs      # 并行回写 Transform
├── Colliders/                 # MBox / MSphere / MCapsule / MMesh
└── Tool/
    ├── GJKTool.cs             # GJK 判交
    ├── EPATool.cs             # EPA 求穿透深度
    ├── OBBTool.cs
    └── GeometryMathTool.cs
```

---

## 运行

1. 用 Unity Hub 以 **2022.3.62f3c1** 打开本目录；
2. 打开 `Assets/Scenes/SampleScene.unity`；
3. 场景中的 `ColliderSystemTest` 会在 `FixedUpdate` 调用 `ColliderSystem.UpdateCollision()`。

> **关于素材**：`Assets/Scenes/Demo.unity` 是完整演示场景，它使用了 Synty POLYGON 系列素材。
> 由于素材授权不允许再分发，`Assets/ArtResources` **未包含在本仓库中**，因此 Demo 场景会缺少模型与材质。
> 碰撞检测逻辑本身不依赖任何美术资源，用基础几何体即可复现。
