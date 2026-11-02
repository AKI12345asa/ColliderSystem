using ColliderMath;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

[BurstCompile]
public struct ColliderHandlerJob:IJobParallelFor
{
    [ReadOnly]public NativeArray<BoxColliderEntity> BoxColliderEntities;
    [ReadOnly]public NativeArray<SphereColliderEntity> SphereColliderEntities;
    [ReadOnly]public NativeArray<CapsuleColliderEntity> CapsuleColliderEntities;
    [ReadOnly]public NativeArray<MeshColliderEntity> MeshColliderEntities;
    [ReadOnly]public NativeStream.Reader ColliderCoupleStream;
    public NativeStream.Writer Result;
    public void Execute(int index)
    {
        var reader = ColliderCoupleStream;
        var writer = Result;
        writer.BeginForEachIndex(index);
        var coupleCount =  reader.BeginForEachIndex(index) / 3; //因为三个数据构成一份碰撞对
        for (int i = 0; i < coupleCount; i++)
        {
            var type = reader.Read<ColliderCoupleType>();
            var index1 = reader.Read<int>();
            var index2 = reader.Read<int>();
            if (!CheckCollide(type, index1, index2, out var couple)) continue;
            writer.Write(couple);
        }
        reader.EndForEachIndex();
        writer.EndForEachIndex();
    }

    private bool CheckCollide(ColliderCoupleType type, int index1, int index2,out ColliderCouple couple)
    {
        couple = default;
        float3 intersectSegment;
        switch (type)
        {
            case ColliderCoupleType.BoxBox:
            {
                BoxColliderEntity box1 = BoxColliderEntities[index1];
                BoxColliderEntity box2 = BoxColliderEntities[index2];
                var result =  OBBIntersectTool.Intersects(box1, box2,out intersectSegment);
                intersectSegment = box1.Id > box2.Id ? -intersectSegment : intersectSegment;
                couple = new ColliderCouple(box1.ColliderId, box2.ColliderId,intersectSegment);
                return result;
            }
            case ColliderCoupleType.BoxSphere:
            {
                BoxColliderEntity box = BoxColliderEntities[index1];
                SphereColliderEntity sphere = SphereColliderEntities[index2];
                var result =  OBBIntersectTool.Intersects(box, sphere,out intersectSegment);
                intersectSegment = box.Id > sphere.Id ? -intersectSegment : intersectSegment;
                couple = new ColliderCouple(box.ColliderId, sphere.ColliderId,intersectSegment);
                return result;
            }
            case ColliderCoupleType.BoxCapsule:
            {
                BoxColliderEntity box = BoxColliderEntities[index1];
                CapsuleColliderEntity capsule = CapsuleColliderEntities[index2];
                var result = OBBIntersectTool.Intersects(box,capsule,out intersectSegment);
                intersectSegment = box.ColliderId > capsule.ColliderId ? -intersectSegment : intersectSegment;
                couple = new ColliderCouple(box.ColliderId, capsule.ColliderId, intersectSegment);
                return result;
            }
            case ColliderCoupleType.BoxMesh:
            {
                BoxColliderEntity box = BoxColliderEntities[index1];
                MeshColliderEntity mesh = MeshColliderEntities[index2];
                var result = MeshTree.IntersectingWithMesh(mesh, box, mesh.LocalTransform,out intersectSegment);
                intersectSegment = box.ColliderId > mesh.ColliderId ? -intersectSegment : intersectSegment;
                couple = new ColliderCouple(box.ColliderId, mesh.ColliderId, intersectSegment);
                return result;
            }
            case ColliderCoupleType.SphereSphere:
            {
                SphereColliderEntity sphere1 = SphereColliderEntities[index1];
                SphereColliderEntity sphere2 = SphereColliderEntities[index2];
                var result = GeometryMathTool.CheckTwoSphereIntersect(sphere1.Center, sphere1.Radius, sphere2.Center, sphere2.Radius,out intersectSegment);
                intersectSegment = sphere1.ColliderId > sphere2.ColliderId ? -intersectSegment : intersectSegment;
                couple = new ColliderCouple(sphere1.ColliderId, sphere2.ColliderId, intersectSegment);
                return result;
            }
            case ColliderCoupleType.SphereCapsule:
            {
                SphereColliderEntity sphere = SphereColliderEntities[index1];
                CapsuleColliderEntity capsule = CapsuleColliderEntities[index2];
                var result = GeometryMathTool.CheckCapsuleIntersectWithSphere(capsule.Center, capsule.Rotation, capsule.Height, capsule.Radius, sphere.Center, sphere.Radius,out intersectSegment);
                intersectSegment = -intersectSegment;
                intersectSegment = sphere.ColliderId > capsule.ColliderId ? -intersectSegment : intersectSegment;
                couple = new ColliderCouple(sphere.ColliderId, capsule.ColliderId, intersectSegment);
                return result;
            }
            case ColliderCoupleType.SphereMesh:
            {
                SphereColliderEntity sphere = SphereColliderEntities[index1];
                MeshColliderEntity mesh = MeshColliderEntities[index2];
                var result = MeshTree.IntersectingWithMesh(mesh, sphere, mesh.LocalTransform,out intersectSegment);
                intersectSegment = sphere.ColliderId > mesh.ColliderId ? -intersectSegment : intersectSegment;
                couple = new ColliderCouple(sphere.ColliderId, mesh.ColliderId, intersectSegment);
                return result;
            }
            case ColliderCoupleType.CapsuleCapsule:
            {
                CapsuleColliderEntity capsule1 = CapsuleColliderEntities[index1];
                CapsuleColliderEntity capsule2 = CapsuleColliderEntities[index2];
                var result = GeometryMathTool.CheckTwoCapsuleIntersect(capsule1.Center, capsule1.Rotation, capsule1.Height, capsule1.Radius, capsule2.Center, capsule2.Rotation, capsule2.Height, capsule2.Radius,out intersectSegment);
                intersectSegment = capsule1.ColliderId > capsule2.ColliderId ? -intersectSegment : intersectSegment;
                couple = new ColliderCouple(capsule1.ColliderId, capsule2.ColliderId, intersectSegment);
                return result;
            }
            case ColliderCoupleType.CapsuleMesh:
            {
                CapsuleColliderEntity capsule = CapsuleColliderEntities[index1];
                MeshColliderEntity mesh = MeshColliderEntities[index2];
                var result = MeshTree.IntersectingWithMesh(mesh, capsule, mesh.LocalTransform,out intersectSegment);
                intersectSegment = capsule.ColliderId > mesh.ColliderId ? -intersectSegment : intersectSegment;
                couple = new ColliderCouple(capsule.ColliderId, mesh.ColliderId, intersectSegment);
                return result;
            }
            case ColliderCoupleType.MeshMesh:
            {
                MeshColliderEntity mesh1 = MeshColliderEntities[index1];
                MeshColliderEntity mesh2 = MeshColliderEntities[index2];
                var result = MeshTree.IntersectingWithMesh(mesh1, mesh1.LocalTransform, mesh2, mesh2.LocalTransform,out intersectSegment);
                intersectSegment = mesh1.ColliderId > mesh2.ColliderId ? intersectSegment : -intersectSegment;
                couple = new ColliderCouple(mesh1.ColliderId, mesh2.ColliderId, intersectSegment);
                return result;
            }
        }
        return false;
    }
}
