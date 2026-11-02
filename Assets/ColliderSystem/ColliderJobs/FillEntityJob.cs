using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using Unity.Mathematics;

[BurstCompile]
public struct FillEntityJob : IJobParallelFor
{
    [NativeDisableParallelForRestriction]
    public NativeArray<BoxColliderEntity> BoxColliderEntities;

    [NativeDisableParallelForRestriction]
    public NativeArray<SphereColliderEntity> SphereColliderEntities;

    [NativeDisableParallelForRestriction]
    public NativeArray<CapsuleColliderEntity> CapsuleColliderEntities;

    [NativeDisableParallelForRestriction]
    public NativeArray<MeshColliderEntity> MeshColliderEntities;
    
    [NativeDisableParallelForRestriction]
    public NativeArray<ColliderBoundsData> ColliderBoundsEntities;

    public void Execute(int index)
    {
        var type = GetColliderTypeByIndex(index, out int typeIndex);
        switch (type)
        {
            case ColliderType.Box:
                var boxEntity = GetBoxEntity(BoxColliderEntities[typeIndex]);
                BoxColliderEntities[typeIndex] = boxEntity;
                ColliderBoundsEntities[index] = new ColliderBoundsData(boxEntity.Id, boxEntity.Bounds, ColliderType.Box, boxEntity.Layer, typeIndex,index);
                break;
            case ColliderType.Sphere:
                var sphereEntity = GetSphereBoundingBox(SphereColliderEntities[typeIndex]);
                SphereColliderEntities[typeIndex] = sphereEntity;
                ColliderBoundsEntities[index] = new ColliderBoundsData(sphereEntity.Id, sphereEntity.Bounds, ColliderType.Sphere, sphereEntity.Layer, typeIndex,index);
                break;
            case ColliderType.Capsule:
                var capsuleEntity = GetCapsuleBoundingBox(CapsuleColliderEntities[typeIndex]);
                CapsuleColliderEntities[typeIndex] = capsuleEntity;
                ColliderBoundsEntities[index] = new ColliderBoundsData(capsuleEntity.Id, capsuleEntity.Bounds, ColliderType.Capsule, capsuleEntity.Layer, typeIndex,index);
                break;
            case ColliderType.Mesh:
                var meshEntity = GetMeshColliderEntity(MeshColliderEntities[typeIndex]);
                MeshColliderEntities[typeIndex] = meshEntity;
                ColliderBoundsEntities[index] = new ColliderBoundsData(meshEntity.Id, meshEntity.Bounds, ColliderType.Mesh, meshEntity.Layer, typeIndex,index);
                break;
        }
    }

    public BoxColliderEntity GetBoxEntity(BoxColliderEntity entity)
    {
        float3 scale = GetAxisScales(entity.LocalTransform);
        float3 size = math.abs(entity.Size) * scale;
        float3 center = math.transform(entity.LocalTransform, entity.Center);
        float3 halfSize = size * 0.5f;

        float3 axisX = math.mul(entity.Rotation, new float3(1f, 0f, 0f));
        float3 axisY = math.mul(entity.Rotation, new float3(0f, 1f, 0f));
        float3 axisZ = math.mul(entity.Rotation, new float3(0f, 0f, 1f));
        float3 extents = math.abs(axisX) * halfSize.x + math.abs(axisY) * halfSize.y + math.abs(axisZ) * halfSize.z;
        entity.Center = center;
        entity.Size = size;
        entity.ColliderBounds = new BoundingBox(center, extents);
        return entity;
    }

    private SphereColliderEntity GetSphereBoundingBox(SphereColliderEntity entity)
    {
        float3 scale = GetAxisScales(entity.LocalTransform);
        float radius = math.abs(entity.Radius) * math.cmax(scale);
        float3 center = math.transform(entity.LocalTransform, entity.Center);
        float3 extents = new float3(radius);

        entity.Center = center;
        entity.Radius = radius;
        entity.ColliderBounds = new BoundingBox(center, extents);
        return entity;
    }

    private CapsuleColliderEntity GetCapsuleBoundingBox(CapsuleColliderEntity entity)
    {
        float3 scale = GetAxisScales(entity.LocalTransform);
        float3 localAxis;
        float axisScale;
        float radiusScale;

        switch (entity.Direction)
        {
            case 0:
                localAxis = new float3(1f, 0f, 0f);
                axisScale = scale.x;
                radiusScale = math.max(scale.y, scale.z);
                break;
            case 2:
                localAxis = new float3(0f, 0f, 1f);
                axisScale = scale.z;
                radiusScale = math.max(scale.x, scale.y);
                break;
            case 1:
            default:
                localAxis = new float3(0f, 1f, 0f);
                axisScale = scale.y;
                radiusScale = math.max(scale.x, scale.z);
                break;
        }

        //这里是在根据当前Capsule轴向和默认轴向Y来计算额外要在Rotation上附加的旋转
        float3 defaultAxis = new float3(0f, 1f, 0f);
        float3 rotationAxis = math.cross(defaultAxis, localAxis);
        float rotationAxisLengthSq = math.lengthsq(rotationAxis);
        if (rotationAxisLengthSq > 1e-12f)
        {
            float angle = math.acos(math.clamp(math.dot(defaultAxis, localAxis), -1f, 1f));
            quaternion axisRotation = quaternion.AxisAngle(rotationAxis * math.rsqrt(rotationAxisLengthSq), angle);
            entity.Rotation = math.normalizesafe(math.mul(entity.Rotation, axisRotation));
        }

        float height = math.abs(entity.Height) * axisScale;
        float radius = math.abs(entity.Radius) * radiusScale;
        float3 center = math.transform(entity.LocalTransform, entity.Center);
        float3 axis = math.normalizesafe(math.mul(entity.Rotation, defaultAxis), defaultAxis);
        float halfSegment = math.max(0f, height * 0.5f - radius);
        float3 segmentExtent = math.abs(axis) * halfSegment;
        
        float3 extents = segmentExtent + new float3(radius);//胶囊体的半轴长还要加上radius作为额外偏移才能形成正确的AABB

        entity.Center = center;
        entity.Height = height;
        entity.Radius = radius;
        entity.ColliderBounds = new BoundingBox(center, extents);
        return entity;
    }

    private unsafe MeshColliderEntity GetMeshColliderEntity(MeshColliderEntity entity)
    {
        float3x3 linearTransform = new float3x3(
            entity.LocalTransform.c0.xyz,
            entity.LocalTransform.c1.xyz,
            entity.LocalTransform.c2.xyz);
        float3 localCenter = entity.ColliderBounds.Center;
        float3 localExtents = math.abs(entity.ColliderBounds.Extent);
        float3 center = math.transform(entity.LocalTransform, localCenter);
        float3 extents = math.abs(linearTransform.c0) * localExtents.x +
                         math.abs(linearTransform.c1) * localExtents.y +
                         math.abs(linearTransform.c2) * localExtents.z;
        for (int i = 0; i < entity.VertexCount; i++)
        {
            entity.VerticesInWorldSpace[i] = math.transform(entity.LocalTransform, entity.Vertices[i]);
        }
        entity.ColliderBounds = new BoundingBox(center, extents);
        return entity;
    }

    /// <summary>
    /// 变化矩阵前三列的长度代表队就是缩放值大小(其实变化矩阵的前三列就是其本地坐标轴的单位向量在世界空间下的实际的表达)
    /// </summary>
    /// <param name="transform"></param>
    /// <returns></returns>
    private float3 GetAxisScales(float4x4 transform)
    {
        return new float3(
            math.length(transform.c0.xyz),
            math.length(transform.c1.xyz),
            math.length(transform.c2.xyz));
    }

    private ColliderType GetColliderTypeByIndex(int index, out int typeIndex)
    {
        int offset = 0;
        if (index >= 0 && index < BoxColliderEntities.Length)
        {
            typeIndex = index;
            return ColliderType.Box;
        }

        offset += BoxColliderEntities.Length;
        if (index >= offset && index < offset + SphereColliderEntities.Length)
        {
            typeIndex = index - offset;
            return ColliderType.Sphere;
        }

        offset += SphereColliderEntities.Length;
        if (index >= offset && index < offset + CapsuleColliderEntities.Length)
        {
            typeIndex = index - offset;
            return ColliderType.Capsule;
        }

        offset += CapsuleColliderEntities.Length;
        if (index >= offset && index < offset + MeshColliderEntities.Length)
        {
            typeIndex = index - offset;
            return ColliderType.Mesh;
        }

        typeIndex = -1;
        return default;

    }
}
