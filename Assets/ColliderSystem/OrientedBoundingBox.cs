using Unity.Collections;
using Unity.Mathematics;

public struct OrientedBoundingBox
{
    public float3 Center;
    public float3 Extents;
    public quaternion Orientation;
    public float3 AxisX;
    public float3 AxisY;
    public float3 AxisZ;
    public BoundingBox LocalBounds;

    public OrientedBoundingBox(float3 center, float3 extents, quaternion orientation)
    {
        Center = center;
        Extents = extents;
        Orientation = orientation;
        AxisX = math.mul(orientation, new float3(1f, 0f, 0f));
        AxisY = math.mul(orientation, new float3(0f, 1f, 0f));
        AxisZ = math.mul(orientation, new float3(0f, 0f, 1f));
        LocalBounds = new BoundingBox(float3.zero, Extents);
    }

    public OrientedBoundingBox ToLocalSpace(OrientedBoundingBox reference)
    {
        quaternion inverse = math.inverse(reference.Orientation);
        float3 center = math.mul(inverse, Center - reference.Center);
        quaternion orientation = math.mul(inverse, Orientation);
        return new OrientedBoundingBox(center, Extents, orientation);
    }


}
