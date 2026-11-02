using Unity.Burst;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Jobs;

[BurstCompile]
public struct ColliderMTVJob:IJobParallelForTransform
{
    public NativeArray<float3> MTVArray;
    
    public void Execute(int index, TransformAccess transform)
    {
        var mtv = MTVArray[index];
        transform.position += (Vector3)mtv;
    }
}
