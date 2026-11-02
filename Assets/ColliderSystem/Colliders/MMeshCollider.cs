using System;
using UnityEngine;

public class MMeshCollider : MColliderBase
{
    [NonSerialized]public Mesh SharedMesh;
    [NonSerialized]public bool Convex;
    [NonSerialized]public MeshColliderCookingOptions CookingOptions;
    protected  override void  OnEnable()
    {
        MeshCollider source = GetComponent<MeshCollider>();
        if (source == null)
        {
            throw new System.NullReferenceException("请输入对应Collider组件作为数据容器");
        }
        SharedMesh = source.sharedMesh;
        Convex = source.convex;
        CookingOptions = source.cookingOptions;
        IsTrigger = source.isTrigger ? 1 : 0;
        Destroy(source);
        base.OnEnable();
    }
}
