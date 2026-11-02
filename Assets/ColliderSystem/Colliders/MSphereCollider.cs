using System;
using UnityEngine;

public class MSphereCollider : MColliderBase
{
    [NonSerialized]public Vector3 Center;
    [NonSerialized]public float Radius;
    protected  override void  OnEnable()
    {
        SphereCollider source = GetComponent<SphereCollider>();
        if (source == null)
        {
            throw new System.NullReferenceException("请输入对应Collider组件作为数据容器");
        }
        Center = source.center;
        Radius = source.radius;
        IsTrigger = source.isTrigger ? 1 : 0;
        Destroy(source);
        base.OnEnable();
    }
}
