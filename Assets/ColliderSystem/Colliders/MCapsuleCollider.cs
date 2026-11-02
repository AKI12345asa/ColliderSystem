using System;
using UnityEngine;

public class MCapsuleCollider : MColliderBase
{
    [NonSerialized]public Vector3 Center;
    [NonSerialized]public float Radius;
    [NonSerialized]public float Height;
    [NonSerialized]public int Direction;
    
    protected  override void  OnEnable()
    {
        CapsuleCollider source = GetComponent<CapsuleCollider>();
        if (source == null)
        {
            throw new System.NullReferenceException("请输入对应Collider组件作为数据容器");
        }
        Center = source.center;
        Radius = source.radius;
        Height = source.height;
        Direction = source.direction;
        IsTrigger = source.isTrigger ? 1 : 0;
        Destroy(source);
        base.OnEnable();
    }

    
}
