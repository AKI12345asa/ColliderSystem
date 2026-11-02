using System;
using Unity.Mathematics;
using UnityEngine;

public class MBoxCollider : MColliderBase
{
    [NonSerialized]public Vector3 Center;
    [NonSerialized]public Vector3 Size;
    protected  override void  OnEnable()
    {
        BoxCollider source = GetComponent<BoxCollider>();
        if (source == null)
        {
            throw new System.NullReferenceException("请输入对应Collider组件作为数据容器");
        }
        Center = source.center;
        Size = source.size;
        IsTrigger = source.isTrigger ? 1 : 0;
        Destroy(source);
        base.OnEnable();
    }
    
}
