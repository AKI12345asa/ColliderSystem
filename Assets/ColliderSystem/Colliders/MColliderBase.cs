using System;
using UnityEngine;
public abstract class MColliderBase : MonoBehaviour
{
    private event Action<MColliderBase> TriggerEnterEvent;
    private event Action<MColliderBase> TriggerStayEvent;
    private event Action<MColliderBase> TriggerExitEvent;
    [NonSerialized]public int IsTrigger;
    public int Layer => gameObject.layer;
    public float InMass => Mass > float.Epsilon ? 1 / Mass : 0;
    public float Mass;
     protected virtual void OnEnable()
     {
         ColliderSystem.RegisterCollider(this); 
     }

     protected virtual void OnDisable()
     {
         ColliderSystem.UnRegisterCollider(this);
     }

     public void RegisterTriggerEnter(Action<MColliderBase> callback)
     {
         TriggerEnterEvent += callback;
     }

     public void UnRegisterTriggerEnter(Action<MColliderBase> callback)
     {
         TriggerEnterEvent -= callback;
     }

     public void RegisterTriggerStay(Action<MColliderBase> callback)
     {
         TriggerStayEvent += callback;
     }

     public void UnRegisterTriggerStay(Action<MColliderBase> callback)
     {
         TriggerStayEvent -= callback;
     }

     public void RegisterTriggerExit(Action<MColliderBase> callback)
     {
         TriggerExitEvent += callback;
     }

     public void UnRegisterTriggerExit(Action<MColliderBase> callback)
     {
         TriggerExitEvent -= callback;
     }

     public virtual void TriggerEnter(MColliderBase other)
     {
         TriggerEnterEvent?.Invoke(other);
     }

     public virtual void TriggerStay(MColliderBase other)
     {
         TriggerStayEvent?.Invoke(other);
     }

     public virtual void TriggerExit(MColliderBase other)
     {
         TriggerExitEvent?.Invoke(other);
     }
     
}
