using System;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine.Jobs;

public enum ColliderCoupleType
{
     BoxBox,
     BoxSphere,
     BoxCapsule,
     BoxMesh,
     SphereSphere,
     SphereCapsule,
     SphereMesh,
     CapsuleCapsule,
     CapsuleMesh,
     MeshMesh
}

public enum ColliderType
{
     Box,
     Sphere,
     Mesh,
     Capsule,
}

public readonly struct ColliderCouple:IEquatable<ColliderCouple>
{
     public readonly int ColliderAId;
     public readonly int ColliderBId;
     public readonly float3 MVT;
     public ColliderCouple(int colliderAId, int colliderBId,float3 mvt)
     {
          if (colliderAId <= colliderBId)
          {
               ColliderAId = colliderAId;
               ColliderBId = colliderBId;
          }
          else
          {
               ColliderAId = colliderBId;
               ColliderBId = colliderAId;
          }

          this.MVT = mvt;
     }
     
     public bool Equals(ColliderCouple other)
     {
          return other.ColliderAId == ColliderAId && other.ColliderBId == ColliderBId;
     }
     
     public override int GetHashCode()
     {
          unchecked
          {
               return ColliderAId * 16777213 + ColliderBId;
          }
     }
}

public interface IColliderEntity
{
     public int Id { get; }
     public BoundingBox Bounds { get; }
     public int Index { get; set; }
     
     public int LayerMask { get; set; }
}

public struct BoxColliderEntity:IColliderEntity
{
     public int ColliderId;
     public quaternion Rotation;
     public BoundingBox ColliderBounds;
     public float3 Center;
     public float4x4 LocalTransform;
     public float3 Size;
     public int IsTrigger;
     public int Id => ColliderId;
     public BoundingBox Bounds => ColliderBounds;
     public int Index { get; set; }
     public int Layer;
     public int LayerMask { get; set; }
     public override int GetHashCode() => Id;
}

public struct SphereColliderEntity:IColliderEntity
{
     public int ColliderId;
     public BoundingBox ColliderBounds;
     public float3 Center;
     public float Radius;
     public int IsTrigger;
     public int Id => ColliderId;
     public float4x4 LocalTransform;
     public BoundingBox Bounds => ColliderBounds;
     public int Index { get; set; }
     public int Layer;
     public int LayerMask { get; set; }
     public override int GetHashCode() => Id;
}

public struct CapsuleColliderEntity:IColliderEntity
{
     public int ColliderId;
     public quaternion Rotation;
     public BoundingBox ColliderBounds;
     public float3 Center;
     public float Radius;
     public float Height;
     public int IsTrigger;
     public int Id => ColliderId;
     public BoundingBox Bounds => ColliderBounds;
     public int Index { get; set; }
     public int Direction;

     public float4x4 LocalTransform;
     
     public int Layer;
     public int LayerMask { get; set; }

     public override int GetHashCode() => Id;
}

public unsafe struct MeshColliderEntity:IColliderEntity
{
     public int ColliderId;
     public BoundingBox ColliderBounds;
     public float3* Vertices;
     public float3* VerticesInWorldSpace;
     public int VertexCount;
     public int* Triangles;
     public int TriangleIndexCount;
     public float4x4 LocalTransform;
     public MeshColliderBVTData BVTData;
     public int IsTrigger;
     public int VertexLength;
     public int Layer;
     public int LayerMask { get; set; }
     public int Id => ColliderId;
     public BoundingBox Bounds => ColliderBounds;
     public int Index { get; set; }
     public override int GetHashCode() => Id;
}

public struct MeshTriangleEntity : IColliderEntity
{
     public int TriangleIndex;
     public ColliderTriangle Triangle;
     public BoundingBox TriangleBounds;
     public int Layer;
     public int LayerMask { get; set; }
     public int Id => TriangleIndex + 1;
     public BoundingBox Bounds => TriangleBounds;
     public int Index { get; set; }
     public override int GetHashCode() => Id;
}



public struct MeshColliderBuildData
{
     public int ColliderId;
     public unsafe float3* Vertices;
     public int VertexCount;
     public unsafe int* Triangles;
     public int TriangleCount;
     public int EntityStart;
}

public struct ColliderTriangle
{
     public float3 P0;
     public float3 P1;
     public float3 P2;

     public ColliderTriangle(float3 p0, float3 p1, float3 p2)
     {
          P0 = p0;
          P1 = p1;
          P2 = p2;
     }

     public float3 this[int index]
     {
          get
          {
               if (index == 0) return P0;
               if (index == 1) return P1;
               return P2;
          }
          set
          {
               if (index == 0) P0 = value;
               else if (index == 1) P1 = value;
               else P2 = value;
          }
     }

     public bool TryGetCommonEdge(ColliderTriangle triangle, out Edge edge)
     {
          float3 p0 = default;
          var check = false;
          edge = default;
          if (P0.Equals(triangle.P0))
          {
               check = true;
               p0 = P0;
          }

          if (P0.Equals(triangle.P1))
          {
               check = true;
               p0 = P0;
          }

          if (P0.Equals(triangle.P2))
          {
               check = true;
               p0 = P0;
          }

          if (P1.Equals(triangle.P0))
          {
               if (check)
               {
                    edge = new Edge(p0, P1);
                    return true;
               }

               check = true;
               p0 = P1;
          }

          if (P1.Equals(triangle.P1))
          {
               if (check)
               {
                    edge = new Edge(p0, P1);
                    return true;
               }

               check = true;
               p0 = P1;
          }

          if (P1.Equals(triangle.P2))
          {
               if (check)
               {
                    edge = new Edge(p0, P1);
                    return true;
               }

               check = true;
               p0 = P1;
          }

          if (!check) return false;

          if (P2.Equals(triangle.P0))
          {
               edge = new Edge(p0, P2);
               return true;
          }

          if (P2.Equals(triangle.P1))
          {
               edge = new Edge(p0, P2);
               return true;
          }

          if (P2.Equals(triangle.P2))
          {
               edge = new Edge(p0, P2);
               return true;
          }

          return false;
          
     }
     

     /// <summary>
     /// 获取法线
     /// </summary>
     /// <param name="ifPointToZero">是否朝向零点</param>
     /// <returns></returns>
     public float3 GetNormal(bool ifPointToZero)
     {
          var f1 = P1 - P0;
          var f2 = P2 - P0;
          var normal = math.cross(f1, f2);
          var dot = math.dot(-P0, normal);
          if (ifPointToZero &&  dot > 0) return normal;
          if( !ifPointToZero && dot <= 0) return normal;
          return -normal;
     }
}

public struct Edge:IComparable<Edge>,IEquatable<Edge>
{
     public float3 P0;
     public float3 P1;

     public Edge(float3 p0, float3 p1)
     {
        if (ComparePoints(p0, p1) >= 0)
        {
             P0 = p0;
             P1 = p1;
        }
        else
        {
             P0 = p1;
             P1 = p0;
        }
     }

     private static int ComparePoints(float3 a, float3 b)
     {
          int comparison = a.x.CompareTo(b.x);
          if (comparison != 0) return comparison;
          comparison = a.y.CompareTo(b.y);
          return comparison != 0 ? comparison : a.z.CompareTo(b.z);
     }

     public int CompareTo(Edge other)
     {
          int comparison = ComparePoints(P0, other.P0);
          return comparison != 0 ? comparison : ComparePoints(P1, other.P1);
     }

     public bool Equals(Edge other)
     {
          return P0.Equals(other.P0) && P1.Equals(other.P1);
     }
     
     public override int GetHashCode()
     {
          return HashCode.Combine(P0, P1);
     }
}

public struct MinHeapEntry
{
     public ColliderTriangle Triangle;
     public float Distance;
     public float3 Normal;
     public MinHeapEntry(ColliderTriangle triangle,float distance)
     {
          Triangle = triangle;
          Distance = distance;
          Normal = triangle.GetNormal(false);
     }
}

public unsafe struct MeshColliderBVTData
{
     public BVTNode* Nodes;
     public MeshTriangleEntity* Entities;
     public int* RootIndies;
     public int Index;

     public int RootIndex => RootIndies == null || Index < 0 ? -1 : RootIndies[Index];
}

public  struct ColliderBoundsData
{
     public int Id;
     public BoundingBox Bounds;
     public ColliderType ColliderType;
     public int Layer;
     public int TypeIndex;
     public int EntityIndex;
     public ColliderBoundsData(int id, BoundingBox bounds,ColliderType colliderType,int layer,int typeIndex,int entityIndex)
     {
          Id = id;
          Bounds = bounds;
          ColliderType = colliderType;
          Layer = layer;
          TypeIndex = typeIndex;
          EntityIndex = entityIndex;
     }
}


