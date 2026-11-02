using Unity.Mathematics;
//这库没啥用
public static class ColliderMathTool
{
      public static float Dot(this float3 v1, float3 v2)
      {
            return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
      }

      public static float3 Cross(this float3 v1, float3 v2)
      {
            return new float3(v1.y * v2.z - v1.z * v2.y,v1.x * v2.z - v1.z * v2.x,v1.x * v2.y - v1.y * v2.x); 
      }

      public static float Distance(this float3 v1, float3 v2)
      {
            return math.sqrt((v1 - v2).SqrDistance());
      }

      public static float3 Normalize(this float3 v)
      {
            var length = math.length(v);
            return new float3(v.x / length, v.y / length, v.z / length);
      }
      
      public static float SqrDistance(this float3 v)
      {
            return v.x * v.x + v.y * v.y + v.z * v.z;
      }

      public static float4 Inverse(this float4 quaternion)
      {
            var lenSq = quaternion.w * quaternion.w + quaternion.z * quaternion.z + quaternion.x * quaternion.x + quaternion.y * quaternion.y;
            return new  float4(-quaternion.x / lenSq,-quaternion.y / lenSq,-quaternion.z / lenSq,quaternion.w / lenSq);
      }
      
      public static float4 MultipleQuaternion(this float4 q1, float4 q2)
      {
            var w = q1.w * q2.w - q1.x * q2.x + q1.y * q2.y + q1.z * q2.z;
            var y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x;
            var x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y;
            var z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w;
            return new float4(x, y, z, w);
      }

      public static bool Equal(this float3 v1, float3 v2)
      {
            return (math.abs(v1.x - v2.x) < math.EPSILON) && (math.abs(v1.y - v2.y) < math.EPSILON) && (math.abs(v1.z - v1.z) < math.EPSILON);
      }

      public static float3 Rotate(this float4 q, float3 v)
      {
            var inverse = q.Inverse();
            var vq = new float4(v.x,v.y,v.z,0);
            var res = q.MultipleQuaternion(vq).MultipleQuaternion(inverse);
            return new float3(res.x,res.y,res.z);
      }
}
