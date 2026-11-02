using UnityEngine;

	/// <summary>由最小值和最大值表示的闭区间，常用于投影和射线参数范围。</summary>
	public struct Interval
	{
		public float min;
		public float max;

		public Interval(float min, float max)
		{
			this.min = min;
			this.max = max;
		}

		public bool Contains(float value)
		{
			// 端点属于区间。
			return value >= min && value <= max;
		}

		public bool IsValid
		{
			get
			{
				// Infinity 被用作未命中/未初始化的区间哨兵值。
				return min != Mathf.Infinity && max != Mathf.Infinity;
			}
		}
	}
