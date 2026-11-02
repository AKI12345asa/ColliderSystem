# 八叉树 UInt32 编码说明

本文说明 `Assets/VTNavigation/OCTree` 中 `VTCode` 与 `HashCode` 的编码逻辑。这里的 `UInt32` 不是通用哈希值，而是把八叉树节点的**层级和三维网格坐标**压缩到一个 32 位整数中。

## 1. 编码里存的是什么

一个节点编码包含：

- `layer`：节点层级。`layer = 0` 是最小叶格；层级每增加 1，节点边长增加一倍。
- `x、y、z`：节点在当前层级网格中的**整数索引**，不是节点中心坐标，也不是直接存储的世界坐标。
- 层级标记位：用 `1 << layer` 标识该编码所属的层级。

当前最大层级为 `MAX_LAYER = 10`。以最小格边长为 1 个树内单位时，`layer = L` 的节点边长为 `2^L`；根节点是 `layer = 10`，边长为 1024。

对于 `layer = 0`，编码位大致如下：

```text
bit 30..21：Y 坐标（10 位）
bit 20..11：Z 坐标（10 位）
bit 10..1 ：X 坐标（10 位）
bit 0     ：层级标记（1 << 0）
```

每层坐标字段的可用位数会随层级变化。较大的节点不需要那么多坐标位，所以编码布局由该层对应的 `MASK` 和 `OFFSET` 决定。

## 2. 位运算符的含义

| 运算 | 含义 | 在编码中的用途 |
|---|---|---|
| `a << n` | 左移 n 位 | 把坐标或标记移动到编码中的目标位置；子节点坐标也通过左移一位细分 |
| `a >> n` | 右移 n 位 | 将已经编码的字段移回低位，恢复坐标数值；父坐标通过右移一位合并 |
| `a & b` | 按位与 | 用掩码筛出需要的位；也用于检查某个状态标记是否置位 |
| `a | b` | 按位或 | 合并位段互不重叠的坐标字段，或设置某个标志位而不影响其他位 |
| `1u << n` | 生成第 n 位为 1 的数 | 表示层级标记或状态字节中的第 n 个子节点标记 |
| `^` | 按位异或 | 两个位不同时结果为 1；这里用于隔离一个数最右边的置位 |

## 3. `X_COORDINATE_MASK` 和 `X_COORDINATE_OFFSET`

坐标要放进一个整数的指定 bit 区域，编码时需要知道两件事：

- `X_COORDINATE_OFFSET[layer]`：X 坐标左移几位，才能对齐到 X 字段的起始位置。
- `X_COORDINATE_MASK[layer]`：哪些 bit 属于当前层级的 X 坐标字段；与它做 `&` 可以清除字段以外的位。

编码 X 坐标的表达式如下：

```csharp
(x << X_COORDINATE_OFFSET[layer]) & X_COORDINATE_MASK[layer]
```

解码 X 坐标时顺序相反：

```csharp
(code & X_COORDINATE_MASK[layer]) >> X_COORDINATE_OFFSET[layer]
```

也就是说，先用 `& MASK` 从总编码里取出 X 字段，再用 `>> OFFSET` 把它移回普通整数位置。

以 `layer = 0` 为例：

```text
X_COORDINATE_OFFSET[0] = 1
X_COORDINATE_MASK[0]   = 0x000007FE
```

掩码 `0x000007FE` 的 bit 10..1 为 1，表示这些位用于 X 坐标。假设 `x = 3`：

```text
3 << 1                 = 0b...00000110
(3 << 1) & 0x000007FE   = 0b...00000110
```

这一步将数值 3 放到编码的 X 区域。解码时：

```text
code & 0x000007FE       // 仅保留 X 字段
结果 >> 1               // 移回普通整数位置，得到 3
```

Y、Z 字段的原理相同，分别使用 `Y_COORDINATE_MASK` / `Y_COORDINATE_OFFSET` 和 `Z_COORDINATE_MASK` / `Z_COORDINATE_OFFSET`。这些表是按层级预先生成的，避免每次编码、解码时重新计算 bit 布局。

注意：在 `layer = 10`，坐标掩码为 0、偏移为 -1，因为根节点坐标固定为 `(0, 0, 0)`，没有可编码的坐标字段；因此不应将根层的偏移用于坐标移位。

## 4. 编码与解码

`VTCode.Encode(x, y, z, layer)` 分别将坐标放入对应字段，然后用按位或合并，并写入层级标记：

```csharp
((y << yOffset) & yMask)
| ((z << zOffset) & zMask)
| ((x << xOffset) & xMask)
| (1u << layer)
```

这些字段使用互不重叠的位，因此可以用 `|` 合并。以最小层 `layer = 0`、坐标 `(x, y, z) = (3, 5, 2)` 为例：

```text
X 字段 = (3 << 1)  & 0x000007FE
Z 字段 = (2 << 11) & 0x001FF800
Y 字段 = (5 << 21) & 0x7FE00000
层级位 = 1 << 0
```

将这些字段按位或合并，编码结果为 `0x00A01007`。通过对应掩码提取、再右移，就能恢复 `x = 3、y = 5、z = 2、layer = 0`。

`VTCode.Decode` 先恢复层级，再按该层级的坐标掩码和偏移解码三个坐标。`DecodeX`、`DecodeY`、`DecodeZ` 是单独提取各坐标的入口。

## 5. 如何从编码识别层级

编码中 `1u << layer` 是层级标记。对于有效编码，取出**最右侧的置位**就能得到这个标记：

```csharp
(v & (v - 1)) ^ v
```

例子：

```text
v             = 10110000
v - 1         = 10101111
v & (v - 1)   = 10100000
异或结果       = 00010000
```

结果只保留原数最右侧的 1。`LOG_MAPPING` 再把 `1、2、4、8……` 映射为对应的层级 `0、1、2、3……`。

该方式依赖编码有效：`INVALID_CODE`（`UInt32.MaxValue`）并不是普通节点编码，不能直接当作正常节点调用层级和坐标解码逻辑。应先用 `HashCode.IsValide` 判断。

## 6. 父子节点编码

八叉树每次细分时，每个轴都切成两半。父节点到子节点的坐标关系是：

```text
childX = (parentX << 1) | xOffset
childY = (parentY << 1) | yOffset
childZ = (parentZ << 1) | zOffset
childLayer = parentLayer - 1
```

其中每个 `offset` 是 0 或 1，表示选取该轴的低半侧或高半侧。左移一位相当于为坐标增加一个二进制位，按位或把本层方向放到最低位。

`HashCode.ToChild(int childOffset)` 接受 `0..7`，按以下位序拆成三个方向位：

```text
bit 0：X
bit 1：Z
bit 2：Y
```

因此子节点编号组合为：

```csharp
(yOffset << 2) | (zOffset << 1) | xOffset
```

`ToParent()` 则对各坐标右移一位、层级加 1，相当于把子格归并回父格。

`DecodeOffset()` 取当前节点三个坐标的最低位，得到它相对父节点所在的方向；`DecodeMergedOffset()` 再将方向组合成 `0..7` 的子节点编号。

## 7. 相邻节点

`ToLeft/ToRight`、`ToBottom/ToTop`、`ToBack/ToFront` 分别对某个轴的网格坐标减一或加一，然后重新编码。

底层函数会在坐标边界回绕，例如负向移动时 `0` 可能变成当前层最大坐标；代码通过比较移动前后坐标识别回绕。如果检测到越界，`HashCode` 方法返回 `INVALID_CODE`，表示该方向没有有效的相邻编码节点，不把回绕后的另一侧节点当成邻居。

## 8. `status` 字节中的位标记

八叉树节点的 `status` 是 `byte`，与节点的 `UInt32` 编码是两种不同用途的数据。状态字节的 8 个 bit 分别表示该节点 8 个直接子格是否阻塞：

```csharp
status & (1 << offset)           // 检查第 offset 个子格是否阻塞
status |= (byte)(1 << offset)    // 将该子格标记为阻塞
```

例如 `status = 00000100` 时，第 2 号子格被标记阻塞。`&` 用于检查位，`|=` 用于设置位且保留其他位。若 8 位全为 1，表示 8 个子格都阻塞，代码可以将这些状态向父节点合并。

## 9. 树内坐标、世界坐标与包围盒

编码中的坐标是树内的网格索引。`layer` 对应边长由 `POW_MAPPING[layer] = 2^layer` 提供。通过 `HashCode.DecodeBounds()` 还原节点包围盒：

```text
节点边长 = 2^layer
节点最小点 = (x, y, z) × 节点边长
节点最大点 = 节点最小点 + 节点边长
```

`HashCode` 本身不保存世界原点或旋转，也不自动应用 `MinSizeScale`。`OCTreeUtil` 提供树内与世界空间的比例换算：

```text
树内坐标 = 世界坐标 / MinSizeScale
世界坐标 = 树内坐标 × MinSizeScale
```

例如 `MinSizeScale = 0.5`，世界位置 `(3, 1, 4)` 对应树内位置 `(6, 2, 8)`。传入 `HashCode.GetHashCodeWithPoint` 的位置应已处于预期的树内坐标空间；该方法本身只按格子边长换算网格索引。

## 10. 点坐标转换方法的差异

`HashCode.GetHashCodeWithPoint(point, layer)` 使用 `FloorToInt` 求网格坐标，所以负坐标向负无穷取整。例如 `-0.2` 会归入索引 `-1`。

`HashCode.PositionToHashCode(position, layer)` 使用整数转换，C# 浮点转整数会向 0 截断。例如 `-0.2` 会得到索引 `0`。因此对于负坐标，两种方法的行为不同；调用前应根据坐标范围和期望的网格归属选择正确的方法。
