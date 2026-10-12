# 路面高程剖面（激光雷达预瞄输出）
# 由 road_preview_node 发布，供悬架控制器 / 可视化使用

std_msgs/Header header

float32 spacing              # 纵向采样间距 (m)
float32 wheel_dist           # 左右轮间距 (m)
float32 preview_dist         # 预瞄距离 (m)

float32[] left_height        # 左轮迹路面高程 (m)，无效点为 NaN
float32[] right_height       # 右轮迹路面高程 (m)，无效点为 NaN
uint8[]   left_valid         # 左轮迹掩码：0=无效 1=实测有效 2=插值
uint8[]   right_valid        # 右轮迹掩码：0=无效 1=实测有效 2=插值

uint8     quality            # 数据质量 0-100（有效格占比）
