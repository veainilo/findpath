import matplotlib.pyplot as plt
import numpy as np
from matplotlib import rcParams
rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'WenQuanYi Micro Hei']  # 设置中文字体
rcParams['axes.unicode_minus'] = False  # 解决负号显示问题

class SimplePathFinder:
    def __init__(self, grid):
        """
        初始化路径规划器
        :param grid: 二维数组，0表示可通行，1表示障碍
        """
        self.grid = grid
        self.rows = len(grid)
        self.cols = len(grid[0]) if self.rows > 0 else 0
        self.visited = set()
        self.max_steps = 500  # 防止无限递归

    def find_path(self, start, end):
        """修改后的主路径查找方法"""
        self.end = end  # 保存终点用于绕行判断
        path = [start]
        current = start
        self.visited = {start}
        retry_count = 0  # 添加重试机制

        for _ in range(self.max_steps):
            if current == end:
                return path

            # 尝试直线通行（增加重试机制）
            if self._direct_path_clear(current, end):
                if retry_count < 2:
                    path.append(end)
                    return path
                else:
                    # 多次重试后需要验证路径
                    verify_path = path + [end]
                    if self._verify_full_path(verify_path):
                        return verify_path

            # 寻找并处理碰撞点（新增障碍物轮廓跟踪）
            collision = self._find_first_collision(current, end)
            if not collision:
                return path + [end]

            # 获取并选择绕行点
            detour_points = self._find_detour_points(collision, current)
            if not detour_points:
                # 尝试回溯
                if len(path) > 1:
                    current = path[-2]
                    path = path[:-1]
                    retry_count += 1
                    continue
                else:
                    return None

            current = min(detour_points, 
                         key=lambda p: (self._heuristic(p, end), len(path)))
            path.append(current)
            self.visited.add(current)
            retry_count = 0

        return None

    def _direct_path_clear(self, a, b):
        """检查直线是否可通行（增加端点检查）"""
        line_points = self._bresenham_line(a, b)
        # 排除起点（因为起点可能位于障碍边缘）
        return all(self.grid[x][y] != 1 for (x, y) in line_points[1:-1])

    def _find_first_collision(self, start, end):
        """沿直线找到第一个障碍物坐标"""
        for point in self._bresenham_line(start, end):
            x, y = point
            if self.grid[x][y] == 1:
                return (x, y)
        return None

    def _find_detour_points(self, collision, from_point):
        """改进的绕行点选择策略"""
        cx, cy = collision
        candidates = []
        
        # 根据行进方向生成优先方向
        dx_dir = cx - from_point[0]
        dy_dir = cy - from_point[1]
        
        # 生成8个绕行方向（优先与行进方向垂直）
        directions = []
        if dx_dir != 0:  # 垂直运动优先左右绕行
            directions += [(0, 1), (0, -1), (1, 0), (-1, 0)]
        if dy_dir != 0:  # 水平运动优先上下绕行
            directions += [(1, 0), (-1, 0), (0, 1), (0, -1)]
        
        # 添加对角线方向
        directions += [(1,1), (1,-1), (-1,1), (-1,-1)]
        
        # 去重并保持顺序
        seen = set()
        directions = [d for d in directions if not (d in seen or seen.add(d))]
        
        # 评估每个方向的可行性
        for dx, dy in directions:
            x, y = cx + dx, cy + dy
            if (0 <= x < self.rows and 0 <= y < self.cols and
                self.grid[x][y] == 0 and (x, y) not in self.visited):
                
                # 检查新位置到终点的视线是否畅通
                if self._direct_path_clear((x,y), self.end):
                    candidates.append((x, y))
        
        # 优先选择离终点更近的点
        return sorted(candidates, key=lambda p: self._heuristic(p, self.end))[:2]

    def _heuristic(self, a, b):
        """估算两点距离"""
        return abs(a[0]-b[0]) + abs(a[1]-b[1])  # 曼哈顿距离

    def _bresenham_line(self, start, end):
        """生成两点间直线经过的格子坐标（修正坐标系）"""
        x0, y0 = start
        x1, y1 = end
        # 移除steep转换，保持原始坐标系
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        error = 0
        y_step = 1 if y0 < y1 else -1
        y = y0
        points = []
        
        for x in range(x0, x1 + 1):
            points.append((x, y))
            error += dy
            if 2 * error >= dx:
                y += y_step
                error -= dx
        return points

    def _verify_full_path(self, path):
        """完整路径验证"""
        for i in range(len(path)-1):
            if not self._direct_path_clear(path[i], path[i+1]):
                return False
        return True

    # 新增可视化方法
    def plot_path(self, path, title='路径可视化'):
        """优化中文显示的可视化方法"""
        plt.figure(figsize=(10, 10), dpi=100)
        
        # 绘制网格背景
        plt.grid(True, which='both', color='lightgray', linestyle='-', linewidth=0.8)
        plt.gca().set_axisbelow(True)

        # 显示网格地图
        grid_array = np.array(self.grid)
        plt.imshow(grid_array, cmap='Greys', origin='lower',
                  extent=(-0.5, self.cols-0.5, -0.5, self.rows-0.5),
                  vmin=0, vmax=1, alpha=0.5)

        # 绘制障碍物（优化中文图例）
        obstacle_y, obstacle_x = np.where(grid_array == 1)
        plt.scatter(obstacle_x + 0.5, obstacle_y + 0.5,
                   c='darkred', s=1500, marker='s',
                   edgecolor='black', linewidth=1.5,
                   alpha=0.7, label='障碍区域')

        # 绘制路径（带箭头指示）
        if path:
            y_coords = [p[0] + 0.5 for p in path]  # 显示在格子中心
            x_coords = [p[1] + 0.5 for p in path]
            plt.plot(x_coords, y_coords, 'b-', linewidth=3, alpha=0.7)
            plt.scatter(x_coords, y_coords, c='blue', s=100, edgecolor='white', zorder=4)
            
            # 添加路径箭头
            for i in range(len(x_coords)-1):
                dx = x_coords[i+1] - x_coords[i]
                dy = y_coords[i+1] - y_coords[i]
                plt.arrow(x_coords[i], y_coords[i], dx*0.8, dy*0.8, 
                         head_width=0.3, head_length=0.4, 
                         fc='dodgerblue', ec='navy', linewidth=2, zorder=5)
        
        # 标记起点终点（带阴影效果）
        if path:
            start = path[0]
            end = path[-1]
            plt.scatter(start[1]+0.5, start[0]+0.5, 
                       c='limegreen', s=400, marker='s',
                       edgecolor='darkgreen', linewidth=2,
                       zorder=5, label='起点')
            plt.scatter(end[1]+0.5, end[0]+0.5, 
                       c='gold', s=600, marker='*',
                       edgecolor='darkorange', linewidth=2,
                       zorder=5, label='终点')
        
        # 添加障碍物边界强调
        if hasattr(self, 'end'):
            # 绘制终点方向指示
            plt.arrow(end[1]+0.5, end[0]+0.5, 
                     (end[1] - start[1])*0.2, (end[0] - start[0])*0.2,
                     color='purple', linestyle=':', width=0.05)
        
        # 添加绕行点标记
        if path and len(path) > 2:
            for i in range(1, len(path)-1):
                plt.text(path[i][1]+0.5, path[i][0]+0.5, str(i),
                        fontsize=10, color='darkred', weight='bold')
        
        # 优化中文标注
        plt.title(title, fontsize=16, pad=20, fontweight='bold')
        plt.xlabel('X 坐标', fontsize=12, labelpad=10)
        plt.ylabel('Y 坐标', fontsize=12, labelpad=10)
        
        plt.xticks(range(self.cols))
        plt.yticks(range(self.rows))
        plt.legend(loc='upper right', fontsize=10)
        plt.tight_layout()
        plt.show()

# 修改测试用例部分
if __name__ == "__main__":
    # 测试环境1：无障碍
    grid1 = [[0]*10 for _ in range(10)]
    finder1 = SimplePathFinder(grid1)
    path1 = finder1.find_path((0,0), (9,9))
    print("Test 1 Path:", path1)
    finder1.plot_path(path1, '无障碍路径')
    
    # 测试环境2：单个障碍
    grid2 = [[0]*10 for _ in range(10)]
    grid2[5][3:7] = [1,1,1,1]
    finder2 = SimplePathFinder(grid2)
    path2 = finder2.find_path((0,0), (9,9))
    print("\nTest 2 Path:", path2)
    finder2.plot_path(path2, '障碍绕行路径')
    
    # 测试环境3：封闭环境
    grid3 = [[1 if 0<x<9 and 0<y<9 else 0 for y in range(10)] for x in range(10)]
    finder3 = SimplePathFinder(grid3)
    path3 = finder3.find_path((0,0), (9,9))
    print("\nTest 3 Result:", path3)
    finder3.plot_path(path3, '无解情况')