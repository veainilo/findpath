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
        self.end = end
        self.visited = set()
        path = [start]
        current = start
        self.visited.add(current)
        
        for _ in range(self.max_steps):
            # 每次迭代都检测完整路径
            if self._direct_path_clear(current, end):
                path.append(end)
                if self._verify_full_path(path):
                    return path
                else:
                    path.pop()  # 移除不可行的终点
            
            # 寻找新的碰撞点（可能变化）
            collision = self._find_first_collision(current, end)
            if not collision:
                return path + [end] if self._direct_path_clear(current, end) else None
            
            # 获取绕行点（考虑后续路径）
            detour_points = self._get_safe_detours(collision, current)
            if not detour_points:
                # 回溯机制
                if len(path) > 1:
                    current = path[-2]
                    path = path[:-1]
                    continue
                else:
                    return None
            
            # 选择最优绕行点（考虑后续障碍）
            current = self._select_best_detour(detour_points, end)
            path.append(current)
            self.visited.add(current)
        
        return None

    def _direct_path_clear(self, a, b):
        """检查直线是否可通行（增加端点检查）"""
        line_points = self._bresenham_line(a, b)
        # 排除起点（因为起点可能位于障碍边缘）
        return all(self.grid[x][y] != 1 for (x, y) in line_points[1:-1])
    
    def _find_first_collision(self, start, end):
        """精确的碰撞检测"""
        line = self._bresenham_line(start, end)
        # 包含终点在内的完整检测
        for point in line:
            x, y = point
            if (x, y) == end:
                return None  # 到达终点
            if self.grid[x][y] == 1:
                return (x, y)
        return None

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
    
    def _get_safe_detours(self, collision, from_point):
        """改进的绕行点获取策略"""
        candidates = []
        # 扩展搜索方向（包含对角线）
        directions = [(-1,0), (1,0), (0,-1), (0,1), 
                     (-1,-1), (-1,1), (1,-1), (1,1)]
        
        for dx, dy in directions:
            x, y = collision[0] + dx, collision[1] + dy
            if (0 <= x < self.rows and 0 <= y < self.cols 
                and self.grid[x][y] == 0 
                and (x, y) not in self.visited):
                
                # 多步预测（而不仅是直线预测）
                if self._multi_step_safety_check((x,y), self.end, depth=2):
                    candidates.append((x, y))
        
        return sorted(candidates, key=lambda p: self._heuristic(p, self.end))[:3]

    def _multi_step_safety_check(self, point, end, depth):
        """递归多步安全检测"""
        if depth == 0:
            return True
        if self._direct_path_clear(point, end):
            return True
        next_collision = self._find_first_collision(point, end)
        if not next_collision:
            return True
        # 检查是否有绕行可能
        return any(self._multi_step_safety_check((next_collision[0]+dx, next_collision[1]+dy), end, depth-1)
                   for dx, dy in [(-1,0), (1,0), (0,-1), (0,1)])

    def _verify_full_path(self, path):
        """验证整个路径是否畅通"""
        for i in range(len(path) - 1):
            if not self._direct_path_clear(path[i], path[i+1]):
                return False
        return True

    def _select_best_detour(self, detours, end):
        """改进的绕行点选择策略"""
        # 优先选择能看到最远路径的点
        best_point = None
        max_visible = -1
        for p in detours:
            visible_length = self._visible_path_length(p, end)
            if visible_length > max_visible:
                max_visible = visible_length
                best_point = p
        return best_point if best_point else min(detours, key=lambda p: self._heuristic(p, end))

    def _visible_path_length(self, point, end):
        """计算可见路径长度"""
        path = self._bresenham_line(point, end)
        count = 0
        for (x,y) in path:
            if self.grid[x][y] == 1:
                break
            count += 1
        return count

    def _heuristic(self, a, b):
        """估算两点距离"""
        return abs(a[0]-b[0]) + abs(a[1]-b[1])  # 曼哈顿距离

    def plot_path(self, path, title='路径可视化'):
        """显示绕行点的可视化方法"""
        plt.figure(figsize=(8, 8))
        
        # 显示网格和障碍物
        plt.imshow(self.grid, cmap='Greys', origin='lower',
                  extent=(-0.5, self.cols-0.5, -0.5, self.rows-0.5),
                  alpha=0.3, vmin=0, vmax=1)
        
        # 绘制路径和绕行点
        if path:
            x = [p[1] for p in path]
            y = [p[0] for p in path]
            plt.plot(x, y, 'r.-', linewidth=2, markersize=10)
            
            # 标记绕行点（排除起点和终点）
            if len(path) > 2:
                detour_points = path[1:-1]
                dx = [p[1] for p in detour_points]
                dy = [p[0] for p in detour_points]
                plt.scatter(dx, dy, c='orange', s=150, 
                           marker='o', edgecolor='black',
                           label='绕行点', zorder=3)

        # 标记起点终点
        if path:
            start = path[0]
            end = path[-1]
            plt.scatter(start[1], start[0], c='g', s=100, marker='s', label='起点')
            plt.scatter(end[1], end[0], c='b', s=100, marker='*', label='终点')
        
        # 添加碰撞点标记
        if path:
            for i in range(len(path)-1):
                collision = self._find_first_collision(path[i], path[i+1])
                if collision:
                    plt.scatter(collision[1], collision[0], c='purple', 
                               marker='x', s=200, label='碰撞点')
        
        # 避免重复图例
        handles, labels = plt.gca().get_legend_handles_labels()
        by_label = dict(zip(labels, handles))
        plt.legend(by_label.values(), by_label.keys())
        
        plt.title(title)
        plt.legend()  # 显示图例
        plt.xticks(range(self.cols))
        plt.yticks(range(self.rows))
        plt.grid(True, color='lightgray', linestyle='--')
        plt.show()
        
if __name__ == "__main__":
    # 创建多层障碍地图
    grid = [
        [0,0,0,0,0,0,0],
        [0,1,1,0,1,1,0],
        [0,0,0,0,0,0,0],
        [0,1,0,1,0,1,0],
        [0,0,0,0,0,0,0]
    ]

    # 初始化路径规划器
    finder = SimplePathFinder(grid)

    # 定义起点和终点
    start = (0, 0)
    end = (4, 6)

    # 寻找路径
    path = finder.find_path(start, end)

    # 打印路径
    print(f"路径: {path}")

    # 可视化路径
    finder.plot_path(path, title='路径可视化')
    plt.show()
