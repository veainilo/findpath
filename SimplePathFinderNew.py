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
        self.current_path = path
        
        for _ in range(self.max_steps):
            while True:
                # 直接可达则完成
                if self._direct_path_clear(current, end):
                    if current != end:  # 只有当当前点不是终点时才添加终点
                        path.append(end)
                    return path
                
                # 获取当前碰撞点
                collision = self._find_first_collision(current, end)
                if not collision:
                    if current != end:  # 只有当当前点不是终点时才添加终点
                        path.append(end)
                    return path
                
                # 获取并选择绕行点
                detour_points = self._get_safe_detours(collision, current)
                if not detour_points:
                    # 回溯处理
                    if len(path) > 1:
                        current = path[-2]
                        path.pop()
                        continue
                    else:
                        return None
                    
                # 选择最佳绕行点并更新状态
                best_point = self._select_best_detour(detour_points, current, end)
                if best_point is None:
                    return None
                    
                # 避免添加重复的点
                if best_point != current:
                    path.append(best_point)
                    self.visited.add(best_point)
                    current = best_point
                break
                
        return None

    def _direct_path_clear(self, a, b):
        """改进的直线可通行检查"""
        if a == b:
            return True
        line_points = self._bresenham_line(a, b)
        # 检查除起点外的所有点
        return all(self.grid[x][y] == 0 for (x, y) in line_points[1:])
    
    def _find_first_collision(self, start, end):
        """改进的碰撞检测"""
        if start == end:
            return None
        line = self._bresenham_line(start, end)
        # 跳过起点，检查其他所有点
        for point in line[1:]:
            x, y = point
            if not (0 <= x < self.rows and 0 <= y < self.cols):
                return point  # 超出边界也视为碰撞
            if self.grid[x][y] == 1:
                return (x, y)
        return None

    def _bresenham_line(self, start, end):
        """修复后的Bresenham算法"""
        x0, y0 = start
        x1, y1 = end
        points = []
        
        # 处理起点等于终点的情况
        if (x0, y0) == (x1, y1):
            return [(x0, y0)]
        
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        x_step = 1 if x1 > x0 else -1 if x1 < x0 else 0
        y_step = 1 if y1 > y0 else -1 if y1 < y0 else 0
        
        # 处理水平线
        if dy == 0:
            x_range = range(x0, x1 + x_step, x_step) if x_step != 0 else [x0]
            return [(x, y0) for x in x_range]
        
        # 处理垂直线
        if dx == 0:
            y_range = range(y0, y1 + y_step, y_step) if y_step != 0 else [y0]
            return [(x0, y) for y in y_range]
        
        # 一般情况
        x, y = x0, y0
        points.append((x, y))
        
        if dx >= dy:
            err = dx / 2
            while x != x1:
                err -= dy
                if err < 0:
                    y += y_step
                    err += dx
                x += x_step
                points.append((x, y))
        else:
            err = dy / 2
            while y != y1:
                err -= dx
                if err < 0:
                    x += x_step
                    err += dy
                y += y_step
                points.append((x, y))
        
        return points

    def _get_safe_detours(self, collision, from_point):
        """改进的绕行点获取策略"""
        candidates = []
        # 扩展搜索半径
        search_radius = 2
        
        # 在更大范围内搜索可能的绕行点
        for dx in range(-search_radius, search_radius + 1):
            for dy in range(-search_radius, search_radius + 1):
                x, y = collision[0] + dx, collision[1] + dy
                
                # 跳过碰撞点本身
                if dx == 0 and dy == 0:
                    continue
                    
                # 检查点是否有效
                if (0 <= x < self.rows and 0 <= y < self.cols 
                    and self.grid[x][y] == 0 
                    and (x, y) not in self.visited
                    and self._is_safe_point((x, y))):
                    
                    # 确保从当前点到候选点的路径是畅通的
                    if self._direct_path_clear(from_point, (x, y)):
                        # 计算点的评分
                        score = self._evaluate_detour_point((x, y), from_point, self.end)
                        candidates.append(((x, y), score))
        
        # 根据评分排序并返回最佳的几个点
        sorted_candidates = sorted(candidates, key=lambda x: x[1])
        return [point for point, _ in sorted_candidates[:3]]

    def _is_safe_point(self, point):
        """检查一个点是否安全（周围没有太多障碍物）"""
        x, y = point
        obstacle_count = 0
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                nx, ny = x + dx, y + dy
                if (0 <= nx < self.rows and 0 <= ny < self.cols 
                    and self.grid[nx][ny] == 1):
                    obstacle_count += 1
        return obstacle_count <= 4  # 周围障碍物不能太多

    def _evaluate_detour_point(self, point, from_point, end_point):
        """评估绕行点的质量"""
        # 计算与起点和终点的距离
        dist_from_start = self._euclidean_distance(point, from_point)
        dist_to_end = self._euclidean_distance(point, end_point)
        
        # 计算路径平滑度（通过检查转角角度）
        smoothness = self._path_smoothness(from_point, point, end_point)
        
        # 计算点的空旷程度
        clearance = self._point_clearance(point)
        
        # 计算到终点的可见性
        visibility = self._visible_path_length(point, end_point)
        
        # 综合评分（较小的值更好）
        return (dist_from_start + dist_to_end) * 0.3 + smoothness * 1.5 - clearance * 0.8 - visibility * 0.4

    def _euclidean_distance(self, a, b):
        """计算欧几里得距离"""
        return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5

    def _path_smoothness(self, p1, p2, p3):
        """计算路径的平滑度（转角角度）"""
        if p1 == p2 or p2 == p3:
            return 0
        
        v1 = (p2[0] - p1[0], p2[1] - p1[1])
        v2 = (p3[0] - p2[0], p3[1] - p2[1])
        
        # 计算向量点积
        dot_product = v1[0] * v2[0] + v1[1] * v2[1]
        # 计算向量模长
        v1_norm = (v1[0] ** 2 + v1[1] ** 2) ** 0.5
        v2_norm = (v2[0] ** 2 + v2[1] ** 2) ** 0.5
        
        # 避免除以零
        if v1_norm == 0 or v2_norm == 0:
            return 0
            
        # 计算夹角的余弦值
        cos_angle = dot_product / (v1_norm * v2_norm)
        # 限制cos_angle在[-1, 1]范围内
        cos_angle = max(min(cos_angle, 1), -1)
        
        # 返回角度（弧度）
        return abs(np.arccos(cos_angle))

    def _point_clearance(self, point):
        """计算点的空旷程度（周围空地的数量）"""
        x, y = point
        clearance = 0
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                nx, ny = x + dx, y + dy
                if (0 <= nx < self.rows and 0 <= ny < self.cols 
                    and self.grid[nx][ny] == 0):
                    clearance += 1
        return clearance

    def _select_best_detour(self, detours, current, end):
        """改进的绕行点选择策略"""
        if not detours:
            return None
            
        # 选择对路径影响最小的点
        return min(detours, key=lambda p: self._evaluate_detour_point(p, current, end))

    def _verify_full_path(self, path):
        """验证整个路径是否畅通"""
        for i in range(len(path) - 1):
            if not self._direct_path_clear(path[i], path[i+1]):
                return False
        return True

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
        plt.figure(figsize=(10, 10))
        
        # 显示网格和障碍物
        plt.imshow(self.grid, cmap='Greys', origin='lower',
                  extent=(-0.5, self.cols-0.5, -0.5, self.rows-0.5),
                  alpha=0.3, vmin=0, vmax=1)
        
        # 绘制路径和绕行点
        if path:
            # 直接连接路径点（红线显示主路径）
            x_path = [p[1] for p in path]
            y_path = [p[0] for p in path]
            plt.plot(x_path, y_path, 'r-', linewidth=2, alpha=0.8, label='主路径')
            
            # 生成并显示所有中间路径点
            all_path_points = []
            for i in range(len(path)-1):
                start = path[i]
                end = path[i+1]
                line_points = self._bresenham_line(start, end)
                all_path_points.extend(line_points)
            
            # 绘制所有路径点和连接线（蓝色）
            x_points = [p[1] for p in all_path_points]
            y_points = [p[0] for p in all_path_points]
            plt.plot(x_points, y_points, 'b-', linewidth=1, alpha=0.5, label='实际路径')
            plt.plot(x_points, y_points, 'bo', markersize=4, alpha=0.4)
            
            # 标记绕行点（排除起点和终点）
            if len(path) > 2:
                detour_points = path[1:-1]
                dx = [p[1] for p in detour_points]
                dy = [p[0] for p in detour_points]
                plt.scatter(dx, dy, c='orange', s=150, 
                           marker='o', edgecolor='black',
                           label='绕行点', zorder=3)

            # 标记起点终点
            start = path[0]
            end = path[-1]
            plt.scatter(start[1], start[0], c='g', s=150, marker='s', label='起点', zorder=4)
            plt.scatter(end[1], end[0], c='r', s=150, marker='*', label='终点', zorder=4)
        
        # 避免重复图例
        handles, labels = plt.gca().get_legend_handles_labels()
        by_label = dict(zip(labels, handles))
        plt.legend(by_label.values(), by_label.keys(), loc='upper right')
        
        plt.title(title)
        plt.grid(True, color='lightgray', linestyle='--')
        plt.show()
        
    def print_path(self, path, title='路径可视化'):
        """使用字符可视化路径"""
        if not path:
            print("未找到路径")
            return
        
        # 创建显示用的网格
        display_grid = []
        for row in self.grid:
            # 转换0和1为相应字符
            display_row = ['■' if cell == 1 else '□' for cell in row]
            display_grid.append(display_row)
        
        # 标记路径
        if path:
            for i in range(len(path)-1):
                start = path[i]
                end = path[i+1]
                # 获取路径点
                line_points = self._bresenham_line(start, end)
                # 标记路径点（除了起点和终点）
                for point in line_points[1:-1]:
                    x, y = point
                    if display_grid[x][y] == '□':  # 只在空地上标记路径
                        display_grid[x][y] = '·'
        
        # 标记起点、终点和绕行点
        for i, point in enumerate(path):
            x, y = point
            if i == 0:
                display_grid[x][y] = 'S'  # 起点
            elif i == len(path) - 1:
                display_grid[x][y] = 'E'  # 终点
            else:
                display_grid[x][y] = '○'  # 绕行点
        
        # 打印标题
        print(f"\n{title}")
        print("─" * (self.cols * 2 + 2))
        
        # 打印网格
        for row in display_grid:
            print("|" + "".join(f"{cell}" for cell in row) + "|")
        
        print("─" * (self.cols * 2 + 2))
        
        # 打印图例
        print("\n图例:")
        print("S: 起点")
        print("E: 终点")
        print("○: 绕行点")
        print("·: 路径")
        print("■: 障碍")
        print("□: 空地")
        
if __name__ == "__main__":
    # 创建更复杂的障碍地图
    grid = [
        [0,0,0,0,0,0,0,0,0,0],
        [0,1,1,0,1,1,0,1,0,0],
        [0,0,0,0,0,0,0,1,0,0],
        [0,1,1,1,0,1,0,0,0,0],
        [0,0,0,0,0,1,1,1,0,0],
        [0,1,1,1,0,0,0,0,0,0],
        [0,0,0,0,0,1,1,1,0,0],
        [0,0,1,1,0,0,0,0,0,0]
    ]

    # 初始化路径规划器
    finder = SimplePathFinder(grid)

    # 定义起点和终点
    start = (0, 0)
    end = (7, 9)

    # 寻找路径
    path = finder.find_path(start, end)

    # 打印路径
    print(f"路径: {path}")

    # 可视化路径
    finder.print_path(path, title='路径可视化')
    finder.plot_path(path, title='路径可视化（图形界面）')
