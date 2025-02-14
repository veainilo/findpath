class RRTPathPlanner {
    constructor(gridSize = 50, options = {}) {
        this.gridSize = gridSize;
        this.stepSize = options.stepSize || 2.5;
        this.maxIterations = options.maxIterations || 5000;
        this.goalBias = options.goalBias || 0.3;
        this.tree = [];
        this.path = [];

        // 障碍物数据接口
        this.staticObstacles = options.staticObstacles || new Set();
        this.dynamicObstacles = options.dynamicObstacles || new Set();

        // 可视化配置
        this.enableVisualization = options.visualization || false;
        if (this.enableVisualization) this.initVisualization();
    }

    static Node = class {
        constructor(position, parent = null) {
            this.position = position;
            this.parent = parent;
            this.children = [];
            this.cost = 0;  // 累计路径代价
        }
    };

    // 核心路径规划方法
    async findPath(start, goal) {
        this.tree = [new RRTPathPlanner.Node(start)];
        this.path = [];

        for (let iter = 0; iter < this.maxIterations; iter++) {
            const randomPoint = this.sampleRandomPoint(goal);
            const nearest = this.findNearestNode(randomPoint);
            const newNode = this.steer(nearest.position, randomPoint);

            if (this.isPathClear(nearest.position, newNode)) {
                const newNodeObj = this.addNode(nearest, newNode);

                if (this.checkGoalReached(newNodeObj.position, goal)) {
                    this.path = this.generatePath(newNodeObj);
                    if (this.enableVisualization) this.visualize(goal);
                    return this.optimizePath(this.path);
                }

                // 动态障碍处理
                if (iter % 100 === 0) {
                    this.pruneCollidedNodes();
                }
            }

            // 可视化更新
            if (this.enableVisualization && iter % 50 === 0) {
                this.visualize(goal);
                await this.delay(5);
            }
        }
        return null;
    }

    // 以下是辅助方法（保持独立性和封装性）
    sampleRandomPoint(goal) {
        return Math.random() < this.goalBias ?
            goal : {
                x: Math.floor(Math.random() * this.gridSize),
                y: Math.floor(Math.random() * this.gridSize)
            };
    }

    findNearestNode(point) {
        let minDist = Infinity;
        return this.tree.reduce((nearest, node) => {
            const dist = this.calculateDistance(node.position, point);
            return dist < minDist ? (minDist = dist, node) : nearest;
        }, this.tree[0]);
    }

    steer(from, to) {
        const dx = to.x - from.x;
        const dy = to.y - from.y;
        const dist = Math.sqrt(dx * dx + dy * dy);
        return {
            x: from.x + (dx / dist) * this.stepSize,
            y: from.y + (dy / dist) * this.stepSize
        };
    }

    addNode(parent, position) {
        const newNode = new RRTPathPlanner.Node(position, parent);
        newNode.cost = parent.cost + this.calculateDistance(parent.position, position);
        parent.children.push(newNode);
        this.tree.push(newNode);
        return newNode;
    }

    generatePath(goalNode) {
        const path = [];
        let current = goalNode;
        while (current) {
            path.unshift([current.position.x, current.position.y]);
            current = current.parent;
        }
        return path;
    }

    // 路径碰撞检测
    isPathClear(start, end) {
        // 使用Bresenham算法进行直线路径检查
        const points = this.getLinePoints(start, end);
        return points.every(point =>
            !this.staticObstacles.has(`${Math.round(point.x)},${Math.round(point.y)}`) &&
            !this.dynamicObstacles.has(`${Math.round(point.x)},${Math.round(point.y)}`)
        );
    }

    // 计算两点间距离
    calculateDistance(a, b) {
        return Math.hypot(a.x - b.x, a.y - b.y);
    }

    // 路径优化（直线化）
    optimizePath(originalPath) {
        const optimized = [originalPath[0]];
        let lastValid = 0;

        while (lastValid < optimized.length - 1) {
            let furthestValid = lastValid + 1;
            for (let i = originalPath.length - 1; i > furthestValid; i--) {
                const start = { x: optimized[lastValid][0], y: optimized[lastValid][1] };
                const end = { x: originalPath[i][0], y: originalPath[i][1] };
                if (this.isPathClear(start, end)) {
                    optimized.push(originalPath[i]);
                    lastValid = optimized.length - 1;
                    break;
                }
            }
        }
        return optimized;
    }

    // 可视化初始化
    initVisualization() {
        this.canvas = document.createElement('canvas');
        this.canvas.width = this.gridSize * 10;
        this.canvas.height = this.gridSize * 10;
        document.body.appendChild(this.canvas);
        this.ctx = this.canvas.getContext('2d');
    }

    // 可视化更新
    visualize(goal) {
        this.ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);

        // 绘制树结构
        this.tree.forEach(node => {
            if (node.parent) {
                this.ctx.beginPath();
                this.ctx.moveTo(node.parent.position.x * 10 + 5, node.parent.position.y * 10 + 5);
                this.ctx.lineTo(node.position.x * 10 + 5, node.position.y * 10 + 5);
                this.ctx.strokeStyle = '#e0e0e0';
                this.ctx.stroke();
            }
        });

        // 绘制路径
        if (this.path.length > 1) {
            this.ctx.beginPath();
            this.ctx.moveTo(this.path[0][0] * 10 + 5, this.path[0][1] * 10 + 5);
            this.path.forEach(p => this.ctx.lineTo(p[0] * 10 + 5, p[1] * 10 + 5));
            this.ctx.strokeStyle = '#ff0000';
            this.ctx.lineWidth = 2;
            this.ctx.stroke();
        }

        // 绘制障碍物
        [...this.staticObstacles].forEach(coord => {
            const [x, y] = coord.split(',').map(Number);
            this.ctx.fillStyle = '#333333';
            this.ctx.fillRect(x * 10, y * 10, 10, 10);
        });

        // 绘制目标点
        this.ctx.fillStyle = '#00ff00';
        this.ctx.beginPath();
        this.ctx.arc(goal.x * 10 + 5, goal.y * 10 + 5, 5, 0, Math.PI * 2);
        this.ctx.fill();
    }

    // 动态障碍处理
    pruneCollidedNodes() {
        this.tree = this.tree.filter(node => {
            const coord = `${Math.round(node.position.x)},${Math.round(node.position.y)}`;
            if (this.dynamicObstacles.has(coord)) {
                if (node.parent) {
                    const index = node.parent.children.indexOf(node);
                    if (index > -1) node.parent.children.splice(index, 1);
                }
                return false;
            }
            return true;
        });
    }

    // Bresenham直线算法
    getLinePoints(start, end) {
        const points = [];
        let dx = Math.abs(end.x - start.x);
        let dy = Math.abs(end.y - start.y);
        const sx = (start.x < end.x) ? 1 : -1;
        const sy = (start.y < end.y) ? 1 : -1;
        let err = dx - dy;

        let current = { ...start };

        while (true) {
            points.push({ x: current.x, y: current.y });
            if (current.x === end.x && current.y === end.y) break;
            const e2 = 2 * err;
            if (e2 > -dy) {
                err -= dy;
                current.x += sx;
            }
            if (e2 < dx) {
                err += dx;
                current.y += sy;
            }
        }
        return points;
    }

    // 异步延迟
    delay(ms) {
        return new Promise(resolve => setTimeout(resolve, ms));
    }
}

export default RRTPathPlanner; 