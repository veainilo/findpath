class PriorityQueue {
    constructor(comparator = (a, b) => a - b) {
        this.items = [];
        this.comparator = comparator;
    }

    enqueue(item) {
        this.items.push(item);
        this.items.sort(this.comparator);
    }

    dequeue() {
        return this.items.shift();
    }

    isEmpty() {
        return this.items.length === 0;
    }

    find(predicate) {
        return this.items.find(predicate);
    }

    remove(item) {
        const index = this.items.indexOf(item);
        if (index !== -1) {
            this.items.splice(index, 1);
        }
    }
}

class DynamicPathPlanner {
    constructor(gridSize = 50, dynamicUpdateInterval = 500) {
        this.gridSize = gridSize;
        this.dynamicObstacles = new Set();
        this.staticObstacles = new Set();
        this.pathCache = {};
        this.updateInterval = dynamicUpdateInterval;
        this.running = false;
        this.nodesExplored = 0; // 添加节点计数器

        // Initialize visualization on canvas
        this.canvas = document.createElement('canvas');
        this.canvas.width = gridSize * 20; // 增大画布尺寸
        this.canvas.height = gridSize * 20;
        document.body.appendChild(this.canvas);
        this.ctx = this.canvas.getContext('2d');

        this.pathLine = [];
        this.robotMarker = null;
        this.obstacles = [];

        // 添加动画相关属性
        this.animationFrame = null;
        this.lastFrameTime = 0;
    }

    // Node类作为静态内部类
    static Node = class {
        constructor(pos, parent = null) {
            this.pos = pos;
            this.parent = parent;
            this.g = 0;  // 实际代价
            this.h = 0;  // 启发式估计
            this.f = 0;  // 总代价
            this.turningCost = 0;  // 拐点代价
        }
    }

    heuristic(a, b) {
        // 使用对角线距离替代曼哈顿距离
        const dx = Math.abs(a.x - b.x);
        const dy = Math.abs(a.y - b.y);
        return (dx + dy) + (Math.sqrt(2) - 2) * Math.min(dx, dy);
    }

    async dynamicObstacleUpdate() {
        while (this.running) {
            const newObstacles = new Set();
            for (const obs of this.dynamicObstacles) {
                // 随机移动方向
                const directions = [
                    { dx: 1, dy: 0 }, { dx: -1, dy: 0 },
                    { dx: 0, dy: 1 }, { dx: 0, dy: -1 }
                ];
                const dir = directions[Math.floor(Math.random() * directions.length)];

                const newX = obs.x + dir.dx;
                const newY = obs.y + dir.dy;

                if (newX >= 0 && newX < this.gridSize &&
                    newY >= 0 && newY < this.gridSize) {
                    newObstacles.add({ x: newX, y: newY });
                } else {
                    newObstacles.add(obs); // 保留无法移动的障碍物
                }
            }
            this.dynamicObstacles = newObstacles;
            await this.sleep(this.updateInterval);
        }
    }

    sleep(ms) {
        return new Promise(resolve => setTimeout(resolve, ms));
    }

    async findPath(start, goal) {
        this.nodesExplored = 0; // 重置计数器
        const openSet = new PriorityQueue((a, b) => a.f - b.f);
        const closedSet = new Set();

        const startNode = new DynamicPathPlanner.Node(start);
        startNode.h = this.heuristic(start, goal);
        startNode.f = startNode.h;
        openSet.enqueue(startNode);

        while (!openSet.isEmpty()) {
            const currentNode = openSet.dequeue();
            this.nodesExplored++; // 计数探索的节点

            if (currentNode.pos.x === goal.x && currentNode.pos.y === goal.y) {
                const path = [];
                let current = currentNode;
                while (current) {
                    path.push([current.pos.x, current.pos.y]);
                    current = current.parent;
                }
                return path.reverse();
            }

            const posKey = `${currentNode.pos.x},${currentNode.pos.y}`;
            if (closedSet.has(posKey)) continue;
            closedSet.add(posKey);

            const neighbors = this.getNeighbors(currentNode, goal);
            for (const neighbor of neighbors) {
                const neighborKey = `${neighbor.pos.x},${neighbor.pos.y}`;
                if (closedSet.has(neighborKey)) continue;

                const existing = openSet.find(n =>
                    `${n.pos.x},${n.pos.y}` === neighborKey);

                if (!existing || neighbor.f < existing.f) {
                    if (existing) openSet.remove(existing);
                    openSet.enqueue(neighbor);
                }
            }
        }
        return null;
    }

    getNeighbors(node, goal) {
        const directions = [
            { x: 1, y: 0 }, { x: -1, y: 0 },
            { x: 0, y: 1 }, { x: 0, y: -1 },
            { x: 1, y: 1 }, { x: 1, y: -1 },
            { x: -1, y: 1 }, { x: -1, y: -1 }
        ];

        let currentDirection = null;
        if (node.parent) {
            currentDirection = {
                x: node.pos.x - node.parent.pos.x,
                y: node.pos.y - node.parent.pos.y
            };
        }

        return directions
            .map(d => {
                const newX = node.pos.x + d.x;
                const newY = node.pos.y + d.y;

                if (newX >= 0 && newX < this.gridSize &&
                    newY >= 0 && newY < this.gridSize &&
                    !this.isBlocked(newX, newY)) {

                    // 新增对角移动碰撞检测
                    if (d.x !== 0 && d.y !== 0) { // 斜向移动
                        const side1Blocked = this.isBlocked(node.pos.x + d.x, node.pos.y);
                        const side2Blocked = this.isBlocked(node.pos.x, node.pos.y + d.y);

                        // 如果两边都被阻挡，禁止移动
                        if (side1Blocked && side2Blocked) {
                            return null;
                        }
                    }

                    const neighborPos = { x: newX, y: newY };
                    const neighborNode = new DynamicPathPlanner.Node(neighborPos, node);

                    // 计算实际代价（考虑对角线移动）
                    neighborNode.g = node.g + (d.x !== 0 && d.y !== 0 ? Math.SQRT2 : 1);

                    // 计算启发式估计
                    neighborNode.h = this.heuristic(neighborPos, goal, currentDirection);

                    // 计算拐点代价
                    if (node.parent && currentDirection &&
                        (d.x !== currentDirection.x || d.y !== currentDirection.y)) {
                        neighborNode.turningCost = node.turningCost + 0.2;
                    }

                    // 计算总代价
                    neighborNode.f = neighborNode.g + neighborNode.h + neighborNode.turningCost;

                    return neighborNode;
                }
                return null;
            })
            .filter(n => n);
    }

    isBlocked(x, y) {
        const posKey = `${x},${y}`;
        return this.staticObstacles.has(posKey) ||
            Array.from(this.dynamicObstacles).some(obs => obs.x === x && obs.y === y);
    }

    visualize() {
        const cellSize = this.canvas.width / this.gridSize;
        this.ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);

        // 绘制网格
        this.ctx.strokeStyle = '#ddd';
        this.ctx.lineWidth = 0.5;
        for (let i = 0; i <= this.gridSize; i++) {
            this.ctx.beginPath();
            this.ctx.moveTo(i * cellSize, 0);
            this.ctx.lineTo(i * cellSize, this.canvas.height);
            this.ctx.stroke();
            this.ctx.beginPath();
            this.ctx.moveTo(0, i * cellSize);
            this.ctx.lineTo(this.canvas.width, i * cellSize);
            this.ctx.stroke();
        }

        // 绘制静态障碍物
        this.ctx.fillStyle = '#666';
        for (let obs of this.staticObstacles) {
            const [x, y] = obs.split(',').map(Number);
            this.ctx.fillRect(x * cellSize, y * cellSize, cellSize, cellSize);
        }

        // 绘制动态障碍物
        this.ctx.fillStyle = '#f00';
        for (let obs of this.dynamicObstacles) {
            this.ctx.fillRect(obs.x * cellSize, obs.y * cellSize, cellSize, cellSize);
        }

        // 绘制路径
        if (this.pathLine.length > 0) {
            this.ctx.strokeStyle = '#00f';
            this.ctx.lineWidth = 2;
            this.ctx.beginPath();
            this.ctx.moveTo(this.pathLine[0][0] * cellSize + cellSize / 2,
                this.pathLine[0][1] * cellSize + cellSize / 2);
            for (let i = 1; i < this.pathLine.length; i++) {
                this.ctx.lineTo(this.pathLine[i][0] * cellSize + cellSize / 2,
                    this.pathLine[i][1] * cellSize + cellSize / 2);
            }
            this.ctx.stroke();
        }

        // 绘制机器人位置
        if (this.robotMarker) {
            this.ctx.fillStyle = '#0f0';
            this.ctx.beginPath();
            this.ctx.arc(this.robotMarker.x * cellSize + cellSize / 2,
                this.robotMarker.y * cellSize + cellSize / 2,
                cellSize / 3, 0, Math.PI * 2);
            this.ctx.fill();
        }
    }

    async run_dynamic_demo(start = { x: 1, y: 1 }, goal = { x: 48, y: 48 }) {
        // 生成随机障碍物
        const obstacleDensity = 0.3;
        const totalCells = this.gridSize * this.gridSize;
        const targetObstacles = Math.floor(totalCells * obstacleDensity);

        // 生成随机静态障碍物
        while (this.staticObstacles.size < targetObstacles) {
            const x = Math.floor(Math.random() * this.gridSize);
            const y = Math.floor(Math.random() * this.gridSize);
            // 避开起点和终点区域
            if ((x > 3 || y > 3) && (x < this.gridSize - 4 || y < this.gridSize - 4)) {
                this.staticObstacles.add(`${x},${y}`);
            }
        }

        // 添加动态障碍物集群
        for (let i = 0; i < 5; i++) { // 添加5个动态障碍物集群
            const clusterX = Math.floor(Math.random() * (this.gridSize - 10)) + 5;
            const clusterY = Math.floor(Math.random() * (this.gridSize - 10)) + 5;

            for (let dx = -2; dx <= 2; dx++) {
                for (let dy = -2; dy <= 2; dy++) {
                    if (Math.random() < 0.7) { // 70%概率生成障碍物
                        this.dynamicObstacles.add({
                            x: clusterX + dx,
                            y: clusterY + dy
                        });
                    }
                }
            }
        }

        this.start = start;
        this.goal = goal;
        this.robotMarker = { ...start };

        // 启动动态更新
        this.running = true;
        this.startAnimation();
        this.dynamicObstacleUpdate();
    }

    startAnimation() {
        const animate = (currentTime) => {
            if (this.lastFrameTime === 0) {
                this.lastFrameTime = currentTime;
            }

            const deltaTime = currentTime - this.lastFrameTime;

            if (deltaTime >= this.updateInterval) {
                this.updatePath();
                this.visualize();
                this.lastFrameTime = currentTime;
            }

            if (this.running) {
                this.animationFrame = requestAnimationFrame(animate);
            }
        };

        this.animationFrame = requestAnimationFrame(animate);
    }

    async updatePath() {
        const path = await this.findPath(this.robotMarker, this.goal);
        if (path) {
            this.pathLine = path;
            // 更新机器人位置到路径的下一个点
            if (path.length > 1) {
                this.robotMarker = { x: path[1][0], y: path[1][1] };
            }
        }
    }

    stopAnimation() {
        this.running = false;
        if (this.animationFrame) {
            cancelAnimationFrame(this.animationFrame);
        }
    }
}

export { DynamicPathPlanner };