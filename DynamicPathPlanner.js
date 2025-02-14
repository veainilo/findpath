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
            this.children = [];  // 添加子节点追踪
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
        this.nodesExplored = 0;
        const openSet = new PriorityQueue((a, b) => a.f - b.f);
        const closedSet = new Set();

        const startNode = new DynamicPathPlanner.Node(start);
        startNode.h = this.heuristic(start, goal);
        startNode.f = startNode.h;
        openSet.enqueue(startNode);

        while (!openSet.isEmpty()) {
            const currentNode = openSet.dequeue();
            this.nodesExplored++;

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

                    node.children.push(neighborNode);  // 维护父子关系

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

        // 再绘制路径线
        if (this.pathLine.length > 0) {
            this.ctx.strokeStyle = '#00f'; // 确保颜色设置正确
            this.ctx.lineWidth = 3;        // 确保线宽足够
            this.ctx.beginPath();
            
            // 起始点坐标转换
            const startX = this.pathLine[0][0] * cellSize + cellSize/2;
            const startY = this.pathLine[0][1] * cellSize + cellSize/2;
            this.ctx.moveTo(startX, startY);

            // 绘制路径线段
            for (let i = 1; i < this.pathLine.length; i++) {
                const x = this.pathLine[i][0] * cellSize + cellSize/2;
                const y = this.pathLine[i][1] * cellSize + cellSize/2;
                this.ctx.lineTo(x, y);
            }
            this.ctx.stroke(); // 确保调用stroke绘制线条

            // 最后绘制拐点标记（黄色点）
            for (let i = 1; i < this.pathLine.length - 1; i++) {
                if (this._isCorner(this.pathLine[i-1], this.pathLine[i], this.pathLine[i+1])) {
                    const [x, y] = this.pathLine[i];
                    this.ctx.fillStyle = '#ff0';
                    this.ctx.beginPath();
                    this.ctx.arc(
                        x * cellSize + cellSize/2,
                        y * cellSize + cellSize/2,
                        cellSize/5, 0, Math.PI * 2
                    );
                    this.ctx.fill();
                }
            }
        }

        // 最后绘制机器人标记
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
        const startTime = performance.now();
        const path = await this.findPath(this.robotMarker, this.goal);
        const endTime = performance.now();
        
        console.log(`路径规划耗时: ${(endTime - startTime).toFixed(2)}ms`);
        console.log(`探索节点数: ${this.nodesExplored}`);
        console.log('规划路径:', path);

        if (path && path.length > 0) {
            this.pathLine = path;
            // 更新机器人位置到下一个路径点
            if (path.length > 1) {
                this.robotMarker = { 
                    x: path[1][0], 
                    y: path[1][1] 
                };
            }
        } else {
            console.warn('路径规划失败');
            this.pathLine = [];
        }
    }

    stopAnimation() {
        this.running = false;
        if (this.animationFrame) {
            cancelAnimationFrame(this.animationFrame);
        }
    }

    async findFirstCorner(start, goal) {
        this.nodesExplored = 0;
        const openSet = new PriorityQueue((a, b) => a.f - b.f);
        const closedSet = new Set();

        const startNode = new DynamicPathPlanner.Node(start);
        startNode.h = this.heuristic(start, goal);
        startNode.f = startNode.h;
        openSet.enqueue(startNode);

        let firstCorner = null;

        while (!openSet.isEmpty()) {
            const currentNode = openSet.dequeue();
            this.nodesExplored++;

            // 找到目标点直接返回完整路径
            if (currentNode.pos.x === goal.x && currentNode.pos.y === goal.y) {
                return this._constructFullPath(currentNode);
            }

            // 检测拐点（当前节点有父节点且父节点有父节点）
            if (currentNode.parent && currentNode.parent.parent) {
                const prevDir = {
                    x: currentNode.parent.pos.x - currentNode.parent.parent.pos.x,
                    y: currentNode.parent.pos.y - currentNode.parent.parent.pos.y
                };
                const currentDir = {
                    x: currentNode.pos.x - currentNode.parent.pos.x,
                    y: currentNode.pos.y - currentNode.parent.pos.y
                };

                // 发现方向变化时记录第一个拐点
                if (prevDir.x !== currentDir.x || prevDir.y !== currentDir.y) {
                    firstCorner = currentNode;
                    break;
                }
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

        // 构造返回结果：包含已走路径和剩余方向
        return {
            partialPath: this._constructPartialPath(firstCorner || openSet.dequeue()),
            remainingDirection: firstCorner ? this._getRemainingDirection(firstCorner, goal) : null
        };
    }

    _constructFullPath(endNode) {
        const path = [];
        let current = endNode;
        while (current) {
            path.push([current.pos.x, current.pos.y]);
            current = current.parent;
        }
        return {
            partialPath: path.reverse(),
            remainingDirection: null
        };
    }

    _constructPartialPath(node) {
        const path = [];
        let current = node;
        while (current) {
            path.push([current.pos.x, current.pos.y]);
            current = current.parent;
        }
        return path.reverse();
    }

    _getRemainingDirection(node, goal) {
        // 计算从拐点到目标的大致方向
        const dx = goal.x - node.pos.x;
        const dy = goal.y - node.pos.y;
        return {
            x: dx !== 0 ? dx / Math.abs(dx) : 0,
            y: dy !== 0 ? dy / Math.abs(dy) : 0
        };
    }

    async findPathWithCorners(start, goal, maxCorners = 1) {
        this.nodesExplored = 0;
        const openSet = new PriorityQueue((a, b) => a.f - b.f);
        const closedSet = new Set();
        const cornerNodes = [];

        const startNode = new DynamicPathPlanner.Node(start);
        startNode.h = this.heuristic(start, goal);
        startNode.f = startNode.h;
        openSet.enqueue(startNode);

        let lastDirection = null;
        let current = startNode;

        while (!openSet.isEmpty()) {
            current = openSet.dequeue();
            this.nodesExplored++;

            if (current.pos.x === goal.x && current.pos.y === goal.y) {
                break;
            }

            if (current.parent) {
                const newDirection = {
                    x: current.pos.x - current.parent.pos.x,
                    y: current.pos.y - current.parent.pos.y
                };

                if (lastDirection &&
                    (newDirection.x !== lastDirection.x ||
                        newDirection.y !== lastDirection.y)) {
                    cornerNodes.push(current);
                    if (cornerNodes.length >= maxCorners) {
                        break;
                    }
                }
                lastDirection = newDirection;
            }

            const posKey = `${current.pos.x},${current.pos.y}`;
            if (closedSet.has(posKey)) continue;
            closedSet.add(posKey);

            const neighbors = this.getNeighbors(current, goal);
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

        return this._constructMultiCornerPath(cornerNodes, current, goal, maxCorners);
    }

    _constructMultiCornerPath(corners, endNode, goal, maxCorners) {
        const fullPath = [];
        let current = endNode;

        // 构建完整路径
        while (current) {
            fullPath.unshift([current.pos.x, current.pos.y]);
            current = current.parent;
        }

        // 如果找到足够拐角
        if (corners.length >= maxCorners) {
            // 找到最后一个拐点的位置
            const lastCorner = corners[maxCorners - 1];

            // 包含拐点后的10个节点（可根据需要调整）
            let extendCount = 0;
            let extendNode = lastCorner;
            const extendedPath = [];

            while (extendNode && extendCount < 10) {
                extendedPath.push([extendNode.pos.x, extendNode.pos.y]);
                // 优先使用子节点
                if (extendNode.children.length > 0) {
                    extendNode = extendNode.children[0];
                } else {
                    // 没有子节点时尝试继续搜索
                    const neighbors = this.getNeighbors(extendNode, goal);
                    if (neighbors.length > 0) {
                        extendNode = neighbors[0];
                    } else {
                        break;
                    }
                }
                extendCount++;
            }

            return {
                path: fullPath.concat(extendedPath),
                remaining: [],  // 不再保留剩余路径
                cornersFound: corners.length
            };
        }

        // 未找到足够拐角时返回完整路径
        return {
            path: fullPath,
            remaining: [],
            cornersFound: corners.length
        };
    }

    _isCorner(prev, current, next) {
        const dir1 = { x: current[0] - prev[0], y: current[1] - prev[1] };
        const dir2 = { x: next[0] - current[0], y: next[1] - current[1] };
        return dir1.x !== dir2.x || dir1.y !== dir2.y;
    }
}

export { DynamicPathPlanner };