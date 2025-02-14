class Node {
    constructor(x, y, parent = null) {
        this.x = x;
        this.y = y;
        this.parent = parent;
        this.g = 0;
        this.h = 0;
        this.f = 0;
    }

    compareTo(other) {
        return this.f - other.f;
    }

    equals(other) {
        return this.x === other.x && this.y === other.y;
    }
}

class PriorityQueue {
    constructor() {
        this.elements = [];
    }

    enqueue(element, priority) {
        this.elements.push({ element, priority });
        this.elements.sort((a, b) => a.priority - b.priority);
    }

    dequeue() {
        return this.elements.shift()?.element;
    }

    isEmpty() {
        return this.elements.length === 0;
    }
}

class JPS {
    constructor(grid) {
        this.grid = grid;
        this.height = grid.length;     // 行数
        this.width = grid[0].length;   // 列数
        this.movements = [
            [0, 1], [1, 0], [0, -1], [-1, 0],  // 直线移动
            [1, 1], [1, -1], [-1, 1], [-1, -1]  // 对角线移动
        ];
        this.debug = true;  // 启用调试输出
        this.nodesExplored = 0;
        this.jumpCalls = 0;
        this.executionTime = 0;
        this.jumpCache = {};
        this.pathLength = 0;
        this.avgJumpDistance = 0;
    }

    log(...args) {
        if (this.debug) {
            console.log(...args);
        }
    }

    heuristic(node, goal) {
        const dx = Math.abs(node.x - goal.x);
        const dy = Math.abs(node.y - goal.y);
        return Math.sqrt(dx * dx + dy * dy) * 10;
    }

    findPath(start, end) {
        this.log('开始寻路...');
        this.log(`起点: (${start[0]}, ${start[1]})`);
        this.log(`终点: (${end[0]}, ${end[1]})`);

        const startTime = performance.now();
        this.nodesExplored = 0;
        this.jumpCalls = 0;

        const openList = new PriorityQueue();
        const startNode = new Node(start[0], start[1]);
        const endNode = new Node(end[0], end[1]);
        const closedSet = new Set();
        
        startNode.g = 0;
        startNode.h = this.heuristic(startNode, endNode);
        startNode.f = startNode.g + startNode.h;
        openList.enqueue(startNode, startNode.f);

        let iterations = 0;
        while (!openList.isEmpty()) {
            iterations++;
            const current = openList.dequeue();
            this.log(`\n迭代 ${iterations}:`);
            this.log(`当前节点: (${current.x}, ${current.y}), f=${current.f.toFixed(2)}, g=${current.g.toFixed(2)}, h=${current.h.toFixed(2)}`);

            if (current.x === endNode.x && current.y === endNode.y) {
                this.log('找到目标节点!');
                const path = [];
                let node = current;
                while (node) {
                    path.unshift([node.x, node.y]);
                    node = node.parent;
                }
                this.executionTime = performance.now() - startTime;
                this.pathLength = path.length;
                return path;
            }

            const key = `${current.x},${current.y}`;
            if (closedSet.has(key)) {
                this.log(`节点 (${current.x}, ${current.y}) 已在关闭列表中`);
                continue;
            }
            closedSet.add(key);

            const neighbors = this.findNeighbors(current);
            this.log(`找到 ${neighbors.length} 个邻居节点`);

            for (const neighbor of neighbors) {
                this.log(`\n检查邻居节点 (${neighbor.x}, ${neighbor.y})`);
                const dx = neighbor.x - current.x;
                const dy = neighbor.y - current.y;
                this.log(`移动方向: [${dx}, ${dy}]`);

                const jumpPoint = this.jump(neighbor.x, neighbor.y, dx, dy, endNode);
                if (jumpPoint) {
                    const [jx, jy] = jumpPoint;
                    const jumpKey = `${jx},${jy}`;
                    
                    if (!closedSet.has(jumpKey)) {
                        const jumpNode = new Node(jx, jy, current);
                        const d = Math.sqrt(
                            Math.pow(jumpNode.x - current.x, 2) + 
                            Math.pow(jumpNode.y - current.y, 2)
                        );
                        jumpNode.g = current.g + d * 10;
                        jumpNode.h = this.heuristic(jumpNode, endNode);
                        jumpNode.f = jumpNode.g + jumpNode.h;
                        this.log(`添加跳点: (${jx}, ${jy}), f=${jumpNode.f.toFixed(2)}, g=${jumpNode.g.toFixed(2)}, h=${jumpNode.h.toFixed(2)}`);
                        openList.enqueue(jumpNode, jumpNode.f);
                    } else {
                        this.log(`跳点 (${jx}, ${jy}) 已在关闭列表中`);
                    }
                } else {
                    this.log(`未找到从 (${neighbor.x}, ${neighbor.y}) 开始的跳点`);
                }
            }

            this.log(`\n当前状态:`);
            this.log(`- 开放列表大小: ${openList.elements.length}`);
            this.log(`- 关闭列表大小: ${closedSet.size}`);
            this.log(`- 已探索节点数: ${iterations}`);
        }

        this.log('未找到路径');
        this.executionTime = performance.now() - startTime;
        return null;
    }

    findNeighbors(node) {
        const neighbors = [];
        const parent = node.parent;

        if (!parent) {
            this.log(`\n节点 (${node.x}, ${node.y}) 没有父节点，检查所有方向`);
            for (const [dx, dy] of this.movements) {
                const x = node.x + dx;
                const y = node.y + dy;
                if (this.isWalkable(x, y)) {
                    this.log(`添加邻居: (${x}, ${y})`);
                    neighbors.push(new Node(x, y, node));
                }
            }
            return neighbors;
        }

        this.log(`\n节点 (${node.x}, ${node.y}) 的父节点是 (${parent.x}, ${parent.y})`);
        const dx = Math.sign(node.x - parent.x);
        const dy = Math.sign(node.y - parent.y);
        this.log(`移动方向: [${dx}, ${dy}]`);

        // 如果是对角线移动
        if (dx !== 0 && dy !== 0) {
            // 继续对角线移动
            if (this.isWalkable(node.x + dx, node.y + dy)) {
                this.log(`可以对角线移动到: (${node.x + dx}, ${node.y + dy})`);
                neighbors.push(new Node(node.x + dx, node.y + dy, node));
            }
            // 检查水平和垂直方向
            if (this.isWalkable(node.x + dx, node.y)) {
                this.log(`可以水平移动到: (${node.x + dx}, ${node.y})`);
                neighbors.push(new Node(node.x + dx, node.y, node));
            }
            if (this.isWalkable(node.x, node.y + dy)) {
                this.log(`可以垂直移动到: (${node.x}, ${node.y + dy})`);
                neighbors.push(new Node(node.x, node.y + dy, node));
            }
        } else {
            // 如果是水平移动
            if (dx !== 0) {
                if (this.isWalkable(node.x + dx, node.y)) {
                    this.log(`可以水平移动到: (${node.x + dx}, ${node.y})`);
                    neighbors.push(new Node(node.x + dx, node.y, node));
                }
                // 检查强制邻居
                if (!this.isWalkable(node.x, node.y + 1)) {
                    if (this.isWalkable(node.x + dx, node.y + 1)) {
                        this.log(`发现强制邻居: (${node.x + dx}, ${node.y + 1})`);
                        neighbors.push(new Node(node.x + dx, node.y + 1, node));
                    }
                }
                if (!this.isWalkable(node.x, node.y - 1)) {
                    if (this.isWalkable(node.x + dx, node.y - 1)) {
                        this.log(`发现强制邻居: (${node.x + dx}, ${node.y - 1})`);
                        neighbors.push(new Node(node.x + dx, node.y - 1, node));
                    }
                }
            }
            // 如果是垂直移动
            else if (dy !== 0) {
                if (this.isWalkable(node.x, node.y + dy)) {
                    this.log(`可以垂直移动到: (${node.x}, ${node.y + dy})`);
                    neighbors.push(new Node(node.x, node.y + dy, node));
                }
                // 检查强制邻居
                if (!this.isWalkable(node.x + 1, node.y)) {
                    if (this.isWalkable(node.x + 1, node.y + dy)) {
                        this.log(`发现强制邻居: (${node.x + 1}, ${node.y + dy})`);
                        neighbors.push(new Node(node.x + 1, node.y + dy, node));
                    }
                }
                if (!this.isWalkable(node.x - 1, node.y)) {
                    if (this.isWalkable(node.x - 1, node.y + dy)) {
                        this.log(`发现强制邻居: (${node.x - 1}, ${node.y + dy})`);
                        neighbors.push(new Node(node.x - 1, node.y + dy, node));
                    }
                }
            }
        }

        return neighbors;
    }

    jump(x, y, dx, dy, endNode) {
        this.log(`\n跳跃检测: 从 (${x}, ${y}) 向 [${dx}, ${dy}] 方向`);
        const nx = x + dx;
        const ny = y + dy;

        if (!this.isWalkable(nx, ny)) {
            this.log(`位置 (${nx}, ${ny}) 不可行走`);
            return null;
        }

        if (nx === endNode.x && ny === endNode.y) {
            this.log(`到达目标点 (${nx}, ${ny})`);
            return [nx, ny];
        }

        // 对角线移动
        if (dx !== 0 && dy !== 0) {
            // 检查水平和垂直方向是否有强制邻居
            if (this.hasForceNeighbor(nx, ny, dx, 0) || 
                this.hasForceNeighbor(nx, ny, 0, dy)) {
                this.log(`在 (${nx}, ${ny}) 发现强制邻居`);
                return [nx, ny];
            }
        } else {
            // 直线移动
            if (dx !== 0) { // 水平移动
                // 检查上下是否有强制邻居
                if ((!this.isWalkable(nx, ny + 1) && this.isWalkable(nx + dx, ny + 1)) ||
                    (!this.isWalkable(nx, ny - 1) && this.isWalkable(nx + dx, ny - 1))) {
                    this.log(`在水平移动时，在 (${nx}, ${ny}) 发现强制邻居`);
                    return [nx, ny];
                }
            } else { // 垂直移动
                // 检查左右是否有强制邻居
                if ((!this.isWalkable(nx + 1, ny) && this.isWalkable(nx + 1, ny + dy)) ||
                    (!this.isWalkable(nx - 1, ny) && this.isWalkable(nx - 1, ny + dy))) {
                    this.log(`在垂直移动时，在 (${nx}, ${ny}) 发现强制邻居`);
                    return [nx, ny];
                }
            }
        }

        // 如果是对角线移动，递归检查水平和垂直方向
        if (dx !== 0 && dy !== 0) {
            if (this.jump(nx, ny, dx, 0, endNode) || this.jump(nx, ny, 0, dy, endNode)) {
                this.log(`在 (${nx}, ${ny}) 发现跳点（通过递归检查）`);
                return [nx, ny];
            }
        }

        // 继续在当前方向跳跃
        return this.jump(nx, ny, dx, dy, endNode);
    }

    hasForceNeighbor(x, y, dx, dy) {
        const reverse = dy !== 0 ? dx : dy;
        const hasForce = (!this.isWalkable(x - reverse, y) && this.isWalkable(x - reverse, y + dy)) ||
                        (!this.isWalkable(x + reverse, y) && this.isWalkable(x + reverse, y + dy));
        
        if (hasForce) {
            this.log(`在 (${x}, ${y}) 检测到强制邻居，方向 [${dx}, ${dy}]`);
        }
        
        return hasForce;
    }

    isWalkable(x, y) {
        const walkable = x >= 0 && x < this.width && y >= 0 && y < this.height && this.grid[y][x] === 0;
        this.log(`检查位置 (${x}, ${y}) 是否可行走: ${walkable}`);
        return walkable;
    }

    hasObstacle(start, end) {
        let [x0, y0] = start;
        let [x1, y1] = end;
        const dx = Math.abs(x1 - x0);
        const dy = Math.abs(y1 - y0);
        const sx = x0 < x1 ? 1 : -1;
        const sy = y0 < y1 ? 1 : -1;
        let err = dx - dy;

        while (true) {
            if (this.grid[y0][x0] === 1) {
                return true;
            }
            if (x0 === x1 && y0 === y1) break;

            const e2 = 2 * err;
            if (e2 > -dy) {
                err -= dy;
                x0 += sx;
            }
            if (e2 < dx) {
                err += dx;
                y0 += sy;
            }
        }
        return false;
    }

    hasForcedNeighbor(x, y, dx, dy) {
        console.log(`\n检查强制邻居 @(${x},${y}) 方向[${dx},${dy}]`);
        
        if (this.isWalkable(x, y)) return false;

        // 调整对角线移动检测逻辑
        if (dx !== 0 && dy !== 0) {
            // 允许单侧阻挡的情况下继续检测
            const nextX = x + dx;
            const nextY = y + dy;
            if (this.isWalkable(nextX, y) && this.isWalkable(x, nextY)) {
                return false; // 仅当两侧都阻挡时返回
            }
        }

        // 修正方向向量处理
        if (dx === 0) { // 垂直移动
            const moveDir = dy > 0 ? 1 : -1;
            return (this.isWalkable(x + 1, y) && !this.isWalkable(x + 1, y + moveDir)) || 
                   (this.isWalkable(x - 1, y) && !this.isWalkable(x - 1, y + moveDir));
        }
        if (dy === 0) { // 水平移动
            const moveDir = dx > 0 ? 1 : -1;
            return (this.isWalkable(x, y + 1) && !this.isWalkable(x + moveDir, y + 1)) || 
                   (this.isWalkable(x, y - 1) && !this.isWalkable(x + moveDir, y - 1));
        }
        return false;
    }
}

export { JPS, Node };

if (process.argv[1].includes('JPS.js')) {
    const testGrid = [
        [0, 0, 0, 0, 0],
        [0, 1, 1, 0, 0],
        [0, 0, 0, 0, 0],
        [0, 1, 1, 1, 0],
        [0, 0, 0, 0, 0]
    ];
    
    console.log('测试网格:');
    testGrid.forEach(row => console.log(row.map(cell => cell === 1 ? '█' : '·').join(' ')));
    console.log('\n起点: [0, 0]');
    console.log('终点: [4, 4]\n');
    
    const jps = new JPS(testGrid);
    const startTime = performance.now();
    const path = jps.findPath([0, 0], [4, 4]);
    const endTime = performance.now();
    
    if (path) {
        console.log('找到路径!');
        console.log('路径坐标:', path);
        
        // 在网格上显示路径
        const visualGrid = testGrid.map(row => [...row]);
        path.forEach(([x, y]) => {
            visualGrid[y][x] = 2;
        });
        
        console.log('\n路径可视化:');
        visualGrid.forEach(row => {
            console.log(row.map(cell => {
                if (cell === 1) return '█';
                if (cell === 2) return '○';
                return '·';
            }).join(' '));
        });
    } else {
        console.log('未找到路径!');
    }
    
    console.log('\n执行时间:', (endTime - startTime).toFixed(2) + 'ms');
}