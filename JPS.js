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
        return this.elements.shift().element;
    }

    isEmpty() {
        return this.elements.length === 0;
    }
}

class JPS {
    constructor(grid) {
        this.grid = grid;
        this.height = grid.length;
        this.width = grid[0].length;
        this.movements = [[0, 1], [1, 0], [0, -1], [-1, 0], [1, 1], [1, -1], [-1, 1], [-1, -1]];
        this.nodesExplored = 0;
        this.jumpCalls = 0;
        this.executionTime = 0;
        this.jumpCache = {};
        this.pathLength = 0;
        this.avgJumpDistance = 0;
    }

    heuristic(node, goal) {
        const dx = Math.abs(node.x - goal.x);
        const dy = Math.abs(node.y - goal.y);
        return 10 * (dx + dy) - 6 * Math.min(dx, dy);
    }

    jump(x, y, dx, dy, goal) {
        console.log(`\n=== 跳跃检测开始 (${x},${y}) 方向[${dx},${dy}] ===`);
        
        const cacheKey = `${x},${y},${dx},${dy}`;
        if (this.jumpCache[cacheKey] !== undefined) {
            console.log(`缓存命中: ${this.jumpCache[cacheKey]}`);
            return this.jumpCache[cacheKey];
        }
        this.jumpCalls += 1;

        let nx = x + dx;
        let ny = y + dy;

        if (this.isBlocked(x, y)) {
            this.jumpCache[cacheKey] = null;
            return null;
        }

        if (x === goal.x && y === goal.y) {
            this.jumpCache[cacheKey] = [x, y];
            return [x, y];
        }

        // 调整递归终止条件
        if (this.isBlocked(nx, ny)) {
            this.jumpCache[cacheKey] = null;
            return null;
        }

        // 添加对角线移动的可行路径检测
        if (dx !== 0 && dy !== 0) {
            if (this.isBlocked(x + dx, y) || this.isBlocked(x, y + dy)) {
                // 允许单侧阻挡时继续检测
                const horizontal = this.jump(nx, ny, dx, 0, goal);
                const vertical = this.jump(nx, ny, 0, dy, goal);
                if (horizontal || vertical) {
                    this.jumpCache[cacheKey] = [nx, ny];
                    return [nx, ny];
                }
            }
        }

        // 检查强制邻居
        const hasForced = this.hasForcedNeighbor(nx, ny, dx, dy);
        console.log(`强制邻居检查结果: ${hasForced ? '存在' : '无'}`);
        
        if (hasForced) {
            this.jumpCache[cacheKey] = [nx, ny];
            return [nx, ny];
        }

        // 对角线移动时需要检查水平和垂直方向
        if (dx !== 0 && dy !== 0) {
            // 恢复正确的节点检测
            const horizontal = this.jump(nx, ny, dx, 0, goal);
            const vertical = this.jump(nx, ny, 0, dy, goal);
            if (horizontal !== null || vertical !== null) {
                this.jumpCache[cacheKey] = [nx, ny];
                return [nx, ny];
            }

            // 修正强制邻居检测位置
            const dirs = [
                [dx, 0],  // 水平方向
                [0, dy],  // 垂直方向
                [dx, dy]  // 对角线方向
            ];

            for (const [ndx, ndy] of dirs) {
                // 修正为检测下一个节点而非当前节点
                if (this.hasForcedNeighbor(nx, ny, ndx, ndy)) {
                    this.jumpCache[cacheKey] = [nx, ny];
                    return [nx, ny];
                }
            }
        }

        // 继续沿原方向跳跃
        const nextPoint = this.jump(nx, ny, dx, dy, goal);
        console.log(`递归跳跃结果: ${nextPoint ? `(${nextPoint[0]},${nextPoint[1]})` : '无'}`);
        
        this.jumpCache[cacheKey] = nextPoint;
        return nextPoint;
    }

    findPath(start, end) {
        const startTime = performance.now();
        this.nodesExplored = 0;
        this.jumpCalls = 0;

        const openList = new PriorityQueue();
        const startNode = new Node(...start);
        const endNode = new Node(...end);
        startNode.g = 0;
        startNode.h = this.heuristic(startNode, endNode);
        startNode.f = startNode.g + startNode.h;
        openList.enqueue(startNode, startNode.f);

        const closedDict = {};
        const gValues = { [`${start[0]},${start[1]}`]: 0 };
        const maxIterations = Math.min(this.height * this.width / 2, 10000);
        let iterations = 0;

        while (!openList.isEmpty() && iterations < maxIterations) {
            console.log(`[迭代 ${iterations}] 开放列表大小: ${openList.elements.length}, 关闭列表大小: ${Object.keys(closedDict).length}`);
            
            const current = openList.dequeue();
            console.log(`当前节点: (${current.x},${current.y}) g=${current.g}, h=${current.h}, f=${current.f}`);

            if (current.x === endNode.x && current.y === endNode.y) {
                const path = [];
                let temp = current;
                while (temp) {
                    path.push([temp.x, temp.y]);
                    temp = temp.parent;
                }
                this.executionTime = performance.now() - startTime;
                this.pathLength = path.length;
                return path; // Directly returning the path without smoothing
            }

            if (closedDict[`${current.x},${current.y}`]) continue;

            this.nodesExplored += 1;
            closedDict[`${current.x},${current.y}`] = current;

            for (const [dx, dy] of this.movements) {
                console.log(`检查方向: [${dx},${dy}]`);
                
                // 处理直线移动
                if (dx === 0 || dy === 0) {
                    const jumpPoint = this.jump(current.x, current.y, dx, dy, endNode);
                    console.log(`直线跳跃结果: ${jumpPoint ? `(${jumpPoint[0]},${jumpPoint[1]})` : '无'}`);
                    
                    if (jumpPoint) {
                        const nx = jumpPoint[0];
                        const ny = jumpPoint[1];

                        // console.log(`Jump Point found: (${nx}, ${ny})`);

                        if (this.isBlocked(nx, ny)) {
                            // console.log(`Jump Point (${nx}, ${ny}) is blocked.`);
                            continue;
                        }

                        if (!closedDict[`${nx},${ny}`]) {
                            const dxTotal = nx - current.x;
                            const dyTotal = ny - current.y;

                            const steps = Math.abs(dxTotal) + Math.abs(dyTotal);
                            const isDiagonal = dxTotal !== 0 && dyTotal !== 0;
                            const cost = isDiagonal ? 14 * Math.min(Math.abs(dxTotal), Math.abs(dyTotal)) + 10 * Math.abs(Math.abs(dxTotal) - Math.abs(dyTotal))
                                : 10 * steps;

                            const newG = current.g + cost;
                            if (newG < (gValues[`${nx},${ny}`] || Infinity)) {
                                const newNode = new Node(nx, ny, current);
                                newNode.g = newG;
                                newNode.h = this.heuristic(newNode, endNode);
                                newNode.f = newNode.g + newNode.h;
                                gValues[`${nx},${ny}`] = newG;
                                openList.enqueue(newNode, newNode.f);
                            }
                        }
                    }
                }
                // 处理对角线移动
                else {
                    console.log('处理对角线移动...');
                    // 先检查水平方向
                    const horizontalJump = this.jump(current.x, current.y, dx, 0, endNode);
                    // 再检查垂直方向
                    const verticalJump = this.jump(current.x, current.y, 0, dy, endNode);

                    if (horizontalJump || verticalJump) {
                        const jumpPoint = this.jump(current.x, current.y, dx, dy, endNode);
                        if (jumpPoint) {
                            const nx = jumpPoint[0];
                            const ny = jumpPoint[1];

                            // console.log(`Jump Point found: (${nx}, ${ny})`);

                            if (this.isBlocked(nx, ny)) {
                                // console.log(`Jump Point (${nx}, ${ny}) is blocked.`);
                                continue;
                            }

                            if (!closedDict[`${nx},${ny}`]) {
                                const dxTotal = nx - current.x;
                                const dyTotal = ny - current.y;

                                const steps = Math.abs(dxTotal) + Math.abs(dyTotal);
                                const isDiagonal = dxTotal !== 0 && dyTotal !== 0;
                                const cost = isDiagonal ? 14 * Math.min(Math.abs(dxTotal), Math.abs(dyTotal)) + 10 * Math.abs(Math.abs(dxTotal) - Math.abs(dyTotal))
                                    : 10 * steps;

                                const newG = current.g + cost;
                                if (newG < (gValues[`${nx},${ny}`] || Infinity)) {
                                    const newNode = new Node(nx, ny, current);
                                    newNode.g = newG;
                                    newNode.h = this.heuristic(newNode, endNode);
                                    newNode.f = newNode.g + newNode.h;
                                    gValues[`${nx},${ny}`] = newG;
                                    openList.enqueue(newNode, newNode.f);
                                }
                            }
                        }
                    }
                }
            }

            // 在终点检测处添加详细日志
            if ((Math.abs(current.x - endNode.x) + Math.abs(current.y - endNode.y)) <= 2) {
                console.log('进入终点检测范围');
                const isReachable = this.hasObstacle([current.x, current.y], [endNode.x, endNode.y]);
                console.log(`终点可达性检查: ${isReachable ? '不可达' : '可达'}`);
                
                if (!isReachable) {
                    console.log('构建最终路径:');
                    let temp = current;
                    const path = [];
                    while (temp) {
                        console.log(`<- (${temp.x},${temp.y})`);
                        path.unshift([temp.x, temp.y]); // 使用unshift确保正确顺序
                        temp = temp.parent;
                    }
                    // 添加终点到路径
                    path.push([endNode.x, endNode.y]);
                    this.pathLength = path.length;
                    return path;
                }
            }
        }

        this.executionTime = performance.now() - startTime;
        return null;
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
            if (this.grid[x0][y0] === 1) {
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
        
        if (this.isBlocked(x, y)) return false;

        // 调整对角线移动检测逻辑
        if (dx !== 0 && dy !== 0) {
            // 允许单侧阻挡的情况下继续检测
            const nextX = x + dx;
            const nextY = y + dy;
            if (this.isBlocked(nextX, y) && this.isBlocked(x, nextY)) {
                return false; // 仅当两侧都阻挡时返回
            }
        }

        // 修正方向向量处理
        if (dx === 0) { // 垂直移动
            const moveDir = dy > 0 ? 1 : -1;
            return (this.isBlocked(x + 1, y) && !this.isBlocked(x + 1, y + moveDir)) || 
                   (this.isBlocked(x - 1, y) && !this.isBlocked(x - 1, y + moveDir));
        }
        if (dy === 0) { // 水平移动
            const moveDir = dx > 0 ? 1 : -1;
            return (this.isBlocked(x, y + 1) && !this.isBlocked(x + moveDir, y + 1)) || 
                   (this.isBlocked(x, y - 1) && !this.isBlocked(x + moveDir, y - 1));
        }
        return false;
    }

    isBlocked(x, y) {
        return x < 0 || x >= this.height ||
            y < 0 || y >= this.width ||
            this.grid[x][y] === 1;
    }
}

export { JPS, Node };