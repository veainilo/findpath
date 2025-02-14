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
        this.elements.push({element, priority});
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
        const cacheKey = `${x},${y},${dx},${dy}`;
        if (this.jumpCache[cacheKey] !== undefined) {
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

        // 检查强制邻居
        if (this.hasForcedNeighbor(nx, ny, dx, dy)) {
            this.jumpCache[cacheKey] = [nx, ny];
            return [nx, ny];
        }

        // 对角线移动时需要检查水平和垂直方向
        if (dx !== 0 && dy !== 0) {
            // 修改为同时检查三个方向
            const horizontal = this.jump(nx, ny, dx, 0, goal);
            const vertical = this.jump(nx, ny, 0, dy, goal);
            if (horizontal !== null || vertical !== null) {
                this.jumpCache[cacheKey] = [nx, ny];
                return [nx, ny];
            }
            
            // 增强对角线强制邻居检测
            const dirs = [
                [dx, 0],  // 原水平方向
                [0, dy],  // 原垂直方向
                [dx, dy]  // 原对角线方向
            ];
            
            for (const [ndx, ndy] of dirs) {
                if (this.hasForcedNeighbor(nx, ny, ndx, ndy)) {
                    this.jumpCache[cacheKey] = [nx, ny];
                    return [nx, ny];
                }
            }
        }

        // 继续沿原方向跳跃
        const nextPoint = this.jump(nx, ny, dx, dy, goal);
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
        startNode.h = this.heuristic(startNode, endNode);
        startNode.f = startNode.h;
        openList.enqueue(startNode, startNode.f);

        const closedDict = {};
        const gValues = { [`${start[0]},${start[1]}`]: 0 };
        const maxIterations = Math.min(this.height * this.width / 2, 10000);
        let iterations = 0;

        while (!openList.isEmpty() && iterations < maxIterations) {
            iterations += 1;
            const current = openList.dequeue();

            // console.log(`Evaluating node: (${current.x}, ${current.y})`);

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
                // 处理直线移动
                if (dx === 0 || dy === 0) {
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
                // 处理对角线移动
                else {
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

            // 修改直接邻居检测部分
            if (Math.abs(current.x - endNode.x) <= 1 && 
                Math.abs(current.y - endNode.y) <= 1) {
                const finalNode = new Node(endNode.x, endNode.y, current);
                
                // 添加路径重建代码
                const path = [];
                let temp = finalNode;
                while (temp) {
                    path.push([temp.x, temp.y]);
                    temp = temp.parent;
                }
                this.executionTime = performance.now() - startTime;
                this.pathLength = path.length;
                return path;
            }
        }

        this.executionTime = performance.now() - startTime;
        return null;
    }

    hasObstacle(start, end) {
        const [x0, y0] = start;
        const [x1, y1] = end;
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
        // 简化并优化强制邻居判断
        if (dx === 0) { // 垂直
            return (this.isBlocked(x+1, y-dy) && !this.isBlocked(x+1, y)) || 
                   (this.isBlocked(x-1, y-dy) && !this.isBlocked(x-1, y));
        } 
        if (dy === 0) { // 水平
            return (this.isBlocked(x-dx, y+1) && !this.isBlocked(x, y+1)) || 
                   (this.isBlocked(x-dx, y-1) && !this.isBlocked(x, y-1));
        }
        // 对角线
        return (this.isBlocked(x-dx, y) || this.isBlocked(x, y-dy)) || 
               (this.isBlocked(x+dx, y-dy) && !this.isBlocked(x+dx, y)) ||
               (this.isBlocked(x-dx, y+dy) && !this.isBlocked(x, y+dy));
    }

    isBlocked(x, y) {
        return x < 0 || x >= this.height || 
               y < 0 || y >= this.width || 
               this.grid[x][y] === 1;
    }
}

export { JPS, Node };