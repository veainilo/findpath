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
        return 10 * (dx + dy) + (14 - 20) * Math.min(dx, dy);
    }

    jump(x, y, dx, dy, goal) {
        const cacheKey = `${x},${y},${dx},${dy}`;
        if (this.jumpCache[cacheKey]) {
            return this.jumpCache[cacheKey];
        }
        this.jumpCalls += 1;
        
        let nx = x + dx;
        let ny = y + dy;

        if (this.isBlocked(nx, ny)) {
            this.jumpCache[cacheKey] = null;
            return null;
        }

        if (nx === goal.x && ny === goal.y) {
            this.jumpCache[cacheKey] = [nx, ny];
            return [nx, ny];
        }

        // 检查强制邻居
        if (this.hasForcedNeighbor(nx, ny, dx, dy)) {
            this.jumpCache[cacheKey] = [nx, ny];
            return [nx, ny];
        }

        // 对角线移动时需要检查水平和垂直方向
        if (dx !== 0 && dy !== 0) {
            if (this.jump(nx, ny, dx, 0, goal) || this.jump(nx, ny, 0, dy, goal)) {
                this.jumpCache[cacheKey] = [nx, ny];
                return [nx, ny];
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

        const openList = [];
        const startNode = new Node(...start);
        const endNode = new Node(...end);
        startNode.h = this.heuristic(startNode, endNode);
        startNode.f = startNode.h;
        openList.push(startNode);

        const closedDict = {};
        const gValues = { [`${start[0]},${start[1]}`]: 0 };
        const maxIterations = Math.min(this.height * this.width / 2, 10000);
        let iterations = 0;

        while (openList.length > 0 && iterations < maxIterations) {
            iterations += 1;
            const current = openList.sort((a, b) => a.compareTo(b)).shift();

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

                            const cost = (dxTotal !== 0 && dyTotal !== 0) ? (14 * Math.min(Math.abs(dxTotal), Math.abs(dyTotal)) + 10 * Math.abs(dxTotal + dyTotal)) : (10 * (Math.abs(dxTotal) + Math.abs(dyTotal)));

                            const newG = current.g + cost;
                            if (newG < (gValues[`${nx},${ny}`] || Infinity)) {
                                const newNode = new Node(nx, ny, current);
                                newNode.g = newG;
                                newNode.h = this.heuristic(newNode, endNode);
                                newNode.f = newNode.g + newNode.h;
                                gValues[`${nx},${ny}`] = newG;
                                openList.push(newNode);
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

                                const cost = (dxTotal !== 0 && dyTotal !== 0) ? (14 * Math.min(Math.abs(dxTotal), Math.abs(dyTotal)) + 10 * Math.abs(dxTotal + dyTotal)) : (10 * (Math.abs(dxTotal) + Math.abs(dyTotal)));

                                const newG = current.g + cost;
                                if (newG < (gValues[`${nx},${ny}`] || Infinity)) {
                                    const newNode = new Node(nx, ny, current);
                                    newNode.g = newG;
                                    newNode.h = this.heuristic(newNode, endNode);
                                    newNode.f = newNode.g + newNode.h;
                                    gValues[`${nx},${ny}`] = newG;
                                    openList.push(newNode);
                                }
                            }
                        }
                    }
                }
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
        if (dx === 0) { // 垂直移动
            return (this.isBlocked(x + 1, y - dy) && !this.isBlocked(x + 1, y)) || 
                   (this.isBlocked(x - 1, y - dy) && !this.isBlocked(x - 1, y));
        } else if (dy === 0) { // 水平移动
            return (this.isBlocked(x - dx, y + 1) && !this.isBlocked(x, y + 1)) || 
                   (this.isBlocked(x - dx, y - 1) && !this.isBlocked(x, y - 1));
        } else { // 对角线移动
            return (this.isBlocked(x - dx, y) || this.isBlocked(x, y - dy)) || 
                   (this.isBlocked(x + dx, y - dy) && !this.isBlocked(x + dx, y)) || 
                   (this.isBlocked(x - dx, y + dy) && !this.isBlocked(x, y + dy));
        }
    }

    isBlocked(x, y) {
        return !(x >= 0 && x < this.height && y >= 0 && y < this.width) || this.grid[x][y] === 1;
    }
}

export { JPS, Node };