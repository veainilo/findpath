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

class AStar {
    constructor(grid) {
        this.grid = grid;
        this.height = grid.length;
        this.width = grid[0].length;
        this.movements = [[0, 1], [1, 0], [0, -1], [-1, 0], [1, 1], [1, -1], [-1, 1], [-1, -1]];
        this.nodesExplored = 0;
        this.executionTime = 0;
        this.pathLength = 0;
    }

    heuristic(node, goal) {
        const dx = Math.abs(node.x - goal.x);
        const dy = Math.abs(node.y - goal.y);
        return 10 * (dx + dy) + (14 - 20) * Math.min(dx, dy);
    }

    findPath(start, end) {
        const startTime = performance.now();
        this.nodesExplored = 0;

        const openList = [];
        const startNode = new Node(...start);
        const endNode = new Node(...end);
        openList.push(startNode);
        const closedDict = new Map();

        while (openList.length > 0) {
            const current = openList.sort((a, b) => a.compareTo(b)).shift(); 
            this.nodesExplored += 1;

            if (current.x === endNode.x && current.y === endNode.y) {
                const path = [];
                let temp = current;
                while (temp) {
                    path.push([temp.x, temp.y]);
                    temp = temp.parent;
                }
                this.executionTime = performance.now() - startTime;
                this.pathLength = path.length;
                return path.reverse();
            }

            if (closedDict.has(`${current.x},${current.y}`)) continue;
            closedDict.set(`${current.x},${current.y}`, current);

            for (const [dx, dy] of this.movements) {
                const nx = current.x + dx;
                const ny = current.y + dy;
                if (nx >= 0 && nx < this.height && ny >= 0 && ny < this.width) {
                    if ((dx !== 0 && dy !== 0) && 
                        (this.grid[current.x + dx][current.y] === 1 || this.grid[current.x][current.y + dy] === 1)) 
                        continue;

                    if (this.grid[nx][ny] === 1 || closedDict.has(`${nx},${ny}`)) continue;

                    const moveCost = (dx !== 0 && dy !== 0) ? 14 : 10;
                    const newNode = new Node(nx, ny, current);
                    newNode.g = current.g + moveCost;
                    newNode.h = this.heuristic(newNode, endNode);
                    newNode.f = newNode.g + newNode.h;

                    openList.push(newNode);
                }
            }
        }

        this.executionTime = performance.now() - startTime;
        return null;
    }
}

class BidirectionalAStar {
    constructor(grid) {
        this.grid = grid;
        this.height = grid.length;
        this.width = grid[0].length;
        this.movements = [[0, 1], [1, 0], [0, -1], [-1, 0], [1, 1], [1, -1], [-1, 1], [-1, -1]];
        this.nodesExplored = 0;
        this.executionTime = 0;
        this.pathLength = 0;
    }

    heuristic(node, goal) {
        const dx = Math.abs(node.x - goal.x);
        const dy = Math.abs(node.y - goal.y);
        return 10 * (dx + dy) + (14 - 20) * Math.min(dx, dy);
    }

    findPath(start, end) {
        const startTime = performance.now();
        this.nodesExplored = 0;

        const forwardOpen = [];
        const backwardOpen = [];
        const startNode = new Node(...start, true);
        const endNode = new Node(...end, false);
        forwardOpen.push(startNode);
        backwardOpen.push(endNode);
        
        const forwardClosed = new Map();
        const backwardClosed = new Map();

        while (forwardOpen.length > 0 && backwardOpen.length > 0) {
            const currentForward = forwardOpen.sort((a, b) => a.compareTo(b)).shift();
            this.nodesExplored += 1;

            if (backwardClosed.has(`${currentForward.x},${currentForward.y}`)) {
                const meetingNode = backwardClosed.get(`${currentForward.x},${currentForward.y}`);
                const path = this._mergePaths(currentForward, meetingNode);
                this.executionTime = performance.now() - startTime;
                return path;
            }

            if (!forwardClosed.has(`${currentForward.x},${currentForward.y}`)) {
                forwardClosed.set(`${currentForward.x},${currentForward.y}`, currentForward);
                this._expandNode(currentForward, forwardOpen, forwardClosed, backwardClosed, true, endNode);
            }

            const currentBackward = backwardOpen.sort((a, b) => a.compareTo(b)).shift();
            this.nodesExplored += 1;

            if (forwardClosed.has(`${currentBackward.x},${currentBackward.y}`)) {
                const meetingNode = forwardClosed.get(`${currentBackward.x},${currentBackward.y}`);
                const path = this._mergePaths(meetingNode, currentBackward);
                this.executionTime = performance.now() - startTime;
                return path;
            }

            if (!backwardClosed.has(`${currentBackward.x},${currentBackward.y}`)) {
                backwardClosed.set(`${currentBackward.x},${currentBackward.y}`, currentBackward);
                this._expandNode(currentBackward, backwardOpen, backwardClosed, forwardClosed, false, startNode);
            }
        }

        this.executionTime = performance.now() - startTime;
        return null;
    }

    _expandNode(current, openList, closedDict, otherClosed, isForward, goal) {
        for (const [dx, dy] of this.movements) {
            const nx = current.x + dx;
            const ny = current.y + dy;
            if (nx >= 0 && nx < this.height && ny >= 0 && ny < this.width) {
                if ((dx !== 0 && dy !== 0) && (this.grid[current.x + dx][current.y] === 1 || this.grid[current.x][current.y + dy] === 1)) 
                    continue;

                if (this.grid[nx][ny] === 1 || otherClosed.has(`${nx},${ny}`)) continue;

                const moveCost = (dx !== 0 && dy !== 0) ? 14 : 10;
                const newNode = new Node(nx, ny, current, isForward);
                newNode.g = current.g + moveCost;
                newNode.h = this.heuristic(newNode, goal);
                newNode.f = newNode.g + newNode.h;

                openList.push(newNode);
            }
        }
    }

    _mergePaths(forwardNode, backwardNode) {
        const forwardPath = [];
        let current = forwardNode;
        while (current) {
            forwardPath.push([current.x, current.y]);
            current = current.parent;
        }

        const backwardPath = [];
        current = backwardNode;
        while (current) {
            backwardPath.push([current.x, current.y]);
            current = current.parent;
        }

        return forwardPath.reverse().concat(backwardPath.slice(1));
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
        const nx = x + dx;
        const ny = y + dy;

        if (nx < 0 || nx >= this.height || ny < 0 || ny >= this.width || this.grid[nx][ny] === 1) {
            this.jumpCache[cacheKey] = null;
            return null;
        }

        if (nx === goal.x && ny === goal.y) {
            this.jumpCache[cacheKey] = [nx, ny];
            return [nx, ny];
        }

        if (this.hasForcedNeighbor(nx, ny, dx, dy)) {
            this.jumpCache[cacheKey] = [nx, ny];
            return [nx, ny];
        }

        if (dx !== 0 && dy !== 0) {
            if ((this.isBlocked(nx - dx, ny) && !this.isBlocked(nx, ny + dy)) || 
                (this.isBlocked(nx, ny - dy) && !this.isBlocked(nx + dx, ny))) {
                this.jumpCache[cacheKey] = [nx, ny];
                return [nx, ny];
            }
        }

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

            console.log(`Evaluating node: (${current.x}, ${current.y})`);

            if (current.x === endNode.x && current.y === endNode.y) {
                const path = [];
                let temp = current;
                while (temp) {
                    path.push([temp.x, temp.y]);
                    temp = temp.parent;
                }
                this.executionTime = performance.now() - startTime;
                this.pathLength = path.length;
                return this.smoothPath(path.reverse());
            }

            if (closedDict[`${current.x},${current.y}`]) continue;

            this.nodesExplored += 1;
            closedDict[`${current.x},${current.y}`] = current;

            for (const [dx, dy] of this.movements) {
                const jumpPoint = this.jump(current.x, current.y, dx, dy, endNode);
                if (jumpPoint) {
                    const nx = jumpPoint[0];
                    const ny = jumpPoint[1];

                    console.log(`Jump Point found: (${nx}, ${ny})`);

                    if (this.isBlocked(nx, ny)) {
                        console.log(`Jump Point (${nx}, ${ny}) is blocked.`);
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

        this.executionTime = performance.now() - startTime;
        return null;
    }

    // Additional JPS functions remain unchanged...
}

// Closing brackets for all classes
