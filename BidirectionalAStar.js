class Node {
    constructor(x, y, parent = null, isForward = true) {
        this.x = x;
        this.y = y;
        this.parent = parent;
        this.g = 0;
        this.h = 0;
        this.f = 0;
        this.isForward = isForward; // Mark search direction
    }

    compareTo(other) {
        return this.f - other.f;
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

export { BidirectionalAStar, Node };