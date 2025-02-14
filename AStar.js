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

export { AStar, Node };

if (process.argv[1].includes('AStar.js')) { // 直接执行时运行
    const testGrid = [
        [0, 0, 0, 0],
        [0, 1, 1, 0],
        [0, 0, 0, 0],
        [0, 1, 0, 0]
    ];
    
    const planner = new AStar(testGrid);
    const result = planner.findPath([0, 0], [3, 3]);
    
    console.log('路径结果:', result);
    console.log('统计信息:', {
        nodes: planner.nodesExplored,
        time: planner.executionTime.toFixed(2) + 'ms',
        length: planner.pathLength
    });
}