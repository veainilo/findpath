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

    // 其他必要方法（isPathClear, calculateDistance等）保持与之前实现一致
    // 可视化方法也保持独立
}

export default RRTPathPlanner; 