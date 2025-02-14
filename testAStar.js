import { AStar } from './AStar.js';

// 创建测试网格（0=可通过，1=障碍）
const grid = [
    [0, 0, 0, 0],
    [0, 1, 1, 0],
    [0, 0, 0, 0],
    [0, 1, 0, 0]
];

const astar = new AStar(grid);
const path = astar.findPath([0, 0], [3, 3]);

console.log('路径结果:', path);
console.log('探索节点数:', astar.nodesExplored);
console.log('执行时间(ms):', astar.executionTime); 