class PriorityQueue {
    constructor(comparator = (a, b) => a - b) {
        this.heap = [];
        this.comparator = comparator;
    }

    enqueue(item) {
        this.heap.push(item);
        this.heapifyUp();
    }

    dequeue() {
        if (this.size() <= 1) return this.heap.pop();
        const top = this.heap[0];
        this.heap[0] = this.heap.pop();
        this.heapifyDown();
        return top;
    }

    // 其他优先队列方法（heapifyUp, heapifyDown, find等）需要完整实现
    // 由于篇幅限制这里省略具体实现，需要补充完整
}

export default PriorityQueue; 