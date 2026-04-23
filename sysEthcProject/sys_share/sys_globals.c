#include "sys_share_messages.h"

bool IsSystemError = false;

bool Is_Zlimit_Open = false;

bool Is_Zout_limit[7][2] = {false};

Info_Show_In_Err info_show_in_err;

// ================ 队列操作函数 ================

// 初始化队列
void queue_init(SafeCircularQueue *q) {
    q->head = 0;
    q->tail = 0;
    q->size = 0;
    pthread_mutex_init(&q->lock, NULL);
    pthread_cond_init(&q->cond, NULL);
}

// 销毁队列（释放资源）
void queue_destroy(SafeCircularQueue *q) {
    pthread_mutex_destroy(&q->lock);
    pthread_cond_destroy(&q->cond);
}

// 获取当前队列大小
int queue_size(SafeCircularQueue *q) {
    pthread_mutex_lock(&q->lock);
    int size = q->size;
    pthread_mutex_unlock(&q->lock);
    return size;
}

// 入队操作（线程安全）
void queue_enqueue(SafeCircularQueue *q, Info_Show_In_Err item) {
    pthread_mutex_lock(&q->lock);
    
    if (q->size == QUEUE_CAPACITY) {
        // 队列满时覆盖最旧数据
        q->head = (q->head + 1) % QUEUE_CAPACITY;
    } else {
        q->size++;
    }
    
    q->data[q->tail] = item;
    q->tail = (q->tail + 1) % QUEUE_CAPACITY;
    
    // 通知等待数据的线程
    pthread_cond_signal(&q->cond);
    pthread_mutex_unlock(&q->lock);
}

// 出队操作（线程安全）
bool queue_dequeue(SafeCircularQueue *q, Info_Show_In_Err *result) {

    if (q->size == 0) {
        return false;
        // 队列空时等待条件变量
        //pthread_cond_wait(&q->cond, &q->lock);
    }
    pthread_mutex_lock(&q->lock);
    
    
    
    *result = q->data[q->head];
    q->head = (q->head + 1) % QUEUE_CAPACITY;
    q->size--;
    
    pthread_mutex_unlock(&q->lock);
    return true;
}