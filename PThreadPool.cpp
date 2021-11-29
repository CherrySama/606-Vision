//#include <pthread.h>
//#include <iostream>
//#include <vector>
//#include <queue>
//#include <memory>
//#include <thread>
//#include <mutex>
//#include <condition_variable>
//#include <future>
//#include <functional>
//#include <stdexcept>
//#include <Windows.h>
//#include <string>
//
//using namespace std;
//
//
//bool operator==(pthread_t x,pthread_t y)
//{
//    if (x == y)
//        return TRUE;
//    else
//        return FALSE;
//}
//
//
//// ��������ṹ��
//using callback = void(*)(void*);
//struct Task
//{
//    Task()
//    {
//        function = nullptr;
//        arg = nullptr;
//    }
//    Task(callback f, void* arg)
//    {
//        function = f;
//        this->arg = arg;
//    }
//    callback function;
//    void* arg;
//};
//
//// �������
//class TaskQueue
//{
//public:
//    TaskQueue();
//    ~TaskQueue();
//
//    // �������
//    void addTask(Task& task);
//    void addTask(callback func, void* arg);
//
//    // ȡ��һ������
//    Task takeTask();
//
//    // ��ȡ��ǰ�������������
//    inline int taskNumber()
//    {
//        return m_queue.size();
//    }
//
//private:
//    pthread_mutex_t m_mutex;    // ������
//    std::queue<Task> m_queue;   // �������
//};
//
///****************************************************************************************/
//// �̳߳�
//class ThreadPool
//{
//public:
//    ThreadPool(int min, int max);
//    ~ThreadPool();
//
//    // �������
//    void addTask(Task task);
//    // ��ȡæ�̵߳ĸ���
//    int getBusyNumber();
//    // ��ȡ���ŵ��̸߳���
//    int getAliveNumber();
//
//private:
//    // �������̵߳�������
//    static void* worker(void* arg);
//    // �������̵߳�������
//    static void* manager(void* arg);
//    void threadExit();
//
//private:
//    pthread_mutex_t m_lock;
//    pthread_cond_t m_notEmpty;
//    pthread_t* m_threadIDs;
//    pthread_t m_managerID;
//    TaskQueue* m_taskQ;
//    int m_minNum;
//    int m_maxNum;
//    int m_busyNum;
//    int m_aliveNum;
//    int m_exitNum;
//    bool m_shutdown = false;
//};
//
//
///****************************************************************************************/
////������еĶ���
//TaskQueue::TaskQueue()
//{
//    pthread_mutex_init(&m_mutex, NULL);
//}
//
//TaskQueue::~TaskQueue()
//{
//    pthread_mutex_destroy(&m_mutex);
//}
//
//void TaskQueue::addTask(Task& task)
//{
//    pthread_mutex_lock(&m_mutex);
//    m_queue.push(task);
//    pthread_mutex_unlock(&m_mutex);
//}
//
//void TaskQueue::addTask(callback func, void* arg)
//{
//    pthread_mutex_lock(&m_mutex);
//    Task task;
//    task.function = func;
//    task.arg = arg;
//    m_queue.push(task);
//    pthread_mutex_unlock(&m_mutex);
//}
//
//Task TaskQueue::takeTask()
//{
//    Task t;
//    pthread_mutex_lock(&m_mutex);
//    if (m_queue.size() > 0)
//    {
//        t = m_queue.front();
//        m_queue.pop();
//    }
//    pthread_mutex_unlock(&m_mutex);
//    return t;
//}
//
///****************************************************************************************/
//
//ThreadPool::ThreadPool(int minNum, int maxNum)
//{
//    // ʵ�����������
//    m_taskQ = new TaskQueue;
//    do {
//        // ��ʼ���̳߳�
//        m_minNum = minNum;
//        m_maxNum = maxNum;
//        m_busyNum = 0;
//        m_aliveNum = minNum;
//
//        // �����̵߳�������޸��߳���������ڴ�
//        m_threadIDs = new pthread_t[maxNum];
//        if (m_threadIDs == nullptr)
//        {
//            cout << "���� thread_t �ڴ�ʧ��...." << endl;;
//            break;
//        }
//        // ��ʼ��
//        memset(m_threadIDs, 0, sizeof(pthread_t) * maxNum);
//        // ��ʼ��������,��������
//        if (pthread_mutex_init(&m_lock, NULL) != 0 ||
//            pthread_cond_init(&m_notEmpty, NULL) != 0)
//        {
//            cout << "init mutex or condition fail..." << endl;
//            break;
//        }
//
//        /////////////////// �����߳� //////////////////
//        // ������С�̸߳���, �����߳�
//        for (int i = 0; i < minNum; ++i)
//        {
//            pthread_create(&m_threadIDs[i], NULL, worker, this);
//            cout << "�������߳�, ID: " << to_string(m_threadIDs[i]) << endl;
//        }
//        // �����������߳�, 1��
//        pthread_create(&m_managerID, NULL, manager, this);
//    } while (0);
//}
//
//ThreadPool::~ThreadPool()
//{
//    m_shutdown = 1;
//    // ���ٹ������߳�
//    pthread_join(m_managerID, NULL);
//    // ���������������߳�
//    for (int i = 0; i < m_aliveNum; ++i)
//    {
//        pthread_cond_signal(&m_notEmpty);
//    }
//
//    if (m_taskQ) delete m_taskQ;
//    if (m_threadIDs) delete[]m_threadIDs;
//    pthread_mutex_destroy(&m_lock);
//    pthread_cond_destroy(&m_notEmpty);
//}
//
//void ThreadPool::addTask(Task task)
//{
//    if (m_shutdown)
//    {
//        return;
//    }
//    // ������񣬲���Ҫ�������������������
//    m_taskQ->addTask(task);
//    // ���ѹ������߳�
//    pthread_cond_signal(&m_notEmpty);
//}
//
//int ThreadPool::getAliveNumber()
//{
//    int threadNum = 0;
//    pthread_mutex_lock(&m_lock);
//    threadNum = m_aliveNum;
//    pthread_mutex_unlock(&m_lock);
//    return threadNum;
//}
//
//int ThreadPool::getBusyNumber()
//{
//    int busyNum = 0;
//    pthread_mutex_lock(&m_lock);
//    busyNum = m_busyNum;
//    pthread_mutex_unlock(&m_lock);
//    return busyNum;
//}
//
//
//// �����߳�������
//void* ThreadPool::worker(void* arg)
//{
//    ThreadPool* pool = static_cast<ThreadPool*>(arg);
//    // һֱ��ͣ�Ĺ���
//    while (true)
//    {
//        // �����������(������Դ)����
//        pthread_mutex_lock(&pool->m_lock);
//        // �ж���������Ƿ�Ϊ��, ���Ϊ�չ����߳�����
//        while (pool->m_taskQ->taskNumber() == 0 && !pool->m_shutdown)
//        {
//            printf(" Thread %ld wating...\n", pthread_self());
//            // �����߳�
//            pthread_cond_wait(&pool->m_notEmpty, &pool->m_lock);
//
//            // �������֮��, �ж��Ƿ�Ҫ�����߳�
//            if (pool->m_exitNum > 0)
//            {
//                pool->m_exitNum--;
//                if (pool->m_aliveNum > pool->m_minNum)
//                {
//                    pool->m_aliveNum--;
//                    pthread_mutex_unlock(&pool->m_lock);
//                    pool->threadExit();
//                }
//            }
//        }
//        // �ж��̳߳��Ƿ񱻹ر���
//        if (pool->m_shutdown)
//        {
//            pthread_mutex_unlock(&pool->m_lock);
//            pool->threadExit();
//        }
//
//        // �����������ȡ��һ������
//        Task task = pool->m_taskQ->takeTask();
//        // �������߳�+1
//        pool->m_busyNum++;
//        // �̳߳ؽ���
//        pthread_mutex_unlock(&pool->m_lock);
//        // ִ������
//        cout << "thread " << to_string(pthread_self()) << " start working..." << endl;
//        task.function(task.arg);
//        delete task.arg;
//        task.arg = nullptr;
//
//        // ���������
//        cout << "thread " << to_string(pthread_self()) << " end working...";
//        pthread_mutex_lock(&pool->m_lock);
//        pool->m_busyNum--;
//        pthread_mutex_unlock(&pool->m_lock);
//    }
//
//    return nullptr;
//}
//
//
//// �������߳�������
//void* ThreadPool::manager(void* arg)
//{
//    ThreadPool* pool = static_cast<ThreadPool*>(arg);
//    // ����̳߳�û�йر�, ��һֱ���
//    while (!pool->m_shutdown)
//    {
//        // ÿ��5s���һ��
//        Sleep(5);
//        // ȡ���̳߳��е����������߳�����
//        //  ȡ���������̳߳�����
//        pthread_mutex_lock(&pool->m_lock);
//        int queueSize = pool->m_taskQ->taskNumber();
//        int liveNum = pool->m_aliveNum;
//        int busyNum = pool->m_busyNum;
//        pthread_mutex_unlock(&pool->m_lock);
//
//        // �����߳�
//        const int NUMBER = 2;
//        // ��ǰ�������>�����߳��� && �����߳���<����̸߳���
//        if (queueSize > liveNum && liveNum < pool->m_maxNum)
//        {
//            // �̳߳ؼ���
//            pthread_mutex_lock(&pool->m_lock);
//            int num = 0;
//            for (int i = 0; i < pool->m_maxNum && num < NUMBER
//                && pool->m_aliveNum < pool->m_maxNum; ++i)
//            {
//                if (pool->m_threadIDs[i] == 0)
//                {
//                    pthread_create(&pool->m_threadIDs[i], NULL, worker, pool);
//                    num++;
//                    pool->m_aliveNum++;
//                }
//            }
//            pthread_mutex_unlock(&pool->m_lock);
//        }
//
//        // ���ٶ�����߳�
//        // æ�߳�*2 < �����߳���Ŀ && �����߳��� > ��С�߳�����
//        if (busyNum * 2 < liveNum && liveNum > pool->m_minNum)
//        {
//            pthread_mutex_lock(&pool->m_lock);
//            pool->m_exitNum = NUMBER;
//            pthread_mutex_unlock(&pool->m_lock);
//            for (int i = 0; i < NUMBER; ++i)
//            {
//                pthread_cond_signal(&pool->m_notEmpty);
//            }
//        }
//    }
//    return nullptr;
//}
//
//// �߳��˳�
//void ThreadPool::threadExit()
//{
//    pthread_t tid = pthread_self();
//    for (int i = 0; i < m_maxNum; ++i)
//    {
//        if (m_threadIDs[i] == tid)
//        {
//            cout << "threadExit() function: thread "
//                << to_string(pthread_self()) << " exiting..." << endl;
//            m_threadIDs[i] = 0;
//            break;
//        }
//    }
//    pthread_exit(NULL);
//}
//
//
