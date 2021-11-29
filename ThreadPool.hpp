//#ifndef THREAD_POOL_H
//#define THREAD_POOL_H
//
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
//
//using namespace std;
//
//class ThreadPool {
//
//public:
//    ThreadPool(size_t);                          //���캯��
//    template<class F, class... Args>             //��ģ��
//    auto enqueue(F&& f, Args&&... args)->std::future<typename std::result_of<F(Args...)>::type>; //��������
//    void Delete();                              //��������
//
//private:
//    std::vector< std::thread > workers;            //�̶߳��У�ÿ��Ԫ��Ϊһ��Thread����
//    std::queue< std::function<void()> > tasks;     //������У�ÿ��Ԫ��Ϊһ����������    
//
//    std::mutex queue_mutex;                        //������
//    std::condition_variable condition;             //��������
//    bool stop;                                     //ֹͣ
//};
//
//// ���캯�������̲߳����̶߳��У�����ʱ����embrace_back()������������lambda��ʼ��Thread����
//inline ThreadPool::ThreadPool(size_t threads) : stop(false) {
//
//    for (size_t i = 0; i < threads; ++i)
//        workers.emplace_back(
//            [this]
//            {
//                while(1)
//                {
//                    
//                    std::function<void()> task;
//                    {
//                        
//                        std::unique_lock<std::mutex> mutex(this->queue_mutex);
//
//                        //������falseʱ�������̣߳�����ʱ�Զ��ͷ���
//                        //������true���ܵ�֪ͨʱ��������Ȼ�����
//                        this->condition.wait(mutex, [this] { return this->stop || !this->tasks.empty(); });
//
//                        if (this->stop && this->tasks.empty())                                               
//                            return;  //�����ж�ѭ��
//                        
//                        //���������ȡ��һ������
//                        task = std::move(this->tasks.front());
//                        this->tasks.pop();
//                    }                            
//                    task();                      // ִ���������
//                }
//            }
//            );
//}
//
//// ����µ�����
//template<class F, class... Args>
//auto ThreadPool::enqueue(F&& f, Args&&... args) -> std::future<typename std::result_of<F(Args...)>::type>
//{
//    // ��ȡ��������ֵ����        
//    using return_type = typename std::result_of<F(Args...)>::type;
//
//    // ����һ��ָ�����������ָ��
//    auto task = std::make_shared< std::packaged_task<return_type()> >(
//        std::bind(std::forward<F>(f), std::forward<Args>(args)...)
//        );
//
//    std::future<return_type> res = task->get_future();
//    {
//        std::unique_lock<std::mutex> lock(queue_mutex);  
//        if (stop)
//            throw std::runtime_error("enqueue on stopped ThreadPool");
//
//        tasks.emplace([task]() { (*task)(); });          //������������
//    }                                                   //�Զ�����
//    condition.notify_one();                             //֪ͨ��������������һ���߳�
//    return res;
//}
//
//// ɾ�������߳�
//inline void ThreadPool::Delete()
//{
//    {
//        std::unique_lock<std::mutex> lock(queue_mutex);
//        stop = true;
//    }
//    condition.notify_all();
//    for (std::thread& worker : workers)
//        worker.join();
//    
//    cout << "All the threads have been deleted" << endl;
//}
//
//#endif
