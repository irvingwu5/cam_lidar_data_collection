/*
 * @Author: Fengqiang
 * @Date: 2021-02-25 12:00:19
 * @LastEditTime: 2021-09-14 19:54:56
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: \IntegratedComputingPlatform\include\msg_queue.h
 */
#ifndef __INCLUDE_MSG_QUEUE_H_
#define __INCLUDE_MSG_QUEUE_H_

#include <iostream>
#include <queue>
#include <condition_variable>
#include <mutex>
#include <string>
namespace benewake
{
template<class T>
class MsgQueue
{
public:
    MsgQueue(std::string queue_name = "msg", int max_size = 3)
    {
        queue_name_ = queue_name;
        max_size_ = max_size;
    }
    ~MsgQueue() {}

	T front();
    T popMsg();
    void pop();
    void pushMsg(const T& msg);
    void pushMsgWait(const T& msg);
    void clear();
	bool empty();
    int size();
    void setPopWaitTime(int time); // in millisecond
private:
    std::queue<T> queue_;
    std::condition_variable cv_space_;
    std::condition_variable cv_data_;
    std::mutex mutex_;
    int max_size_;
    std::string queue_name_;
    int pop_wait_time_ = 1000; // in milisecond
};

/**
 * @description: 向消息队列中发送消息
 * @param {const T& }msg
 * @return {*}
 * @note 消息队列长度超过max_size_则pop之前的旧消息
 */
template<class T>
void MsgQueue<T>::pushMsg(const T& msg)
{
    std::unique_lock<std::mutex> lock{mutex_};
    queue_.push(msg);
    if (queue_.size() > max_size_)
    {
        queue_.pop();
    }
    cv_data_.notify_one();
    lock.unlock();
}

/**
 * @description: 向消息队列中发送消息
 * @param {const T& }msg
 * @return {*}
 * @note 消息队列长度如果已经为max_size_,则等待直到队列长队最多为max_size_-1
 */
template<class T>
void MsgQueue<T>::pushMsgWait(const T& msg)
{
    std::unique_lock<std::mutex> lock{mutex_};
    while (queue_.size() == max_size_)
    {
        cv_space_.wait(lock);
    }
    queue_.push(msg);
    cv_data_.notify_one();
    lock.unlock();
}

template<class T>
T MsgQueue<T>::popMsg()
{
    T data;
    std::unique_lock<std::mutex> lock{mutex_};
    while (queue_.size() == 0)
    {
        auto status = cv_data_.wait_for(lock, std::chrono::milliseconds(pop_wait_time_));
        if (std::cv_status::timeout == status)
        {
            std::cout << queue_name_ << " popMsg timeout" << std::endl;
            return data;
        }
    }
    data = queue_.front();
    queue_.pop();
    cv_space_.notify_one();
    lock.unlock();
    return data;
}

template<class T>
inline void MsgQueue<T>::pop()
{
    std::unique_lock<std::mutex> lock{ mutex_ };
    if (queue_.size() != 0)
        queue_.pop();
    lock.unlock();
}

template<class T>
T MsgQueue<T>::front()
{
	std::unique_lock<std::mutex> lock{ mutex_ };
	while (queue_.size() == 0)
	{
		cv_data_.wait(lock);
	}
	T data = queue_.front();
	lock.unlock();
	return data;
}

template<class T>
void MsgQueue<T>::clear()
{
    std::unique_lock<std::mutex> lock{mutex_};
    int length = queue_.size();
    for (auto i = 0; i < length; ++i)
        queue_.pop();
    lock.unlock();
}

template<class T>
int MsgQueue<T>::size()
{
    std::unique_lock<std::mutex> lock{mutex_};
    return queue_.size();
}

template<class T>
inline void MsgQueue<T>::setPopWaitTime(int time)
{
    pop_wait_time_ = time;
}

template<class T>
bool MsgQueue<T>::empty()
{
	std::unique_lock<std::mutex> lock{ mutex_ };
	return queue_.size() == 0;
}

}
#endif