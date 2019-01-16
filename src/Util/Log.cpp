//
// Created by 98420 on 2018/8/31.
//

#include "Log.h"

namespace util
{
    
    Log::Log(int _interval)
    {
        m_interval = _interval;
        m_update_flag = false;
    }
    
    void Log::update()
    {
        std::unique_lock<std::mutex> lock(m_update_mutex);
    
        time_t now = std::time(nullptr);
        struct tm *lt = std::localtime(&now);
        m_time_stamp = "[ " + std::to_string(lt->tm_hour) + "-" + std::to_string(lt->tm_min) + "-" +
                     std::to_string(lt->tm_sec) + " ]";
        m_word = m_cur_word;
        m_cur_word = "";
    
        lock.unlock();
    }
    
    void Log::start()
    {
        m_update_flag = true;
        m_update_thread = std::thread([&]() {
            while (m_update_flag)
            {
                std::unique_lock<std::mutex> lock(m_update_mutex);
                if (!m_word.empty())
                {
                    std::cout << m_time_stamp << std::endl;
                    std::cout << m_word << std::endl;
                }
                lock.unlock();
                if(m_interval!=0)
                    std::this_thread::sleep_for(std::chrono::seconds(m_interval));
                else
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        });
        m_update_thread.detach();
    }
    
    void Log::stop()
    {
        m_update_flag = false;
    }
}
