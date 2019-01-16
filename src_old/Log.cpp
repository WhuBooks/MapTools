//
// Created by books on 2017/11/25.
//

#include "Log.h"

namespace MapBase
{
    Log::Log(int _interval)
    {
        interval = _interval;
        update_flag = false;
    }

    void Log::Update()
    {
//        if (word == cur_word)
//        {
//            cur_word="";
//            return;
//        }

        std::unique_lock<std::mutex> lock(update_mutex);

        time_t now = std::time(nullptr);
        struct tm *lt = std::localtime(&now);
        time_stamp = "[ " + std::to_string(lt->tm_hour) + "-" + std::to_string(lt->tm_min) + "-" +
                     std::to_string(lt->tm_sec) + " ]";
        word = cur_word;
        cur_word = "";

        lock.unlock();
    }

    void Log::Start()
    {
        update_flag = true;
        update_thread = std::thread([&]() {
            while (update_flag)
            {
                std::unique_lock<std::mutex> lock(update_mutex);
                if (!word.empty())
                {
					std::cout << time_stamp << std::endl;
                    std::cout << word << std::endl;
                }
                lock.unlock();
                if(interval!=0)
                    std::this_thread::sleep_for(std::chrono::seconds(interval));
                else
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        });
        update_thread.detach();
    }

    void Log::Stop()
    {
        update_flag = false;
    }



}


