//
// Created by books on 2017/11/25.
//

#ifndef MAPTOOLS_LOG_H
#define MAPTOOLS_LOG_H

#include <iostream>
#include <sstream>
#include <iomanip>
#include <thread>
#include <mutex>
#include <chrono>
#include <condition_variable>
#include <ctime>

namespace MapBase
{
	class Log
	{
	public:
		Log(int _interval=1);

		~Log() = default;

		void Ends(const std::string &str)
		{
			cur_word=cur_word+str+" ";
		};

		void Endl(const std::string &str)
		{
			cur_word=cur_word+str+"\n";
		};

		void Update();

		void Start();

		void Stop();

	private:
		bool update_flag;
		std::mutex update_mutex;
		std::thread update_thread;
		int interval;
		std::string cur_word;
		std::string word;
		std::string time_stamp;
	};
}


#endif //MAPTOOLS_LOG_H
