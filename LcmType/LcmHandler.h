#pragma once
#include <string>

#include <lcm/lcm-cpp.hpp>
#include <memory>

template<typename T>
class LcmHandler
{
private:
	std::string m_net;
	std::string m_channel;
	lcm::LCM *m_lcm;
	T m_data;

	void handleMessage(const lcm::ReceiveBuffer* rbuf,
		const std::string& chan,
		const T * msg)
	{
		m_data = *msg;
	}

public:
	~LcmHandler() 
	{
		if (m_lcm != NULL)
		{
			delete m_lcm;
		}
	}
	LcmHandler()
	{
		m_net = "";
		m_channel = "";
		m_lcm = NULL;
	}
	LcmHandler(std::string net, std::string channel)
	{
		m_net = net;
		m_channel = channel;
	}
	LcmHandler(std::string channel)
	{
		m_net = "";
		m_channel = channel;
	}

	void SetNet(std::string net)
	{
		m_net = net;
	}
	void SetChannel(std::string channel)
	{
		m_channel = channel;
	}

	int InitialSend()
	{
		m_lcm = new lcm::LCM(m_net);
		if (!m_lcm->good())
		{
			return 0;
		}
		return 1;
	}
	void SendLcm(T* data)
	{
		if (m_lcm->good())
		{
			m_lcm->publish(m_channel, data);
		}
	}

	int InitialListen()
	{
		m_lcm = new lcm::LCM(m_net);
		if (!m_lcm->good())
			return 0;
		m_lcm->subscribe(m_channel, &LcmHandler::handleMessage, this);
		return 1;
	}

	void GetData(T &data)
	{
		m_lcm->handle();
		data = m_data;
	}

	bool GetData(T &data, int time)
	{
		int res=m_lcm->handleTimeout(time);
		if (res > 0)
		{
			data = m_data;
			return true;
		}
		else
			return false;
	}
	typedef std::shared_ptr<LcmHandler<T>> Ptr;
};
