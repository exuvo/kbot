#ifndef UI_H
#define UI_H

#include <thread>
#include <atomic>

class UI{


public:
	UI() : m_stop(), m_thread() { };
	virtual ~UI() { if(m_thread.joinable()) stop(); }
	void start() { m_thread = std::thread(&UI::run, this); }
	void stop() { m_stop = true; }
	void join() { m_thread.join(); }

private:
	std::atomic<bool> m_stop;
	std::thread m_thread;

	void run();
	
};


#endif //UI_H
