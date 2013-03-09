#ifndef UI_H
#define UI_H

#include <thread>
#include <atomic>
#include <ncurses.h>
#include <panel.h>
#include <string>
#include <vector>

class tab {
public:
	tab(std::string name): _name(name) {
		win = newwin(LINES-2, COLS, 2, 0);
		pan = new_panel(win);
	}
	virtual ~tab() { del_panel(pan); delwin(win); }
	PANEL* getPanel() {return pan;}
	WINDOW* getWindow() {return win;}
	std::string& getName() {return _name;}
	virtual bool input(int key) {return false;};
	virtual void update() {};

private:
	std::string _name;
	WINDOW *win;
	PANEL *pan;
};

class UI{
public:
	UI() : m_stop(), m_thread(), tabs() { };
	virtual ~UI() { if(m_thread.joinable()) stop(); }
	void start() { m_thread = std::thread(&UI::run, this); }
	void stop() { m_stop = true; }
	void join() { m_thread.join(); }

private:
	std::atomic<bool> m_stop;
	std::thread m_thread;
	std::vector<tab*> tabs;
	WINDOW* headerW;
	PANEL* headerP;
	tab* currentTab;

	void run();
	void initHeader();
	void drawHeader();
};

#endif //UI_H
