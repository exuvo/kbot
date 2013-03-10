#ifndef UI_H
#define UI_H

#include <thread>
#include <atomic>
#include <ncurses.h>
#include <panel.h>
#include <string>
#include <vector>
#include "dbg.h"

using namespace std;

enum class color {
	ACTIVE=COLOR_PAIR(1),
	INFO=COLOR_PAIR(2)
};

class tab {
public:
	tab(string name): _name(name) {
		win = newwin(LINES-2, COLS, 2, 0);
		pan = new_panel(win);
	}
	virtual ~tab() { del_panel(pan); delwin(win); }
	PANEL* getPanel() {return pan;}
	WINDOW* getWindow() {return win;}
	string& getName() {return _name;}
	//Sends KEY_CTAB when tab loses focus and KEY_STAB at gain focus.
	virtual bool input(int key)  = 0;//{debug("i"); return false;};
	virtual void update() = 0;

private:
protected:
	string _name;
	WINDOW *win;
	PANEL *pan;
};

class UI{
public:
	UI() : m_stop(), m_thread(), tabs() { };
	virtual ~UI() {
		del_panel(headerP);
		delwin(headerW);
		for(tab *t : tabs){
			delete t;
		}
		if(m_thread.joinable()) stop(); 
	}
	void start() { m_thread = thread(&UI::run, this); }
	void stop() { m_stop = true; }
	void join() { m_thread.join(); }

private:
	atomic<bool> m_stop;
	thread m_thread;
	vector<tab*> tabs;
	WINDOW* headerW;
	PANEL* headerP;
	tab* currentTab;

	void run();
	void initHeader();
	void drawHeader();
};

class dummyTab : public tab {
public:
	dummyTab(string name) : tab(name) {}
	void update(){}
	bool input(int key){return false;}
};

#endif /* UI_H */
