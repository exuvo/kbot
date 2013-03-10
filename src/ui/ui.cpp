#include <ncurses.h>
#include "ui.h"
#include <iostream>
#include <thread>
#include <menu.h>
#include "dbg.h"


#define HIGHLIGHT COLOR_PAIR(1)

class menuWidget {
	std::vector<std::string> names, desc;
	MENU *menu;
	ITEM **items;
	unsigned int size;
	WINDOW *w;

	void rebuild(){
		items = (ITEM **) calloc(names.size()+1, sizeof(ITEM*));
		check_mem(items);
		for(unsigned int i=0; i<names.size(); i++){
			items[i] = new_item(names[i].c_str(), desc[i].c_str());
			check(items[i], "");
		}
		menu = new_menu(items);
		check(menu, "");
		set_menu_win(menu, w);
		set_menu_sub(menu, derwin(w,6,38,4,4));
		set_menu_mark(menu, "*");
//		int sy,sx,ey,ex,h,w;
//		getbegyx(w, sy, sx);
//		getmaxyx(w, ey, ex);
//		h = ey-sy;
//		w = ex-sx;
		box(w,0,0);
//		mvwprintw(w, 2, w/2, "My menu");
//		mvwaddch(w, 2, 0, ACS_LTEE);
//		mvwhline(w, 2, 1, ACS_HLINE, w-2);
//		mvwaddch(w, 2, w-1, ACS_RTEE);
		size = names.size();

		return;
		error:
			debug("e");
	}

	void clean(){
		for(unsigned int i=0; i<names.size(); i++){
			free_item(items[i]);
		}
		free(items);
		free_menu(menu);
	}

public:
	menuWidget(WINDOW *win): names(), desc(), size(0), w(win) {}
	~menuWidget() {clean();}

	void add(std::string name, std::string descript){
		names.push_back(name);
		desc.push_back(descript);
	}

	void update(){
		if(size != names.size()){
			rebuild();
			post_menu(menu);
		}
		wrefresh(w);
	}

	bool input(int &key){
		switch(key){
			case KEY_DOWN:
				menu_driver(menu, REQ_DOWN_ITEM);
				break;
			case KEY_UP:
				menu_driver(menu, REQ_UP_ITEM);
				break;
			default:
				return false;
		}
		return true;
	}
};

class mainTab : public tab {
private:
	WINDOW *menuW;
	PANEL *menuP;
	menuWidget menu;
public:
	mainTab(): tab("main"), menuW(derwin(win, 10,10,5,5)), menuP(new_panel(menuW)), menu(menuW) {
//	mainTab(): tab("main"), menuW(getWindow()), menu(menuW) {
		menu.add("a","b"); 
		menu.add("c","d");
	}
	~mainTab() {}

	void update(){
		menu.update();
	}

	bool input(int key){
		switch(key){
			default:
				return menu.input(key);
		}
	}
};

void UI::initHeader(){
	headerW = newwin(2, COLS, 0, 0);
	WINDOW *w = headerW;
	headerP = new_panel(w);

	mvwhline(w, 1, 0, ACS_HLINE, COLS);
	mvwaddch(w, 1, 0, ACS_LLCORNER);
	mvwaddch(w, 0, 0, ACS_VLINE);

	int col=1;
	for(tab* t : tabs){
		std::string &name = t->getName();
		col += name.size();
		mvwaddch(w, 1, col, ACS_BTEE);
		mvwaddch(w, 0, col, ACS_VLINE);
		col++;
	}

}

void UI::drawHeader(){
	WINDOW *w = headerW;

	int col=1;
	for(tab* t : tabs){
		std::string &name = t->getName();

		if(currentTab == t) wattron(w, HIGHLIGHT);
		mvwaddstr(w, 0, col, name.c_str());
		if(currentTab == t) wattroff(w, HIGHLIGHT);
		col += name.size() + 1;
	}
}

void UI::run(){
	std::cout << "Starting UI thread\n";
	initscr();
	cbreak();//Disable line buffering
	noecho();
	keypad(stdscr, true);
	timeout(0);
	start_color();

	init_pair(1, COLOR_RED, COLOR_BLACK);
	init_pair(2, COLOR_GREEN, COLOR_BLACK);
	init_pair(3, COLOR_BLUE, COLOR_BLACK);
	init_pair(4, COLOR_CYAN, COLOR_BLACK);

	//tabs.push_back(dynamic_cast<tab*>(new mainTab()));	
	tabs.push_back(new mainTab());	
	//tabs.push_back(new tab("nav"));	
	//tabs.push_back(new tab("settings"));	
	//tabs.push_back(new tab("log"));	
	//tabs.push_back(new tab("_"));	

	for(tab* t : tabs){
		hide_panel(t->getPanel());
	}

	currentTab = tabs[0];
	show_panel(currentTab->getPanel());

	initHeader();
	drawHeader();

	while(!m_stop){
		currentTab->update();
		update_panels();
		doupdate();

		int ch = getch();
		if(ch != ERR) mvwprintw(headerW, 0, COLS-3, "%03d", ch);
		switch(ch){
			case KEY_F(1):
			case KEY_F(2):
			case KEY_F(3):
			case KEY_F(4):
			case KEY_F(5):
			case KEY_F(6):
			case KEY_F(7):
			case KEY_F(8):
			case KEY_F(9):
			case KEY_F(10):
			case KEY_F(11):
			case KEY_F(12):
				ch -= KEY_F(1);
				if(ch < (int)tabs.size()){
					hide_panel(currentTab->getPanel());
					currentTab = tabs[ch];
					show_panel(currentTab->getPanel());
					drawHeader();
				}
				break;
			case 'q':
				stop();
				break;
			case ERR:
				std::this_thread::sleep_for(std::chrono::milliseconds(200));
				break;
			default:
				currentTab->input(ch);
				break;
		}
	}

	endwin();
	std::cout << "Stopping UI thread\n";
}

