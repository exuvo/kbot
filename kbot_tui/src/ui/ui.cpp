#include "ui.h"
#include <iostream>
#include <thread>
#include "widgets.h"
#include "kbotpi.h"
#include "settings.h"

class mainTab : public tab {
private:
	menuWidget menu;
public:
//	mainTab(): tab("main"), menu(getWindow(),"Main", 2) {
	mainTab(): tab("main"), menu(getWindow(),"Main", 
		{
			menuItem("a","b",nullptr),
			menuItem("Quit","Ends the program", [](){ui.stop();})
		}) {
//		menu.add("c","A very long description is placed here.", [] () {});
//		menu.add("Quit","Ends the program", [] () {ui.stop();});
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

		if(currentTab == t) wattron(w, color::ACTIVE);
		mvwaddstr(w, 0, col, name.data());
		if(currentTab == t) wattroff(w, color::ACTIVE);
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
	curs_set(0);//Invisible cursor

	init_pair(1, COLOR_RED, COLOR_BLACK);
	init_pair(2, COLOR_GREEN, COLOR_BLACK);
	init_pair(3, COLOR_BLUE, COLOR_BLACK);
	init_pair(4, COLOR_CYAN, COLOR_BLACK);

	tabs.push_back(new mainTab());	
	tabs.push_back(new dummyTab("nav"));//2D map
	tabs.push_back(new settingsTab());//bios style
	tabs.push_back(new dummyTab("log"));//read error file
	tabs.push_back(new dummyTab("sensors"));//menu list with subsystems	
	tabs.push_back(new dummyTab("network"));//list current connections

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
					currentTab->input(KEY_CTAB);
					hide_panel(currentTab->getPanel());
					currentTab = tabs[ch];
					show_panel(currentTab->getPanel());
					currentTab->input(KEY_STAB);
					drawHeader();
				}
				break;
			case 'q':
				stop();
				break;
			case ERR:
				this_thread::sleep_for(chrono::milliseconds(200));
				break;
			default:
				currentTab->input(ch);
				break;
		}
	}

	endwin();
	std::cout << "Stopping UI thread\n";
}

