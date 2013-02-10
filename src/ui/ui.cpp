#include <ncurses.h>
#include "ui.h"
#include <iostream>
#include <thread>

void UI::run(){
	std::cout << "Starting UI thread\n";
	initscr();
	printw("Starting UI..");
	refresh();

	while(!m_stop){
		printw("UI Running");
		refresh();
		std::this_thread::sleep_for(std::chrono::milliseconds(200));
	}

	endwin();
	std::cout << "Stopping UI thread\n";
}
