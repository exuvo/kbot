#include "widgets.h"
#include "settings.h"

using namespace std;

settingsTab::settingsTab(): tab("settings"), desc(), name() {
	int width, height;
	getmaxyx(win, height, width);
	int descWidth = 25;

	mainW = derwin(win, height, width-descWidth, 0, 0);
	mainP = new_panel(mainW);
	box(mainW, 0, 0);

	descW = derwin(win, height, descWidth, 0, width-descWidth);
	descP = new_panel(descW);
	box(descW, 0, 0);
	mvwaddch(descW, 2, 0, ACS_LTEE);
	mvwhline(descW, 2, 1, ACS_HLINE, descWidth-2);
	mvwaddch(descW, 2, descWidth-1, ACS_RTEE);

	hide_panel(mainP);
	hide_panel(descP);
}
settingsTab::~settingsTab() {}

void settingsTab::update() {}

void settingsTab::redraw(){
  redrawDescription();
}

void settingsTab::redrawDescription(){
	//TODO clear
	//
	int cols = getmaxx(descW);
	wattron(descW, color::INFO); 
	mvwprintw(descW, 1, cols/2 - name.size()/2, name.data());
	wattroff(descW, color::INFO); 

	const char *c = desc.data();
	int line = 3,col=1;
	while(*c){
		if(*c == '\n'){
		 	line++;
			col = 1;
		}else{
			if(col > cols - 2){
				line++;
				col = 1;
			}
			mvwaddch(descW, line, col, *c);
			col++;
		}
		c++;
	}
}

bool settingsTab::input(int key){
	switch(key){
			case KEY_STAB:
				redraw();
				show_panel(mainP);
				show_panel(descP);
				break;
			case KEY_CTAB:
				hide_panel(mainP);
				hide_panel(descP);
				break;
		default:
			return false;
	}
  return true;
}
