#include "ui.h"
#include "widgets.h"

using namespace std;

infoWidget::infoWidget(string name,string desc): name(name), desc(desc) {
		int lines=4,cols=2+name.size();

		{
			int tmp_cols=2;
			const char *c = desc.data();
			do{
				if(*c == '\n' || *c == 0){
				 	lines++;
					if(tmp_cols > cols) cols = tmp_cols;
					tmp_cols = 2;
				}else{
					tmp_cols++;
				}
			}while(*c++);
		}

		w = newwin(lines, cols,(LINES-lines)/2,(COLS-cols)/2);
		p = new_panel(w);
	
		wattron(w, color::INFO); 
		mvwprintw(w, 1, cols/2 - name.size()/2, name.data());
		wattroff(w, color::INFO); 
		box(w,0,0);
		mvwaddch(w, 2, 0, ACS_LTEE);
		mvwhline(w, 2, 1, ACS_HLINE, cols-2);
		mvwaddch(w, 2, cols-1, ACS_RTEE);

		const char *c = desc.data();
		int line = 3,col=1;
		while(*c){
			if(*c == '\n'){
			 	line++;
				col=1;
			}else{
				mvwaddch(w, line, col, *c);
				col++;
			}
			c++;
		}
	}

infoWidget::~infoWidget() {
		del_panel(p);
		delwin(w);		
	}

void menuWidget::rebuild(){
		items = (ITEM **) calloc(names.size()+1, sizeof(ITEM*));
		dbg_check_mem(items);
		for(unsigned int i=0; i<names.size(); i++){
			items[i] = new_item(names[i].data(), desc[i].data());
			dbg_check(items[i], "");
			set_item_userptr(items[i], (void*)(intptr_t)i);
		}
		menu = new_menu(items);
		dbg_check(menu, "");
		set_menu_win(menu, w);
		set_menu_sub(menu, derwin(w,getmaxy(w)-4,getmaxx(w)-2,3,1));
		set_menu_mark(menu, " * ");
		menu_opts_off(menu, O_SHOWDESC|O_NONCYCLIC);

		int width,height;
		getmaxyx(w,height,width);
		box(w,0,0);
		mvwaddch(w, 2, 0, ACS_LTEE);
		mvwhline(w, 2, 1, ACS_HLINE, width-2);
		mvwaddch(w, 2, width-1, ACS_RTEE);

		mvwaddch(w, height-3, 0, ACS_LTEE);
		mvwhline(w, height-3, 1, ACS_HLINE, width-2);
		mvwaddch(w, height-3, width-1, ACS_RTEE);

		wattron(w, color::INFO); 
		mvwprintw(w, 1, width-2, "?");
		wattroff(w, color::INFO);
		mvwprintw(w, 1, width/2 - name.size()/2, name.data());

		size = names.size();

		return;
		error:
			dbg_debug("e");
	}

void menuWidget::clean(){
		if(items){
			for(unsigned int i=0; i<size; i++){
				free_item(items[i]);
			}
			free(items);
		}
		if(menu) free_menu(menu);
	}

menuWidget::menuWidget(WINDOW *win, string name, initializer_list<menuItem> list): 
		name(name), names(), desc(), callbacks(), size(0) {
	for(menuItem i : list){
		add(i);
	}
	
	int lines=6+names.size();
	int cols=3+name.size();

	for(unsigned int i=0; i<names.size(); i++){
		int ns = 5 + names[i].size();
		int ds = 2 + desc[i].size();
		if(ns > cols) cols = ns;
		if(ds > cols) cols = ds;
	}
	
	w = derwin(win, lines, cols, (getmaxy(win)-lines)/2-1, (getmaxx(win)-cols)/2);
	p = new_panel(w);
}

menuWidget::menuWidget(WINDOW *win, string name, int border): 
		name(name), names(), desc(), callbacks(), size(0), 
		w(derwin(win, getmaxy(win) - 2*border, getmaxx(win) - 4*border, border > 0 ? border-1: 0, 2*border)),
	 	p(new_panel(w)) {}
menuWidget::menuWidget(WINDOW *win, string name, int height, int width):
		name(name), names(), desc(), callbacks(), size(0),
		w(derwin(win, height, width, (getmaxy(win)-height)/2-1, (getmaxx(win)-width)/2)),
	 	p(new_panel(w)) {}
menuWidget::~menuWidget() {clean(); if(i) delete i;}

void menuWidget::add(string name, string description, function<void()> callback){
		names.push_back(name);
		desc.push_back(description);
		callbacks.push_back(callback);
	}

void menuWidget::add(menuItem item){
	add(get<0>(item), get<1>(item), get<2>(item));
}

void menuWidget::update(){
		if(size != names.size()){
			clean();
			rebuild();
			post_menu(menu);
			redraw();
		}
	}

void menuWidget::redraw(){
		int lines,cols;
	  getmaxyx(w, lines, cols);
		mvwhline(w, lines-2, 1, ' ', cols-2);
		string& text = desc[(intptr_t)item_userptr(current_item(menu))];
		mvwprintw(w, lines-2, cols/2 - text.size()/2, text.data());
}

bool menuWidget::input(int &key){
		if(i){
			delete i;
			i = nullptr;
			if(key != KEY_STAB && key != KEY_CTAB) return true;
		}
		switch(key){
			case KEY_STAB:
				show_panel(p);
				break;
			case KEY_CTAB:
				hide_panel(p);
				break;
			case KEY_DOWN:
				menu_driver(menu, REQ_DOWN_ITEM);
				redraw();
				break;
			case KEY_UP:
				menu_driver(menu, REQ_UP_ITEM);
				redraw();
				break;
			case KEY_RIGHT:
				menu_driver(menu, REQ_RIGHT_ITEM);
				redraw();
				break;
			case KEY_LEFT:
				menu_driver(menu, REQ_LEFT_ITEM);
				redraw();
				break;
			case KEY_NPAGE:
				menu_driver(menu, REQ_SCR_DPAGE);
				redraw();
				break;
			case KEY_PPAGE:
				menu_driver(menu, REQ_SCR_UPAGE);
				redraw();
				break;
			case KEY_ENTER://Keypad Enter
			case ' ':
			case 10://Enter
				if(menu_opts(menu) & O_ONEVALUE){
					function<void()> fun = callbacks[(intptr_t)item_userptr(current_item(menu))];
					if(fun) fun();
				}else{//multi valued menu
					menu_driver(menu, REQ_TOGGLE_ITEM);
				}
				break;
			case KEY_BACKSPACE:
				menu_driver(menu, REQ_BACK_PATTERN);
				break;
			case ',':
				menu_driver(menu, REQ_PREV_MATCH);
				redraw();
				break;
			case '.':
				menu_driver(menu, REQ_NEXT_MATCH);
				redraw();
				break;
			case KEY_MOUSE:
				menu_driver(menu, KEY_MOUSE);
				redraw();
				break;
			case 'a':
			case 'b':
			case 'c':
			case 'd':
			case 'e':
			case 'f':
			case 'g':
			case 'h':
			case 'i':
			case 'j':
			case 'k':
			case 'l':
			case 'm':
			case 'n':
			case 'o':
			case 'p':
			case 'q':
			case 'r':
			case 's':
			case 't':
			case 'u':
			case 'v':
			case 'w':
			case 'x':
			case 'y':
			case 'z':
				menu_driver(menu, key);
				redraw();
				break;
			case '?':
				i = new infoWidget("Menu Help","Arrow keys to move around: < > ^ v\nPageUp and PageDown to scroll.\nSearch by writing item name. Backspace works.\nUse , and . to jump between matching items.\nArrows clears the current search.\nEnter/Space selects the highlighted item.");
				break;
			default:
				return false;
		}
		return true;
	}
