#ifndef WIDGETS_H
#define WIDGETS_H

#include <menu.h>
#include <functional>
#include <stdint.h>

using namespace std;

class infoWidget {
	string name,desc;
	WINDOW *w;
	PANEL *p;

public:
	infoWidget(string name,string desc);
	~infoWidget();
};

typedef tuple <string, string, function<void()>> menuItem;
class menuWidget {
	string name;
	vector<string> names, desc;
	vector<function<void()>> callbacks;
	MENU *menu = nullptr;
	ITEM **items = nullptr;
	unsigned int size;
	WINDOW *w;
	PANEL *p;
	infoWidget *i= nullptr;

	void rebuild();
	void clean();
	void redraw();

public:
	menuWidget(WINDOW *win, string name, initializer_list<menuItem> list);
	menuWidget(WINDOW *win, string name, int border);
	menuWidget(WINDOW *win, string name, int height, int width);
	~menuWidget();
	void add(string name, string description, function<void()> callback);
	void add(menuItem item);
	void update();
	bool input(int &key);
};

#endif /* WIDGETS_H */
