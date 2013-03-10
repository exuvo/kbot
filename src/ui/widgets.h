#ifndef WIDGETS_H
#define WIDGETS_H

#include <menu.h>
#include <functional>
#include <stdint.h>

class infoWidget {
	std::string name,desc;
	WINDOW *w;
	PANEL *p;

public:
	infoWidget(std::string name,std::string desc);
	~infoWidget();
};

typedef std::tuple <std::string, std::string, std::function<void()>> menuItem;
class menuWidget {
	std::string name;
	std::vector<std::string> names, desc;
	std::vector<std::function<void()>> callbacks;
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
	menuWidget(WINDOW *win, std::string name, std::initializer_list<menuItem> list);
	menuWidget(WINDOW *win, std::string name, int border);
	menuWidget(WINDOW *win, std::string name, int height, int width);
	~menuWidget();
	void add(std::string name, std::string description, std::function<void()> callback);
	void add(menuItem item);
	void update();
	bool input(int &key);
};

#endif /* WIDGETS_H */
