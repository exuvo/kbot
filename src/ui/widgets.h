#ifndef WIDGETS_H
#define WIDGETS_H

#include <menu.h>

class infoWidget {
	std::string name,desc;
	WINDOW *w;
	PANEL *p;

public:
	infoWidget(std::string name,std::string desc);
	~infoWidget();
};

class menuWidget {
	std::string name;
	std::vector<std::string> names, desc;
	MENU *menu = nullptr;
	ITEM **items = nullptr;
	unsigned int size;
	WINDOW *w;
	PANEL *p;
	infoWidget *i= nullptr;

	void rebuild();
	void clean();

public:
	menuWidget(WINDOW *win, std::string name, int border);
	menuWidget(WINDOW *win, std::string name, int height, int width);
	~menuWidget();
	void add(std::string name, std::string descript);
	void update();
	bool input(int &key);
};












#endif /* WIDGETS_H */
