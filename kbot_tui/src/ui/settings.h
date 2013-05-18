#ifndef SETTINGS_H
#define SETTINGS_H

#include "ui.h"

class setting {
private:
	string name, desc;
public:
	setting(string name, string description, initializer_list<settingValues>);
	setting(string name, string description, initializer_list<setting> subList);
  ~setting();
}

class settingsTab : public tab {
private:
	WINDOW *mainW, *descW;
  PANEL *mainP, *descP;
	string desc,name;

	void redraw();
	void redrawDescription();
public:
	settingsTab();
	~settingsTab();

	void update();

	bool input(int key);
	void add(setting);
};

#endif /* SETTINGS_H */
