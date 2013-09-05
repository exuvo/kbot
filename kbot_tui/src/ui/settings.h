#ifndef SETTINGS_H
#define SETTINGS_H

#include "ui.h"
#include <boost/any.hpp>

template <class T> class settingValue{
private:
public:
	string desc;
	T value;
};

class setting {
private:
	string name, desc, ROS_URL;
	function<void()> callback;
public:
	setting(string name, string description, string ROS_URL);
  setting(string name, string description, string ROS_URL, function<void()> apply);
  ~setting();
	virtual bool input(int key);
	virtual void load();
};

template <typename T> class settingChoice : setting{
private:
	settingValue<T> *value;
  vector<settingValue<T>> values;	
public:
	settingChoice(string name, string description, string ROS_URL, initializer_list<settingValue<T>>, function<void()> apply);
  ~settingChoice();
	bool input(int key);
};

template <typename T> class settingNum: setting{
private:
	int value, min, max;
public:
  settingNum(string name, string description, string ROS_URL, int min, int max, function<void()> apply);
	~settingNum();
	bool input(int key);
};

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
