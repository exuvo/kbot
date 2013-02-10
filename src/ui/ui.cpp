#include <ncurses.h>


UI::UI(){
	std::thread t(run);
	//t.detach();
}

void UI::run(){
	std::cout << "Starting UI thread" << endl;
	std::this_thread::sleep_for(std::chrono::seconds(2));
}
