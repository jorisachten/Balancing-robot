#ifndef JOYPAD_CLIENT
#define JOYPAD_CLIENT
	int ask_joypad_update(char ip[]);
	int get_ax(char ax_number);
	char get_btn(char btn_number);
	char get_ax_count();
	char get_btn_count();
#endif
