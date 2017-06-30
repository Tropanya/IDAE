#ifndef DISPLAY_H
#define DISPLAY_H

#include <iostream>

#include <GLEW/glew.h>
#include <GLFW/glfw3.h>

#define MAX_KEYS	1024
#define MAX_BUTTONS 32

class CDisplay
{
private:
	int m_Width, m_Height;
	const char* m_Title;
	GLFWwindow* m_Window;
	bool m_Closed;

	bool m_Keys[MAX_KEYS];
	bool m_KeyState[MAX_KEYS];
	bool m_KeyTyped[MAX_KEYS];
	bool m_MouseButtons[MAX_BUTTONS];
	bool m_MouseState[MAX_BUTTONS];
	bool m_MouseClicked[MAX_BUTTONS];
	double m_MX, m_MY;
private:
	bool init();
	friend void window_resize(GLFWwindow* window, int width, int height);
	friend void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods);
	friend void mouse_button_callback(GLFWwindow* window, int button, int action, int mods);
	friend void cursor_position_callback(GLFWwindow* window, double xPos, double yPos);
public:
	CDisplay(const char* title, int width, int height);
	~CDisplay();

	void Clear() const;
	void Update();
	bool Closed() const;

	bool IsKeyPressed(unsigned int keyCode) const;
	bool IsKeyTyped(unsigned int keyCode) const;
	bool IsMouseButtonPressed(unsigned int mouseButton) const;
	bool IsMouseButtonClicked(unsigned int mouseButton) const;
	void GetMousePosition(double& x, double& y) const;

	inline int GetWidth() const { return m_Width; }
	inline int GetHeight() const { return m_Height; }
};

#endif // DISPLAY_H