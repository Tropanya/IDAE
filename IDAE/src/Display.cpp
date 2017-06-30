#include "Display.h"

CDisplay::CDisplay(const char* title, int width, int height)
{
	m_Title = title;
	m_Width = width;
	m_Height = height;
	if (!init())
		glfwTerminate();

	for (int i = 0; i < MAX_KEYS; i++)
	{
		m_Keys[i] = false;
		m_KeyState[i] = false;
		m_KeyTyped[i] = false;
	}

	for (int i = 0; i < MAX_BUTTONS; i++)
	{
		m_MouseButtons[i] = false;
		m_MouseState[i] = false;
		m_MouseClicked[i] = false;
	}
}

CDisplay::~CDisplay()
{
	glfwTerminate();
}

bool CDisplay::init()
{
	if (!glfwInit())
	{
		std::cout << "Failde to initialize GLFW!" << std::endl;
		return false;
	}

	m_Window = glfwCreateWindow(m_Width, m_Height, m_Title, NULL, NULL);

	if (!m_Window)
	{
		std::cout << "Failed to create window!" << std::endl;
		return false;
	}

	glfwMakeContextCurrent(m_Window);
	glfwSetWindowUserPointer(m_Window, this);
	glfwSetFramebufferSizeCallback(m_Window, window_resize);
	glfwSetKeyCallback(m_Window, key_callback);
	glfwSetMouseButtonCallback(m_Window, mouse_button_callback);
	glfwSetCursorPosCallback(m_Window, cursor_position_callback);
	glfwSwapInterval(0.0);

	if (glewInit() != GLEW_OK)
	{
		std::cout << "Failde to initialize GLEW!" << std::endl;
		return false;
	}

	std::cout << "OpenGL: " << glGetString(GL_VERSION) << std::endl;

	return true;
}

void CDisplay::Clear() const
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

void CDisplay::Update()
{
	for (int i = 0; i < MAX_KEYS; i++)
		m_KeyTyped[i] = m_Keys[i] && !m_KeyState[i];

	for (int i = 0; i < MAX_BUTTONS; i++)
		m_MouseClicked[i] = m_MouseButtons[i] && !m_MouseState[i];

	memcpy(m_KeyState, m_Keys, MAX_KEYS * sizeof(bool));
	memcpy(m_MouseState, m_MouseButtons, MAX_BUTTONS * sizeof(bool));

	GLenum error = glGetError();
	if (error != GL_NO_ERROR)
		std::cout << "OpenGL ERROR: " << error << std::endl;

	glfwPollEvents();
	glfwSwapBuffers(m_Window);
}

bool CDisplay::Closed() const
{
	return glfwWindowShouldClose(m_Window) == 1;
}

bool CDisplay::IsKeyPressed(unsigned int keyCode) const
{
	if (keyCode >= MAX_KEYS)
		return false;

	return m_Keys[keyCode];
}

bool CDisplay::IsKeyTyped(unsigned int keyCode) const
{
	if (keyCode >= MAX_KEYS)
		return false;

	return m_KeyTyped[keyCode];
}

bool CDisplay::IsMouseButtonPressed(unsigned int mouseButton) const
{
	if (mouseButton >= MAX_BUTTONS)
		return false;

	return m_MouseButtons[mouseButton];
}

bool CDisplay::IsMouseButtonClicked(unsigned int mouseButton) const
{
	if (mouseButton >= MAX_BUTTONS)
		return false;

	return m_MouseClicked[mouseButton];
}

void CDisplay::GetMousePosition(double& x, double& y) const
{
	x = m_MX;
	y = m_MY;
}

void window_resize(GLFWwindow* window, int width, int height)
{
	glViewport(0, 0, width, height);
	CDisplay* win = (CDisplay*)glfwGetWindowUserPointer(window);
	win->m_Width = width;
	win->m_Height = height;
}

void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
	CDisplay* win = (CDisplay*)glfwGetWindowUserPointer(window);
	win->m_Keys[key] = action != GLFW_RELEASE;
}

void mouse_button_callback(GLFWwindow* window, int button, int action, int mods)
{
	CDisplay* win = (CDisplay*)glfwGetWindowUserPointer(window);
	win->m_MouseButtons[button] = action != GLFW_RELEASE;
}

void cursor_position_callback(GLFWwindow* window, double xPos, double yPos)
{
	CDisplay* win = (CDisplay*)glfwGetWindowUserPointer(window);
	win->m_MX = xPos;
	win->m_MY = yPos;
}