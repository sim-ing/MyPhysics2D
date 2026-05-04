#pragma once
#define SDL_MAIN_HANDELED
#include <SDL2/SDL.h>
#include <SDL2/SDL_image.h>
#include <Eigen/Dense>
#include <iostream>
#include <vector>

struct Color
{
	float r, g, b, a;

	Color(float x = 255, float y = 255, float z = 255, float w = 255);
	
};

class Renderer
{
public:
	int w, h;

	SDL_Window* window;

	SDL_Renderer* renderer;

	Renderer(const char* name, int width = 640, int height = 480);

	~Renderer();

	//”√”⁄ª≠œﬂ
	void drawL(Eigen::Vector2f v0, Eigen::Vector2f v1, Color col);

	void drawP(Eigen::Vector2f v, Color col);

	void drawR(Eigen::Vector2f v, int w, int h, Color col);

	void fillR(Eigen::Vector2f v, int w, int h, Color col);

	void rendT(SDL_Texture* texture, Eigen::Vector2f v, int w, int h);

	void drawTr(std::array<Eigen::Vector2f,3> ver, Color col);

	void drawPl(std::vector<Eigen::Vector2f> ver, Color col);

	void clear(Color col = { 255, 255, 255, 255 });

	void present();

	void delay(const float& dt);
	
};

class Texture
{
public:

	SDL_Surface* surface;

	SDL_Texture* texture;

	Texture();

	~Texture();
	
	bool loadBMP(SDL_Renderer* renderer, const char* file_name);
	
};

bool init();
