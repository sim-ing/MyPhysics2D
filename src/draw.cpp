#include "draw.h"


Color::Color(float x, float y, float z, float w )
	:r(x), g(y), b(z), a(w) {
}

Renderer::Renderer(const char* name, int width, int height) :w(width), h(height)
{
	window = SDL_CreateWindow(name, SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, w, h, SDL_WINDOW_SHOWN);
	if (!window) {
		std::cout << "´´½¨´°ÌåÊ§°Ü" << SDL_GetError() << std::endl;
		system("pause");
	}
	renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
	if (!renderer) {
		std::cout << "´´½¨äÖÈ¾Æ÷Ê§°Ü" << SDL_GetError() << std::endl;
		system("pause");
	}
}
Renderer::~Renderer()
{
	SDL_DestroyRenderer(renderer);
	renderer = NULL;
	SDL_DestroyWindow(window);
	window = NULL;
}
void Renderer::drawL(Eigen::Vector2f v0, Eigen::Vector2f v1, Color col)
{
	SDL_SetRenderDrawColor(renderer, col.r, col.g, col.b, col.a);
	SDL_RenderDrawLine(renderer, v0[0], h - v0[1], v1[0], h - v1[1]);
}
void Renderer::drawP(Eigen::Vector2f v, Color col)
{
	SDL_SetRenderDrawColor(renderer, col.r, col.g, col.b, col.a);
	SDL_RenderDrawPoint(renderer, v[0], h - v[1]);
}
void Renderer::drawR(Eigen::Vector2f v, int wr, int hr, Color col)
{
	SDL_Rect dest;
	dest.x = v[0];
	dest.y = h - v[1];
	dest.w = wr;
	dest.h = hr;
	SDL_SetRenderDrawColor(renderer, col.r, col.g, col.b, col.a);
	SDL_RenderDrawRect(renderer, &dest);
}
void Renderer::fillR(Eigen::Vector2f v, int w, int h, Color col)
{
	SDL_Rect dest;
	dest.x = v[0];
	dest.y = v[1];
	dest.w = w;
	dest.h = h;
	SDL_SetRenderDrawColor(renderer, col.r, col.g, col.b, col.a);
	SDL_RenderFillRect(renderer, &dest);
}
void Renderer::rendT(SDL_Texture* texture, Eigen::Vector2f v, int w, int h)
{
	SDL_Rect dest;
	dest.x = v[0];
	dest.y = v[1];
	dest.w = w;
	dest.h = h;
	SDL_RenderCopy(renderer, texture, NULL, &dest);
}
void Renderer::drawTr(std::array<Eigen::Vector2f, 3> ver, Color col)
{
	SDL_SetRenderDrawColor(renderer, col.r, col.g, col.b, col.a);
	SDL_RenderDrawLine(renderer, ver[0][0], h - ver[0][1], ver[1][0], h - ver[1][1]);
	SDL_RenderDrawLine(renderer, ver[0][0], h - ver[0][1], ver[2][0], h - ver[2][1]);
	SDL_RenderDrawLine(renderer, ver[2][0], h - ver[2][1], ver[1][0], h - ver[1][1]);
}
void Renderer::drawPl(std::vector<Eigen::Vector2f> ver, Color col)
{
	SDL_SetRenderDrawColor(renderer, col.r, col.g, col.b, col.a);
	int j;
	int size = ver.size();
	for (int i = 0; i < size; i++)
	{
		j = (i + 1) % size;
		SDL_RenderDrawLine(renderer, ver[i][0], h - ver[i][1], ver[j][0], h - ver[j][1]);
	}
}
void Renderer::clear(Color col )
{
	SDL_SetRenderDrawColor(renderer, col.r, col.g, col.b, col.a);
	SDL_RenderClear(renderer);
}
void Renderer::present()
{
	SDL_RenderPresent(renderer);
}
void Renderer::delay(const float& dt)
{
	SDL_Delay(dt);
}


Texture::Texture()
{
	SDL_Surface* surface = NULL;
	SDL_Texture* Texture = NULL;
}
Texture::~Texture()
{
	SDL_FreeSurface(surface);
	surface = NULL;
	SDL_DestroyTexture(texture);
}
bool Texture::loadBMP(SDL_Renderer* renderer, const char* file_name)
{
	surface = SDL_LoadBMP(file_name);
	if (!surface) {
		std::cout << "Í¼Æ¬¼ÓÔØ´íÎó" << SDL_GetError() << std::endl;
		return false;
	}
	texture = SDL_CreateTextureFromSurface(renderer, surface);
}

bool init()
{
	if (SDL_Init(SDL_INIT_EVERYTHING) < 0) {
		std::cout << "SDL³õÊ¼»¯´íÎó" << SDL_GetError() << std::endl;
		return false;
	}
	if (IMG_Init(IMG_INIT_JPG) < 0) {
		std::cout << "SDLimage³õÊ¼»¯´íÎó" << SDL_GetError() << std::endl;
		return false;
	}
}