#include "main.h"


int main(int argc, char** args)
{

	//初始化sdl
	if (!init()) return -1;

	//创建窗体,创建窗体渲染器
	Renderer ren("Fenster", 640, 480);

	Physics world;
	



	int count;
	
	float phi = PI/1000000;
	bool run = true;
	SDL_Event ev;
	while (run)//主循环
	{
		ren.clear({ 0, 0, 0 });
		while (SDL_PollEvent(&ev))//抓取事件
		{
			switch (ev.type)//事件处理
			{
			case SDL_QUIT:
				run = false;
				break;
			case SDL_MOUSEBUTTONDOWN:
				Eigen::Vector2f pos({ ev.button.x, ren.h - ev.button.y });
				world.createRB(pos);

				break;
			}
		}
		
		count = world.getBodySize();
		//std::cout << "size: " << count << std::endl;

		ren.drawPl(world.getCol(0)->getWVertices(), { 255, 255, 0 });
		//没撞
		if (world.aT())
		{
			for (int i = 1; i < count; i++)
			{
				ren.drawPl(world.getCol(i)->getWVertices(), { 255, 255, 255 });
			}
		}
		//撞了
		else
		{
			for (int i = 1; i < count; i++)
			{
				ren.drawPl(world.getCol(i)->getWVertices(), { 255, 255, 0 });
			}
		}

		
		ren.present();
		
		ren.delay(1);
		
		world.step();
	
	}

	return 0;
}
