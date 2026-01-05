#include <SDL3/SDL.h>
#include <SDL3_image/SDL_image.h>
#include <vector>
#include <random>

const SDL_Rect windowRect{ 0, 0, 1280, 720 };

class boids
{
public:
	int w;
	int h;
	SDL_FColor defaultColor;
	std::vector<int> midX;
	std::vector<int> midY;
	std::vector<int> degree;
	std::vector<SDL_Vertex> vertices;
	boids(int boidC, int w = 20, int h = 20, SDL_FColor defaultColor = { 1.0f, 0.84f, 0.0f, 1.0f });
private:

};

boids::boids(int boidC, int w, int h, SDL_FColor defaultColor)
{
	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_int_distribution<int> midX(50, 1230);
	std::uniform_int_distribution<int> midY(50, 770);
	std::uniform_int_distribution<int> degree(0, 359);
	this->w = w;
	this->h = h;
	this->defaultColor = defaultColor;
	for (int i = 0; i < 10; i++)
	{
		printf("%d\n", midX(gen));
	}
}


int main(int argc, char** argv)
{
	boids Boids(1);
	SDL_Window* window = NULL;
	SDL_Renderer* renderer = NULL;
	SDL_Event event;

	bool isRunning = false;

	SDL_SetAppMetadata("Boids Simulator", "on develop", "CSY");

	if (!SDL_Init(SDL_INIT_VIDEO))
	{
		SDL_Log("ERROR: %s", SDL_GetError());
		return 0;
	}
	if (!SDL_Init(SDL_INIT_EVENTS))
	{
		SDL_Log("ERROR: %s", SDL_GetError());
		return 0;
	}

	if (!SDL_CreateWindowAndRenderer("Boids Simulator", windowRect.w, windowRect.h, SDL_WINDOW_RESIZABLE, &window, &renderer))
	{
		SDL_Log("ERROR: %s", SDL_GetError());
		return SDL_APP_FAILURE;
	}

	SDL_SetRenderLogicalPresentation(renderer, windowRect.w, windowRect.h, SDL_LOGICAL_PRESENTATION_STRETCH);

	isRunning = true;

	while (isRunning)
	{
		SDL_PollEvent(&event);
		if (event.type == SDL_EVENT_QUIT)
		{
			isRunning = false;
		}


		// SDL_RenderGeometry(renderer, NULL, vertices, 3, NULL, 0);
		SDL_RenderPresent(renderer);
	}
	


	return 0;
}

