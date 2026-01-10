#include <SDL3/SDL.h>
#include <SDL3_image/SDL_image.h>
#include <vector>
#include <random>

const SDL_Rect windowRect{ 0, 0, 1280, 720 };

class boids
{
public:
	float w;
	float h;
	int count;
	SDL_FColor defaultColor;
	std::vector<float> midX;
	std::vector<float> midY;
	std::vector<float> degree;
	std::vector<float> v0;
	std::vector<float> vx;
	std::vector<float> vy;
	std::vector<SDL_Vertex> vertices;
	boids(int boidC, int w = 20, int h = 40, SDL_FColor defaultColor = { 1.0f, 0.84f, 0.0f, 1.0f });
	void updateVectices(float dt);
	void makeVertices();
private:

};

boids::boids(int boidC, int w, int h, SDL_FColor defaultColor)
{
	SDL_Vertex temp{};
	this->w = w;
	this->h = h;
	this->count = boidC;
	this->defaultColor = defaultColor;
	temp.color = defaultColor;
	
	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_int_distribution<int> genX(this->w, windowRect.w - this->w);
	std::uniform_int_distribution<int> genY(this->h, windowRect.h - this->h);
	std::uniform_int_distribution<int> genDegree(0, 359);
	
	for (int i = 0; i < count; i++)
	{
		midX.push_back(genX(gen));
		midY.push_back(genY(gen));
		degree.push_back(genDegree(gen));
		v0.push_back(5.f);
		vx.push_back(v0[i] * SDL_cosf(degree[i] * (SDL_PI_F / 180.f)));
		vy.push_back(v0[i] * SDL_sinf(degree[i] * (SDL_PI_F / 180.f)));
		for (int j = 0; j < 3; j++)
		{
			this->vertices.push_back(temp);	// init vertices
		}
	}
}

void boids::updateVectices(float dt)
{
	for (int i = 0; i < this->count; i++)
	{
		this->midX[i] += vx[i] * dt;
		this->midY[i] += vy[i] * dt;

		if (midX[i] > windowRect.w)
		{
			midX[i] -= windowRect.w;
		}
		if (midX[i] < 0)
		{
			midX[i] += windowRect.w;
		}
		if (midY[i] > windowRect.h)
		{
			midY[i] -= windowRect.h;
		}
		if (midY[i] < 0)
		{
			midY[i] += windowRect.h;
		}
	}

	return;
}

void boids::makeVertices()
{

	for (int i = 0; i < this->count; i++)
	{
		float theta = degree[i] * (SDL_PI_F / 180.f);

		this->vertices[3*i].position.x = this->midX[i] + this->h * SDL_cosf(theta);							// P1x
		this->vertices[3*i].position.y = this->midY[i] + this->h * SDL_sinf(theta);							// P1y
		vertices[3 * i].color = { 1.0, 0.0, 0.0, 1.0 };

		this->vertices[3*i+1].position.x = this->midX[i] + this->w / 2 * SDL_cosf(theta + (SDL_PI_F / 2.f));	// P2x
		this->vertices[3*i+1].position.y = this->midY[i] + this->w / 2 * SDL_sinf(theta + (SDL_PI_F / 2.f));	// P2y
		vertices[3 * i + 1].color = { 0.0, 1.0, 0.0, 1.0 };

		this->vertices[3*i+2].position.x = this->midX[i] + this->w / 2 * SDL_cosf(theta + (SDL_PI_F / 2.f * 3.f));	// P3x
		this->vertices[3*i+2].position.y = this->midY[i] + this->w / 2 * SDL_sinf(theta + (SDL_PI_F / 2.f * 3.f));	// p3y
		vertices[3 * i + 2].color = { 0.0, 0.0, 1.0, 1.0 };
	}

	return;
}


int main(int argc, char** argv)
{
	float dt = 0.1;
	boids Boids(10);
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

		Boids.updateVectices(dt);
		Boids.makeVertices();

		SDL_RenderClear(renderer);
		SDL_RenderGeometry(renderer, NULL, Boids.vertices.data(), Boids.count * 3, NULL, 0);
		SDL_RenderPresent(renderer);
	}
	
	

	return 0;
}

