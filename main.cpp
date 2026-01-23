#define DELTA 0.01
#define MAXSPEED 0.001

#include <SDL3/SDL.h>
#include <SDL3_image/SDL_image.h>
#include <vector>
#include <random>

// 커밋 메세지: 가속도, 힘 추가, 이동방향 == 앵글
// TODO topX, Y 만들어서 밀고 땡기는게 더 정확하게 만들기

const SDL_Rect windowRect{ 0, 0, 1280, 720 };
const float dt = 0.1;

class boids
{
public:
	float w = 20.f;
	float h = 40.f;
	int count;
	SDL_FColor defaultColor = { 1.0f, 0.84f, 0.0f, 1.0f };
	std::vector<float> midX;
	std::vector<float> midY;
	std::vector<float> angle;
	std::vector<float> v0;
	std::vector<float> vx;
	std::vector<float> vy;
	std::vector<float> ax;
	std::vector<float> ay;
	std::vector<SDL_Vertex> vertices;
	std::vector<float> distances;
	std::vector<float> distancesX;
	std::vector<float> distancesY;
	boids(int boidC);
	float getDistance(float x1, float y1, float x2, float y2);
	void updateDistances();
	void updateAcceleration();
	void updateAlign();
	void updateVectices();
	void makeVertices();
private:

};

boids::boids(int boidC)
{
	SDL_Vertex temp{};
	count = boidC;
	temp.color = defaultColor;
	
	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_int_distribution<int> genX(w, windowRect.w - w);
	std::uniform_int_distribution<int> genY(h, windowRect.h - h);
	std::uniform_real_distribution<float> genangle(0.f, 2.f*SDL_PI_F);
	
	for (int i = 0; i < count; i++)
	{
		midX.push_back(genX(gen));
		midY.push_back(genY(gen));
		angle.push_back(genangle(gen));
		v0.push_back(1.f);
		vx.push_back(v0[i] * SDL_cosf(angle[i]));
		vy.push_back(v0[i] * SDL_sinf(angle[i]));
		ax.push_back(0.f);
		ay.push_back(0.f);
		for (int j = 0; j < 3; j++)
		{
			vertices.push_back(temp);	// init vertices
		}
	}
	
	distances.resize(count * count);
	distancesX.resize(count * count);
	distancesY.resize(count * count);
}

float boids::getDistance(float x1, float y1, float x2, float y2)
{
	float dx = x2 - x1;
	float dy = y2 - y1;
	float d = SDL_sqrtf(dx * dx + dy * dy);
	return d;
}

void boids::updateDistances()
{
	for (int i = 0; i < count; i++)
	{
		for (int j = 0; j < count; j++)
		{
			distances[count * i + j] = getDistance(midX[i], midY[i], midX[j], midY[j]);
			distancesX[count * i + j] = midX[i] - midX[j];
			distancesY[count * i + j] = midY[i] - midY[j];

		}
	}
	return;
}

void boids::updateAcceleration()
{
	for (int i = 0; i < count; i++)
	{
		ax[i] = 0.f;
		ay[i] = 0.f;
		for (int j = 0; j < count; j++)
		{
			if (i != j)
			{
				ax[i] -= distancesX[count * i + j] / powf(distances[count * i + j], 3.f) + distancesX[count * i + j] / powf(distances[count * i + j], 2.f);
				ay[i] -= distancesY[count * i + j] / powf(distances[count * i + j], 3.f) + distancesY[count * i + j] / powf(distances[count * i + j], 2.f);

			}
		}
	}
}

void boids::updateAlign()
{
	float averageSpeedx = 0;
	float averageSpeedy = 0;

	for (int i = 0; i < count; i++)
	{
		averageSpeedx += vx[i]; 
		averageSpeedy += vy[i];
	}
	averageSpeedx = averageSpeedx / float(count);
	averageSpeedy = averageSpeedy / float(count);

	for (int i = 0; i < count; i++)
	{
		if (vx[i] > averageSpeedx + DELTA || vx[i] < averageSpeedx - DELTA) // equal to vx[i] != averageSpeedx
		{
			if (vx[i] < averageSpeedx)
			{
				ax[i] += averageSpeedx * 0.1; // align speed
			}
			else if (vx[i] > averageSpeedx)
			{
				ax[i] -= averageSpeedx * 0.1; // align speed
			}
		}
		else
		{
			ax[i] = 0;
		}
		
		if (vy[i] > averageSpeedy + DELTA || vy[i] < averageSpeedy - DELTA) // equal to vy[i] != averageSpeedx
		{
			if (vy[i] < averageSpeedy)
			{
				ay[i] += averageSpeedy * 0.01; // align speed
			}
			else if (vy[i] > averageSpeedy)
			{
				ay[i] -= averageSpeedy * 0.01; // align speed
			}
		}
		else
		{
			ay[i] = 0;
		}
	}
	
}

void boids::updateVectices()
{
	for (int i = 0; i < count; i++)
	{
		vx[i] += ax[i] * dt;
		vy[i] += ay[i] * dt;
		if (vx[i] > MAXSPEED)
		{
			ax[i] = ax[i] / -10;
			ay[i] = ay[i] / -10;
		}
		midX[i] += vx[i] * dt;
		midY[i] += vy[i] * dt;
		
		angle[i] = SDL_atan2f(vy[i], vx[i]);

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

	for (int i = 0; i < count; i++)
	{
		float theta = angle[i];

		vertices[3*i].position.x = midX[i] + h * SDL_cosf(theta);									// P1x
		vertices[3*i].position.y = midY[i] + h * SDL_sinf(theta);									// P1y
		vertices[3 * i].color = { 1.0, 0.0, 0.0, 1.0 };		// It sets boids rainbow
		
		vertices[3*i+1].position.x = midX[i] + w / 2 * SDL_cosf(theta + (SDL_PI_F / 2.f));			// P2x
		vertices[3*i+1].position.y = midY[i] + w / 2 * SDL_sinf(theta + (SDL_PI_F / 2.f));			// P2y
		vertices[3 * i + 1].color = { 0.0, 1.0, 0.0, 1.0 };	// It sets boids rainbow

		vertices[3*i+2].position.x = midX[i] + w / 2 * SDL_cosf(theta + (SDL_PI_F / 2.f * 3.f));		// P3x
		vertices[3*i+2].position.y = midY[i] + w / 2 * SDL_sinf(theta + (SDL_PI_F / 2.f * 3.f));		// p3y
		vertices[3 * i + 2].color = { 0.0, 0.0, 1.0, 1.0 };	// It sets boids rainbow
	}

	return;
}



int main(int argc, char** argv)
{
	boids Boids(50);
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

		Boids.updateDistances();
		Boids.updateAcceleration();
		// Boids.updateAlign();

		Boids.updateVectices();
		Boids.makeVertices();

		SDL_RenderClear(renderer);
		SDL_RenderGeometry(renderer, NULL, Boids.vertices.data(), Boids.count * 3, NULL, 0);
		SDL_RenderPresent(renderer);
	}
	
	

	return 0;
}

