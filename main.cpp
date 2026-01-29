#include <SDL3/SDL.h>
#include <vector>
#include <random>

// 커밋 메세지: 가속도, 힘 추가, 이동방향 == 앵글
// TODO topX, Y 만들어서 밀고 땡기는게 더 정확하게 만들기

const SDL_Rect windowRect{ 0, 0, 1280, 720 };
const float dt = 0.1f;

class boids
{
public:
	float w = 10.f;
	float h = 20.f;
	int count;
	float v0 = 1.f;
	SDL_FColor defaultColor = { 1.0f, 0.84f, 0.0f, 1.0f };
	std::vector<float> midX;
	std::vector<float> midY;
	std::vector<float> angle;
	
	std::vector<float> vx;
	std::vector<float> vy;
	std::vector<float> ax;
	std::vector<float> ay;
	std::vector<SDL_Vertex> vertices;
	std::vector<float> distances;
	std::vector<float> distancesX;
	std::vector<float> distancesY;
	std::vector<bool> isMigrating;    // 현재 무리를 옮기는 중인가? // GEMINI #2
	std::vector<float> migrateTimer;  // 이동 지속 시간 // GEMINI #2
	boids(int boidC);
	float getDistance(float x1, float y1, float x2, float y2);
	void updateDistances();
	void updateAcceleration();
	void updateAlign();
	void updateMigration(); // 이탈 및 전입 로직 // GEMINI #2
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
		vx.push_back(v0 * SDL_cosf(angle[i]));
		vy.push_back(v0 * SDL_sinf(angle[i]));
		ax.push_back(0.f);
		ay.push_back(0.f);
		isMigrating.push_back(false);
		migrateTimer.push_back(0.0f);
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
	/* GEMINI #3 */
	// --- 파라미터 조정 포인트 ---
	const float sigma = 25.0f;    // 척력 범위를 좁힘 (기존 25.0 -> 20.0)
	const float epsilon = 0.7f;   // 힘의 강도를 약간 높여 응집력 강화
	const float cutoff = 100.0f;  // 감지 반경을 크게 확대 (멀리 있는 보이드들도 무리에 합류)
	const float accelLimit = 0.5f; // 가속도 제한을 약간 완화하여 기동성 부여
	// -------------------------

	std::fill(ax.begin(), ax.end(), 0.0f);
	std::fill(ay.begin(), ay.end(), 0.0f);

	for (int i = 0; i < count; i++)
	{
		for (int j = 0; j < count; j++)
		{
			if (i == j) continue;

			float r = distances[count * i + j];

			// Cutoff가 커질수록 더 많은 보이드가 서로 인력을 주고받아 무리를 형성합니다.
			if (r > 2.0f && r < cutoff)
			{
				float s_r = sigma / r;
				float s_r6 = powf(s_r, 6);
				float s_r12 = s_r6 * s_r6;

				// L-J Force 계산
				float forceMag = (24.0f * epsilon / (r * r)) * (2.0f * s_r12 - s_r6);

				// distancesX[idx] = midX[i] - midX[j] 이므로
				// forceMag이 음수일 때(인력) i는 j 방향으로 끌려갑니다.
				ax[i] += forceMag * distancesX[count * i + j];
				ay[i] += forceMag * distancesY[count * i + j];
			}
		}

		// 가속도 클램핑
		float currentAccMag = sqrtf(ax[i] * ax[i] + ay[i] * ay[i]);
		if (currentAccMag > accelLimit) {
			ax[i] = (ax[i] / currentAccMag) * accelLimit;
			ay[i] = (ay[i] / currentAccMag) * accelLimit;
		}
	}
}

void boids::updateAlign()
{
	/* GEMINI #4 */
	const float alignRadius = 150.0f;
	const float alignStrength = 0.5f; // 강도를 조금 더 높여서 반응성 개선

	for (int i = 0; i < count; i++)
	{
		float avgVx = 0.0f;
		float avgVy = 0.0f;
		int neighborCount = 0;

		if (isMigrating[i]) continue;
		for (int j = 0; j < count; j++)
		{
			if (i != j && distances[count * i + j] < alignRadius)
			{
				avgVx += vx[j];
				avgVy += vy[j];
				neighborCount++;
			}
		}

		if (neighborCount > 0)
		{
			avgVx /= (float)neighborCount;
			avgVy /= (float)neighborCount;

			// 주변의 평균 속도 방향과 내 현재 속도 사이의 차이를 가속도로 전환
			// 이 가속도가 updateVectices로 넘어가서 방향을 바꿉니다.
			ax[i] += (avgVx - vx[i]) * alignStrength;
			ay[i] += (avgVy - vy[i]) * alignStrength;
		}
	}
}

void boids::updateMigration()
{
	const float dt = 0.01f;	// dt = 0.01 이 가장 좋은 값
	const float migrationChance = 0.004f; // 이탈 발생 확률 (낮게 설정)
	const float migrationForce = 1.2f;    // 새로운 무리로 향하는 힘

	for (int i = 0; i < count; i++) {
		// 1. 일정 확률로 이탈 시작 (한 번에 너무 많지 않게)
		if (!isMigrating[i] && (float)rand() / RAND_MAX < migrationChance) {
			isMigrating[i] = true;
			migrateTimer[i] = 3.0f; // 3초 동안 다른 무리 탐색
		}

		if (isMigrating[i]) {
			migrateTimer[i] -= dt;

			// 2. 전방에 있는 다른 무리 찾기
			int targetIdx = -1;
			float minDist = 500.0f;

			for (int j = 0; j < count; j++) {
				if (i == j) continue;
				float d = distances[count * i + j];

				// 너무 가깝지 않고(현재 무리 탈피), 적당히 멀리 전방에 있는 보이드
				if (d > 150.0f && d < 400.0f) {
					// 내 이동 방향(vx, vy)과 상대 위치의 각도 확인 (내적 활용)
					float dot = (vx[i] * -distancesX[count * i + j] + vy[i] * -distancesY[count * i + j]);
					if (dot > 0) { // 전방에 위치함
						targetIdx = j;
						break;
					}
				}
			}

			// 3. 대상이 있다면 그쪽으로 강한 응집력 발생
			if (targetIdx != -1) {
				ax[i] += (-distancesX[count * i + targetIdx] / distances[count * i + targetIdx]) * migrationForce;
				ay[i] += (-distancesY[count * i + targetIdx] / distances[count * i + targetIdx]) * migrationForce;
			}

			if (migrateTimer[i] <= 0) isMigrating[i] = false;
		}
	}
}

void boids::updateVectices()
{

	/* GEMINI #1 */
	const float targetSpeed = 3.0f; // 원하는 이동 속도 (너무 빠르면 줄이세요)

	for (int i = 0; i < count; i++)
	{
		// 1. L-J 가속도 + Alignment 가속도 통합 적용
		vx[i] += ax[i] * dt;
		vy[i] += ay[i] * dt;

		// 2. 현재 속도 크기(속력) 계산
		float currentSpeed = SDL_sqrtf(vx[i] * vx[i] + vy[i] * vy[i]);

		if (currentSpeed > 0.01f)
		{
			// 3. 방향은 유지하되 속력만 targetSpeed로 고정 (추진력 핵심)
			vx[i] = (vx[i] / currentSpeed) * targetSpeed;
			vy[i] = (vy[i] / currentSpeed) * targetSpeed;
		}

		// 4. 위치 이동
		midX[i] += vx[i] * dt;
		midY[i] += vy[i] * dt;

		angle[i] = SDL_atan2f(vy[i], vx[i]);

		// 화면 래핑
		if (midX[i] > windowRect.w) midX[i] -= windowRect.w;
		if (midX[i] < 0) midX[i] += windowRect.w;
		if (midY[i] > windowRect.h) midY[i] -= windowRect.h;
		if (midY[i] < 0) midY[i] += windowRect.h;
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
		vertices[3 * i].color = { 1.0, 1.0, 0.0, 1.0 };
		
		vertices[3*i+1].position.x = midX[i] + w / 2 * SDL_cosf(theta + (SDL_PI_F / 2.f));			// P2x
		vertices[3*i+1].position.y = midY[i] + w / 2 * SDL_sinf(theta + (SDL_PI_F / 2.f));			// P2y
		vertices[3 * i + 1].color = { 0.0, 0.0, 1.0, 1.0 };

		vertices[3*i+2].position.x = midX[i] + w / 2 * SDL_cosf(theta + (SDL_PI_F / 2.f * 3.f));		// P3x
		vertices[3*i+2].position.y = midY[i] + w / 2 * SDL_sinf(theta + (SDL_PI_F / 2.f * 3.f));		// p3y
		vertices[3 * i + 2].color = { 0.0, 0.0, 1.0, 1.0 };
	}

	return;
}



int main(int argc, char** argv)
{
	boids Boids(100);
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
		Boids.updateMigration();
		Boids.updateAlign();

		Boids.updateVectices();
		Boids.makeVertices();

		SDL_RenderClear(renderer);
		SDL_RenderGeometry(renderer, NULL, Boids.vertices.data(), Boids.count * 3, NULL, 0);
		SDL_RenderPresent(renderer);
	}
	
	

	return 0;
}

