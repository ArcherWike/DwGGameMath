#include "dwgSimpleGraphics.h"

#include "dwgSimpleGraphics.h"
#include <assert.h>
#include <vector>
#include <windows.h>

float RandomRange(float min, float max)
{
	return (rand() % 1001) / 1000.0f * (max - min) + min;
}

float RandomSign()
{
	return rand() % 2 == 0 ? -1.0f : 1.0f;
}

Vector3 RandomPosition()
{
	return Vector3(RandomRange(-10.0f, 10.0f), RandomRange(-10.0f, 10.0f), 0.0f);
}

Vector3 RandomVector2D()
{
	const float x = RandomRange(-1.0f, 1.0f);
	const float y = sqrt(1.0f - x * x) * RandomSign();
	return Vector3(x, y, 0.0f);
}

void LimitVector(Vector3& v, float bounds)
{
	const float limit = 12.0f;
	const float limitTwice = 2 * limit;

	if (v.getX() < -limit)
	{
		v.setX(v.getX() + limitTwice);
	}

	if (v.getY() < -limit)
	{
		v.setY(v.getY() + limitTwice);
	}

	if (v.getX() > limit)
	{
		v.setX(v.getX() - limitTwice);
	}

	if (v.getY() > limit)
	{
		v.setY(v.getY() - limitTwice);
	}

	v.setZ(0.0f);
}


class Flock
{
public:
	Flock(int numPrey, int numPredators)
		: m_numAll(numPrey + numPredators)
		, m_numPrey(numPrey)
		, m_numPredators(numPredators)
	{
		m_data = new Vector3[m_numAll * 2];
		m_positions = m_data;
		m_velocities = m_data + m_numAll;

		// random init
		for (int i = 0; i < m_numAll; ++i)
		{
			m_positions[i] = RandomPosition();
			m_velocities[i] = RandomVector2D();
		}
	}

	~Flock()
	{
		delete[] m_data;
	}

	void simulate(float dt)
	{
		const float maxPreySpeed = 4.0f;
		const float maxPredatorSpeed = 6.0f;
		const float maxPreyDistance = 1.0f;
		const float maxPredatorDistance = 5.0f;

		// iterate all prey boids (steering for prey)
		for (int i = 0; i < m_numPrey; ++i)
		{
			Vector3 avgPosition = Vector3(0.0f);
			Vector3 avgVelocity = Vector3(0.0f);
			Vector3 avgPush = Vector3(0.0f);
			int numNeighbours = 0;

			// iterate over all other prey
			for (int j = 0; j < m_numPrey; ++j)
			{
				if (i == j)
					continue;

				const Vector3 dirToNeighbour = m_positions[j] - m_positions[i];
				float distanceToNeighbour = length(dirToNeighbour);
				if (distanceToNeighbour < 0.001f)
					distanceToNeighbour = 0.001f;

				if (distanceToNeighbour < maxPreyDistance)
				{
					avgPosition += m_positions[j];
					avgVelocity += m_velocities[j];
					avgPush -= dirToNeighbour / (distanceToNeighbour * distanceToNeighbour);
					numNeighbours += 1;
				}
			}

			// iterate over all predators
			for (int j = m_numPrey; j < m_numAll; ++j)
			{
				const Vector3 dirToPredator = m_positions[j] - m_positions[i];
				float distanceToPredator = length(dirToPredator);
				if (distanceToPredator < 0.001f)
					distanceToPredator = 0.001f;

				if (distanceToPredator < maxPredatorDistance)
				{
					avgPush -= 50.0f * dirToPredator / (distanceToPredator * distanceToPredator);
					numNeighbours += 1;
				}
			}

			if (numNeighbours == 0)
				continue;

			avgPosition /= numNeighbours;
			avgVelocity /= numNeighbours;
			avgPush /= numNeighbours;

			Vector3 steer = Vector3(0.0f);

			// coherence
			Vector3 dirToAvgPosition = avgPosition - m_positions[i];
			if (length(dirToAvgPosition) > 0.001f)
			{
				steer += 0.5f * normalize(dirToAvgPosition);
			}

			// alignment
			steer += 0.7f * avgVelocity;

			// separation
			steer += 1.0f * avgPush;

			// apply steering
			m_velocities[i] += steer;

			// clamp velocity to max speed
			if (length(m_velocities[i]) > maxPreySpeed)
			{
				m_velocities[i] = normalize(m_velocities[i]) * maxPreySpeed;
			}
		}

		// iterate over all predators (steering for predators)
		int numPrey = m_numPrey;
		for (int i = numPrey; i < m_numAll; ++i)
		{
			int closestPreyIndex = -1;
			float closestPreyDistance = 9999.0f;

			// iterate over all prey
			for (int j = 0; j < numPrey; ++j)
			{
				const Vector3 dirToPrey = m_positions[j] - m_positions[i];
				float distanceToPrey = length(dirToPrey);
				if (distanceToPrey < closestPreyDistance)
				{
					closestPreyDistance = distanceToPrey;
					closestPreyIndex = j;
				}
			}

			Vector3 steer = Vector3(0.0f);

			if (closestPreyIndex != -1)
			{
				steer += 0.4f * normalize(m_positions[closestPreyIndex] - m_positions[i]);
			}

			if (closestPreyDistance < 0.04f)
			{
				std::swap(m_positions[closestPreyIndex], m_positions[m_numPrey - 1]);
				std::swap(m_velocities[closestPreyIndex], m_velocities[m_numPrey - 1]);
				m_numPrey -= 1;
			}

			// apply steering
			m_velocities[i] += steer;

			// clamp velocity to max speed
			if (length(m_velocities[i]) > maxPredatorSpeed)
			{
				m_velocities[i] = normalize(m_velocities[i]) * maxPredatorSpeed;
			}
		}

		// apply velocity to boids
		for (int i = 0; i < m_numAll; ++i)
		{
			m_positions[i] += m_velocities[i] * dt;
			LimitVector(m_positions[i], 10.0f);
		}
	}

	void draw()
	{
		for (int i = 0; i < m_numPrey; ++i)
		{
			dwgDebugSphere(m_positions[i], Vector3(0.04f), Vector3(0.2f, 1.0f, 1.0f));
			dwgDebugLine(m_positions[i], m_positions[i] + m_velocities[i] * 0.04f, Vector3(0.2f, 1.0f, 1.0f));
		}

		for (int i = m_numPrey; i < m_numAll; ++i)
		{
			dwgDebugSphere(m_positions[i], Vector3(0.04f), Vector3(1.0f, 0.2f, 0.2f));
			dwgDebugLine(m_positions[i], m_positions[i] + m_velocities[i] * 0.04f, Vector3(1.0f, 0.2f, 0.2f));
		}
	}

private:
	Vector3* m_data = nullptr;
	Vector3* m_positions = nullptr;
	Vector3* m_velocities = nullptr;

	int m_numPrey = 0;
	int m_numPredators = 0;
	int m_numAll = 0;
};




// entry point for the app (using WinMain, so no console appears, just the rendering window)
int WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nShowCmd)
{
	// init window and rendering with given width, height, and title of the window
	if (!dwgInitApp(2540, 1300, "DwG - Game Math - SteeringBehaviours"))
		return 1;


	Flock flock(50, 10);

	// main game loop, each iteration is a single frame
	while (!dwgShouldClose())
	{
		const double globalTime = dwgGlobalTime();	// global time - time since the start of the app
		const float dt = dwgDeltaTime();			// delta time - time since last frame

		//dwgDebugSphere(Vector3(0.0f, 0.0f, 20.0f), Vector3(11.0f, 11.0f, 0.1f), Vector3(1.0f, 1.0f, 1.0f));

		//axes
		dwgDebugLine({ -9.0f,-5.0f,0.0f }, { -10.0f,-5.0f,0.0f }, { 1.0f,0.0f,0.0f });
		dwgDebugLine({ -10.0f,-5.0f,0.0f }, { -10.0f,-4.0f,0.0f }, { 0.0f,1.0f,0.0f });

		// your code goes here...

		flock.simulate(dt);
		flock.draw();



		// prepare camera
		const Matrix4 camera = Matrix4::orthographic(-10.f, 10.0f, -10.0f, 10.0f, 0.1f, 1000.0f);	// create camera view matrix
		const float fov = 120.0f;								// field of view - angle of view of the camera (for perspective)

		// draw scene
		dwgRender(camera, fov);
	}

	// release rendering and window app
	dwgReleaseApp();

	return 0;
}
