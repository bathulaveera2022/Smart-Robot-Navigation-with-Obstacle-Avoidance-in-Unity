# Smart-Robot-Navigation-with-Obstacle-Avoidance-in-Unity
Autonomous Robot Navigation in Unity
Overview:
This project simulates a robot moving autonomously in a 3D environment created with Unity. The robot follows a predefined set of waypoints, avoids obstacles, and adapts to its surroundings using Unity’s built-in physics and AI tools. The environment can be customized by adding different terrains and obstacles.
Key Features:
1. Autonomous Navigation: The robot follows a path made up of waypoints and navigates autonomously.
2. Obstacle Avoidance: The robot detects and avoids obstacles in real-time.
3. Real-Time Environmental Interaction: It reacts to changes in the environment, such as avoiding moving obstacles.
4. Customizable Terrain: You can easily change the terrain and add obstacles to the 3D world.
Technologies Used:
• Unity 3D Engine: To create the environment and manage physics.
• NavMesh System: For pathfinding and navigation.
• AI/Physics: Used for real-time interactions with the environment.
How It Works:
1. Setup Waypoints for Navigation:
The robot follows a series of waypoints in the scene to get to its destination. Each waypoint is linked together to create a path.
C# Script: PathFollower
    public class PathFollower : MonoBehaviour
    {
        public Transform[] waypoints; // Array of waypoints the robot will follow
        private int currentWaypointIndex = 0;
        public float speed = 5f;

        void Update()
        {
            if (currentWaypointIndex < waypoints.Length)
            {
                MoveTowardsWaypoint();
            }
        }

        void MoveTowardsWaypoint()
        {
            Transform targetWaypoint = waypoints[currentWaypointIndex];
            Vector3 direction = targetWaypoint.position - transform.position;
            transform.position += direction.normalized * speed * Time.deltaTime;

            if (Vector3.Distance(transform.position, targetWaypoint.position) < 0.1f)
            {
                currentWaypointIndex++;
            }
        }
    }
    
2. Obstacle Avoidance:
The robot uses a NavMeshAgent to avoid obstacles while navigating. This system helps the robot find the shortest path and avoid any dynamic objects that may block the way.
C# Script: ObstacleAvoidance
    using UnityEngine;
    using UnityEngine.AI;

    public class ObstacleAvoidance : MonoBehaviour
    {
        public Transform destination; // The goal destination of the robot
        private NavMeshAgent agent;

        void Start()
        {
            agent = GetComponent<NavMeshAgent>();
            agent.SetDestination(destination.position);
        }

        void Update()
        {
            if (agent.pathPending || agent.remainingDistance > agent.stoppingDistance) return;

            // Code to handle any other dynamic behavior after reaching the destination
            Debug.Log("Robot has reached its destination!");
        }
    }
    
3. Customizing the Environment:
You can easily modify the environment by adding various obstacles and terrains to the scene in Unity’s editor. For instance, you can add obstacles like rocks or walls, and the robot will automatically navigate around them.
C# Script: PlaceObstacles
    void PlaceObstacles()
    {
        GameObject obstacle = GameObject.CreatePrimitive(PrimitiveType.Cube);
        obstacle.transform.position = new Vector3(10, 0.5f, 10);
        obstacle.AddComponent<NavMeshObstacle>(); // This makes the obstacle block the robot's path
    }
    
How to Run:
1. Clone the repository or download the project files.
2. Open the project in Unity.
3. Set up the environment with different terrains and obstacles (optional).
4. Attach the PathFollower and ObstacleAvoidance scripts to the robot object.
5. Press Play in Unity to see the robot navigate through the scene autonomously.
Purpose:
This project demonstrates how AI can be used in robotics to solve navigation problems. You can customize and expand this code to fit other use cases, like self-driving cars or warehouse robots, by adjusting the terrain, obstacles, and robot movement behavior.
