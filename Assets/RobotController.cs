using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Threading.Tasks;

public class RobotController : MonoBehaviour
{
    // naming constraints do not change
    //Wheel Controller
    [SerializeField] private WheelCollider FLC;
    [SerializeField] private WheelCollider FRC;
    [SerializeField] private WheelCollider RLC;
    [SerializeField] private WheelCollider RRC;
    //Transform 
    [SerializeField] private Transform FLT;
    [SerializeField] private Transform FRT;
    [SerializeField] private Transform RLT;
    [SerializeField] private Transform RRT;
    [SerializeField] private Transform FRS;
    [SerializeField] private Transform L1S;
    [SerializeField] private Transform L2S;
    [SerializeField] private Transform L3S;
    [SerializeField] private Transform R1S;
    [SerializeField] private Transform R2S;
    [SerializeField] private Transform R3S;
    [SerializeField] private Transform ORS;
    //Steering Angles
    [SerializeField] private float steeringAngle;
    [SerializeField] private float brakeForce;
    [SerializeField] private float motorForce;
    [SerializeField] private float thresholdValue = 8f;
    float stoppingDistance = 5;
    float turnSpeed = 4f;
    float sensorLength = 4;
    private float maxSpeed = 46;
    List<Vector3> newTrack = new List<Vector3>();
    private List<Vector3> targetWaypoints = new List<Vector3>();
    private float amountOfTrack = 3f;

    private Vector3[] WAYPOINTS = new Vector3[37];
    //private float currentSpeed;
    public int currentWaypoint = 0;
    private float targetSteering = 0;
    private bool applyBrake;
    private bool avoidingObstacles = false;
    private Rigidbody rigidBody;
    private LayerMask landscape;
    private LayerMask road;
    private LayerMask obstacle;
    private bool halt;
    bool isEndPoint, isRightBoundaryDetected, isLeftBoundaryDetected, isLeftEdgeDetected, isRightEdgeDetected, isFrontDetected, isBackDetected, isTopLeftDetected, isTopRightDetected;
    private float targetXPosition;
    private bool isReturningToCenter = false;
    bool isLeftSafe;
    bool isRightSafe;
    bool isCarStop;

    [Header("Road Detection Sensor")]
    [SerializeField] float roadDetectionSensorLength = 5f;
    [SerializeField] float edgeSteerStrength = 25f;
    [SerializeField] float steerStrength = 6f;
    [Header("Obstacle Detection Sensor")]
    [SerializeField] float obstacleDetectionSensorLength = 7f; // Length of the ray to detect in front
    [SerializeField] float safeDistance = 8f; // Safe distance to consider obstacle close
    [SerializeField] float obstacleDetectionSensorSteerAngle = 22f; // Steering angle for obstacle avoidance
    [SerializeField] float obstacleDetectionSensorSideRayLength = 8f; // Length of side ray for checking left and right

    private void Start()
    {
        rigidBody = GetComponent<Rigidbody>();
        rigidBody.centerOfMass = new Vector3(0f, -0.318f, -0.3600006f);

        rigidBody.drag = 0.75f;
        rigidBody.mass = 1000f;
        turnSpeed = 5f;
        road = 1 << LayerMask.NameToLayer("Road");
        obstacle = 1 << LayerMask.NameToLayer("Obs");
        landscape = 1 << LayerMask.NameToLayer("LS");

        //InitializeSensors(ORS, 50, 180, 0);
        //InitializeSensors(L1S, 0, -15, 0);
        //InitializeSensors(L2S, 0, -25, 0);
        //InitializeSensors(L3S, 0, -90, 0);
        //InitializeSensors(FRS, 0, 0, 0);
        //InitializeSensors(R1S, 0, 15, 0);
        //InitializeSensors(R2S, 0, 25, 0);
        //InitializeSensors(R3S, 0, 90, 0);
    }
    private void Awake()
    {
        InitializeTrackPoints();
        GenerateTrack();

        steeringAngle = 4f;
        brakeForce = 10f;
    }

   

    private void FixedUpdate()
    {
        if (EvaluateRoadEnd())
        {
            StopCar();
            return;
        }
        InspectSensors();
        ManageCarSpeed();
        SteerCar();
        CheckWaypointDistance();
        AdjustSteerAngle();
        CheckRoadEdges();
        RaycastFromSensorSFR();
    }

    private void InitializeSensors(Transform sensor, float xAngle, float yAngle, float zAngle)
    {
        sensor.transform.Rotate(xAngle, yAngle, zAngle);
    }

    private void Update()
    {
        var wheelPos = Vector3.zero;
        var wheelRot = Quaternion.identity;

        RRC.GetWorldPose(out wheelPos, out wheelRot);
        RRT.position = wheelPos;
        RRT.rotation = wheelRot;

        RLC.GetWorldPose(out wheelPos, out wheelRot);
        RLT.position = wheelPos;
        RLT.rotation = wheelRot;

        if (!isCarStop)
        {
            FRC.GetWorldPose(out wheelPos, out wheelRot);
            FRT.position = wheelPos;
            FRT.rotation = wheelRot;

            FLC.GetWorldPose(out wheelPos, out wheelRot);
            FLT.position = wheelPos;
            FLT.rotation = wheelRot;
        }
    }


    private void CheckWaypointDistance()
    {

        if (Vector3.Distance(transform.position, targetWaypoints[currentWaypoint]) < stoppingDistance)
        {
            if (currentWaypoint == targetWaypoints.Count - 6)
                applyBrake = true;
            else
                currentWaypoint++;
        }

        if (Vector3.Distance(transform.position, new Vector3(4.62f, 21.41f, -198f)) < 1f)
        {
            FLC.brakeTorque = 1000f;
            FRC.brakeTorque = 1000f;
        }
    }

    
    private void SteerCar()
    {
        if (avoidingObstacles) return;

        Vector3 relativeVector = transform.InverseTransformPoint(targetWaypoints[currentWaypoint]);
        float newSteer = (relativeVector.x / relativeVector.magnitude) * steeringAngle;
        targetSteering = newSteer;
    }

    private void ManageCarSpeed()
    {
        float carVelocity = rigidBody.velocity.magnitude;

        if (carVelocity < thresholdValue)
        {
            motorForce += 3.5f;
        }
        else
        {
            motorForce = 10000f;
        }
        if (!applyBrake)
        {
            FLC.motorTorque = motorForce;
            FRC.motorTorque = motorForce;
        }
        else
        {
            FLC.motorTorque = 0;
            FRC.motorTorque = 0;
            FLC.brakeTorque = brakeForce;
            FRC.brakeTorque = brakeForce;
            RLC.brakeTorque = brakeForce;
            RRC.brakeTorque = brakeForce;
        }
    }

    public void InspectSensors()
    {
        RaycastHit hit;
        avoidingObstacles = false;
        float avoidMultiplier = 0;

        // Left Sensors
        if (Physics.Raycast(L1S.position, L1S.forward, out hit, sensorLength))
        {
            avoidingObstacles = true;
            avoidMultiplier += 0.5f;
        }
        else if (Physics.Raycast(L2S.position, L2S.forward, out hit, sensorLength))
        {
            avoidingObstacles = true;
            avoidMultiplier += 1f;
        }
        else if (Physics.Raycast(L3S.position, L3S.forward, out hit, sensorLength))
        {
            avoidingObstacles = true;
            avoidMultiplier += 0.25f;
        }

        // Right sensors
        if (Physics.Raycast(R1S.position, R1S.forward, out hit, sensorLength))
        {
            avoidingObstacles = true;
            avoidMultiplier -= 0.5f;
        }
        else if (Physics.Raycast(R2S.position, R2S.forward, out hit, sensorLength))
        {
            avoidingObstacles = true;
            avoidMultiplier -= 1f;
        }

        if (avoidingObstacles)
        {
            targetSteering = steeringAngle * avoidMultiplier;
        }
    }

    public void GenerateTrack()
    {
        newTrack.Clear();
        for (int i = 0; i < WAYPOINTS.Length; i++)
        {
            Vector3 path0;
            Vector3 path1;
            Vector3 path2;
            Vector3 path3;

            if (i == 0)
            {
                path0 = WAYPOINTS[WAYPOINTS.Length - 1];
                path1 = WAYPOINTS[i];
                path2 = WAYPOINTS[i + 1];
                path3 = WAYPOINTS[i + 2];
            }
            else if (i == (WAYPOINTS.Length - 1))
            {
                path0 = WAYPOINTS[WAYPOINTS.Length - 2];
                path1 = WAYPOINTS[i];
                path2 = WAYPOINTS[0];
                path3 = WAYPOINTS[1];
            }
            else if (i == (WAYPOINTS.Length - 2))
            {
                path0 = WAYPOINTS[WAYPOINTS.Length - 3];
                path1 = WAYPOINTS[i];
                path2 = WAYPOINTS[i + 1];
                path3 = WAYPOINTS[0];
            }
            else
            {
                path0 = WAYPOINTS[i - 1];
                path1 = WAYPOINTS[i];
                path2 = WAYPOINTS[i + 1];
                path3 = WAYPOINTS[i + 2];
            }
            float track0 = 0.0f;
            float track1 = GeT(track0, path0, path1);
            float track2 = GeT(track1, path1, path2);
            float track3 = GeT(track2, path2, path3);

            for (float t = track1; t < track2; t += ((track2 - track1) / amountOfTrack))
            {
                Vector3 A1 = (track1 - t) / (track1 - track0) * path0 + (t - track0) / (track1 - track0) * path1;
                Vector3 A2 = (track2 - t) / (track2 - track1) * path1 + (t - track1) / (track2 - track1) * path2;
                Vector3 A3 = (track3 - t) / (track3 - track2) * path2 + (t - track2) / (track3 - track2) * path3;

                Vector3 B1 = (track2 - t) / (track2 - track0) * A1 + (t - track0) / (track2 - track0) * A2;
                Vector3 B2 = (track3 - t) / (track3 - track1) * A2 + (t - track1) / (track3 - track1) * A3;

                Vector3 C = (track2 - t) / (track2 - track1) * B1 + (t - track1) / (track2 - track1) * B2;
                newTrack.Add(C);
            }
        }

        targetWaypoints = newTrack;
    }

    private void AdjustSteerAngle()
    {
        FLC.steerAngle = Mathf.Lerp(FLC.steerAngle, targetSteering, Time.deltaTime * turnSpeed);
        FRC.steerAngle = Mathf.Lerp(FRC.steerAngle, targetSteering, Time.deltaTime * turnSpeed);
    }

    float GeT(float t, Vector3 path0, Vector3 path1)
    {
        float a = Mathf.Pow((path1.x - path0.x), 2.0f) + Mathf.Pow((path1.y - path0.y), 2.0f) + Mathf.Pow((path1.z - path0.z), 2.0f);
        float b = Mathf.Pow(a, 0.5f);
        float c = Mathf.Pow(b, 1);

        return (c + t);
    }

    void InitializeTrackPoints()
    {
        WAYPOINTS[0] = new Vector3(-128.66f, 0f, 126.97f);
        WAYPOINTS[1] = new Vector3(-39.6f, 0f, 126.97f);
        WAYPOINTS[2] = new Vector3(48f, 0f, 126.97f);
        WAYPOINTS[3] = new Vector3(122.3f, 0f, 126.97f);
        WAYPOINTS[4] = new Vector3(200.1f, 0f, 126.97f);
        WAYPOINTS[5] = new Vector3(239f, 0f, 120.2f);
        WAYPOINTS[6] = new Vector3(243.3f, 0f, 105.2f);
        WAYPOINTS[7] = new Vector3(240.62f, 8.5f, 61.8f);
        WAYPOINTS[8] = new Vector3(243.3f, 16f, 14.24f);
        WAYPOINTS[9] = new Vector3(239.4f, 16f, 3.27f);
        WAYPOINTS[10] = new Vector3(221.6f, 16f, -2.4f);
        WAYPOINTS[11] = new Vector3(193.9f, 16f, -2.4f);
        WAYPOINTS[12] = new Vector3(135.57f, 10.6f, -2.4f);
        WAYPOINTS[13] = new Vector3(104.98f, 16.22f, -2.4f);
        WAYPOINTS[14] = new Vector3(65.6f, 16.22f, -0.25f);
        WAYPOINTS[15] = new Vector3(56.03f, 16.22f, -10.55f);
        WAYPOINTS[16] = new Vector3(54.26f, 16.22f, -18.3f);
        WAYPOINTS[17] = new Vector3(53.7f, 16.22f, -35.7f);
        WAYPOINTS[18] = new Vector3(79f, 16.22f, -43f);
        WAYPOINTS[19] = new Vector3(94.57f, 16.22f, -59.74f);
        WAYPOINTS[20] = new Vector3(92.77f, 15.13f, -73.54f);
        WAYPOINTS[21] = new Vector3(76.52f, 11.86f, -79.6f);
        WAYPOINTS[22] = new Vector3(29.9f, 11.86f, -76.4f);
        WAYPOINTS[23] = new Vector3(0.8f, 11.86f, -81.6f);
        WAYPOINTS[24] = new Vector3(-28.23f, 11.86f, -74.96f);
        WAYPOINTS[25] = new Vector3(-36.7f, 15.98f, -50f);
        WAYPOINTS[26] = new Vector3(-44f, 15.98f, -15.3f);
        WAYPOINTS[27] = new Vector3(-59.61f, 15.98f, -12.64f);
        WAYPOINTS[28] = new Vector3(-74.45f, 15.98f, -34.5f);
        WAYPOINTS[29] = new Vector3(-74.45f, 15.98f, -63.8f);
        WAYPOINTS[30] = new Vector3(-60.3f, 15.98f, -80.7f);
        WAYPOINTS[31] = new Vector3(-38.9f, 15.98f, -90.7f);
        WAYPOINTS[32] = new Vector3(-32.3f, 15.98f, -112f);
        WAYPOINTS[33] = new Vector3(0.2f, 15.98f, -128.6f);
        WAYPOINTS[34] = new Vector3(3.2f, 15.98f, -141.5f);
        WAYPOINTS[35] = new Vector3(4.41f, 21.27f, -174.73f);
        WAYPOINTS[36] = new Vector3(4.62f, 21.27f, -198f);
    }

    #region Obstacle Detection and Avoidence, Car on track and End Point  
    private void StopCar()
    {
        FLC.motorTorque = 0f;
        FRC.motorTorque = 0f;

        FLC.brakeTorque = 1000f;
        FRC.brakeTorque = 1000f;
        RLC.brakeTorque = 1000f;
        RRC.brakeTorque = 1000f;
        //motorForce = Mathf.Lerp(motorForce, 0f, Time.fixedDeltaTime * 2f);
    }
    private void RaycastFromSensorSFR()
    {
        DetectObstacle(FRS);
    }

    private void SmoothAvoidance(float turnAngle)
    {
        FLC.steerAngle = turnAngle;
        FRC.steerAngle = turnAngle;

        isReturningToCenter = true;
    }

    private void CheckRoadEdges()
    {
        Vector3 leftBoundaryRayOrigin = transform.position - transform.right * 6.8f + Vector3.up * 2f;
        Vector3 rightBoundaryRayOrigin = transform.position + transform.right * 6.8f + Vector3.up * 2f;

        Vector3 leftRayOrigin = transform.position - transform.right * 3.8f + Vector3.up * 2f;
        Vector3 rightRayOrigin = transform.position + transform.right * 3.8f + Vector3.up * 2f;
        Vector3 frontRayOrigin = transform.position + transform.forward * 2f + Vector3.up * 1f;
        Vector3 backRayOrigin = transform.position - transform.forward * 2f + Vector3.up * 1f;
        Vector3 topLeftRayOrigin = transform.position + transform.forward * 1.5f - transform.right * 1.5f + Vector3.up * 1f;
        Vector3 topRightRayOrigin = transform.position + transform.forward * 1.5f + transform.right * 1.5f + Vector3.up * 1f;

        // Perform raycasts
        isLeftBoundaryDetected = !Physics.Raycast(leftBoundaryRayOrigin, Vector3.down, roadDetectionSensorLength, road);
        isRightBoundaryDetected = !Physics.Raycast(rightBoundaryRayOrigin, Vector3.down, roadDetectionSensorLength, road);
        isRightEdgeDetected = !Physics.Raycast(rightRayOrigin, Vector3.down, roadDetectionSensorLength, road);
        isFrontDetected = !Physics.Raycast(frontRayOrigin, Vector3.down, roadDetectionSensorLength, road);
        isBackDetected = !Physics.Raycast(backRayOrigin, Vector3.down, roadDetectionSensorLength, road);
        isTopLeftDetected = !Physics.Raycast(topLeftRayOrigin, Vector3.down, roadDetectionSensorLength, road);
        isTopRightDetected = !Physics.Raycast(topRightRayOrigin, Vector3.down, roadDetectionSensorLength, road);





        if (isLeftBoundaryDetected)
        {
            SmoothAvoidance(steerStrength);
        }
        else if (isRightBoundaryDetected)
        {
            SmoothAvoidance(-steerStrength);
        }

        if (isLeftEdgeDetected || isTopLeftDetected)
        {
            Debug.DrawRay(rightRayOrigin, Vector3.down * roadDetectionSensorLength, isLeftEdgeDetected ? Color.red : Color.red);
            Debug.Log("Near left edge! Steering right.");
            SmoothAvoidance(edgeSteerStrength);
        }
        else if (isRightEdgeDetected || isTopRightDetected)
        {
            Debug.Log("Near right edge! Steering left.");
            Debug.DrawRay(rightRayOrigin, Vector3.down * roadDetectionSensorLength, isRightEdgeDetected ? Color.red : Color.red);
            SmoothAvoidance(-edgeSteerStrength);
        }
        else if (isFrontDetected || isBackDetected)
        {
            SmoothAvoidance(steerStrength);
        }
    }

    private bool EvaluateRoadEnd()
    {
        Vector3 frontRayOrigin = transform.position + transform.forward * 5f + Vector3.up * 1f;
        isEndPoint = !Physics.Raycast(frontRayOrigin, Vector3.down, roadDetectionSensorLength, road);
        Debug.DrawRay(frontRayOrigin, Vector3.down * roadDetectionSensorLength, isFrontDetected ? Color.red : Color.magenta);
        return isEndPoint;

    }
   

    private void DetectObstacle(Transform origin)
    {
        obstacleDetectionSensorLength = 8f; // Length of the ray to detect in front
        safeDistance = 8f; // Safe distance to consider obstacle close
        obstacleDetectionSensorSteerAngle = 25f; // Steering angle for obstacle avoidance
        obstacleDetectionSensorSideRayLength = 25f; // Length of side ray for checking left and right

        Debug.DrawRay(origin.position, transform.forward * roadDetectionSensorLength, Color.blue); // Front Ray

        if (Physics.Raycast(origin.position, transform.forward, out RaycastHit hit, roadDetectionSensorLength, obstacle))
        {
            Debug.DrawRay(origin.position, transform.forward * roadDetectionSensorLength, Color.red); // Obstacle detected in front

            if (Vector3.Distance(hit.point, transform.position) < safeDistance)
            {
                Debug.Log("Obstacle Detected: Taking Action");

                // Check if it's safe to go left or right
                bool isLeftSafe = CheckSideSafety(-transform.right);
                bool isRightSafe = CheckSideSafety(transform.right);

                // Determine which direction to steer based on the safety check
                if (isLeftSafe)
                {
                    Debug.Log("Steering Left: Safe to go left.");
                    SmoothAvoidance(obstacleDetectionSensorSteerAngle);
                }
                else if (isRightSafe)
                {
                    Debug.Log("Steering Right: Safe to go right.");
                    SmoothAvoidance(-obstacleDetectionSensorSteerAngle);
                }
                else
                {
                }
            }
        }
    }

    private bool CheckSideSafety(Vector3 direction)
    {
        float sideRayLength = 10f;
        Vector3 sideRayOrigin = transform.position + Vector3.up * 1f;

        Debug.DrawRay(sideRayOrigin, direction * sideRayLength, Color.green);

        if (Physics.Raycast(sideRayOrigin, direction, sideRayLength, obstacle))
        {
            return false;
        }

        return true;
    }

    #endregion
}