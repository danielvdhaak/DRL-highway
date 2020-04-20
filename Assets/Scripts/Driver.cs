using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

//[RequireComponent(typeof(TrajectoryManager))]
[RequireComponent(typeof(Rigidbody))]
public class Driver : MonoBehaviour
{
    public enum TrackingMode { leftLaneChange, laneKeeping, rightLaneChange };

    [Header("Properties")]
    public float steeringAngle;
    public float velocity;
    public float maxSpeed;
    public bool laneChanging = false;

    [Header("Initial Conditions")]
    [SerializeField] private float initVelocity = 0f;

    [Header("Wheels")]
    [SerializeField] private GameObject wheelFrontLeft;
    [SerializeField] private GameObject wheelFrontRight;
    [SerializeField] private GameObject wheelBackLeft;
    [SerializeField] private GameObject wheelBackRight;
    private WheelCollider wheelcolFL, wheelcolFR, wheelcolBL, wheelcolBR;

    [Header("Car behaviour")]
    [SerializeField] private float downForce = 0.0f;
    [SerializeField] private Transform centerOfMass;

    [Header("Trajectory Tracking Behavior")]
    public TrackingMode trackingMode = TrackingMode.laneKeeping;
    [SerializeField] private Transform wayPointTracker;
    [SerializeField] private Transform target;
    [SerializeField] private float gainParameter = 1.0f;
    private float trackingTime = 0.0f;

    [Header("Lane change parameters")]
    [SerializeField] private Vector3 advancePoint;
    [SerializeField] private float lc_Width = 3.5f;
    [SerializeField] private float lc_Time = 3f;
    [SerializeField] private float v = 10f;
    [SerializeField] private float lc_Length = 100;
    private List<Vector3> trajectoryPoints = new List<Vector3>();
    private int currentNode = 0;
    private Vector3 error;

    private float l, w;
    private Rigidbody rigidBody;

    //private TrajectoryManager trajectoryManager;

    private void Start()
    {
        // Initialize rigid body and center of mass
        rigidBody = GetComponent<Rigidbody>();
        if (rigidBody != null && centerOfMass != null)
        {
            rigidBody.centerOfMass = centerOfMass.localPosition;
        }

        // Initialize wheelcolliders
        wheelcolFL = wheelFrontLeft.GetComponent<WheelCollider>();
        wheelcolFR = wheelFrontRight.GetComponent<WheelCollider>();
        wheelcolBL = wheelBackLeft.GetComponent<WheelCollider>();
        wheelcolBR = wheelBackRight.GetComponent<WheelCollider>();

        // Calculate wheel seperation w and base l
        w = Math.Abs(wheelFrontLeft.transform.localPosition.x - wheelFrontRight.transform.localPosition.x);
        l = Math.Abs(wheelFrontLeft.transform.localPosition.z - wheelBackLeft.transform.localPosition.z);

        // Initialize trajectory manager class
        //trajectoryManager = GetComponent<TrajectoryManager>();

        // Set initial conditions
        rigidBody.velocity = transform.TransformDirection(initVelocity*Vector3.forward);

        CalcLCTrajectory();
    }

    private void FixedUpdate()
    {
        GetSpeed();
        FollowTrajectory();
        //SteerWheels();
        //Accelerate();
    }


    private void GetSpeed()
    {
        // Receives vehicle velocity in km/h
        velocity = transform.InverseTransformDirection(rigidBody.velocity).z * 3.6f;
    }

    private void SteerWheels()
    {
        // Steers wheels using Ackermann steering principle
        double angle = steeringAngle * Mathf.Deg2Rad;
        wheelcolFL.steerAngle = (float)Math.Atan((2 * l * Math.Sin(angle)) / (2 * l * Math.Cos(angle) + w * Math.Sin(angle))) * Mathf.Rad2Deg;
        wheelcolFR.steerAngle = (float)Math.Atan((2 * l * Math.Sin(angle)) / (2 * l * Math.Cos(angle) - w * Math.Sin(angle))) * Mathf.Rad2Deg;
    }

    private void Accelerate()
    {
        wheelcolBL.motorTorque = 100;
        wheelcolBR.motorTorque = 100;
    }

    private void ChangeLane()
    {
        laneChanging = true;

        trackingTime = 0.0f;
    }

    public void StanleyMethod(Transform target)
    {
        
    }


    private void FollowTrajectory()
    {
        switch (trackingMode)
        {
            case TrackingMode.laneKeeping:
                Debug.Log("Keeping lane");
                break;
            case TrackingMode.leftLaneChange:
                Debug.Log("Switching to left lane");


                break;
            case TrackingMode.rightLaneChange:
                Debug.Log("Switching to right lane");
                break;
            default:
                Debug.Log("No tracking mode set");
                break;
        }




        float errorRadius = 0;

        Vector3 target = trajectoryPoints[currentNode];
        error = target - wayPointTracker.position;

        if (error.magnitude <= errorRadius)
        {
            currentNode += 1;

            if (currentNode >= trajectoryPoints.Count -1)
            {
                currentNode = trajectoryPoints.Count - 1;
            }
        }

        Vector3 headingAngle = transform.TransformDirection(Vector3.forward);

        if (error.x >= 0)
        {
            steeringAngle = Vector3.Angle(error.normalized, headingAngle);
        } else
        {
            steeringAngle = - Vector3.Angle(error.normalized, headingAngle);
        }
        
    }

    private void CalcLCTrajectory()
    {
        currentNode = 0;
        trajectoryPoints.Clear();

        if (advancePoint == null)
        {
            Debug.LogError("Missing <Vector3> advancePoint!");
            Debug.Break();
        }

        float x;
        float z;
        Vector3 startingPoint = transform.position;

        for (float t = 0; t < lc_Time; t += 0.1f)
        {
            x = lc_Width * (10 * Mathf.Pow((t / lc_Time), 3) - 15 * Mathf.Pow((t / lc_Time), 4) + 6 * Mathf.Pow((t / lc_Time), 5));
            z = t * v + (lc_Length - lc_Time * v) * (10 * Mathf.Pow((t / lc_Time), 3) - 15 * Mathf.Pow((t / lc_Time), 4) + 6 * Mathf.Pow((t / lc_Time), 5));

            Vector3 pos = new Vector3(x, transform.position.y, z);
            pos = pos + advancePoint + startingPoint;

            trajectoryPoints.Add(pos);
        }
    }

    private void OnDrawGizmos()
    {
        float errorRadius = 0;

        Gizmos.color = Color.green;

        foreach (Vector3 pos in trajectoryPoints)
        {
            Gizmos.DrawSphere(pos, 0.1f);
        }

        Gizmos.color = Color.red;
        Gizmos.DrawLine(transform.position, transform.position + error);

        Gizmos.color = Color.cyan;
        Gizmos.DrawWireSphere(wayPointTracker.position, errorRadius);
    }
}
