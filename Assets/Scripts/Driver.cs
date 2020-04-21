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

    [Header("Initial Conditions")]
    [SerializeField] private float initVelocity = 0f;

    [Header("Wheels")]
    [SerializeField] private GameObject wheelFrontLeft;
    [SerializeField] private GameObject wheelFrontRight;
    [SerializeField] private GameObject wheelBackLeft;
    [SerializeField] private GameObject wheelBackRight;
    private WheelCollider wheelcolFL, wheelcolFR, wheelcolBL, wheelcolBR;

    [Header("Car behaviour")]
    public float desiredSpeed = 100f;
    public float proptionalGain, integralGain, derivativeGain;
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
    private Vector3 delta = Vector3.zero;

    private float l, w;
    private Rigidbody rigidBody;

    private float[] speedError = new float[] { 0f, 0f, 0f, 0f};

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
        rigidBody.velocity = transform.TransformDirection(initVelocity/3.6f*Vector3.forward);
    }

    private void FixedUpdate()
    {
        GetSpeed();
        FollowTrajectory();
        //SteerWheels();
        Accelerate();
    }


    private void GetSpeed()
    {
        // Receives vehicle velocity in m/s
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
        speedError[0] = desiredSpeed - velocity;
        speedError[2] = speedError[2] + speedError[0] * Time.deltaTime;
        speedError[3] = (speedError[0] - speedError[1]) / Time.deltaTime;
        speedError[1] = speedError[0];

        // PID control
        float correction = speedError[0] * proptionalGain + speedError[2] * integralGain + speedError[3] * derivativeGain;
        Debug.Log(correction);

        wheelcolBL.motorTorque = correction;
        wheelcolBR.motorTorque = correction;
    }


    private void FollowTrajectory()
    {

        if (trackingTime == 0.0f) { delta = transform.position; }

        switch (trackingMode)
        {
            case TrackingMode.laneKeeping:
                trackingTime = 0.0f;
                break;
            case TrackingMode.leftLaneChange:
                
                float t = trackingTime;
                float x = -lc_Width * (10 * Mathf.Pow((t / lc_Time), 3) - 15 * Mathf.Pow((t / lc_Time), 4) + 6 * Mathf.Pow((t / lc_Time), 5));
                float y = transform.position.y;
                float z = t * v + (lc_Length - lc_Time * v) * (10 * Mathf.Pow((t / lc_Time), 3) - 15 * Mathf.Pow((t / lc_Time), 4) + 6 * Mathf.Pow((t / lc_Time), 5));
                float angle = (float)-Math.Atan((30*Math.Pow(t,2)*lc_Width*Math.Pow(t-lc_Time,2))/((Math.Pow(lc_Time,5)*(v + (30*Math.Pow(t,2)*(lc_Length- v * lc_Time)*Math.Pow(t-lc_Time,2))/(Math.Pow(lc_Time,5))))));

                Vector3 pos = new Vector3(x, y, z);
                Quaternion rotation = Quaternion.Euler(0, angle, 0);

                target.transform.position = pos + delta;
                target.transform.rotation = rotation;

                trackingTime += Time.deltaTime;

                break;
            case TrackingMode.rightLaneChange:
                trackingTime += Time.fixedDeltaTime;

                break;
            default:
                Debug.Log("No tracking mode set");
                break;
        }

        if (trackingTime >= lc_Time & trackingMode != TrackingMode.laneKeeping)
        {
            trackingMode = TrackingMode.laneKeeping;
            return;
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
        Gizmos.color = Color.green;
        Gizmos.DrawSphere(target.position, 1.0f);
    }
}
