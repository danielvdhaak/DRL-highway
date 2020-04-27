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
    public int targetLane = 2;

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
    [SerializeField] private float proptionalGain, integralGain, derivativeGain;
    [SerializeField] private Transform centerOfMass;
    private float[] speedError = new float[] { 0f, 0f, 0f, 0f };

    [Header("Trajectory Tracking Behavior")]
    public TrackingMode trackingMode = TrackingMode.laneKeeping;
    [SerializeField] private Transform wayPointTracker;
    [SerializeField] private Transform target;
    [SerializeField] private float gainParameter = 0.2f;
    [SerializeField] private GameObject environmentManager;
    private List<float> laneCenterlist = new List<float>();
    private float trackingTime = 0.0f;
    private Vector3 error;
    private float cte;
    private Vector3 delta = Vector3.zero;
    private Vector3 progress;

    [Header("Lane change parameters")]
    [SerializeField] private float lc_Width = 3.5f;
    [SerializeField] private float lc_Length = 100;

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

        // Initialize environment mananger and lanes
        if (environmentManager != null)
        {
            laneCenterlist = environmentManager.GetComponent<EnvironmentManager>().laneCenterList;
        } else
        {
            Debug.LogError("Class environmentManager not referenced!");
        }

        // Set initial conditions
        rigidBody.velocity = transform.TransformDirection(initVelocity/3.6f*Vector3.forward);
    }

    private void FixedUpdate()
    {
        GetSpeed();
        FollowTrajectory();
        SteerWheels();
        SetSpeed();
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

    private void SetSpeed()
    {
        speedError[0] = desiredSpeed - velocity;
        speedError[2] = speedError[2] + speedError[0] * Time.deltaTime;
        speedError[3] = (speedError[0] - speedError[1]) / Time.deltaTime;
        speedError[1] = speedError[0];

        // PID control
        float correction = speedError[0] * proptionalGain + speedError[2] * integralGain + speedError[3] * derivativeGain;
        //Debug.Log(correction);

        wheelcolBL.motorTorque = correction;
        wheelcolBR.motorTorque = correction;
    }


    private void FollowTrajectory()
    {
        float x, y, z, angle;
        Vector3 pos;
        Quaternion rotation;

        // Calculate starting position
        if (trackingTime == 0.0f)
        {
            progress = environmentManager.transform.InverseTransformPoint(wayPointTracker.transform.position);
            delta = progress;
            delta.x = laneCenterlist[targetLane - 1];
            //delta = environmentManager.transform.TransformPoint(new Vector3())
            //delta = wayPointTracker.transform.position;
        }

        switch (trackingMode)
        {
            case TrackingMode.laneKeeping:
                trackingTime = 0.0f;
           
                pos = delta + new Vector3(0, 0, 5);
                pos = environmentManager.transform.TransformPoint(pos);

                target.transform.position = pos;
                target.transform.rotation = environmentManager.transform.rotation;

                //cte = target.transform.InverseTransformDirection(target.transform.position - wayPointTracker.transform.position).x;
                cte = -(target.transform.InverseTransformPoint(wayPointTracker.transform.position)).x;
                //Debug.Log(cte);

                // Stanley method
                Debug.Log("cte: " + cte + ", 1st term: " + (0 - transform.rotation.eulerAngles.y) + ", 2nd term: " + (Mathf.Rad2Deg * (float)Math.Atan(gainParameter * cte / velocity)));
                steeringAngle = environmentManager.transform.rotation.eulerAngles.y - transform.rotation.eulerAngles.y + Mathf.Rad2Deg * (float)Math.Atan(gainParameter * cte / velocity);
                break;
            case TrackingMode.leftLaneChange:
                // Calculate target coordinates
                z = wayPointTracker.transform.position.z - delta.z;
                y = transform.position.y;
                x = -lc_Width * (10 * Mathf.Pow((z / lc_Length), 3) - 15 * Mathf.Pow((z / lc_Length), 4) + 6 * Mathf.Pow((z / lc_Length), 5));
                pos = new Vector3(x, y, z);
                pos = environmentManager.transform.TransformPoint(pos);
                angle = -Mathf.Rad2Deg * (float)Math.Atan(30 * lc_Width * Math.Pow(z, 2) * Math.Pow(lc_Length - z, 2) * Math.Pow(lc_Length, -5));
                rotation = Quaternion.Euler(0, angle, 0);
                
                target.transform.position = pos + delta;
                target.transform.rotation = rotation;

                // Stanley method
                //error = rotation * (target.transform.position - wayPointTracker.transform.position);
                //cte = error.x;
                cte = -(target.transform.InverseTransformPoint(wayPointTracker.transform.position)).x;
                Debug.Log("cte: " + cte + ", 1st term: " + (0 - transform.rotation.eulerAngles.y) + ", 2nd term: " + (Mathf.Rad2Deg * (float)Math.Atan(gainParameter * cte / velocity)));
                steeringAngle = angle - transform.rotation.eulerAngles.y + Mathf.Rad2Deg * (float)Math.Atan(gainParameter * cte / velocity);

                trackingTime += Time.deltaTime;

                if (wayPointTracker.transform.position.z - delta.z >= lc_Length)
                {
                    trackingMode = TrackingMode.laneKeeping;
                    targetLane = targetLane - 1;
                    //steeringAngle = 0.0f;
                    trackingTime = 0.0f;
                }
                break;
            case TrackingMode.rightLaneChange:
                // Calculate target coordinates
                z = wayPointTracker.transform.position.z - delta.z;
                y = transform.position.y;
                x = lc_Width * (10 * Mathf.Pow((z / lc_Length), 3) - 15 * Mathf.Pow((z / lc_Length), 4) + 6 * Mathf.Pow((z / lc_Length), 5));
                pos = new Vector3(x, y, z);
                pos = environmentManager.transform.TransformPoint(pos);
                angle = Mathf.Rad2Deg*(float)Math.Atan(30 * lc_Width * Math.Pow(z, 2) * Math.Pow(lc_Length - z, 2) * Math.Pow(lc_Length, -5));
                rotation = Quaternion.Euler(0, angle, 0);

                target.transform.position = pos + delta;
                target.transform.rotation = rotation;

                // Stanley method
                error = rotation * (target.transform.position - wayPointTracker.transform.position);
                cte = error.x;
                steeringAngle = angle - transform.rotation.eulerAngles.y + Mathf.Rad2Deg * (float)Math.Atan(gainParameter * cte / velocity);

                trackingTime += Time.deltaTime;

                if (wayPointTracker.transform.position.z - delta.z >= lc_Length)
                {
                    trackingMode = TrackingMode.laneKeeping;
                    targetLane = targetLane + 1;
                    //steeringAngle = 0.0f;
                    trackingTime = 0.0f;
                }
                break;
            default:
                Debug.Log("No tracking mode set");
                break;
        }

    }


    private void OnDrawGizmos()
    {
        Gizmos.color = Color.green;
        Gizmos.DrawSphere(target.position, 0.2f);

        Gizmos.color = Color.cyan;
        Gizmos.DrawLine(wayPointTracker.transform.position, target.transform.position);
    }
}
