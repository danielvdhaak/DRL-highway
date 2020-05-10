using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
public class VehicleControl : MonoBehaviour
{
    public enum TrackingMode { leftLaneChange, laneKeeping, rightLaneChange };

    [Header("Properties")]
    public float steeringAngle;
    public float velocity;
    public int targetLane = 2;

    [Header("Car components")]
    [SerializeField] private GameObject wheelFrontLeft;
    [SerializeField] private GameObject wheelFrontRight;
    [SerializeField] private GameObject wheelBackLeft;
    [SerializeField] private GameObject wheelBackRight;
    private WheelCollider wheelcolFL, wheelcolFR, wheelcolBL, wheelcolBR;
    [SerializeField] private Transform centerOfMass;
    [SerializeField] private Transform frontRadar;

    /*
    [Header("Car velocity control")]
    public float desiredVelocity = 100f;
    [SerializeField] private float timeToCollision = 2.0f;
    private bool frontWarning = false;
    [SerializeField] private float proptionalGain, integralGain, derivativeGain;
    [SerializeField] private int brakingForce = 1000;
    private float[] velocityError = new float[] { 0f, 0f, 0f, 0f };
    */

    [Header("ACC parameters")]
    public float throttle = 0f;
    [Range(0,200)] public int desiredVelocity = 100;
    [SerializeField] [Range(50,200)] private int m_MeasureDistance = 100;
    [SerializeField] private float m_K = 50f;
    [SerializeField] private float m_Kt = 1.0f;
    [SerializeField] private float m_Kv = 60f;
    [SerializeField] private float m_Kd = 30f;
    [SerializeField] private int m_maxMotorTorque = 1000;
    [SerializeField] private int m_maxBrakeTorque = 1000;

    [Header("Trajectory tracking Behavior")]
    public TrackingMode trackingMode = TrackingMode.laneKeeping;
    [SerializeField] private Transform wayPointTracker;
    [SerializeField] private Transform target;
    [SerializeField] private GameObject environmentManager;
    [SerializeField] private float m_GainParameter = 0.2f;
    
    private float laneCenter;
    private float trackingTime = 0.0f;
    private float cte;
    private Vector3 delta;
    private Vector3 progress;

    [Header("Lane change parameters")]
    [SerializeField] private float lc_Width = 3.5f;
    [SerializeField] private float lc_Length = 100;

    private float l, w;
    private Rigidbody rigidBody;


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

        // Initialize environment mananger
        if (environmentManager == null)
        {
            Debug.LogError("Class environmentManager not referenced!");
        } 

        // Set initial conditions
        rigidBody.velocity = transform.TransformDirection(velocity/3.6f*Vector3.forward);
    }

    private void FixedUpdate()
    {
        GetSpeed();
        FollowTrajectory();
        SteerWheels();
        ACC();
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

    private void ACC()
    {
        float gap = 3f + 0.0019f * (velocity/3.6f) + 0.0448f * (float)Math.Pow(velocity/3.6f, 2);
        Vector3 origin = frontRadar.transform.position;
        Vector3 direction = frontRadar.transform.TransformDirection(frontRadar.transform.forward);
        RaycastHit hit;

        float freeThrottle;
        float referenceThrottle;

        if (trackingMode == TrackingMode.laneKeeping)
        {
            if (Physics.Raycast(origin, direction, out hit, m_MeasureDistance) && hit.transform.tag == "Car")
            {
                Debug.DrawRay(origin, gap * direction, Color.red);

                // Gather preceding car information
                Vector3 carInfrontPos = hit.transform.position;
                float carInfrontVelocity = hit.transform.InverseTransformDirection(hit.rigidbody.velocity).z * 3.6f;
                VehicleControl carInfront = hit.transform.GetComponent<VehicleControl>();
                float carInfrontThrottle = carInfront.throttle;
                

                // Determine minimum and throttle
                freeThrottle = m_K * (desiredVelocity - velocity);
                referenceThrottle = m_Kt * carInfrontThrottle + m_Kv * ((carInfrontVelocity - velocity)/3.6f) + m_Kd * (hit.distance - gap);

                throttle = Mathf.Min(freeThrottle, referenceThrottle);
            }
            else
            {
                throttle = m_K * (desiredVelocity - velocity);
            }
        }
        else
        {
            throttle = m_K * (desiredVelocity - velocity);
        }

        // Apply throttle
        float motorTorque = Mathf.Clamp(throttle, 0f, m_maxMotorTorque);
        float brakeTorque = Mathf.Clamp(throttle, -m_maxBrakeTorque, 0f);

        wheelcolBL.motorTorque = motorTorque;
        wheelcolBR.motorTorque = motorTorque;
        wheelcolBL.brakeTorque = -brakeTorque;
        wheelcolBR.brakeTorque = -brakeTorque;
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
            delta.x = environmentManager.GetComponent<EnvironmentManager>().laneInfo[targetLane - 1].center;
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
                //Debug.Log("cte: " + cte + ", 1st term: " + (0 - transform.rotation.eulerAngles.y) + ", 2nd term: " + (Mathf.Rad2Deg * (float)Math.Atan(gainParameter * cte / velocity)));
                steeringAngle = environmentManager.transform.rotation.eulerAngles.y - transform.rotation.eulerAngles.y + Mathf.Rad2Deg * (float)Math.Atan(m_GainParameter * cte / velocity);
                break;
            case TrackingMode.leftLaneChange:
                // Calculate target coordinates
                z = environmentManager.transform.InverseTransformPoint(wayPointTracker.transform.position).z - delta.z;
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
                //Debug.Log("cte: " + cte + ", 1st term: " + (0 - transform.rotation.eulerAngles.y) + ", 2nd term: " + (Mathf.Rad2Deg * (float)Math.Atan(gainParameter * cte / velocity)));
                steeringAngle = angle - transform.rotation.eulerAngles.y + Mathf.Rad2Deg * (float)Math.Atan(m_GainParameter * cte / velocity);

                trackingTime += Time.deltaTime;

                if (environmentManager.transform.InverseTransformPoint(wayPointTracker.transform.position).z - delta.z >= lc_Length)
                {
                    trackingMode = TrackingMode.laneKeeping;
                    targetLane = targetLane - 1;
                    //steeringAngle = 0.0f;
                    trackingTime = 0.0f;
                }
                break;
            case TrackingMode.rightLaneChange:
                // Calculate target coordinates
                z = environmentManager.transform.InverseTransformPoint(wayPointTracker.transform.position).z - delta.z;
                y = transform.position.y;
                x = lc_Width * (10 * Mathf.Pow((z / lc_Length), 3) - 15 * Mathf.Pow((z / lc_Length), 4) + 6 * Mathf.Pow((z / lc_Length), 5));
                pos = new Vector3(x, y, z);
                pos = environmentManager.transform.TransformPoint(pos);
                angle = Mathf.Rad2Deg*(float)Math.Atan(30 * lc_Width * Math.Pow(z, 2) * Math.Pow(lc_Length - z, 2) * Math.Pow(lc_Length, -5));
                rotation = Quaternion.Euler(0, angle, 0);

                target.transform.position = pos + delta;
                target.transform.rotation = rotation;

                // Stanley method
                cte = -(target.transform.InverseTransformPoint(wayPointTracker.transform.position)).x;
                steeringAngle = angle - transform.rotation.eulerAngles.y + Mathf.Rad2Deg * (float)Math.Atan(m_GainParameter * cte / velocity);

                trackingTime += Time.deltaTime;

                if (environmentManager.transform.InverseTransformPoint(wayPointTracker.transform.position).z - delta.z >= lc_Length)
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


    /* OLD FUNCTIONS
    private void SpeedControl()
    {
        float thresholdDistance = timeToCollision * velocity / 3.6f;
        Vector3 origin = frontRadar.transform.position;
        Vector3 direction = frontRadar.transform.TransformDirection(frontRadar.transform.forward);
        RaycastHit hit;

        if (trackingMode == TrackingMode.laneKeeping)
        {
            if (Physics.Raycast(origin, direction, out hit, thresholdDistance))
            {
                float carInfrontVelocity = hit.transform.InverseTransformDirection(hit.rigidbody.velocity).z * 3.6f;
                Vector3 carInfrontPos = hit.transform.position;
                targetVelocity = carInfrontVelocity;

                Debug.DrawRay(origin, thresholdDistance * direction, Color.red);
                frontWarning = true;
            }
            else
            {
                targetVelocity = desiredVelocity;
                Debug.DrawRay(origin, thresholdDistance * direction, Color.green);
                frontWarning = false;
            }
        } else
        {
            targetVelocity = desiredVelocity;
            frontWarning = false;
        }    
    }

    private void SetSpeed()
    {
        velocityError[0] = targetVelocity - velocity;
        velocityError[2] = velocityError[2] + velocityError[0] * Time.deltaTime;
        velocityError[3] = (velocityError[0] - velocityError[1]) / Time.deltaTime;
        velocityError[1] = velocityError[0];

        // PID control
        float correction = velocityError[0] * proptionalGain + velocityError[2] * integralGain + velocityError[3] * derivativeGain;
        Debug.Log("PID correction: " + correction);

        // Clamp to prevent oscilatory behavior
        correction = Mathf.Clamp(correction, -1000f, 1000f);

        if (correction >= 0)
        {
            wheelcolBL.motorTorque = correction;
            wheelcolBR.motorTorque = correction;
            wheelcolBL.brakeTorque = 0f;
            wheelcolBR.brakeTorque = 0f;
        }
        else
        {
            wheelcolBL.motorTorque = 0f;
            wheelcolBR.motorTorque = 0f;
            wheelcolBL.brakeTorque = brakingForce;
            wheelcolBR.brakeTorque = brakingForce;
        }

    }
    */
}
