/*
 * This code is part of DRL-Highway by Daniel (2020)
 * 
 */

using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
public class VehicleControl : MonoBehaviour
{
    public enum _TrackingMode { leftLaneChange, keepLane, rightLaneChange };
    
    private EnvironmentManager environment;
    private VehicleRanker ranker;

    // Properties
    public float SteeringAngle { get; set; }
    public int Lane { get; set; }
    public float LaneCenter { get; set; }
    public float Velocity { get; private set; }
    public float TargetVelocity { get; set; }
    public float Throttle { get; private set; }
    public float Spacing { get; private set; }
    public float Headway { get; private set; } = 1f;
    public int Rank { get; set; }
    public float Compare { get; set; }
    public WheelCollider[] Wheels { get; private set; }
    public _TrackingMode TrackingMode
    {
        get { return trackingMode; }
        set
        {
            trackingMode = value;
            OnTrackingModeChanged(trackingMode);
        }
    }
    
    [Header("Car components")]
    [SerializeField] private GameObject wheelFrontLeft;
    [SerializeField] private GameObject wheelFrontRight;
    [SerializeField] private GameObject wheelBackLeft;
    [SerializeField] private GameObject wheelBackRight;
    private WheelCollider wheelcolFL, wheelcolFR, wheelcolBL, wheelcolBR;
    [SerializeField] private Transform centerOfMass;
    public Transform Front;
    public Transform Back;
    private float l, w;
    private Rigidbody rBody;

    [Header("ACC")]
    public bool isFollowing;
    public VehicleControl followTarget;
    private float mTorque;
    private float bTorque;
    [SerializeField] private float m_K = 250f;
    [SerializeField] private float m_Kt = 1.0f;
    [SerializeField] private float m_Kv = 60f;
    [SerializeField] private float m_Kd = 30f;
    [SerializeField] private int m_maxMotorTorque = 1000;
    [SerializeField] private int m_maxBrakeTorque = 1000;

    [Header("Trajectory tracking")]
    [SerializeField] private _TrackingMode trackingMode;
    [SerializeField] private Transform tracker;
    [SerializeField] private float m_GainParameter = 0.4f;
    private float m_Delta;
    private float m_CTE;
    private float m_HeadingError;

    [Header("Lane change parameters")]
    private float lc_Width;
    [SerializeField] private float lc_Time = 2.5f;
    private float lc_Length;
    private const int k_RightLC = 1;
    private const int k_LeftLC = -1;



    private void Awake()
    {
        environment = GetComponentInParent<EnvironmentManager>();
        lc_Width = environment.laneWidth;

        // Initialize rigid body and center of mass
        rBody = GetComponent<Rigidbody>();
        if (rBody != null && centerOfMass != null)
        {
            rBody.centerOfMass = centerOfMass.localPosition;
        }

        // Initialize wheelcolliders
        wheelcolFL = wheelFrontLeft.GetComponent<WheelCollider>();
        wheelcolFR = wheelFrontRight.GetComponent<WheelCollider>();
        wheelcolBL = wheelBackLeft.GetComponent<WheelCollider>();
        wheelcolBR = wheelBackRight.GetComponent<WheelCollider>();
        Wheels = new WheelCollider[] { wheelcolFL, wheelcolFR, wheelcolBL, wheelcolBR };

        // Calculate wheel seperation w and base l
        w = Math.Abs(wheelFrontLeft.transform.localPosition.x - wheelFrontRight.transform.localPosition.x);
        l = Math.Abs(wheelFrontLeft.transform.localPosition.z - wheelBackLeft.transform.localPosition.z);
    }

    private void OnEnable()
    {
        TrackingMode = _TrackingMode.keepLane;
        isFollowing = true;
    }

    private void FixedUpdate()
    {
        Velocity = GetSpeed();
        Spacing = CalcSpacing(Velocity);

        if (followTarget != null)
        {
            float gap = environment.transform.InverseTransformDirection(followTarget.Back.position - Front.position).z;
            Headway = Mathf.Clamp(gap / Spacing, 0f, 1f);
            Debug.DrawLine(Front.position, Front.position + Front.forward * Spacing, Color.green);
            Debug.DrawLine(Front.position, Front.position + Front.forward * 0.6f * Spacing, Color.red);
            
            if (isFollowing)
                (mTorque, bTorque) = CalcTorques(Velocity, TargetVelocity, (gap - Spacing), followTarget.Velocity, followTarget.Throttle);
            else
                (mTorque, bTorque) = CalcTorques(Velocity, TargetVelocity, Mathf.Infinity, 0f, 0f);
        }
        else
        {
            Headway = 1f;
            (mTorque, bTorque) = CalcTorques(Velocity, TargetVelocity, Mathf.Infinity, 0f, 0f);
        }
            
        wheelcolBL.motorTorque = mTorque;
        wheelcolBL.brakeTorque = bTorque;
        wheelcolBR.motorTorque = mTorque;
        wheelcolBR.brakeTorque = bTorque;

    }

    /// <summary>
    /// Calls and stops coroutines upon changing the tracking mode enum.
    /// </summary>
    /// <param name="trackingMode"></param>
    protected void OnTrackingModeChanged(_TrackingMode trackingMode)
    {
        switch (trackingMode)
        {
            case _TrackingMode.keepLane:
                StopAllCoroutines();
                StartCoroutine(KeepLane(LaneCenter));
                break;
            case _TrackingMode.leftLaneChange:
                StopAllCoroutines();
                StartCoroutine(ChangeLane(k_LeftLC, LaneCenter));
                break;
            case _TrackingMode.rightLaneChange:
                StopAllCoroutines();
                StartCoroutine(ChangeLane(k_RightLC, LaneCenter));
                break;
        }
    }

    IEnumerator KeepLane (float center)
    {
        while (true)
        {
            m_CTE = center - environment.transform.InverseTransformPoint(tracker.position).x;
            m_HeadingError = -transform.localEulerAngles.y;
            SteeringAngle = CalcSteeringAngle(m_CTE, m_HeadingError, Velocity);
            (wheelcolFL.steerAngle, wheelcolFR.steerAngle) = Ackermann(SteeringAngle, l, w);

            yield return new WaitForFixedUpdate();
        }
    }

    IEnumerator ChangeLane (int dir, float center)
    {
        lc_Length = (Velocity / 3.6f) * lc_Time;
        m_Delta = environment.transform.InverseTransformPoint(tracker.position).z;

        while (environment.transform.InverseTransformPoint(tracker.position).z - m_Delta <= lc_Length)
        {
            Vector3 pos = environment.transform.InverseTransformPoint(tracker.position);
            float x = dir * lc_Width * (10 * Mathf.Pow(((pos.z - m_Delta) / lc_Length), 3) - 15 * Mathf.Pow(((pos.z - m_Delta) / lc_Length), 4) + 6 * Mathf.Pow(((pos.z - m_Delta) / lc_Length), 5)) + center;
            float a = dir * (float)Math.Atan(30 * lc_Width * Math.Pow((pos.z - m_Delta), 2) * Math.Pow(lc_Length - (pos.z - m_Delta), 2) * Math.Pow(lc_Length, -5));

            m_CTE = (x - pos.x) * Mathf.Cos(-a);
            m_HeadingError = Mathf.Rad2Deg * a - transform.localEulerAngles.y;
            SteeringAngle = CalcSteeringAngle(m_CTE, m_HeadingError, Velocity);
            (wheelcolFL.steerAngle, wheelcolFR.steerAngle) = Ackermann(SteeringAngle, l, w);

            yield return new WaitForFixedUpdate();
        }

        TrackingMode = _TrackingMode.keepLane;
    }

    /// <summary>
    /// Sets the (initial) forward velocity of the vehicles rigidbody.
    /// </summary>
    /// <param name="velocity"></param>
    public void SetInitialVelocity(float velocity)
    {
        if (rBody == null)
            rBody = GetComponent<Rigidbody>();

        rBody.angularVelocity = Vector3.zero;
        rBody.velocity = transform.TransformDirection((velocity / 3.6f) * Vector3.forward);
        Velocity = velocity;
    }


    /// <summary>
    /// Returns left- and right wheel steering angles calculated using Ackermann steering principle.
    /// </summary>
    private (float, float) Ackermann(float steeringAngle, float l, float w)
    {
        double angle = steeringAngle * Mathf.Deg2Rad;
        float leftAngle = (float)Math.Atan((2 * l * Math.Sin(angle)) / (2 * l * Math.Cos(angle) + w * Math.Sin(angle))) * Mathf.Rad2Deg;
        float rightAngle = (float)Math.Atan((2 * l * Math.Sin(angle)) / (2 * l * Math.Cos(angle) - w * Math.Sin(angle))) * Mathf.Rad2Deg;

        return (leftAngle, rightAngle);
    }

    /// <summary>
    /// Returns a steering angle calculated using the Stanley method.
    /// </summary>
    private float CalcSteeringAngle(float CTE, float headingError, float velocity)
    {
        float steeringAngle = headingError + Mathf.Rad2Deg * Mathf.Atan(m_GainParameter * CTE / (velocity / 3.6f));
        
        if (steeringAngle > 180f)
            steeringAngle -= 360f;
        else if (steeringAngle < -180f)
            steeringAngle += 360f;

        return steeringAngle;
    }

    /// <summary>
    /// Returns the safe spacing in meters to a preceding car.
    /// </summary>
    private float CalcSpacing(float velocity)
    {
        return 3f + 0.0019f * (velocity / 3.6f) + 0.0448f * (float)Math.Pow(velocity / 3.6f, 2);
    }

    /// <summary>
    /// Returns motor and braking torques calculated using Adaptive Cruise Control (ACC).
    /// </summary>
    private (float, float) CalcTorques(float velocity, float desVelocity, float spacingError, float fCarVelocity, float fCarThrottle)
    {
        float freeThrottle = m_K * ((desVelocity - velocity) / 3.6f);
        float referenceThrottle = m_Kt * fCarThrottle + m_Kv * ((fCarVelocity - velocity) / 3.6f) + m_Kd * spacingError;
        Throttle = Mathf.Min(freeThrottle, referenceThrottle);

        return (Mathf.Clamp(Throttle, 0f, m_maxMotorTorque), -Mathf.Clamp(Throttle, -m_maxBrakeTorque, 0f));
    }

    /// <summary>
    /// Returns the vehicle speed in km/h.
    /// </summary>
    private float GetSpeed()
    {
        return transform.InverseTransformDirection(rBody.velocity).z * 3.6f;
    }
}
