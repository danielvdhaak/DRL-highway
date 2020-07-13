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
    public enum TrackingMode { leftLaneChange, keepLane, rightLaneChange };

    private EnvironmentManager environment;     

    [Header("Properties (READ ONLY)")]
    public float steeringAngle;
    public float velocity;
    public float throttle;
    [HideInInspector] public int currentLane;

    [Header("Parameters (INPUT)")]
    public TrackingMode trackingMode;
    public VehicleControl followTarget;
    public float laneCenter;
    [HideInInspector] public float targetVelocity;

    [Header("Car components")]
    [SerializeField] private GameObject wheelFrontLeft;
    [SerializeField] private GameObject wheelFrontRight;
    [SerializeField] private GameObject wheelBackLeft;
    [SerializeField] private GameObject wheelBackRight;
    private WheelCollider wheelcolFL, wheelcolFR, wheelcolBL, wheelcolBR;
    [SerializeField] private Transform centerOfMass;
    public Transform frontOffset;
    public Transform backOffset;
    private float l, w;
    private Rigidbody rBody;

    [Header("ACC")]
    private float mTorque;
    private float bTorque;
    [SerializeField] private float m_K = 120f;
    [SerializeField] private float m_Kt = 1.0f;
    [SerializeField] private float m_Kv = 60f;
    [SerializeField] private float m_Kd = 30f;
    [SerializeField] private int m_maxMotorTorque = 1000;
    [SerializeField] private int m_maxBrakeTorque = 1000;

    [Header("Trajectory tracking")]
    [SerializeField] private Transform tracker;
    [SerializeField] private float m_GainParameter = 0.4f;
    private float m_Delta;
    private float m_CTE;
    private float m_HeadingError;

    [Header("Lane change parameters")]
    private float lc_Width;
    [SerializeField] private float lc_Time = 2.5f;
    private float lc_Length;
    private readonly int k_RightLC = 1;
    private readonly int k_LeftLC = -1;

    // Setter that calls event upon value change
    public TrackingMode _TrackingMode
    {
        get { return trackingMode; }
        set
        {
            trackingMode = value;
            OnTrackingModeChanged(trackingMode);
        }
    }

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

        // Calculate wheel seperation w and base l
        w = Math.Abs(wheelFrontLeft.transform.localPosition.x - wheelFrontRight.transform.localPosition.x);
        l = Math.Abs(wheelFrontLeft.transform.localPosition.z - wheelBackLeft.transform.localPosition.z);
    }

    private void OnEnable()
    {
        _TrackingMode = TrackingMode.keepLane;
    }

    private void FixedUpdate()
    {
        velocity = GetSpeed();

        // Torques
        if (followTarget != null)
        {
            float gap = environment.transform.InverseTransformDirection(followTarget.backOffset.position - frontOffset.position).z;
            (mTorque, bTorque) = CalcTorques(velocity, targetVelocity, gap, followTarget.velocity, followTarget.throttle);
        }
        else
            (mTorque, bTorque) = CalcTorques(velocity, targetVelocity, Mathf.Infinity, 0f, 0f);
            
        wheelcolBL.motorTorque = mTorque;
        wheelcolBL.brakeTorque = bTorque;
        wheelcolBR.motorTorque = mTorque;
        wheelcolBR.brakeTorque = bTorque;

    }

    /// <summary>
    /// Calls and stops coroutines upon changing the tracking mode enum.
    /// </summary>
    /// <param name="trackingMode"></param>
    protected void OnTrackingModeChanged(TrackingMode trackingMode)
    {
        switch (trackingMode)
        {
            case TrackingMode.keepLane:
                StopAllCoroutines();
                StartCoroutine(KeepLane(laneCenter));
                break;
            case TrackingMode.leftLaneChange:
                StopAllCoroutines();
                StartCoroutine(ChangeLane(k_LeftLC, laneCenter));
                break;
            case TrackingMode.rightLaneChange:
                StopAllCoroutines();
                StartCoroutine(ChangeLane(k_RightLC, laneCenter));
                break;
        }
    }

    IEnumerator KeepLane (float center)
    {
        while (true)
        {
            m_CTE = center - environment.transform.InverseTransformPoint(tracker.position).x;
            m_HeadingError = -transform.localEulerAngles.y;
            steeringAngle = CalcSteeringAngle(m_CTE, m_HeadingError, velocity);
            (wheelcolFL.steerAngle, wheelcolFR.steerAngle) = Ackermann(steeringAngle, l, w);

            yield return new WaitForFixedUpdate();
        }
    }

    IEnumerator ChangeLane (int dir, float center)
    {
        lc_Length = (velocity / 3.6f) * lc_Time;
        m_Delta = environment.transform.InverseTransformPoint(tracker.position).z;

        while (environment.transform.InverseTransformPoint(tracker.position).z - m_Delta <= lc_Length)
        {
            Vector3 pos = environment.transform.InverseTransformPoint(tracker.position);
            float x = dir * lc_Width * (10 * Mathf.Pow(((pos.z - m_Delta) / lc_Length), 3) - 15 * Mathf.Pow(((pos.z - m_Delta) / lc_Length), 4) + 6 * Mathf.Pow(((pos.z - m_Delta) / lc_Length), 5)) + center;
            float a = dir * (float)Math.Atan(30 * lc_Width * Math.Pow((pos.z - m_Delta), 2) * Math.Pow(lc_Length - (pos.z - m_Delta), 2) * Math.Pow(lc_Length, -5));

            m_CTE = (x - pos.x) * Mathf.Cos(-a);
            m_HeadingError = Mathf.Rad2Deg * a - transform.localEulerAngles.y;
            steeringAngle = CalcSteeringAngle(m_CTE, m_HeadingError, velocity);
            (wheelcolFL.steerAngle, wheelcolFR.steerAngle) = Ackermann(steeringAngle, l, w);

            yield return new WaitForFixedUpdate();
        }

        _TrackingMode = TrackingMode.keepLane;
    }

    public void SetInitialVelocity(float velocity)
    {
        if (rBody == null)
            rBody = GetComponent<Rigidbody>();

        rBody.angularVelocity = Vector3.zero;
        rBody.velocity = transform.TransformDirection((velocity / 3.6f) * Vector3.forward);
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
    /// Returns motor and braking torques calculated using Adaptive Cruise Control (ACC).
    /// </summary>
    private (float, float) CalcTorques(float velocity, float desVelocity, float gap, float fCarVelocity, float fCarThrottle)
    {
        float spacing = 3f + 0.0019f * (velocity / 3.6f) + 0.0448f * (float)Math.Pow(velocity / 3.6f, 2);
        Debug.DrawLine(frontOffset.position, frontOffset.position + frontOffset.forward * spacing, Color.red);

        float freeThrottle = m_K * ((desVelocity - velocity) / 3.6f);
        float referenceThrottle = m_Kt * fCarThrottle + m_Kv * ((fCarVelocity - velocity) / 3.6f) + m_Kd * (gap - spacing);
        throttle = Mathf.Min(freeThrottle, referenceThrottle);

        return (Mathf.Clamp(throttle, 0f, m_maxMotorTorque), -Mathf.Clamp(throttle, -m_maxBrakeTorque, 0f));
    }

    /// <summary>
    /// Returns the vehicle speed in km/h.
    /// </summary>
    private float GetSpeed()
    {
        return transform.InverseTransformDirection(rBody.velocity).z * 3.6f;
    }
}
