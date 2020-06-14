/*
 * This code is part of DRL-Highway by Daniel (2020)
 * 
 */

using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
public class SimpleVehicle : MonoBehaviour
{
    [Header("Properties")]
    public float steeringAngle;
    public float velocity;
    public int targetLane;

    [Header("Car components")]
    [SerializeField] private GameObject wheelFrontLeft;
    [SerializeField] private GameObject wheelFrontRight;
    [SerializeField] private GameObject wheelBackLeft;
    [SerializeField] private GameObject wheelBackRight;
    private WheelCollider wheelcolFL, wheelcolFR, wheelcolBL, wheelcolBR;
    [SerializeField] private Transform centerOfMass;
    [SerializeField] private Transform frontRadar;

    [Header("IDM parameters")]
    [SerializeField] private float m_MaxMotorTorque = 1000f;
    [SerializeField] private float m_MaxBrakingTorque = 1000f;
    [SerializeField] [Range(50, 200)] private int m_MeasureDistance = 100;
    [SerializeField] private float m_Exponent = 4f;
    [SerializeField] private float m_SafeHeadway = 1.6f;

    [Header("Stanley steering parameters")]
    [SerializeField] private Transform tracker;
    [SerializeField] private Transform target;
    [SerializeField] private float m_GainParameter = 0.2f;
    private float laneCenter;

    private GameObject environment;

    private float l, w;
    private Rigidbody rigidBody;

    private void Awake()
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

        // Initialize environment
        environment = transform.parent.gameObject;
        if (environment == null)
        {
            Debug.LogError("Can not reference environment!");
        }
    }

    private void Start()
    {
        // Calculate wheel seperation w and base l
        w = Math.Abs(wheelFrontLeft.transform.localPosition.x - wheelFrontRight.transform.localPosition.x);
        l = Math.Abs(wheelFrontLeft.transform.localPosition.z - wheelBackLeft.transform.localPosition.z);

        // Get targetlane center
        laneCenter = environment.GetComponent<EnvironmentManager>().laneData[targetLane - 1].center;

        // Set initial conditions
        rigidBody.velocity = transform.TransformDirection(velocity / 3.6f * Vector3.forward);
    }

    private void FixedUpdate()
    {
        // Do physics related tasks
        GetSpeed();
        FollowTrajectory();
    }

    private void GetSpeed()
    {
        // Measure vehicle velocity in km/h
        velocity = transform.InverseTransformDirection(rigidBody.velocity).z * 3.6f;
    }

    private void FollowTrajectory()
    {
        float cte = -(environment.transform.InverseTransformPoint(tracker.position).x - laneCenter);
        float steeringError = -transform.localEulerAngles.y;

        steeringAngle = steeringError + Mathf.Rad2Deg * (float)Math.Atan(m_GainParameter * cte / velocity);

        if (steeringAngle > 180f)
            steeringAngle -= 360f;

        if (steeringAngle < -180f)
            steeringAngle += 360f;


        double steeringAngleRad = steeringAngle * Mathf.Deg2Rad;
        wheelcolFL.steerAngle = (float)Math.Atan((2 * l * Math.Sin(steeringAngleRad)) / (2 * l * Math.Cos(steeringAngleRad) + w * Math.Sin(steeringAngleRad))) * Mathf.Rad2Deg;
        wheelcolFR.steerAngle = (float)Math.Atan((2 * l * Math.Sin(steeringAngleRad)) / (2 * l * Math.Cos(steeringAngleRad) - w * Math.Sin(steeringAngleRad))) * Mathf.Rad2Deg;
    }


}
