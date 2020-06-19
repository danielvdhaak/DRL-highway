/*
 * This code is part of DRL-Highway by Daniel (2020)
 * 
 */

using System;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;
using MLAgents;
using MLAgents.Sensors;

[RequireComponent(typeof(Rigidbody))]
public class VehicleAgent : Agent
{
    public EnvironmentManager environment;
    public Transform Target;
    public float reward;

    const int k_LeftLaneChange = 1;
    const int k_KeepLane = 2;
    const int k_RightLaneChange = 3;

    [Header("Properties")]
    public float steeringAngle;
    public float velocity;
    public int targetLane = 2;

    [Header("Car components")]
    private Rigidbody rBody;
    private float l, w;
    [SerializeField] private GameObject wheelFrontLeft;
    [SerializeField] private GameObject wheelFrontRight;
    [SerializeField] private GameObject wheelBackLeft;
    [SerializeField] private GameObject wheelBackRight;
    private WheelCollider wheelcolFL, wheelcolFR, wheelcolBL, wheelcolBR;
    [SerializeField] private Transform centerOfMass;
    [SerializeField] private Transform frontRadar;

    [Header("ACC parameters")]
    public float throttle = 0f;
    [Range(0, 200)] public int desiredVelocity = 100;
    [SerializeField] [Range(50, 200)] private int m_MeasureDistance = 100;
    [SerializeField] private float m_K = 50f;
    [SerializeField] private float m_Kt = 1.0f;
    [SerializeField] private float m_Kv = 60f;
    [SerializeField] private float m_Kd = 30f;
    [SerializeField] private int m_maxMotorTorque = 1000;
    [SerializeField] private int m_maxBrakeTorque = 1000;

    [Header("Trajectory tracking Behavior")]
    private bool m_laneChanging;
    private float m_Delta;
    [SerializeField] private Transform tracker;
    [SerializeField] private Transform waypoint;
    [SerializeField] private float m_GainParameter = 0.2f;

    [Header("Lane change parameters")]
    [SerializeField] private float lc_Width = 3.5f;
    [SerializeField] private float lc_Length = 100;


    public override void Initialize()
    {
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

        // Check whether environment is correctly referenced
        if (environment == null)
        {
            Debug.LogError("Missing <GameObject> environment reference!");
            Debug.Break();
        }
    }

    public override void OnEpisodeBegin()
    {
        // Set initial conditions:
            // - speed
            // - position
            // - rotation
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        
    }

    public override void CollectDiscreteActionMasks(DiscreteActionMasker actionMasker)
    {
        // Ensures the car does not crash into the barriers
    }

    public override void OnActionReceived(float[] vectorAction)
    {
        var action = Mathf.FloorToInt(vectorAction[0]);

        // Shift LC delta point if not lanechanging
        if (!m_laneChanging)
            m_Delta = environment.transform.InverseTransformPoint(vehicle.wayPointTracker.position).z;

        float x, y, z, angle;
        switch (action)
        {
            case k_KeepLane:
                m_laneChanging = false;
                break;
            case k_LeftLaneChange:
                m_laneChanging = true;

                z = environment.transform.InverseTransformPoint(vehicle.wayPointTracker.position).z - m_Delta;
                y = transform.position.y;
                x = -lc_Width * (10 * Mathf.Pow((z / lc_Length), 3) - 15 * Mathf.Pow((z / lc_Length), 4) + 6 * Mathf.Pow((z / lc_Length), 5));



                break;
            case k_RightLaneChange:
                m_laneChanging = true;
                break;
            default:
                throw new ArgumentException("Invalid action value");
        }
    }


    public override float[] Heuristic()
    {
        if (Input.GetKey(KeyCode.A))
            return new float[] { k_LeftLaneChange };
        if (Input.GetKey(KeyCode.D))
            return new float[] { k_RightLaneChange };
        else
            return new float[] { k_KeepLane };
    }

    private void FixedUpdate()
    {
        // Only requests a new decision if agent is not performing a lane change
        if (m_laneChanging)
        {
            RequestAction();
        } else
        {
            RequestDecision();
        }
    }

    private int DetermineCurrentLane(List<float> latErrorList)
    {
        // Determines whether agent is on the highway and if so, determines current lane
        float laneWidth = environment.laneWidth;
        float minLatError = latErrorList.Min();
        int lane = latErrorList.IndexOf(minLatError) + 1;

        if (minLatError <= 0.5*laneWidth)
        {
            //Debug.Log("Car is on lane " + lane);
            return lane;
        } else
        {
            //Debug.Log("Car is off the road");
            return 99;
        }
    }

    /// <summary>
    /// Returns the vehicle speed in km/h.
    /// </summary>
    public float GetSpeed()
    {
        return transform.InverseTransformDirection(rBody.velocity).z * 3.6f;
    }



}
