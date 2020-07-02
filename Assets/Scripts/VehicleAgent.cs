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
[RequireComponent(typeof(VehicleControl))]
public class VehicleAgent : Agent
{
    private EnvironmentManager environment;
    private VehicleControl vehicle;
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
    public float mTorque;
    public float bTorque;
    [Range(0, 200)] public int desiredVelocity = 100;

    [Header("Trajectory tracking Behavior")]
    private bool m_laneChanging = false;
    private float m_Delta;
    private float m_CTE;
    private float m_headingError;
    [SerializeField] private Transform tracker;

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

        // Initialize environment
        environment = GetComponentInParent<EnvironmentManager>();
        if (environment == null)
        {
            Debug.LogError("Missing <GameObject> environment reference!");
            Debug.Break();
        }

        // Initialize vehicle control module
        vehicle = GetComponent<VehicleControl>();
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
        sensor.AddObservation(GetSpeed());
    }

    public override void CollectDiscreteActionMasks(DiscreteActionMasker actionMasker)
    {
        // Ensures the car does not crash into the barriers
    }

    public override void OnActionReceived(float[] vectorAction)
    {
        var action = Mathf.FloorToInt(vectorAction[0]);
        //Debug.Log("Action: " + action);

        Vector3 pos = environment.transform.InverseTransformPoint(tracker.position);
        velocity = GetSpeed();

        // Shift LC delta point if not lanechanging
        if (!m_laneChanging)
            m_Delta = pos.z;
        //environment.laneData[targetLane-1].center,
        //environment.transform.InverseTransformPoint(tracker.position).y,
        //environment.transform.InverseTransformPoint(tracker.position).z
        //);

        // Trajectory generation
        float x, a;
        switch (action)
        {
            case k_KeepLane:
                m_laneChanging = false;
                pos = environment.transform.InverseTransformPoint(tracker.position);
                x = environment.laneData[targetLane - 1].center;
                a = 0f;
                break;
            case k_LeftLaneChange:
                m_laneChanging = true;
                x = -lc_Width * (10 * Mathf.Pow(((pos.z - m_Delta) / lc_Length), 3) - 15 * Mathf.Pow(((pos.z - m_Delta) / lc_Length), 4) + 6 * Mathf.Pow(((pos.z - m_Delta) / lc_Length), 5)) + environment.laneData[targetLane - 1].center;
                a = -(float)Math.Atan(30 * lc_Width * Math.Pow((pos.z - m_Delta), 2) * Math.Pow(lc_Length - (pos.z - m_Delta), 2) * Math.Pow(lc_Length, -5));
                if (pos.z - m_Delta >= lc_Length)
                {
                    m_laneChanging = false;
                    targetLane = targetLane - 1;
                }
                break;
            case k_RightLaneChange:
                m_laneChanging = true;
                x = lc_Width * (10 * Mathf.Pow(((pos.z - m_Delta) / lc_Length), 3) - 15 * Mathf.Pow(((pos.z - m_Delta) / lc_Length), 4) + 6 * Mathf.Pow(((pos.z - m_Delta) / lc_Length), 5)) + environment.laneData[targetLane - 1].center;
                a = (float)Math.Atan(30 * lc_Width * Math.Pow((pos.z - m_Delta), 2) * Math.Pow(lc_Length - (pos.z - m_Delta), 2) * Math.Pow(lc_Length, -5));
                if (pos.z - m_Delta >= lc_Length)
                {
                    m_laneChanging = false;
                    targetLane = targetLane + 1;
                }
                break;
            default:
                throw new ArgumentException("Invalid action value");
        }

        // Steering
        m_CTE = (x - pos.x) * Mathf.Cos(-a);
        m_headingError = Mathf.Rad2Deg * a - transform.rotation.eulerAngles.y;
        steeringAngle = vehicle.CalcSteeringAngle(m_CTE, m_headingError, velocity);
        Debug.Log(steeringAngle);
        (wheelcolFL.steerAngle, wheelcolFR.steerAngle) = vehicle.Ackermann(steeringAngle, l, w);

        // Torque
        (mTorque, bTorque) = vehicle.CalcTorques(velocity, desiredVelocity, Mathf.Infinity, 0f, 0f);
        wheelcolBL.motorTorque = mTorque;
        wheelcolBL.brakeTorque = bTorque;
        wheelcolBR.motorTorque = mTorque;
        wheelcolBR.brakeTorque = bTorque;

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
