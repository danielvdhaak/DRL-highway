/*
 * This code is part of DRL-Highway by Daniel (2020)
 * 
 */

using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using MLAgents;
using MLAgents.Sensors;
using System.Linq;

[RequireComponent(typeof(Rigidbody))]
public class VehicleAgent : Agent
{
    public EnvironmentManager environment;
    public Transform Target;

    [Header("Car components")]
    private Rigidbody rBody;
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

    public override void Initialize()
    {
        // Check whether environment is correctly referenced
        if (environment == null)
        {
            Debug.LogError("Missing <GameObject> environment reference!");
            Debug.Break();
        }

        // Initialize rigidbody component
        rBody = GetComponent<Rigidbody>();
        if (rBody != null && centerOfMass != null)
            rBody.centerOfMass = centerOfMass.localPosition;

    }

    public override void OnEpisodeBegin()
    {
        Debug.Log("OnEpisodeBegin() executed!");

        // Debugging for now, should probably move to CollectObservations()
        //laneCenterList = environmentManager.laneCenterList;


        // If the agent falls
        if (transform.position.y < 0)
        {
            this.rBody.angularVelocity = Vector3.zero;
            this.rBody.velocity = Vector3.zero;
            this.transform.position = new Vector3(0, 0.5f, 0);
        }

        // Car collision
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        sensor.AddObservation(rBody.velocity.x);
    }

    public override void OnActionReceived(float[] vectorAction)
    {
        rBody.AddForce(Vector3.zero);
    }

    public override float[] Heuristic()
    {
        return base.Heuristic();
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
}
