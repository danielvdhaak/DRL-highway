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
    [Header("ML-Agents")]
    private EnvironmentManager environment;
    private VehicleControl control;
    private Transform Target;
    public float reward;

    const int k_LeftLaneChange = 1;
    const int k_KeepLane = 2;
    const int k_RightLaneChange = 3;

    [Header("Parameters")]
    public float desiredVelocity;

    [Header("Properties")]
    public int targetLane;
    public bool m_laneChanging = false;

    private float laneWidth;
    private Vector3 initialLocalPos;
    private float initialVelocity;

    private float center;
    private float z;
    private float dz;

    private RandomNumber randomNumber = new RandomNumber();

    public override void Initialize()
    {
        // Initialize environment
        environment = GetComponentInParent<EnvironmentManager>();
        if (environment == null)
        {
            Debug.LogError("Missing <GameObject> environment reference!");
            Debug.Break();
        }
        laneWidth = environment.laneWidth;

        // Initialize vehicle control module
        control = GetComponent<VehicleControl>();
    }

    public override void OnEpisodeBegin()
    {
        // set position
        (initialLocalPos, initialVelocity) = environment.ResetArea(2, 2);
        control.SetInitialVelocity(initialVelocity);
    }

    private void FixedUpdate()
    {
        // Only requests a new decision if agent is not performing a lane change
        if (m_laneChanging)
        {
            RequestAction();
        }
        else
        {
            RequestDecision();
        }
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        //sensor.AddObservation(GetSpeed());
    }

    public override void CollectDiscreteActionMasks(DiscreteActionMasker actionMasker)
    {
        // Ensures the car does not crash into the barriers
    }

    public override void OnActionReceived(float[] vectorAction)
    {
        var action = Mathf.FloorToInt(vectorAction[0]);
        //Debug.Log("Action: " + action);

        switch (action)
        {
            case k_KeepLane:

                break;
            case k_LeftLaneChange:

                break;
            case k_RightLaneChange:

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

    private VehicleControl GetClosestVehicle(List<VehicleControl> vehicles)
    {
        VehicleControl target = null;
        float z = transform.localPosition.z;
        float dz = Mathf.Infinity;

        foreach (VehicleControl vehicle in vehicles)
        {
            if (vehicle == control)
                continue;

            if (vehicle.transform.localPosition.x >= center - 0.5f * laneWidth &&
                vehicle.transform.localPosition.x <= center + 0.5f * laneWidth &&
                vehicle.transform.localPosition.z - z < dz)
            {
                target = vehicle;
                dz = vehicle.transform.localPosition.z - z;
            }

        }

        return target;
    }

}
