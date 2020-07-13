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

    private const int k_LeftLaneChange = 1;
    private const int k_KeepLane = 2;
    private const int k_RightLaneChange = 3;

    [Header("Parameters")]
    public int targetLane;
    public float targetVelocity;
    private List<float> laneCenters;

    private const float m_MaxFollowDistance = 100f;

    private Vector3 initialLocalPos;
    private float initialVelocity;

    private int currentLane;
    private float center;
    private float z;
    private float dz;

    private RandomNumber randomNumber = new RandomNumber();

    public override void Initialize()
    {
        // Initialize environment
        environment = GetComponentInParent<EnvironmentManager>();
        if (environment == null)
            Debug.LogError("Missing environment reference!");
        laneCenters = environment.centerList;

        // Initialize vehicle control module
        control = GetComponent<VehicleControl>();

        // Find target
        Target = GameObject.FindGameObjectWithTag("Target").transform;
    }

    public override void OnEpisodeBegin()
    {
        // set position
        (initialLocalPos, initialVelocity) = environment.ResetArea(2, 2);
        transform.localPosition = initialLocalPos;
        transform.localRotation = Quaternion.identity;
        control.SetInitialVelocity(initialVelocity);
    }

    private void FixedUpdate()
    {


        // Only request a new decision if agent is not performing a lane change
        if (control._TrackingMode == VehicleControl.TrackingMode.keepLane)
        {
            control.followTarget = GetClosestVehicle(environment.trafficList, targetLane, 1);
            RequestDecision();
        }
            
        else
            RequestAction();
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        //sensor.AddObservation(GetSpeed());
        
    }

    public override void CollectDiscreteActionMasks(DiscreteActionMasker actionMasker)
    {
        // Ensures the car does not crash into the barriers
        int leftMostLane = 1;
        int rightMostLane = laneCenters.Count;

        if (targetLane == leftMostLane)
            actionMasker.SetMask(0, new int[] { k_LeftLaneChange });

        if (targetLane == rightMostLane)
            actionMasker.SetMask(0, new int[] { k_RightLaneChange });
    }

    public override void OnActionReceived(float[] vectorAction)
    {
        var action = Mathf.FloorToInt(vectorAction[0]);

        switch (action)
        {
            case k_KeepLane:
                control.followTarget = GetClosestVehicle(environment.trafficList, targetLane, 1);
                // Allocate reward for driving normalized velocity
                break;
            case k_LeftLaneChange:
                control._TrackingMode = VehicleControl.TrackingMode.leftLaneChange;
                // set acc target
                // negative reward for commiting a lane change
                break;
            case k_RightLaneChange:
                control._TrackingMode = VehicleControl.TrackingMode.rightLaneChange;
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

    private int GetCurrentLane()
    {
        List<float> errors = laneCenters.Select(c => Math.Abs(c - transform.localPosition.x)).ToList();


        return errors.IndexOf(errors.Min()) + 1;
    }

    private VehicleControl GetClosestVehicle(List<VehicleControl> vehicles, int lane, int dir)
    {
        VehicleControl target = null;
        float z = transform.localPosition.z;
        float dz = Mathf.Infinity;

        foreach (VehicleControl vehicle in vehicles)
        {
            if (vehicle == control || vehicle.currentLane != lane || !vehicle.isActiveAndEnabled)
                continue;

            if (vehicle.transform.localPosition.z > z && Math.Abs(vehicle.transform.localPosition.z - z) <= m_MaxFollowDistance)
            {
                if (Math.Abs(vehicle.transform.localPosition.z - z) < dz)
                {
                    target = vehicle;
                    dz = Math.Abs(vehicle.transform.localPosition.z - z);
                }
            }
        }

        return target;
    }

    private void OnTriggerEnter(Collider other)
    {
        if (other.gameObject.CompareTag("Target"))
        {
            // Add reward
            EndEpisode();
        }
    }

    private void OnCollisionEnter(Collision collision)
    {
        if (collision.gameObject.CompareTag("Vehicle"))
        {
            // Add penalty
            EndEpisode();
        }
    }

}
