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

    private bool hasInitiated = false;

    private int currentLane;
    private float center;
    private float z;
    private float dz;

    private RandomNumber randomNumber = new RandomNumber();

    public override void Initialize()
    {
        environment = GetComponentInParent<EnvironmentManager>();
        if (environment == null)
            Debug.LogError("Missing environment reference!");
        laneCenters = environment.centerList;

        control = GetComponent<VehicleControl>();

        Target = GameObject.FindGameObjectWithTag("Target").transform;
    }

    public override void OnEpisodeBegin()
    {
        int startLane = randomNumber.Next(2, 4);
        int startPos = randomNumber.Next(1, 1);

        (initialLocalPos, initialVelocity) = environment.ResetArea(startLane, startPos);
        transform.localPosition = initialLocalPos;
        transform.localRotation = Quaternion.identity;
        control.targetVelocity = targetVelocity;
        control.SetInitialVelocity(initialVelocity);

        targetLane = startLane;
        control.currentLane = startLane;
        control.laneCenter = laneCenters[targetLane - 1];
        control._TrackingMode = VehicleControl.TrackingMode.keepLane;
    }

    private void FixedUpdate()
    {
        // Only request a new decision if agent is not performing a lane change
        if (control._TrackingMode == VehicleControl.TrackingMode.keepLane)
            RequestDecision();
        else
            RequestAction();
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        sensor.AddObservation(control.velocity);
        
    }

    public override void CollectDiscreteActionMasks(DiscreteActionMasker actionMasker)
    {
        // Ensures the car does not crash into the barriers
        int leftMostLane = 1;
        int rightMostLane = laneCenters.Count;

        if (targetLane == leftMostLane)
            actionMasker.SetMask(0, new int[] { k_LeftLaneChange - 1 });

        if (targetLane == rightMostLane)
            actionMasker.SetMask(0, new int[] { k_RightLaneChange - 1 });
    }

    public override void OnActionReceived(float[] vectorAction)
    {
        var lateralAction = Mathf.FloorToInt(vectorAction[0]);
        var longiAction = Mathf.FloorToInt(vectorAction[1]);

        switch (lateralAction)
        {
            case k_KeepLane:
                // Allocate reward for driving normalized velocity
                break;
            case k_LeftLaneChange:
                if (control._TrackingMode != VehicleControl.TrackingMode.leftLaneChange)
                {
                    control._TrackingMode = VehicleControl.TrackingMode.leftLaneChange;
                    targetLane--;
                    // LC penalty
                }
                break;
            case k_RightLaneChange:
                if (control._TrackingMode != VehicleControl.TrackingMode.rightLaneChange)
                {
                    control._TrackingMode = VehicleControl.TrackingMode.rightLaneChange;
                    targetLane++;
                    // LC penalty
                }
                    
                break;
            default:
                throw new ArgumentException("Invalid action value");
        }

        control.currentLane = GetCurrentLane();
        control.laneCenter = laneCenters[targetLane - 1];

        switch (longiAction)
        {
            case 1:
                control.followTarget = GetClosestVehicle(environment.trafficList, targetLane, 1);
                break;
            case 2:
                control.followTarget = null;
                break;
            default:
                throw new ArgumentException("Invalid action value");
        }

    }


    public override float[] Heuristic()
    {
        int lat, longi;

        //if (Input.GetKey(KeyCode.A))
        //    return new float[] { k_LeftLaneChange };
        //if (Input.GetKey(KeyCode.D))
        //    return new float[] { k_RightLaneChange };
        //else
        //    return new float[] { k_KeepLane };

        if (Input.GetKey(KeyCode.A))
            lat = k_LeftLaneChange;
        else if (Input.GetKey(KeyCode.D))
            lat = k_RightLaneChange;
        else
            lat = k_KeepLane;

        if (Input.GetKey(KeyCode.W))
            longi = 2;
        else
            longi = 1;

        return new float[] { lat, longi };
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
