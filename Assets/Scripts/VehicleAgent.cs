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
    private const int k_Follow = 1;
    private const int k_Cruise = 2;

    [Header("Parameters")]
    public int targetLane;
    public float targetVelocity;
    private List<float> laneCenters;

    private const float m_MaxFollowDistance = 100f;

    private Vector3 initialLocalPos;
    private float initialVelocity;

    private RandomNumber randomNumber = new RandomNumber();

    public override void Initialize()
    {
        environment = GetComponentInParent<EnvironmentManager>();
        if (environment == null)
            Debug.LogError("Missing environment reference!");
        laneCenters = environment.centerList;

        control = GetComponent<VehicleControl>();

        Events.Instance.onCutOff += OnCutOff;
    }

    public override void OnEpisodeBegin()
    {
        Target = GameObject.FindGameObjectWithTag("Target").transform;

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
                // Do nothing
                break;
            case k_LeftLaneChange:
                if (control._TrackingMode != VehicleControl.TrackingMode.leftLaneChange)
                {
                    control._TrackingMode = VehicleControl.TrackingMode.leftLaneChange;
                    targetLane--;
                    SetReward(-0.05f);
                }
                break;
            case k_RightLaneChange:
                if (control._TrackingMode != VehicleControl.TrackingMode.rightLaneChange)
                {
                    control._TrackingMode = VehicleControl.TrackingMode.rightLaneChange;
                    targetLane++;
                    SetReward(-0.05f);
                }
                    
                break;
            default:
                throw new ArgumentException("Invalid action value");
        }

        control.currentLane = GetCurrentLane();
        control.laneCenter = laneCenters[targetLane - 1];
        control.followTarget = GetClosestVehicle(environment.trafficList, targetLane, 1);

        switch (longiAction)
        {
            case k_Follow:
                control.isFollowing = true;
                break;
            case k_Cruise:
                control.isFollowing = false;
                break;
            default:
                throw new ArgumentException("Invalid action value");
        }

        float normSpeed = control.velocity / targetVelocity;
        SetReward(0.001f * normSpeed);

    }


    public override float[] Heuristic()
    {
        int lat, longi;

        if (Input.GetKey(KeyCode.A))
            lat = k_LeftLaneChange;
        else if (Input.GetKey(KeyCode.D))
            lat = k_RightLaneChange;
        else
            lat = k_KeepLane;

        if (Input.GetKey(KeyCode.W))
            longi = k_Cruise;
        else
            longi = k_Follow;

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

    private void OnCutOff(int InstanceID)
    {
        if (InstanceID == control.GetInstanceID())
        {
            Debug.Log("Cut off vehicle!");
            SetReward(-0.2f);
        }
            
    }

    private void OnTriggerEnter(Collider other)
    {
        if (other.gameObject.CompareTag("Target"))
        {
            SetReward(1f);
            EndEpisode();
        }
    }

    private void OnCollisionEnter(Collision collision)
    {
        if (collision.gameObject.CompareTag("Vehicle") || collision.gameObject.CompareTag("Railing"))
        {
            SetReward(-1f);
            EndEpisode();
        }
    }

    private void OnDestroy()
    {
        Events.Instance.onCutOff -= OnCutOff;
    }

}
