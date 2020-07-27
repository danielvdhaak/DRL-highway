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
    [SerializeField] private RaySensor lidar;
    [SerializeField] private RaySensor sideRadarL;
    [SerializeField] private RaySensor sideRadarR;
    [SerializeField] private RaySensor backRadarL;
    [SerializeField] private RaySensor backRadarR;

    private const int k_DoNothing = 0;
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
        //float episodeLength = Academy.Instance.FloatProperties.GetPropertyWithDefault("episode_length", 6000f);
        //Target.localPosition = new Vector3(0f, 0.5f, episodeLength);

        int startLane = randomNumber.Next(2, 4);
        int startPos = randomNumber.Next(1, 1);

        float trafficFlow = Academy.Instance.FloatProperties.GetPropertyWithDefault("traffic_flow", 6000f);
        (initialLocalPos, initialVelocity) = environment.ResetArea(startLane, startPos, trafficFlow);
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
        //if (control._TrackingMode == VehicleControl.TrackingMode.keepLane)
        //    RequestDecision();
        //else
        //    RequestAction();
        RequestDecision();
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        sensor.AddObservation(transform.localPosition.z / Target.localPosition.z);
        //sensor.AddObservation(control.velocity);
        sensor.AddObservation(control.velocity / targetVelocity);

        foreach (float lane in laneCenters)
        {
            if (control.currentLane == lane)
                sensor.AddObservation(true);
            else
                sensor.AddObservation(false);
        }

        sensor.AddObservation(control.isFollowing);
        sensor.AddObservation(control.headway);

        foreach (RaySensor raySensor in new RaySensor[] { lidar, sideRadarL, sideRadarR, backRadarL, backRadarR })
        {
            SensorData data = raySensor.GetData();
            foreach (RayPoint point in data.sensorPoints)
            {
                sensor.AddObservation(point.normDistance);
            }
        }
    }

    public override void CollectDiscreteActionMasks(DiscreteActionMasker actionMasker)
    {
        // Ensures the car does not crash into the barriers
        int leftMostLane = 1;
        int rightMostLane = laneCenters.Count;
        if (control._TrackingMode == VehicleControl.TrackingMode.keepLane)
        {
            if (targetLane == leftMostLane)
                actionMasker.SetMask(0, new int[] { k_LeftLaneChange });
            if (targetLane == rightMostLane)
                actionMasker.SetMask(0, new int[] { k_RightLaneChange });
        }

        // Ensures the car does not crash into the preceding car
        if (control.headway < 0.1f)
            actionMasker.SetMask(1, new int[] { k_Cruise });

        // Ensures that a lane change can not be cancelled once initiated
        if (control._TrackingMode == VehicleControl.TrackingMode.leftLaneChange)
            actionMasker.SetMask(0, new int[] { k_KeepLane, k_RightLaneChange });
        if (control._TrackingMode == VehicleControl.TrackingMode.rightLaneChange)
            actionMasker.SetMask(0, new int[] { k_LeftLaneChange, k_KeepLane });
    }

    public override void OnActionReceived(float[] vectorAction)
    {
        var lateralAction = Mathf.FloorToInt(vectorAction[0]);
        var longiAction = Mathf.FloorToInt(vectorAction[1]);

        float normSpeed = (control.velocity - 80f) / (targetVelocity - 80f);
        AddReward(0.1f * normSpeed);
        switch (lateralAction)
        {
            //case k_DoNothing:
            //    normSpeed = control.velocity / targetVelocity;
            //    SetReward(0.01f * normSpeed);
            //    break;
            //case k_KeepLane:
            //    normSpeed = control.velocity / targetVelocity;
            //    SetReward(0.01f * normSpeed);
            //    break;
            case k_LeftLaneChange:
                if (control._TrackingMode != VehicleControl.TrackingMode.leftLaneChange)
                {
                    control._TrackingMode = VehicleControl.TrackingMode.leftLaneChange;
                    if (targetLane != 1)
                        targetLane--;
                    //AddReward(-0.05f);
                }
                AddReward(-0.02f);
                break;
            case k_RightLaneChange:
                if (control._TrackingMode != VehicleControl.TrackingMode.rightLaneChange)
                {
                    control._TrackingMode = VehicleControl.TrackingMode.rightLaneChange;
                    if (targetLane != laneCenters.Count)
                        targetLane++;
                    //AddReward(-0.05f);
                }
                AddReward(-0.02f);
                break;
            default:
                //normSpeed = control.velocity / targetVelocity;
                //AddReward(0.1f * normSpeed);
                //throw new ArgumentException("Invalid action value");
                break;
        }

        control.currentLane = GetCurrentLane();
        control.laneCenter = laneCenters[targetLane - 1];
        control.followTarget = GetClosestVehicle(environment.trafficList, targetLane, 1);

        switch (longiAction)
        {
            //case k_DoNothing:
            //    control.isFollowing = true;
            //    break;
            case k_Follow:
                control.isFollowing = true;
                break;
            case k_Cruise:
                control.isFollowing = false;
                break;
            default:
                control.isFollowing = true;
                //throw new ArgumentException("Invalid action value");
                break;
        }

        // Penalty for tailgating
        if (control.headway < 0.9f)
            AddReward(-0.03f * (1 - control.headway));

        // Penalty for driving on the left
        if (control.currentLane <= Mathf.CeilToInt(laneCenters.Count / 3))
            AddReward(-0.02f);

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
            //Debug.Log("Cut off vehicle!");
            SetReward(-0.2f);
        }   
    }

    private void OnTriggerEnter(Collider other)
    {
        if (other.gameObject.CompareTag("Target"))
        {
            SetReward(5f);
            EndEpisode();
        }
    }

    private void OnCollisionEnter(Collision collision)
    {
        if (collision.gameObject.CompareTag("Vehicle") || collision.gameObject.CompareTag("Railing"))
        {
            SetReward(-5f);
            EndEpisode();
        }
    }

    private void OnDisable()
    {
        //Events.Instance.onCutOff -= OnCutOff;
    }

}
