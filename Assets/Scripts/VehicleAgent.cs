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
    [SerializeField] private RaySensor frontRadarL;
    [SerializeField] private RaySensor frontRadarR;
    [SerializeField] private RaySensor backRadarL;
    [SerializeField] private RaySensor backRadarR;

    private const int k_LeftLaneChange = 1;
    private const int k_KeepLane = 2;
    private const int k_RightLaneChange = 3;
    private const int k_Follow = 1;
    private const int k_Cruise = 2;

    [Header("Parameters")]
    public int targetLane;
    public float targetVelocity;
    public float minVelocity;
    private List<float> laneCenters;

    private const float m_MaxFollowDistance = 55f;
    private const float m_GridSize = 100f;

    private Vector3 initialLocalPos;
    private float initialVelocity;
    public List<float> observationGrid;
    public List<bool> laneObs = new List<bool>();

    private RandomNumber randomNumber = new RandomNumber();

    public override void Initialize()
    {
        environment = GetComponentInParent<EnvironmentManager>();
        if (environment == null)
            Debug.LogError("Missing environment reference!");
        laneCenters = environment.centerList;

        control = GetComponent<VehicleControl>();
    }

    public override void OnEpisodeBegin()
    {
        Target = GameObject.FindGameObjectWithTag("Target").transform;
        //float episodeLength = Academy.Instance.FloatProperties.GetPropertyWithDefault("episode_length", 6000f);
        //Target.localPosition = new Vector3(0f, 0.5f, episodeLength);

        int startLane = randomNumber.Next(2, 4);
        int startPos = randomNumber.Next(1, 1);

        float trafficFlow = Academy.Instance.FloatProperties.GetPropertyWithDefault("traffic_flow", 6000f);
        (initialLocalPos, initialVelocity) = environment.ResetArea(startLane, startPos);
        transform.localPosition = initialLocalPos;
        transform.localRotation = Quaternion.identity;
        control.targetVelocity = targetVelocity;
        control.SetInitialVelocity(initialVelocity);

        targetLane = startLane;
        control.currentLane = startLane;
        control.laneCenter = laneCenters[targetLane - 1];
        control._TrackingMode = VehicleControl.TrackingMode.keepLane;

        Events.Instance.NewEpisode();
    }

    private void FixedUpdate()
    {
        // Only request a new decision if agent is not performing a lane change
        if (control._TrackingMode == VehicleControl.TrackingMode.keepLane)
            RequestDecision();

        // Regulate car controls
        control.currentLane = GetCurrentLane();
        control.laneCenter = laneCenters[targetLane - 1];
        control.followTarget = GetClosestVehicle(environment.trafficList, targetLane, 1, m_MaxFollowDistance);
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        //sensor.AddObservation(transform.localPosition.z / Target.localPosition.z);
        sensor.AddObservation((control.velocity - minVelocity) / (targetVelocity - minVelocity));
        sensor.AddObservation(control.headway);

        laneObs = new List<bool>();
        for (int i = 0; i < laneCenters.Count; i++)
        {
            sensor.AddObservation(control.currentLane == i + 1 ? true : false);
            laneObs.Add(control.currentLane == i + 1 ? true : false);
        }

        ////sensor.AddObservation(control.isFollowing);

        //if (control.followTarget != null)
        //    sensor.AddObservation(true);
        //else
        //    sensor.AddObservation(false);

        //SensorData data = sideRadarL.GetData();
        //foreach (RayPoint point in data.sensorPoints)
        //{
        //    sensor.AddObservation(point.normDistance);
        //}

        //foreach (RaySensor raySensor in new RaySensor[] { lidar, sideRadarL, sideRadarR, backRadarL, backRadarR })
        //{
        //    SensorData data = raySensor.GetData();
        //    foreach (RayPoint point in data.sensorPoints)
        //    {
        //        sensor.AddObservation(point.normDistance);
        //    }
        //}

        observationGrid = GetObservationGrid(environment.trafficList);
        foreach (float obs in observationGrid)
        {
            sensor.AddObservation(obs);
        }
    }

    public override void CollectDiscreteActionMasks(DiscreteActionMasker actionMasker)
    {
        // Ensures the car does not crash into the barriers
        if (control._TrackingMode == VehicleControl.TrackingMode.keepLane)
        {
            if (targetLane == 1)
                actionMasker.SetMask(0, new int[] { k_LeftLaneChange });
            if (targetLane == laneCenters.Count)
                actionMasker.SetMask(0, new int[] { k_RightLaneChange });
        }

        // Ensures no lane change when warnings are on
        foreach (RaySensor raySensor in new RaySensor[] { sideRadarL, frontRadarL, backRadarL })
        {
            SensorData data = raySensor.GetData();
            foreach (RayPoint point in data.sensorPoints)
            {
                if (point.isHit)
                {
                    actionMasker.SetMask(0, new int[] { k_LeftLaneChange });
                    break;
                }

            }
        }

        // Ensures no lane change when warnings are on
        foreach (RaySensor raySensor in new RaySensor[] { sideRadarR, frontRadarR, backRadarR })
        {
            SensorData data = raySensor.GetData();
            foreach (RayPoint point in data.sensorPoints)
            {
                if (point.isHit)
                {
                    actionMasker.SetMask(0, new int[] { k_RightLaneChange });
                    break;
                }

            }
        }

        // Ensures that a lane change can not be cancelled once initiated
        //if (control._TrackingMode == VehicleControl.TrackingMode.leftLaneChange)
        //    actionMasker.SetMask(0, new int[] { k_KeepLane, k_RightLaneChange });
        //if (control._TrackingMode == VehicleControl.TrackingMode.rightLaneChange)
        //    actionMasker.SetMask(0, new int[] { k_LeftLaneChange, k_KeepLane });


    }

    public override void OnActionReceived(float[] vectorAction)
    {
        var lateralAction = Mathf.FloorToInt(vectorAction[0]);

        //var longiAction = Mathf.FloorToInt(vectorAction[1]);
        //switch (longiAction)
        //{
        //    //case k_DoNothing:
        //    //    control.isFollowing = true;
        //    //    break;
        //    case k_Follow:
        //        control.isFollowing = true;
        //        break;
        //    case k_Cruise:
        //        control.isFollowing = false;
        //        break;
        //    default:
        //        control.isFollowing = true;
        //        //throw new ArgumentException("Invalid action value");
        //        break;
        //}

        // Normalized speed reward
        float normSpeed = (control.velocity - minVelocity) / (targetVelocity - minVelocity);
        AddReward(0.01f * normSpeed);

        switch (lateralAction)
        {
            case k_LeftLaneChange:
                if (control._TrackingMode != VehicleControl.TrackingMode.leftLaneChange)
                {
                    control._TrackingMode = VehicleControl.TrackingMode.leftLaneChange;
                    if (targetLane != 1)
                        targetLane--;
                    //SetReward(-0.05f);
                    Events.Instance.LaneChange();
                }
                //control.isFollowing = true;
                AddReward(-0.005f);
                break;
            case k_RightLaneChange:
                if (control._TrackingMode != VehicleControl.TrackingMode.rightLaneChange)
                {
                    control._TrackingMode = VehicleControl.TrackingMode.rightLaneChange;
                    if (targetLane != laneCenters.Count)
                        targetLane++;
                    //SetReward(-0.05f);
                    Events.Instance.LaneChange();
                }
                //control.isFollowing = true;
                AddReward(-0.005f);
                break;
            default:
                break;
        }

        // Penalty for tailgating i.e. dangerous driving
        if (control.headway < 0.9f)
            AddReward(-0.005f);
        //AddReward(-0.03f * (1 - control.headway));

        // Reward for driving as much right as possible
        AddReward(control.currentLane * 0.001f);
        //if (control.currentLane <= Mathf.CeilToInt(laneCenters.Count / 3))
        //    AddReward(-0.005f);

    }

    public override float[] Heuristic()
    {
        //int lat, longi;

        //if (Input.GetKey(KeyCode.A))
        //    lat = k_LeftLaneChange;
        //else if (Input.GetKey(KeyCode.D))
        //    lat = k_RightLaneChange;
        //else
        //    lat = k_KeepLane;

        //if (Input.GetKey(KeyCode.W))
        //    longi = k_Cruise;
        //else
        //    longi = k_Follow;

        //return new float[] { lat, longi };

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



    private List<float> GetObservationGrid(List<VehicleControl> vehicles)
    {
        List<float> proximity = new List<float>();
        Vector3 position = transform.localPosition;

        for (int i = 0; i < laneCenters.Count; i++)
        {
            foreach (int dir in new int[] { 1, -1 })
            {
                VehicleControl inProx = GetClosestVehicle(vehicles, i + 1, dir, m_GridSize);
                float pos = (inProx?.transform.localPosition.z ?? dir * m_GridSize + position.z) - position.z;
                proximity.Add(pos / m_GridSize);
                //float vel = (inProx?.velocity ?? control.velocity) - control.velocity;
                //proximity.Add((vel - minVelocity) / (targetVelocity - minVelocity));
                float normVel = ((inProx?.velocity ?? control.velocity) - minVelocity) / (targetVelocity - minVelocity);
                float agentVel = (control.velocity - minVelocity) / (targetVelocity - minVelocity);
                proximity.Add(normVel - agentVel);
            }
        }

        return proximity;
    }

    private VehicleControl GetClosestVehicle(List<VehicleControl> vehicles, int lane, int dir, float maxDis)
    {
        VehicleControl target = null;
        float z = transform.localPosition.z;
        float dz = Mathf.Infinity;

        foreach (VehicleControl vehicle in vehicles)
        {
            if (vehicle == control || vehicle.currentLane != lane || !vehicle.isActiveAndEnabled)
                continue;

            float distance = vehicle.transform.localPosition.z - z;
            if (Mathf.Sign(distance) == dir && Math.Abs(distance) <= maxDis)
            {
                target = vehicle;
                dz = Math.Abs(distance);
            }
        }

        return target;
    }

    private void OnCutOff(int InstanceID)
    {
        if (InstanceID == control.GetInstanceID())
        {
            //Debug.Log("Cut off vehicle!");
            SetReward(-0.5f);
        }   
    }

    private void OnTriggerEnter(Collider other)
    {
        if (other.gameObject.CompareTag("Target"))
        {
            //SetReward(5f);
            EndEpisode();
        }
    }

    private void OnCollisionEnter(Collision collision)
    {
        if (collision.gameObject.CompareTag("Vehicle") || collision.gameObject.CompareTag("Railing"))
        {
            SetReward(-1f);
            Events.Instance.Crash();
            EndEpisode();
        }
    }

    private void OnDisable()
    {
        //Events.Instance.onCutOff -= OnCutOff;
    }

    private List<float> GetVehiclesInProximity(List<VehicleControl> vehicles, int lane)
    {
        List<float> proximity = new List<float>();
        Vector3 position = transform.localPosition;

        foreach (int i in new int[] { lane - 2, lane - 1, lane, lane + 1, lane + 2 })
        {
            foreach (int dir in new int[] { 1, -1 })
            {
                if (i < 1 || i > laneCenters.Count)
                {
                    float x = i < 1 ? -environment.laneWidth : environment.laneWidth;
                    proximity.AddRange(new float[] { x, dir * m_GridSize, 0f });
                }
                else
                {
                    VehicleControl vehicle = GetClosestVehicle(vehicles, i, dir, m_GridSize);
                    if (vehicle != null)
                    {
                        proximity.Add(vehicle.transform.localPosition.x - position.x);
                        proximity.Add(vehicle.transform.localPosition.z - position.z);
                        proximity.Add(vehicle.velocity - control.velocity);
                    }
                    else
                        proximity.AddRange(new float[] { dir * environment.laneWidth, dir * m_GridSize, 0f });
                }
            }
        }

        return proximity;
    }

}
