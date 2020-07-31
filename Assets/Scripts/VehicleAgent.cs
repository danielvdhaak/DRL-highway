/*
 * This code is part of DRL-Highway by Daniel (2020)
 * 
 */

using System;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;

[RequireComponent(typeof(Rigidbody))]
[RequireComponent(typeof(VehicleControl))]
public class VehicleAgent : Agent
{
    [Header("ML-Agents")]
    [SerializeField] private RaySensor lidar;
    [SerializeField] private RaySensor sideRadarL;
    [SerializeField] private RaySensor sideRadarR;
    [SerializeField] private RaySensor frontRadarL;
    [SerializeField] private RaySensor frontRadarR;
    [SerializeField] private RaySensor backRadarL;
    [SerializeField] private RaySensor backRadarR;
    private EnvironmentManager environment;
    private VehicleControl control;
    private Transform Target;
    public List<float> observationGrid;
    public List<bool> laneObs = new List<bool>();

    private const int k_LeftLaneChange = 1;
    private const int k_KeepLane = 2;
    private const int k_RightLaneChange = 3;
    private const int k_Follow = 1;
    private const int k_Cruise = 2;

    [Header("Safety module")]
    public float minClearance = 5f;
    public float minTTC = 1.5f;

    [Header("Reward function")]
    public float r_Speed = 0.01f;
    public float r_RightDriving = 0.001f;
    public float r_Collision = -1f;
    public float r_HeadwayViolation = -0.005f;
    public float r_LaneChange = -0.05f;
    public float r_DangerousDriving = -0.005f;

    [Header("Parameters")]
    public int targetLane;
    public float targetVelocity;
    public float minVelocity;
    private List<float> laneCenters;

    private const float m_MaxFollowDistance = 55f;
    private const float m_GridSize = 200f;

    private Vector3 initialLocalPos;
    private float initialVelocity;

    private StatsRecorder m_Recorder;
    private RandomNumber randomNumber = new RandomNumber();

    public override void Initialize()
    {
        environment = GetComponentInParent<EnvironmentManager>();
        if (environment == null)
            Debug.LogError("Missing environment reference!");
        laneCenters = environment.centerList;

        control = GetComponent<VehicleControl>();

        m_Recorder = Academy.Instance.StatsRecorder;
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

    private void Update()
    {
        // Log velocity in Tensorboard
        if ((Time.frameCount % 100) == 0)
        {
            m_Recorder.Add("Metrics/Velocity", (control.velocity - minVelocity) / (targetVelocity - minVelocity));
        }
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

        observationGrid = GetObservationGrid(environment.trafficList);
        foreach (float obs in observationGrid)
        {
            sensor.AddObservation(obs);
        }
    }

    public override void CollectDiscreteActionMasks(DiscreteActionMasker actionMasker)
    {
        // Ensures the car does not crash into the barriers
        if (control.currentLane == 1)
            actionMasker.SetMask(0, new int[] { k_LeftLaneChange });
        if (control.currentLane == laneCenters.Count)
            actionMasker.SetMask(0, new int[] { k_RightLaneChange });

        // Ensures no lane changes when cars are in minimum clearance
        if (control.currentLane != 1)
        {
            VehicleControl front = GetClosestVehicle(environment.trafficList, control.currentLane - 1, 1, 100f);
            VehicleControl back = GetClosestVehicle(environment.trafficList, control.currentLane - 1, -1, 100f);

            if (!IsClearTo(front) || !IsClearTo(back))
                actionMasker.SetMask(0, new int[] { k_LeftLaneChange });
        }
        if (control.currentLane != laneCenters.Count)
        {
            VehicleControl front = GetClosestVehicle(environment.trafficList, control.currentLane + 1, 1, 100f);
            VehicleControl back = GetClosestVehicle(environment.trafficList, control.currentLane + 1, -1, 100f);

            if (!IsClearTo(front) || !IsClearTo(back))
                actionMasker.SetMask(0, new int[] { k_RightLaneChange });
        }


        //// Ensures no lane change when warnings are on
        //foreach (RaySensor raySensor in new RaySensor[] { sideRadarL, frontRadarL, backRadarL })
        //{
        //    SensorData data = raySensor.GetData();
        //    foreach (RayPoint point in data.sensorPoints)
        //    {
        //        if (point.isHit)
        //        {
        //            actionMasker.SetMask(0, new int[] { k_LeftLaneChange });
        //            break;
        //        }
        //    }
        //}

        //// Ensures no lane change when warnings are on
        //foreach (RaySensor raySensor in new RaySensor[] { sideRadarR, frontRadarR, backRadarR })
        //{
        //    SensorData data = raySensor.GetData();
        //    foreach (RayPoint point in data.sensorPoints)
        //    {
        //        if (point.isHit)
        //        {
        //            actionMasker.SetMask(0, new int[] { k_RightLaneChange });
        //            break;
        //        }
        //    }
        //}
    }

    public override void OnActionReceived(float[] vectorAction)
    {
        var action = Mathf.FloorToInt(vectorAction[0]);

        // Normalized speed reward
        float normSpeed = (control.velocity - minVelocity) / (targetVelocity - minVelocity);
        AddReward(r_Speed * normSpeed);

        // Penalty for tailgating i.e. dangerous driving
        if (control.headway < 0.95f)
            AddReward(r_HeadwayViolation);

        // Implement TTC violations here! If it does not work, we have to RequestAction() each FixedUpdate() and implement the mask


        // Reward for driving as much right as possible
        AddReward(control.currentLane * r_RightDriving);

        // Lane change initiation
        switch (action)
        {
            case k_LeftLaneChange:
                control._TrackingMode = VehicleControl.TrackingMode.leftLaneChange;
                if (targetLane != 1)
                    targetLane--;
                Events.Instance.LaneChange();
                SetReward(r_LaneChange);
                break;
            case k_RightLaneChange:
                control._TrackingMode = VehicleControl.TrackingMode.rightLaneChange;
                if (targetLane != laneCenters.Count)
                    targetLane++;
                Events.Instance.LaneChange();
                SetReward(r_LaneChange);
                break;
            default:
                break;
        }
    }

    public override void Heuristic(float[] actionsOut)
    {
        actionsOut[0] = k_KeepLane;

        if (Input.GetKey(KeyCode.A))
            actionsOut[0] = k_LeftLaneChange;
        if (Input.GetKey(KeyCode.D))
            actionsOut[0] = k_RightLaneChange;
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
                float pos = inProx?.transform.localPosition.z ?? (dir * m_GridSize + position.z);
                proximity.Add((pos - position.z) / m_GridSize);
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

    private bool IsClearTo(VehicleControl vehicle)
    {
        if (vehicle == null)
            return true;

        float gap;
        float TTC;
        int dir = Math.Sign(vehicle.transform.localPosition.z - transform.localPosition.z);

        switch (dir)
        {
            case -1:
                // Vehicle is behind
                gap = environment.transform.InverseTransformDirection(control.backOffset.position - vehicle.frontOffset.position).z;
                TTC = gap / (vehicle.velocity - control.velocity);
                break;
            case 1:
                // Vehicle is in front
                gap = environment.transform.InverseTransformDirection(vehicle.backOffset.position - control.frontOffset.position).z;
                TTC = gap / (control.velocity - vehicle.velocity);
                break;
            default:
                return true;
        }

        // Not clear when gap is smaller than minimal clearance
        if (gap < minClearance)
            return false;

        // Check TTC
        if (Math.Sign(TTC) == 1 && TTC <= minTTC)
        {
            return false;
        }
        else
            return true;
    }

    // Will become obsolete!!
    private void OnCutOff(int InstanceID)
    {
        if (InstanceID == control.GetInstanceID())
        {
            Debug.Log("Cut off vehicle!");
            AddReward(-0.005f);
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
            SetReward(r_Collision);
            EndEpisode();
            Events.Instance.Crash();
        }
    }
}
