/*
 * This code is part of DRL-Highway by Daniel (2020)
 * 
 */

using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using System.Linq;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;

[RequireComponent(typeof(Rigidbody))]
[RequireComponent(typeof(VehicleControl))]
public class VehicleAgent : Agent
{
    [Header("ML-Agents")]
    public float normSpeed;
    public List<float> observationGrid;
    public List<bool> laneObs = new List<bool>();
    public bool keepRight;
    [SerializeField] private float decisionInterval = 0.1f;
    private float timeSinceDecision;
    [SerializeField] private Text rewardField;
    [SerializeField] private Text speedField;
    private EnvironmentManager environment;
    private VehicleControl control;
    private Transform Target;

    private const int k_LeftLaneChange = 0;
    private const int k_KeepLane = 1;
    private const int k_RightLaneChange = 2;

    [Header("Safety module")]
    public bool applyToHeuristic = false;
    public float minClearance = 5f;
    public float minTTC = 1.5f;

    [Header("Reward function")]
    public float r_Speed = 0.01f;
    public float r_Overtake = 0.02f;
    public float r_LaneChange = -0.005f;
    public float r_Collision = -1f;
    public float r_HeadwayViolation = -0.05f;
    public float r_DangerousDriving = -0.05f;
    public float r_LeftDriving = -0.002f;

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
        laneCenters = environment.LaneCenters;

        control = GetComponent<VehicleControl>();

        Events.Instance.OnCutOff += OnCutOff;
        Events.Instance.OnOvertake += OnOvertake;

        m_Recorder = Academy.Instance.StatsRecorder;
    }

    public override void OnEpisodeBegin()
    {
        Target = GameObject.FindGameObjectWithTag("Target").transform;

        int startLane = randomNumber.Next(2,4);
        startLane = 2;
        int startPos = randomNumber.Next(2,6);

        //(initialLocalPos, initialVelocity) = environment.ResetArea(startLane, startPos);
        //transform.localPosition = initialLocalPos;
        //transform.localRotation = Quaternion.identity;
        environment.ResetArea(startLane, startPos);
        control.TargetVelocity = targetVelocity;
        //control.SetInitialVelocity(initialVelocity);

        targetLane = startLane;
        control.Lane = startLane;
        control.LaneCenter = laneCenters[targetLane - 1];
        control.TrackingMode = VehicleControl._TrackingMode.keepLane;

        Events.Instance.NewEpisode();
    }


    private void Update()
    {
        // Log velocity in Tensorboard
        if ((Time.frameCount % 100) == 0)
        {
            m_Recorder.Add("Metrics/Velocity", (control.Velocity - minVelocity) / (targetVelocity - minVelocity));
        }

        rewardField.text = GetCumulativeReward().ToString();
        speedField.text = normSpeed.ToString();
    }

    private void FixedUpdate()
    {
        // Check whether right lane has space
        if (control.Lane <= Mathf.CeilToInt(laneCenters.Count / 3))
        {
            bool hasSpaceFront, hasSpaceBehind;
            VehicleControl rightLead = GetClosestVehicle(environment.Traffic, control.Lane + 1, 1, 300f);
            VehicleControl rightBehind = GetClosestVehicle(environment.Traffic, control.Lane + 1, -1, 300f);

            // Check space in front
            if (rightLead == null)
                hasSpaceFront = true;
            else
            {
                float gap = environment.transform.InverseTransformDirection(rightLead.Back.position - control.Front.position).z;
                float TTC = gap / ((control.Velocity - rightLead.Velocity) / 3.6f);
                hasSpaceFront = (Math.Sign(gap) == 1 && Math.Sign(TTC) == 1 && TTC >= 20f) ? true : false;
            }

            // Check space behind
            if (rightBehind == null)
                hasSpaceBehind = true;
            else
            {
                float gap = environment.transform.InverseTransformDirection(control.Back.position - rightBehind.Front.position).z;
                hasSpaceBehind = (Math.Sign(gap) == 1 && Mathf.Clamp(gap / rightBehind.Spacing, 0f, 1f) > 0.6f) ? true : false;
            }

            // Determine clearance flag
            keepRight = (hasSpaceFront & hasSpaceBehind) ? true : false;
        }
        else
            keepRight = false;

        // Only request decision while lanekeeping per interval
        if (control.TrackingMode == VehicleControl._TrackingMode.keepLane)
        {
            if (timeSinceDecision >= decisionInterval)
            {
                timeSinceDecision = 0f;
                RequestDecision();
            }
            else
            {
                //RequestAction();
                timeSinceDecision += Time.fixedDeltaTime;
            }
        }

        // Normalized speed reward
        normSpeed = (control.Velocity - minVelocity) / (targetVelocity - minVelocity);
        AddReward(r_Speed * normSpeed);

        // Penalty for tailgating i.e. dangerous driving
        if (control.Headway < 0.6f)
            AddReward(r_HeadwayViolation);

        //if (control.Lane > Mathf.CeilToInt(laneCenters.Count / 3))
        //    AddReward(r_LeftDriving);

        // Penalty for lane changing
        if (control.TrackingMode == VehicleControl._TrackingMode.leftLaneChange || control.TrackingMode == VehicleControl._TrackingMode.rightLaneChange)
            AddReward(r_LaneChange);

        // Penalty for driving on overtake lanes
        if (keepRight)
            AddReward(r_LeftDriving);

        // Regulate car controls
        control.Lane = GetCurrentLane();
        control.LaneCenter = laneCenters[targetLane - 1];
        control.followTarget = GetClosestVehicle(environment.Traffic, targetLane, 1, m_MaxFollowDistance);
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // Normalized velocity and headway
        sensor.AddObservation((control.Velocity - minVelocity) / (targetVelocity - minVelocity));
        sensor.AddObservation(control.Headway);

        // One hot-style lane position
        laneObs = new List<bool>();
        for (int i = 0; i < laneCenters.Count; i++)
        {
            sensor.AddObservation(GetCurrentLane() == i + 1 ? 1.0f : 0.0f);
            laneObs.Add(GetCurrentLane() == i + 1 ? true : false);
        }

        // Observation grid (relative positions & velocities)
        observationGrid = GetObservationGrid(environment.Traffic);
        foreach (float obs in observationGrid)
        {
            sensor.AddObservation(obs);
        }

        // Clear overtake lane flag
        sensor.AddObservation(keepRight ? 1.0f : 0.0f);
    }

    public override void CollectDiscreteActionMasks(DiscreteActionMasker actionMasker)
    {
        // Ensures the car can not abort lane changes
        if (control.TrackingMode == VehicleControl._TrackingMode.leftLaneChange)
        {
            actionMasker.SetMask(0, new int[] { k_KeepLane, k_RightLaneChange });
            return;
        }
        if (control.TrackingMode == VehicleControl._TrackingMode.rightLaneChange)
        {
            actionMasker.SetMask(0, new int[] { k_KeepLane, k_LeftLaneChange });
            return;
        }

        // Ensures the car does not crash into the barriers
        if (control.Lane == 1)
            actionMasker.SetMask(0, new int[] { k_LeftLaneChange });
        if (control.Lane == laneCenters.Count)
            actionMasker.SetMask(0, new int[] { k_RightLaneChange });

        // Ensures no lane changes when cars are in minimum clearance
        if (control.Lane != 1)
        {
            VehicleControl front = GetClosestVehicle(environment.Traffic, control.Lane - 1, 1, 100f);
            VehicleControl back = GetClosestVehicle(environment.Traffic, control.Lane - 1, -1, 100f);

            if (!IsClearTo(front) || !IsClearTo(back))
                actionMasker.SetMask(0, new int[] { k_LeftLaneChange });
        }
        if (control.Lane != laneCenters.Count)
        {
            VehicleControl front = GetClosestVehicle(environment.Traffic, control.Lane + 1, 1, 100f);
            VehicleControl back = GetClosestVehicle(environment.Traffic, control.Lane + 1, -1, 100f);

            if (!IsClearTo(front) || !IsClearTo(back))
                actionMasker.SetMask(0, new int[] { k_RightLaneChange });
        }
    }

    public override void OnActionReceived(float[] vectorAction)
    {
        var action = Mathf.FloorToInt(vectorAction[0]);

        // Lane change initiation
        switch (action)
        {
            case k_LeftLaneChange:
                if (control.TrackingMode != VehicleControl._TrackingMode.leftLaneChange)
                {
                    control.TrackingMode = VehicleControl._TrackingMode.leftLaneChange;
                    if (targetLane != 1)
                        targetLane--;
                    Events.Instance.LaneChange();
                }
                //AddReward(r_LaneChange);
                break;
            case k_RightLaneChange:
                if (control.TrackingMode != VehicleControl._TrackingMode.rightLaneChange)
                {
                    control.TrackingMode = VehicleControl._TrackingMode.rightLaneChange;
                    if (targetLane != laneCenters.Count)
                        targetLane++;
                    Events.Instance.LaneChange();
                }
                //AddReward(r_LaneChange);
                break;
            default:
                break;
        }
    }

    public override void Heuristic(float[] actionsOut)
    {
        actionsOut[0] = k_KeepLane;

        if (Input.GetKey(KeyCode.A))
        {
            if (applyToHeuristic)
            {
                if (control.Lane != 1)
                {
                    VehicleControl front = GetClosestVehicle(environment.Traffic, control.Lane - 1, 1, 100f);
                    VehicleControl back = GetClosestVehicle(environment.Traffic, control.Lane - 1, -1, 100f);

                    if (IsClearTo(front) && IsClearTo(back))
                        actionsOut[0] = k_LeftLaneChange;
                }
            }
            else
                actionsOut[0] = k_LeftLaneChange;
        }

        if (Input.GetKey(KeyCode.D))
        {
            if (applyToHeuristic)
            {
                if (control.Lane != laneCenters.Count)
                {
                    VehicleControl front = GetClosestVehicle(environment.Traffic, control.Lane + 1, 1, 100f);
                    VehicleControl back = GetClosestVehicle(environment.Traffic, control.Lane + 1, -1, 100f);

                    if (IsClearTo(front) && IsClearTo(back))
                        actionsOut[0] = k_RightLaneChange;
                }
            }
            else
                actionsOut[0] = k_RightLaneChange;
        }
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

        //for (int i = 0; i < laneCenters.Count; i++)
        //{
        //    foreach (int dir in new int[] { 1, -1 })
        //    {
        //        VehicleControl inProx = GetClosestVehicle(vehicles, i + 1, dir, m_GridSize);
        //        float pos = inProx?.transform.localPosition.z ?? (dir * m_GridSize + position.z);
        //        proximity.Add((pos - position.z) / m_GridSize);
        //        float normVel = ((inProx?.Velocity ?? control.Velocity) - minVelocity) / (targetVelocity - minVelocity);
        //        float agentVel = (control.Velocity - minVelocity) / (targetVelocity - minVelocity);
        //        proximity.Add(normVel - agentVel);
        //    }
        //}

        foreach (int i in new int[] { -1, 0, 1 })
        {
            foreach (int dir in new int[] { 1, -1 })
            {
                VehicleControl inProx = GetClosestVehicle(vehicles, control.Lane + i, dir, m_GridSize);
                float pos = inProx?.transform.localPosition.z ?? (dir * m_GridSize + position.z);
                proximity.Add((pos - position.z) / m_GridSize);
                float normVel = ((inProx?.Velocity ?? control.Velocity) - minVelocity) / (targetVelocity - minVelocity);
                float agentVel = (control.Velocity - minVelocity) / (targetVelocity - minVelocity);
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
            if (vehicle == control || vehicle.Lane != lane || !vehicle.isActiveAndEnabled)
                continue;

            float distance = vehicle.transform.localPosition.z - z;
            if (Mathf.Sign(distance) == dir && (Math.Abs(distance) <= maxDis) && (Math.Abs(distance) < dz))
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
                gap = environment.transform.InverseTransformDirection(control.Back.position - vehicle.Front.position).z;
                TTC = gap / ((vehicle.Velocity - control.Velocity) / 3.6f);
                break;
            case 1:
                // Vehicle is in front
                gap = environment.transform.InverseTransformDirection(vehicle.Back.position - control.Front.position).z;
                TTC = gap / ((control.Velocity - vehicle.Velocity) / 3.6f);
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

    private void OnCutOff(int InstanceID)
    {
        if (InstanceID == control.GetInstanceID())
            AddReward(r_DangerousDriving);
    }

    private void OnOvertake(int InstanceID, int dir)
    {
        if (InstanceID == control.GetInstanceID())
        {
            // Reward left and penalize right overtakes
            switch (dir)
            {
                case -1:
                    AddReward(r_Overtake);
                    break;
                case 1:
                    AddReward(-r_Overtake);
                    break;
                default:
                    return;
            }
        }
    }

    private void OnTriggerEnter(Collider other)
    {
        if (other.gameObject.CompareTag("Target"))
        {
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
