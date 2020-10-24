/*
 * This code is part of DRL-Highway by Daniel (2020)
 * 
 */

using System.Collections;
using System.Collections.Generic;
using System;
using System.Linq;
using UnityEngine;
using UnityEngine.UI;

[RequireComponent(typeof(Rigidbody))]
[RequireComponent(typeof(VehicleControl))]
public class MobilAgent : MonoBehaviour
{
    [Header("Agent")]
    [SerializeField] private uint MaxSteps = 0;
    private uint fixedUpdateCount = 0;
    [SerializeField] private float decisionInterval = 0.1f;
    public List<float> trafficGrid;
    private float timeSinceDecision;
    private float[] action;
    private EnvironmentManager environment;
    private VehicleControl control;

    [Header("UI/Debugging")]
    [SerializeField] private Text rewardField;
    [SerializeField] private Text velocityField;

    private const int k_LeftLaneChange = 0;
    private const int k_KeepLane = 1;
    private const int k_RightLaneChange = 2;

    [Header("MOBIL parameters")]
    [SerializeField] private float politenessFactor = 0.5f;
    [SerializeField] private float bias = 500f;
    [SerializeField] private float threshold = 200f;
    [SerializeField] private float safetyCriterion = -500f;


    [Header("Parameters")]
    public int targetLane;
    public float targetVelocity;
    public float minVelocity;
    private List<float> laneCenters;

    private const float m_MaxFollowDistance = 55f;
    private const float m_GridSize = 200f;

    private RandomNumber randomNumber = new RandomNumber();

    private void OnEnable()
    {
        // Initialize environment
        environment = GetComponentInParent<EnvironmentManager>();
        if (environment == null)
            Debug.LogError("Missing environment reference!");
        laneCenters = environment.LaneCenters;

        // Initialize vehicle control
        control = GetComponent<VehicleControl>();
    }

    private void Start()
    {
        OnEpisodeBegin();
    }

    private void OnEpisodeBegin()
    {
        // Reset area with given start location
        int startLane = 2;
        int startPos = randomNumber.Next(2, 3);
        environment.ResetArea(startLane, startPos);

        // Set initial control values
        control.TargetVelocity = targetVelocity;
        targetLane = startLane;
        control.Lane = startLane;
        control.LaneCenter = laneCenters[targetLane - 1];
        control.TrackingMode = VehicleControl._TrackingMode.keepLane;

        // Reset FixedUpdate counter
        fixedUpdateCount = 0;

        Events.Instance.NewEpisode();
    }

    private void FixedUpdate()
    {
        // Get current state
        control.Lane = GetCurrentLane();
        trafficGrid = GetTrafficGrid(environment.Traffic);

        // Only request decision while lanekeeping per interval
        if (control.TrackingMode == VehicleControl._TrackingMode.keepLane)
        {
            if (timeSinceDecision >= decisionInterval)
            {
                timeSinceDecision = 0f;
                RequestDecision();
            }
            else
                timeSinceDecision += Time.fixedDeltaTime;
        }

        // Regulate car controls
        control.LaneCenter = laneCenters[targetLane - 1];
        control.followTarget = GetClosestVehicle(environment.Traffic, targetLane, 1, m_MaxFollowDistance);

        // Manually update fixedUpdate counter and terminate episode if needed
        fixedUpdateCount++;
        if (MaxSteps != 0 && fixedUpdateCount >= MaxSteps)
            OnEpisodeBegin();

    }

    private void RequestDecision()
    {
        if (IsRightLCViable())
        {
            if (control.TrackingMode != VehicleControl._TrackingMode.rightLaneChange)
            {
                control.TrackingMode = VehicleControl._TrackingMode.rightLaneChange;
                if (targetLane != laneCenters.Count)
                    targetLane++;
                Events.Instance.LaneChange();

            }
        } 
        else if (IsLeftLCViable())
        {
            if (control.TrackingMode != VehicleControl._TrackingMode.leftLaneChange)
            {
                control.TrackingMode = VehicleControl._TrackingMode.leftLaneChange;
                if (targetLane != 1)
                    targetLane--;
                Events.Instance.LaneChange();
            }
        }
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

    private int GetCurrentLane()
    {
        List<float> errors = laneCenters.Select(c => Math.Abs(c - transform.localPosition.x)).ToList();

        return errors.IndexOf(errors.Min()) + 1;
    }

    private float[] GetAction()
    {
        switch (control.TrackingMode)
        {
            case VehicleControl._TrackingMode.leftLaneChange:
                return new float[] { k_LeftLaneChange };

            case VehicleControl._TrackingMode.keepLane:
                return new float[] { k_KeepLane };

            case VehicleControl._TrackingMode.rightLaneChange:
                return new float[] { k_RightLaneChange };

            default:
                return new float[] { 0f };
        }
    }

    private bool IsRightLCViable()
    {
        // Always return false if vehicle is in right-most lane
        if (control.Lane == laneCenters.Count)
            return false;

        bool isSafe, hasIncentive;
        VehicleControl lag = GetClosestVehicle(environment.Traffic, control.Lane, -1, m_GridSize);
        VehicleControl lead = GetClosestVehicle(environment.Traffic, control.Lane, 1, m_GridSize);
        VehicleControl RightLead = GetClosestVehicle(environment.Traffic, control.Lane + 1, 1, m_GridSize);
        VehicleControl RightLag = GetClosestVehicle(environment.Traffic, control.Lane + 1, -1, m_GridSize);

        // Throttles
        float T = control.Throttle;
        float newT = control.CalcThrottle(RightLead);
        float lagT = lag?.Throttle ?? 0f;
        float lagNewT = lag?.CalcThrottle(lead) ?? 0f;
        float rightNewT = RightLag?.CalcThrottle(control) ?? 0f;

        // Safety criterion
        isSafe = (rightNewT >= safetyCriterion) ? true : false;

        // Lane change incentive
        hasIncentive = (newT - T + politenessFactor * (lagNewT - lagT) > threshold - bias) ? true : false;

        return (isSafe & hasIncentive);
    }

    private bool IsLeftLCViable()
    {
        // Always return false if vehicle is in left-most lane
        if (control.Lane == 1)
            return false;

        bool isSafe, hasIncentive;
        VehicleControl leftLead = GetClosestVehicle(environment.Traffic, control.Lane - 1, 1, m_GridSize);
        VehicleControl leftLag = GetClosestVehicle(environment.Traffic, control.Lane - 1, -1, m_GridSize);

        // Passing rule
        float T = (control.Velocity > (leftLead?.Velocity ?? control.Velocity)) ? Mathf.Min(control.Throttle, control.CalcThrottle(leftLead)) : control.Throttle;

        // Other throttles
        float newT = control.CalcThrottle(leftLead);
        float leftT = leftLag?.Throttle ?? 0f;
        float leftNewT = leftLag?.CalcThrottle(control) ?? 0f;

        // Safety criterion
        isSafe = (leftNewT >= safetyCriterion) ? true : false;

        // Lane change incentive
        hasIncentive = (newT - T + politenessFactor * (leftNewT - leftT) > threshold + bias) ? true : false;

        return (isSafe & hasIncentive);
    }

    private List<float> GetTrafficGrid(List<VehicleControl> vehicles)
    {
        List<float> grid = new List<float>();
        Vector3 position = transform.localPosition;

        for (int lane = 0; lane < laneCenters.Count; lane++)
        {
            foreach (int dir in new int[] { 1, -1 })
            {
                VehicleControl inProx = GetClosestVehicle(vehicles, lane + 1, dir, m_GridSize);

                // Relative position
                float dz = (inProx?.transform.localPosition.z ?? (dir * m_GridSize + position.z)) - position.z;
                grid.Add(Mathf.Clamp(dz / m_GridSize, -1f, 1f));

                // Relative velocity
                float dv = (inProx?.Velocity ?? control.Velocity) - control.Velocity;
                grid.Add(Mathf.Clamp(dv / (targetVelocity - minVelocity), -1f, 1f));
            }
        }

        return grid;
    }

    public EnvironmentState LogStats()
    {
        float[] a = GetAction();
        float speed = control.Velocity;
        int currentLane = control.Lane;
        float[] tGrid = trafficGrid.ToArray();

        return new EnvironmentState(a, speed, currentLane, tGrid);
    }

    private void OnTriggerEnter(Collider other)
    {
        if (other.gameObject.CompareTag("Target"))
            OnEpisodeBegin();
    }

    private void OnCollisionEnter(Collision collision)
    {
        if (collision.gameObject.CompareTag("Vehicle") || collision.gameObject.CompareTag("Railing"))
        {
            Events.Instance.Crash();
            OnEpisodeBegin();
        }
    }
}
