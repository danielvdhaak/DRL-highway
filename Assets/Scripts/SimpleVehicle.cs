/*
 * This code is part of DRL-Highway by Daniel (2020)
 * 
 */

using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
[RequireComponent(typeof(VehicleControl))]
public class SimpleVehicle : MonoBehaviour
{ 
    private VehicleControl control;

    [Header("Environment parameters (INPUT)")]
    public List<VehicleControl> trafficList;
    public List<float> laneCenters;
    public float laneWidth;
    public int targetLane;
    public float initialVelocity;
    public float desiredVelocity;

    private float center;
    private float z;
    private float dz;
    private int currentLane;



    private void Awake()
    {
        control = GetComponent<VehicleControl>();
    }

    private void OnEnable()
    {
        control.environmentSpace = transform.parent.transform;
        control.lc_Width = laneWidth;
        control.laneCenter = laneCenters[targetLane - 1];
        control.currentLane = targetLane;
        control.desiredVelocity = desiredVelocity;
        control.SetInitialVelocity(initialVelocity);
    }

    private void FixedUpdate()
    {
        currentLane = GetCurrentLane();
        control.currentLane = currentLane;
        control.followTarget = GetClosestVehicle(trafficList);
    }

    private int GetCurrentLane()
    {
        List<float> errors = laneCenters.Select(c => Math.Abs(c - transform.localPosition.x)).ToList();

        return errors.IndexOf(errors.Min()) + 1;
    }

    private VehicleControl GetClosestVehicle(List<VehicleControl> vehicles)
    {
        VehicleControl target = null;
        float z = transform.localPosition.z;
        float dz = Mathf.Infinity;

        foreach(VehicleControl vehicle in vehicles)
        {
            if (vehicle == control || vehicle.currentLane != currentLane)
                continue;

            if (vehicle.transform.localPosition.z > z && Math.Abs(vehicle.transform.localPosition.z - z) < dz)
            {
                target = vehicle;
                dz = Math.Abs(vehicle.transform.localPosition.z - z);
            }
        }

        return target;
    }

}
