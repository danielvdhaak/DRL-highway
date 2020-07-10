/*
 * This code is part of DRL-Highway by Daniel (2020)
 * 
 */

using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
[RequireComponent(typeof(VehicleControl))]
public class SimpleVehicle : MonoBehaviour
{ 
    private GameObject environment;
    private VehicleControl control;

    public List<VehicleControl> trafficList;
    public List<float> laneCenters;
    public float laneWidth;
    public int targetLane;
    public float initialVelocity;
    public float desiredVelocity;

    private float center;
    private float z;
    private float dz;



    private void Awake()
    {
        // Initialize environment
        environment = transform.parent.gameObject;
        if (environment == null)
        {
            Debug.LogError("Can not reference environment!");
        }

        // Initialize vehicle control module
        control = GetComponent<VehicleControl>();
    }

    private void OnEnable()
    {
        control.lc_Width = laneWidth;
        control.laneCenter = laneCenters[targetLane - 1];
        control.desiredVelocity = desiredVelocity;
        control.environmentSpace = transform.parent.transform;
        control.SetInitialVelocity(initialVelocity);
    }

    private void FixedUpdate()
    {
        control.followTarget = GetClosestVehicle(trafficList);
    }

    private VehicleControl GetClosestVehicle(List<VehicleControl> vehicles)
    {
        VehicleControl target = null;
        float z = transform.localPosition.z;
        float dz = Mathf.Infinity;

        foreach(VehicleControl vehicle in vehicles)
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
