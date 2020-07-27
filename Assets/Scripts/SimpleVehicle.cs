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
    private EnvironmentManager environment;

    private float m_MaxFollowDistance = 100f;

    [Header("Parameters")]
    public int targetLane;
    public float initialVelocity;
    public float targetVelocity;
    private List<float> laneCenters;

    private bool isCutOff;

    private int currentLane;
    private float center;
    private float z;
    private float dz;

    private void Awake()
    {
        environment = GetComponentInParent<EnvironmentManager>();
        laneCenters = environment.centerList;

        control = GetComponent<VehicleControl>();
    }

    private void OnEnable()
    {
        isCutOff = false;
        control.laneCenter = laneCenters[targetLane - 1];
        control.currentLane = targetLane;
        control.targetVelocity = targetVelocity;
        control.SetInitialVelocity(initialVelocity);
    }

    private void FixedUpdate()
    {
        currentLane = GetCurrentLane();
        control.currentLane = currentLane;
        control.followTarget = GetClosestVehicle(environment.trafficList);

        if (control.followTarget != null)
        {
            if (control.headway <= 0.4f && control.followTarget._TrackingMode != VehicleControl.TrackingMode.keepLane)
            {
                if (!isCutOff)
                {
                    isCutOff = true;
                    Events.Instance.CutOff(control.followTarget.GetInstanceID());
                }
            }
            else if (control.headway > 0.4f)
                isCutOff = false;
        }
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
            if (vehicle == control || vehicle.currentLane != currentLane || !vehicle.isActiveAndEnabled)
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

    private void OnCollisionEnter(Collision collision)
    {
        if (collision.gameObject.CompareTag("Vehicle"))
            environment.Despawn(gameObject);
    }

    private void OnTriggerEnter(Collider other)
    {
        if (other.gameObject.CompareTag("Finish"))
        {
            environment.Despawn(gameObject);
            environment.Spawn(new Vector3(laneCenters[targetLane - 1], 0f, 0f), targetLane, targetVelocity);
        }
    }

}
