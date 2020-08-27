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

    // Properties
    public int TargetLane { get; set; }
    public float TargetVelocity { get; set; }

    [Header("Appearance")]
    [SerializeField] private MeshRenderer[] bodyMeshes;
    [SerializeField] private Color[] colors;

    private const float m_MaxFollowDistance = 100f;
    private List<float> laneCenters;
    private int currentLane;

    private RandomNumber randomNumber = new RandomNumber();

    private void Awake()
    {
        environment = GetComponentInParent<EnvironmentManager>();
        laneCenters = environment.LaneCenters;

        control = GetComponent<VehicleControl>();

        Color color = colors[randomNumber.Next(colors.Length - 1)];
        foreach (MeshRenderer mesh in bodyMeshes)
        {
            mesh.material.color = color;
        }
    }

    private void OnEnable()
    {
        control.LaneCenter = laneCenters[TargetLane - 1];
        control.Lane = TargetLane;
    }

    private void FixedUpdate()
    {
        currentLane = GetCurrentLane();
        control.Lane = currentLane;
        control.followTarget = GetClosestVehicle(environment.Traffic);

        if (control.followTarget != null && control.followTarget?.TrackingMode != VehicleControl._TrackingMode.keepLane)
        {
            if (control.Headway <= 0.6f || control.Throttle <= -500f)
                Events.Instance.CutOff(control.followTarget.GetInstanceID());
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

        foreach (VehicleControl vehicle in vehicles)
        {
            if (vehicle == control || vehicle.Lane != currentLane || !vehicle.isActiveAndEnabled)
                continue;

            float distance = vehicle.transform.localPosition.z - z;
            if (Mathf.Sign(distance) == 1 && (Math.Abs(distance) <= m_MaxFollowDistance) && (Math.Abs(distance) < dz))
            {
                target = vehicle;
                dz = Math.Abs(distance);
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
            environment.Spawn(new Vector3(laneCenters[TargetLane - 1], 0f, 0f), TargetLane, TargetVelocity);
        }
    }

}
