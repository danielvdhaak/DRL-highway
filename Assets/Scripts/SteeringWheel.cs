/*
 * This code is part of DRL-Highway by Daniel (2020)
 * 
 */

using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SteeringWheel : MonoBehaviour
{
    private VehicleControl control;
    [SerializeField] private List<GameObject> meshes;
    [SerializeField] private float multiplier;


    private void OnEnable()
    {
        control = GetComponentInParent<VehicleControl>();
    }

    void Update()
    {
        if (control == null)
            return;

        float steeringAngle = control.SteeringAngle;

        foreach(GameObject mesh in meshes)
        {
            Quaternion localRotate = Quaternion.Euler(0f, multiplier * steeringAngle, 0f);
            mesh.transform.localRotation = localRotate;
        }
    }
}
