/*
 * This code is part of DRL-Highway by Daniel (2020)
 * 
 */

using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

public class WheelSuspension : MonoBehaviour
{
    [Serializable]
    public class Wheel
    {
        public WheelCollider wheelCollider;
        public List<GameObject> wheelMeshes;
        public List<GameObject> brakeCalMeshes;
    }

    [SerializeField] private List<Wheel> wheels;

    private void Update()
    {
        foreach(Wheel wheel in wheels)
        {
            ApplyLocalPositionToVisuals(wheel);
        }
    }

    private void ApplyLocalPositionToVisuals(Wheel wheel)
    {
        Vector3 position;
        Quaternion rotation;

        wheel.wheelCollider.GetWorldPose(out position, out rotation);

        // Rotate wheel meshes
        foreach (GameObject mesh in wheel.wheelMeshes)
        {
            mesh.transform.position = position;
            mesh.transform.rotation = rotation;
        }

        // Locally rotate wheel calliphers around Y-axis only
        Quaternion localRotation = (Quaternion.Inverse(transform.rotation) * rotation);
        foreach (GameObject mesh in wheel.brakeCalMeshes)
        {
            mesh.transform.position = position;
            mesh.transform.localEulerAngles = new Vector3(0f, localRotation.eulerAngles.y, 0f);
        }
    }
}
