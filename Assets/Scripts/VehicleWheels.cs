/*
 * This code is part of DRL-Highway by Daniel (2020)
 * 
 */

using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace CarBehaviour {
    [RequireComponent(typeof(VehicleControl))]

    public class VehicleWheels : MonoBehaviour
    {
        [SerializeField] bool renderSteerAngle = true;
        public Vector3 localRotOffset;

        public void TurnWheel(GameObject wheel)
        {
            // Turn the models of an input wheel gameobject
            Transform wheelTransform = wheel.GetComponent<Transform>();
            WheelCollider wheelCol = wheel.GetComponent<WheelCollider>();

            foreach (Transform wheelModel in wheelTransform.transform)
            {
                Vector3 pos = new Vector3(0, 0, 0);
                Quaternion quat = new Quaternion();
                wheelCol.GetWorldPose(out pos, out quat);


                if (renderSteerAngle)
                {
                    wheelModel.transform.rotation = quat;
                }


                wheelModel.transform.localRotation *= Quaternion.Euler(localRotOffset);
                wheelModel.transform.position = pos;

                WheelHit wheelHit;
                wheelCol.GetGroundHit(out wheelHit);
            }
        }
    }
}

