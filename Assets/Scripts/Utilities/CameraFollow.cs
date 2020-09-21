/*
 * This code is part of DRL-Highway by Daniel (2020)
 * 
 */

using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace CameraBehaviour
{
    [RequireComponent(typeof(Camera))]
    public class CameraFollow : MonoBehaviour
    {
        public enum _CameraMode { Fixed, LookAt };
        [SerializeField] private _CameraMode cameraMode = _CameraMode.Fixed;
        [SerializeField] Transform objectToFollow;

        [Header("Fixed mode configuration")]
        [SerializeField] Vector3 localPosition;
  
        [Header("LockedOn mode configuration")]
        [SerializeField] Vector3 offset;
        [SerializeField] float followSpeed = 10f;
        [SerializeField] float turnSpeed = 10f;

        private void FixedUpdate()
        {
            if (objectToFollow == null)
                return;

            switch(cameraMode)
            {
                case _CameraMode.Fixed:
                    transform.SetParent(objectToFollow, false);
                    transform.localRotation = Quaternion.identity;
                    transform.localPosition = localPosition;
                    break;
                case _CameraMode.LookAt:
                    transform.SetParent(null);
                    Quaternion rotation = Quaternion.LookRotation(objectToFollow.position - transform.position, Vector3.up);
                    Vector3 position = objectToFollow.TransformPoint(offset);
                    transform.rotation = Quaternion.Lerp(transform.rotation, rotation, turnSpeed * Time.fixedDeltaTime);
                    transform.position = Vector3.Lerp(transform.position, position, followSpeed * Time.fixedDeltaTime);
                    break;
                default:
                    break; 
            }

        }

        public void LookAtTarget()
        {
            Vector3 lookDirection = objectToFollow.position - transform.position;
            Quaternion _rot = Quaternion.LookRotation(lookDirection, Vector3.up);
            transform.rotation = Quaternion.Lerp(transform.rotation, _rot, turnSpeed * Time.deltaTime);
        }

        public void MoveToTarget()
        {
            Vector3 _targetPos = objectToFollow.position +
                                 objectToFollow.forward * offset.z +
                                 objectToFollow.right * offset.x +
                                 objectToFollow.up * offset.y;
            transform.position = Vector3.Lerp(transform.position, _targetPos, followSpeed * Time.deltaTime);
        }

    }
}

