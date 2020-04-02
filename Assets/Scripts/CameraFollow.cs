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
        public void LookAtTarget()
        {
            Vector3 lookDirection = objectToFollow.position - transform.position;
            Quaternion _rot = Quaternion.LookRotation(lookDirection, Vector3.up);
            transform.rotation = Quaternion.Lerp(transform.rotation, _rot, lookSpeed * Time.deltaTime);
        }

        public void MoveToTarget()
        {
            Vector3 _targetPos = objectToFollow.position +
                                 objectToFollow.forward * offset.z +
                                 objectToFollow.right * offset.x +
                                 objectToFollow.up * offset.y;
            transform.position = Vector3.Lerp(transform.position, _targetPos, followSpeed * Time.deltaTime);
        }

        private void FixedUpdate()
        {
            LookAtTarget();
            MoveToTarget();
        }

        [SerializeField] Transform objectToFollow;
        [SerializeField] Vector3 offset;
        [SerializeField] float followSpeed = 10;
        [SerializeField] float lookSpeed = 10;
    }
}

