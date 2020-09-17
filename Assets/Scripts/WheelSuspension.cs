/*
 * This code is part of DRL-Highway by Daniel (2020)
 * 
 */

using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(WheelCollider))]
public class WheelSuspension : MonoBehaviour
{
    [SerializeField] private List<GameObject> wheelModels;
    [SerializeField] private List<GameObject> brakeCals;
    private WheelCollider wheelCollider;


    void Start()
    {
        
    }

    void Update()
    {
        
    }
}
