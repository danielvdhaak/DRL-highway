/*
 * This code is part of DRL-Highway by Daniel (2020)
 * 
 */

using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Simple class for rotating a windturbine rotor purely for cosmetic purposes.
/// </summary>
public class WindTurbineRotor : MonoBehaviour
{
    float turnSpeed = 50f;
    [SerializeField] Transform rotor;
    RandomNumber random = new RandomNumber();

    // Start is called before the first frame update
    void Start()
    {
        rotor.Rotate(Vector3.right * random.Uniform(359f));
    }

    // Update is called once per frame
    void Update()
    {
        rotor.Rotate(Vector3.right * turnSpeed * Time.deltaTime);
    }
}
