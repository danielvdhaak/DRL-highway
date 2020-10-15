/*
 * This code is part of DRL-Highway by Daniel (2020)
 * 
 */

using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class Speedometer : MonoBehaviour
{
    [SerializeField] private Text textField;
    private VehicleControl control;

    private void OnEnable()
    {
        control = GetComponentInParent<VehicleControl>();
    }

    void Update()
    {
        float velocity = Mathf.CeilToInt(control.Velocity);
        textField.text = velocity.ToString();
    }
}
