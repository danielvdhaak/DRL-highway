/*
 * This code is part of DRL-Highway by Daniel (2020)
 * 
 */

using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Events : MonoBehaviour
{
    #region Singleton
    public static Events Instance;

    private void Awake()
    {
        Instance = this;
    }
    #endregion

    public event Action<int> onCutOff;
    public void CutOff(int InstanceID)
    {
        onCutOff?.Invoke(InstanceID);
    }
}
