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
    public static Events Instance
    {
        get
        {
            if (instance == null)
                instance = FindObjectOfType(typeof(Events)) as Events;
            return instance;
        }
        set
        {
            instance = value;
        }
    }
    private static Events instance;
    #endregion

    public event Action OnLaneChange;
    public void LaneChange()
    {
        OnLaneChange?.Invoke();
    }

    public event Action<int> OnCutOff;
    public void CutOff(int InstanceID)
    {
        OnCutOff?.Invoke(InstanceID);
    }

    public event Action OnCrash;
    public void Crash()
    {
        OnCrash?.Invoke();
    }

    public event Action OnNewEpisode;
    public void NewEpisode()
    {
        OnNewEpisode?.Invoke();
    }
}
