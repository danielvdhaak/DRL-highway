using System.Collections;
using System.Collections.Generic;
using System;
using System.IO;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.SideChannels;

public class Logger : MonoBehaviour
{
    [Header("Counters")]
    public uint episodeCount = 0;
    public uint laneChangeCount = 0;
    public uint collisionCount = 0;
    
    StatsRecorder m_Recorder;

    private void OnEnable()
    {
        // Subscribe to events
        Events.Instance.OnNewEpisode += OnNewEpisode;
        Events.Instance.OnCrash += OnCrash;
        Events.Instance.OnLaneChange += OnLaneChange;

        m_Recorder = Academy.Instance.StatsRecorder;
    }

    private void Update()
    {
        // Log velocity in Tensorboard
        if ((Time.frameCount % 100) == 0)
        {
            m_Recorder.Add("Metrics/Lane changes per episode", laneChangeCount);
            m_Recorder.Add("Metrics/Collisions", collisionCount);
        }
    }

    private void OnNewEpisode()
    {
        episodeCount++;
        laneChangeCount = 0;
    }

    private void OnCrash()
    {
        collisionCount++;
    }

    private void OnLaneChange()
    {
        laneChangeCount++;
    }

}
