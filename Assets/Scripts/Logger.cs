using System.Collections;
using System.Collections.Generic;
using System;
using System.IO;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.SideChannels;

public class EnvironmentState
{
    public float[] a;
    public float agentVel;
    public int currentLane;
    public float[] observationGrid;

    public EnvironmentState(float[] a, float agentVel, int cLane, float[] oGrid)
    {
        this.a = a;
        this.agentVel = agentVel;
        this.currentLane = cLane;
        this.observationGrid = oGrid;
    }
}

public class Logger : MonoBehaviour
{
    [Header("Settings")]
    [SerializeField] private bool saveToCSV;
    [SerializeField] private string fileName;
    [SerializeField] private float logFrequency;
    private uint t = 0;
    [SerializeField] private string weatherCondition;
    [SerializeField] private VehicleAgent agent;
    private EnvironmentState environmentState;

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

        // Initialize tensorboard
        m_Recorder = Academy.Instance.StatsRecorder;
    }

    private void Start()
    {
        if (saveToCSV)
            StartCoroutine(LogData());
    }

    IEnumerator LogData()
    {
        while (true)
        {
            environmentState = agent.LogStats();


            yield return new WaitForSeconds(logFrequency);
        }
    }

    private void OnNewEpisode()
    {
        episodeCount++;
        m_Recorder.Add("Metrics/Lane changes per episode", laneChangeCount);
        laneChangeCount = 0;
    }

    private void OnCrash()
    {
        collisionCount++;
        m_Recorder.Add("Metrics/Collisions", collisionCount);
    }

    private void OnLaneChange()
    {
        laneChangeCount++;
    }

}
