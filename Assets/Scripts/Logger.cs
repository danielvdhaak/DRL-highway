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
    public float[] traffic;

    public EnvironmentState(float[] a, float agentVel, int currentLane, float[] traffic)
    {
        this.a = a;
        this.agentVel = agentVel;
        this.currentLane = currentLane;
        this.traffic = traffic;
    }
}

public class Logger : MonoBehaviour
{
    public enum AgentType { MOBIL, NeuralNetwork };

    [Header("Settings")]
    [SerializeField] private bool saveToCSV;
    [SerializeField] private string fileName;
    [SerializeField] private float logFrequency;
    private float t = 0f;
    [SerializeField] private string weatherCondition;
    [SerializeField] private AgentType selectedAgent;
    [SerializeField] private VehicleAgent DRLAgent;
    [SerializeField] private MobilAgent MobilAgent;
    private EnvironmentState environmentState;

    [Header("Counters")]
    public uint episodeCount = 0;
    public uint laneChangeCount = 0;
    public uint collisionCount = 0;
    
    StatsRecorder m_Recorder;
    DateTime dt;
    CSVReport report;
    EnvironmentManager environment;

    private void OnEnable()
    {
        // Subscribe to events
        Events.Instance.OnNewEpisode += OnNewEpisode;
        Events.Instance.OnCrash += OnCrash;
        Events.Instance.OnLaneChange += OnLaneChange;

        // Initialize tensorboard
        m_Recorder = Academy.Instance.StatsRecorder;

        // Initialize environment
        environment = FindObjectOfType<EnvironmentManager>();

        // Prepare CSV file
        if (saveToCSV)
        {
            string[] headers = new string[]
            {
                "episode_nr",
                "time",
                "action",
                "velocity",
                "lane",
                "car1_f_dz",
                "car1_f_dv",
                "car1_b_dz",
                "car1_b_dv",
                "car2_f_dz",
                "car2_f_dv",
                "car2_b_dz",
                "car2_b_dv",
                "car3_f_dz",
                "car3_f_dv",
                "car3_b_dz",
                "car3_b_dv",
                "traffic_flow",
                "weather"
            };

            report = new CSVReport(fileName, headers);
            report.Create();
        }
            
    }

    private void Start()
    {
        if (saveToCSV)
            InvokeRepeating("LogToCSV", logFrequency, logFrequency);
    }

    private void LogToCSV()
    {
        switch (selectedAgent)
        {
            case AgentType.MOBIL:
                environmentState = MobilAgent.LogStats();
                break;
            case AgentType.NeuralNetwork:
                environmentState = DRLAgent.LogStats();
                break;
            default:
                Debug.LogError("No agent selected for logging report!");
                break;
        }

        string[] data = new string[]
        {
                episodeCount.ToString(),
                Math.Round(Time.time - t, 1).ToString(),
                environmentState.a[0].ToString(),
                Mathf.CeilToInt(environmentState.agentVel).ToString(),
                environmentState.currentLane.ToString(),
                Math.Round(environmentState.traffic[0], 3).ToString(),
                Math.Round(environmentState.traffic[1], 3).ToString(),
                Math.Round(environmentState.traffic[2], 3).ToString(),
                Math.Round(environmentState.traffic[3], 3).ToString(),
                Math.Round(environmentState.traffic[4], 3).ToString(),
                Math.Round(environmentState.traffic[5], 3).ToString(),
                Math.Round(environmentState.traffic[6], 3).ToString(),
                Math.Round(environmentState.traffic[7], 3).ToString(),
                Math.Round(environmentState.traffic[8], 3).ToString(),
                Math.Round(environmentState.traffic[9], 3).ToString(),
                Math.Round(environmentState.traffic[10], 3).ToString(),
                Math.Round(environmentState.traffic[11], 3).ToString(),
                environment.trafficFlow.ToString(),
                weatherCondition
        };
        report.Append(data);
    }

    private void OnNewEpisode()
    {
        episodeCount++;
        m_Recorder.Add("Metrics/Lane changes per episode", laneChangeCount);
        laneChangeCount = 0;
        t = Time.time;
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
