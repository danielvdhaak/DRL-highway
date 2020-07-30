using System.Collections;
using System.Collections.Generic;
using System;
using System.IO;
using UnityEngine;
using MLAgents;
using MLAgents.SideChannels;

public class Logger : MonoBehaviour
{
    [Serializable]
    public class Log
    {
        public int episode;
        public int step;

        public Log(int episode, int step)
        {
            this.episode = episode;
            this.step = step;
        }
    }

    [Header("Counters")]
    [SerializeField] private int episodeCount = 0;

    [Header("Data logging")]
    [SerializeField] private bool logData = false;
    private int laneChangeCount = 0;
    private int crashCount = 0;
    private List<Log> crashes = new List<Log>();
    private List<Log> laneChanges = new List<Log>();
    private string crashCSV_dir;
    private string laneChangeCSV_dir;

    [Header("Automatic pausing")]
    [SerializeField] private bool pauseGame = false;
    [SerializeField] private int pauseAtEpisode = 10;


    private void OnEnable()
    {
        crashCSV_dir = Application.dataPath + "/Logs/Crashlog.csv";
        laneChangeCSV_dir = Application.dataPath + "/Logs/LaneChangelog.csv";

        Events.Instance.OnNewEpisode += OnNewEpisode;

        if (logData)
        {
            Events.Instance.OnCrash += OnCrash;
            Events.Instance.OnLaneChange += OnLaneChange;
        }
    }

    private void OnDisable()
    {
        //Events.Instance.OnNewEpisode -= OnNewEpisode;

        if (logData)
        {
            //Events.Instance.OnCrash -= OnCrash;
            //Events.Instance.OnLaneChange -= OnLaneChange;

            WriteLogs(laneChangeCSV_dir, laneChanges);
            WriteLogs(crashCSV_dir, crashes);
        }
    }

    private void OnNewEpisode()
    {
        episodeCount++;

        if (episodeCount != 1 && episodeCount % 10 == 1)
        {
            Debug.Log("Episodes  " + (episodeCount - 10) + "-" + (episodeCount - 1) + ": " + laneChangeCount + " lane changes, " + crashCount + " crashes.");
            crashCount = 0;
            laneChangeCount = 0;
        }

        if (pauseGame && episodeCount == pauseAtEpisode)
        {
            Debug.Log("Episode limit reached!");
            Debug.Break();
        }   
    }

    private void OnCrash()
    {
        crashCount++;
        crashes.Add(new Log(episodeCount, Academy.Instance.TotalStepCount));
    }

    private void OnLaneChange()
    {
        laneChangeCount++;
        laneChanges.Add(new Log(episodeCount, Academy.Instance.TotalStepCount));
    }

    private void WriteLogs(string path, List<Log> data)
    {
        StreamWriter csvWriter = new StreamWriter(path);

        csvWriter.WriteLine("episode, step");
        foreach(Log item in data)
        {
            csvWriter.WriteLine(item.episode.ToString() + "," + item.step.ToString());
        }

        csvWriter.Flush();
        csvWriter.Close();
    }
}
