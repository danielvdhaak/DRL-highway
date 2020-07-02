/*
 * This code is part of DRL-Highway by Daniel (2020)
 * 
 */

using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using TMPro;

public class EnvironmentManager : MonoBehaviour
{
    [Serializable]
    public class StartParameters
    {
        public GameObject prefab;
        public float pos;
        public float speed;
        public float headway;
    }

    [Serializable]
    public class LaneData
    {
        public float center;
        public float meanVehicleSpeed;
        public float stdVehicleSpeed;
        public float trafficFraction;
        public GameObject[] vehiclePrefabs;
        public StartParameters[] startParameters;

        public void Init(int N)
        {
            startParameters = new StartParameters[N];
        }
    }

    public VehicleAgent vehicleAgent;
    public TextMeshPro cumulativeRewardText;

    public int numberOfLanes;
    public float laneWidth = 3.5f;
    public int trafficFlow = 6000;
    public float minHeadway = 1f;
    public LaneData[] laneData;

    private RandomNumber randomNumber = new RandomNumber();

    private void Awake()
    {
        DetermineLaneCenters();
        //SetTrafficParameters();
    }

    private void Update()
    {
        //Debug.Log("Random number: " + randomNumber.Uniform(0.01f, 1.0f));
    }

    /// <summary>
    /// Generates a N long float array whose sum is M.
    /// </summary>
    private float[] Disperse(int N, float M)
    {
        float[] array = new float[N];
        float sum = 0f;

        // Fill array with uniform numbers
        for(int i = 0; i < N; i++)
        {
            array[i] = randomNumber.Uniform();
            sum += array[i];
        }

        // Normalize sum to M
        for (int i = 0; i < N; i++)
        {
            array[i] /= sum;
            array[i] *= M;
        }

        return array;
    }

    private void DetermineLaneCenters()
    {
        numberOfLanes = laneData.Length;

        for (int i = 0; i < numberOfLanes; i++)
        {
            laneData[i].center = -0.5f * numberOfLanes * laneWidth + ((float)i + 0.5f) * laneWidth;
        }
    }

    private void SetTrafficParameters()
    {
        foreach(LaneData laneData in laneData)
        {
            float laneTrafficFlow = trafficFlow * laneData.trafficFraction;     // [veh/h]
            float laneDensity = laneTrafficFlow / laneData.meanVehicleSpeed;    // [veh/km]
            float meanHeadway = 3600 / laneTrafficFlow;                         // [s/veh]

            laneData.Init((int)laneDensity);
            for (int i = 0; i < laneData.startParameters.Length; i++)
            {
                laneData.startParameters[i] = new StartParameters();

                laneData.startParameters[i].prefab = null;
                laneData.startParameters[i].speed = randomNumber.Gaussian(laneData.meanVehicleSpeed, laneData.stdVehicleSpeed);
                laneData.startParameters[i].headway = randomNumber.Exponential(meanHeadway, minHeadway);

                try
                {
                    laneData.startParameters[i].pos = laneData.startParameters[i - 1].pos + laneData.startParameters[i - 1].headway * (laneData.startParameters[i - 1].speed / 3.6f);
                }
                catch (IndexOutOfRangeException)
                {
                    laneData.startParameters[i].pos = 0f;
                }
            }
        }
    }

    private void AddAgent(int startLane, int startPos)
    {
        float deltaZ = laneData[startLane - 1].startParameters[startPos - 1].pos;
        foreach(LaneData lane in laneData)
        {
            for (int i = 0; i < lane.startParameters.Length; i++)
            {
                lane.startParameters[i].pos -= deltaZ;
            }
        }
    }

    private void OnDrawGizmosSelected()
    {
        Vector3 position = transform.position;
        Vector3 direction = transform.TransformDirection(transform.forward);

        // Draw highway center and borders
        Gizmos.color = Color.red;
        Gizmos.DrawRay(position, 100 * direction);
        Gizmos.color = Color.green;
        Gizmos.DrawRay(transform.TransformPoint(new Vector3(-0.5f * numberOfLanes * laneWidth, position.y, position.z)), 100 * direction);
        Gizmos.DrawRay(transform.TransformPoint(new Vector3(0.5f * numberOfLanes * laneWidth, position.y, position.z)), 100 * direction);

        // Draw lane centers
        DetermineLaneCenters();
        Gizmos.color = Color.cyan;
        foreach(LaneData lane in laneData)
        {
            float center = lane.center;
            Gizmos.DrawRay(transform.TransformPoint(new Vector3(center, position.y, position.z)), 100 * direction);
        }
    } 
}
