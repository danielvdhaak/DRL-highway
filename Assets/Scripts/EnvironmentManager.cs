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
    public class VehicleParameters
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
        public VehicleParameters[] vehicleParameters;

        public void Init(int N)
        {
            vehicleParameters = new VehicleParameters[N];
        }
    }

    public VehicleAgent vehicleAgent;
    public TextMeshPro cumulativeRewardText;

    [Range(2,6)] public int numberOfLanes = 5;
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
        Debug.Log("Random number: " + randomNumber.Uniform(0.01f, 1.0f));
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
            for (int i = 0; i < laneData.vehicleParameters.Length; i++)
            {
                laneData.vehicleParameters[i] = new VehicleParameters();

                laneData.vehicleParameters[i].prefab = null;
                laneData.vehicleParameters[i].speed = randomNumber.Gaussian(laneData.meanVehicleSpeed, laneData.stdVehicleSpeed);
                laneData.vehicleParameters[i].headway = randomNumber.Exponential(meanHeadway, minHeadway);

                try
                {
                    laneData.vehicleParameters[i].pos = laneData.vehicleParameters[i - 1].pos + laneData.vehicleParameters[i - 1].headway * (laneData.vehicleParameters[i - 1].speed / 3.6f);
                }
                catch (IndexOutOfRangeException)
                {
                    laneData.vehicleParameters[i].pos = 0f;
                }
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
