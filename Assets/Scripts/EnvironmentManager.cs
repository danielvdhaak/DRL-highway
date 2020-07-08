﻿/*
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
    public class LaneData
    {
        [Serializable]
        public class StartParameters
        {
            public float pos;
            public float speed;
            public float headway;
        }

        public float center;
        public float meanVehicleSpeed;
        public float stdVehicleSpeed;
        public float trafficFraction;
        public StartParameters[] startParameters;
    }

    [Serializable]
    public class VehiclePosition
    {
        public GameObject gameObject;
        public float x;
        public float z;
        public int lane;
    }

    [Header("ML-Agents")]
    public VehicleAgent vehicleAgent;
    public TextMeshPro cumulativeRewardText;

    [Header("Environment parameters")]
    [HideInInspector] public int numberOfLanes;
    public float laneWidth = 3.5f;
    public int trafficFlow = 6000;
    public float minHeadway = 1f;
    public GameObject[] vehiclePrefabs;
    public LaneData[] laneData;

    [Header("Traffic")]
    private List<GameObject> vehicleList = new List<GameObject>();
    public VehiclePosition[] vehiclePositions;


    private RandomNumber randomNumber = new RandomNumber();

    private void Awake()
    {
        DetermineLaneCenters();

        // Initialize arrays
        int numberOfCars = 0;
        foreach (LaneData laneData in laneData)
        {
            float laneTrafficFlow = trafficFlow * laneData.trafficFraction;     // [veh/h]
            float laneDensity = laneTrafficFlow / laneData.meanVehicleSpeed;    // [veh/km]
            laneData.startParameters = new LaneData.StartParameters[(int)laneDensity];

            numberOfCars += (int)laneDensity;
        }
        vehiclePositions = new VehiclePosition[numberOfCars];

        // Seed vehicle list
        vehicleList.Clear();
        for (int i = 0; i < numberOfCars; i++)
        {
            //GameObject obj = Instantiate(vehiclePrefabs[randomNumber.Next(vehiclePrefabs.Length - 1)]);
            //obj.SetActive(false);
            //vehicleList.Add(obj);
        }
    }

    public void ResetArea(int startLane, int startPos)
    {
        // Despawn cars
        foreach (GameObject obj in vehicleList)
        {
            Despawn(obj);
        }

        SetStartParameters();

        // Shift all positions so that agent starts at z=0
        float deltaZ = laneData[startLane - 1].startParameters[startPos - 1].pos;
        foreach (LaneData lane in laneData)
        {
            for (int i = 0; i < lane.startParameters.Length; i++)
            {
                lane.startParameters[i].pos -= deltaZ;
            }
        }

        // Spawn cars
        int vID = 0;
        for (int l = 0; l < numberOfLanes; l++)
        {
            for (int p = 0; p < laneData[l].startParameters.Length; p++)
            {
                Vector3 pos = new Vector3(laneData[l].center, 0f, laneData[l].startParameters[p].pos);
                Spawn(vehicleList[vID], pos, l + 1, laneData[l].startParameters[p].speed);
                vID++;
            }
        }

    }

    private void DetermineLaneCenters()
    {
        numberOfLanes = laneData.Length;

        for (int i = 0; i < numberOfLanes; i++)
        {
            laneData[i].center = -0.5f * numberOfLanes * laneWidth + ((float)i + 0.5f) * laneWidth;
        }
    }

    private void SetStartParameters()
    {
        foreach(LaneData laneData in laneData)
        {
            float laneTrafficFlow = trafficFlow * laneData.trafficFraction;     // [veh/h]
            float laneDensity = laneTrafficFlow / laneData.meanVehicleSpeed;    // [veh/km]
            float meanHeadway = 3600 / laneTrafficFlow;                         // [s/veh]

            for (int i = 0; i < laneData.startParameters.Length; i++)
            {
                laneData.startParameters[i] = new LaneData.StartParameters();

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

    public void Spawn(GameObject obj, Vector3 pos, int targetLane, float velocity)
    {
        obj.transform.localPosition = pos;
        obj.transform.localRotation = Quaternion.identity;

        SimpleVehicle vehicle = obj.GetComponent<SimpleVehicle>();
        vehicle.desiredVelocity = velocity;
        vehicle.targetLane = targetLane;

        obj.SetActive(true);

        // POS IS LOCAL!!
        /*
        // Instantiate car prefab
        GameObject vehicleInstance;
        vehicleInstance = Instantiate(carPrefabs[0], new Vector3(x, y, z), transform.rotation) as GameObject;
        GameObject vehicleInstance = Instantiate(vehiclePrefab, new Vector3(x, y, z), transform.rotation) as GameObject;

        // Set driver properties
        Driver driver = vehicleInstance.GetComponent<Driver>();
        driver.targetLane = lane + 1;
        //driver.initVelocity = 0f;
        driver.desiredVelocity = 0;
        driver.velocity = 0f;
        // Set car properties
        VehicleControl vehiclecontrol = vehicleInstance.GetComponent<VehicleControl>();
        vehiclecontrol.targetLane = laneNr;
        vehiclecontrol.desiredVelocity = 10;
        vehiclecontrol.velocity = 10f;
        */
    }

    public void Despawn(GameObject obj)
    {
        obj.SetActive(false);
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
