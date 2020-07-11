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
    public class TrafficParameters
    {
        [Serializable]
        public class SpawnParameters
        {
            public float pos;
            public float speed;
            public float headway;

            public SpawnParameters(float pos, float speed, float headway)
            {
                this.pos = pos;
                this.speed = speed;
                this.headway = headway;
            }
        }

        public float trafficFraction;
        public float meanVehicleSpeed;
        public float stdVehicleSpeed;
        public int laneDensity;
        public List<SpawnParameters> spawnParameters = new List<SpawnParameters>();
    }

    [Header("ML-Agents")]
    public VehicleAgent agent;
    public TextMeshPro cumulativeRewardText;

    [Header("Environment parameters")]
    private int numberOfLanes;
    public float laneWidth = 3.5f;
    [SerializeField] private int trafficFlow = 6000;
    private int density;
    [SerializeField] private float minHeadway = 1f;

    [Header("Traffic")]
    [SerializeField] private List<GameObject> vehiclePrefabs = new List<GameObject>();
    [SerializeField] private TrafficParameters[] trafficParameters;
    private List<GameObject> vehicleList = new List<GameObject>();
    public List<VehicleControl> trafficList = new List<VehicleControl>();
    public List<float> centerList = new List<float>();

    private RandomNumber randomNumber = new RandomNumber();

    private void Awake()
    {
        numberOfLanes = trafficParameters.Length;
        centerList = DetermineLaneCenters();

        // Determine total density and density per lane
        density = 0;
        foreach (TrafficParameters laneData in trafficParameters)
        {
            laneData.laneDensity = (int)Math.Ceiling((trafficFlow * laneData.trafficFraction) / laneData.meanVehicleSpeed);
            density += laneData.laneDensity;
        }

        // Seed vehicle lists
        vehicleList.Clear();
        trafficList.Clear();
        trafficList.Add(agent.GetComponent<VehicleControl>());
        for (int i = 0; i < density - 1; i++)
        {
            GameObject obj = vehiclePrefabs[randomNumber.Next(vehiclePrefabs.Count - 1)];
            obj.SetActive(false);
            GameObject instance = Instantiate(obj, transform);
            VehicleControl vc = instance.GetComponent<VehicleControl>();
            vehicleList.Add(instance);
            trafficList.Add(vc);
        }
    }

    private void Start()
    {
        ResetArea(2, 1);
    }

    private void GenerateSpawnParameters(TrafficParameters lane)
    {
        float meanHeadway = 3600 / (trafficFlow * lane.trafficFraction);

        float pos = randomNumber.Uniform(0f, 40f);
        float speed = randomNumber.Gaussian(lane.meanVehicleSpeed, lane.stdVehicleSpeed);
        float headway = randomNumber.Exponential(meanHeadway, minHeadway);
        for (int i = 0; i < lane.laneDensity; i++)
        {
            lane.spawnParameters.Add(new TrafficParameters.SpawnParameters(pos, speed, headway));

            pos = pos + headway * (speed / 3.6f);
            speed = randomNumber.Gaussian(lane.meanVehicleSpeed, lane.stdVehicleSpeed);
            headway = randomNumber.Exponential(meanHeadway, minHeadway);
        }
    }

    public (Vector3, float) ResetArea(int startLane, int startPos)
    {
        // Despawn cars
        foreach (GameObject obj in vehicleList)
        {
            Despawn(obj);
        }
        
        // For each lane
        foreach (TrafficParameters lane in trafficParameters)
        {
            GenerateSpawnParameters(lane);
        }

        // Spawn cars with offset
        int vID = 0;
        for (int l = 0; l < numberOfLanes; l++)
        {
            if (trafficParameters[l].spawnParameters.Count == 0)
                continue;

            for (int p = 0; p < trafficParameters[l].spawnParameters.Count; p++)
            {
                if (l == startLane - 1 && p == startPos - 1)
                    continue;

                Vector3 location = new Vector3(
                    centerList[l],
                    0f,
                    trafficParameters[l].spawnParameters[p].pos - trafficParameters[startLane - 1].spawnParameters[startPos - 1].pos);
                Spawn(vehicleList[vID], location, l + 1, trafficParameters[l].spawnParameters[p].speed);
                vID++;
            }
        }

        return (new Vector3(centerList[startLane - 1], 0f, 0f), trafficParameters[startLane - 1].spawnParameters[startPos - 1].speed);
    }

    private List<float> DetermineLaneCenters()
    {
        List<float> list = new List<float>();
        for (int i = 0; i < trafficParameters.Length; i++)
        {
            list.Add(-0.5f * trafficParameters.Length * laneWidth + ((float)i + 0.5f) * laneWidth);
        }

        return list;
    }

    public void Spawn(GameObject obj, Vector3 pos, int targetLane, float velocity)
    {
        obj.transform.localPosition = pos;
        obj.transform.localRotation = Quaternion.identity;

        SimpleVehicle sv = obj.GetComponent<SimpleVehicle>();
        sv.trafficList = trafficList;
        sv.laneCenters = centerList;
        sv.laneWidth = laneWidth;
        sv.targetLane = targetLane;
        sv.desiredVelocity = velocity;
        sv.initialVelocity = velocity;

        obj.SetActive(true);
    }

    public void Despawn(GameObject obj)
    {
        obj.SetActive(false);
    }

    private void OnDrawGizmosSelected()
    {
        Vector3 position = transform.position;
        Vector3 direction = transform.TransformDirection(transform.forward);
        
        // Draw centerline, borders and lane centers
        List<float> centers = DetermineLaneCenters();

        Gizmos.color = Color.red;
        Gizmos.DrawRay(position, 100 * direction);

        Gizmos.color = Color.green;
        Gizmos.DrawRay(transform.TransformPoint(new Vector3(-0.5f * centers.Count * laneWidth, position.y, position.z)), 100 * direction);
        Gizmos.DrawRay(transform.TransformPoint(new Vector3(0.5f * centers.Count * laneWidth, position.y, position.z)), 100 * direction);

        Gizmos.color = Color.cyan;
        foreach(float center in centers)
        {
            Gizmos.DrawRay(transform.TransformPoint(new Vector3(center, position.y, position.z)), 100 * direction);
        }
    } 
}
