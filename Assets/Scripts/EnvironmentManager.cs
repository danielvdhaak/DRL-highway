/*
 * This code is part of DRL-Highway by Daniel (2020)
 * 
 */

using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using TMPro;
using Unity.MLAgents;

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
    public List<SpawnParameters> spawnParameters;
}

public class EnvironmentManager : MonoBehaviour
{
    // Properties
    public int NumberOfLanes { get { return numberOfLanes; } }
    public float LaneWidth { get { return laneWidth; } }
    public List<float> LaneCenters { get; private set; }
    public List<VehicleControl> Traffic { get; private set; }

    [Header("ML-Agents")]
    public VehicleAgent agent;
    private int targetVelocity;
    private VehicleRanker ranker;

    [Header("Environment parameters")]
    public float laneWidth = 3.5f;
    [SerializeField] private int numberOfLanes;
    private int trafficFlow = 4000;
    [SerializeField] private float amountOfKms = 1f;
    private float minHeadway = 1f;

    [Header("Traffic")]
    private int density;
    [SerializeField] private List<TrafficTemplate> trafficTemplates = new List<TrafficTemplate>();
    [SerializeField] private List<GameObject> vehiclePrefabs = new List<GameObject>();
    private TrafficParameters[] trafficParameters;
    private Queue<GameObject> vehiclePool;

    private RandomNumber randomNumber = new RandomNumber();

    private void Awake()
    {
        // Initialize lists & arrays
        LaneCenters = DetermineLaneCenters();
        vehiclePool = new Queue<GameObject>();
        Traffic = new List<VehicleControl>();

        // Initialize vehicle ranker
        ranker = FindObjectOfType<VehicleRanker>();
    }

    private void FixedUpdate()
    {
        ranker.UpdateRanks(Traffic);
    }

    /// <summary>
    /// Returns a list of local lateral lane center coordinates.
    /// </summary>
    private List<float> DetermineLaneCenters()
    {
        List<float> list = new List<float>();
        for (int i = 0; i < numberOfLanes; i++)
        {
            list.Add(-0.5f * numberOfLanes * laneWidth + (i + 0.5f) * laneWidth);
        }

        return list;
    }

    public void ResetArea(int startLane, int startPos)
    {
        // Despawn cars
        DespawnAll();

        // Import random traffic template from list
        if (trafficTemplates.Count > 0)
        {
            TrafficTemplate template = trafficTemplates[randomNumber.Next(trafficTemplates.Count - 1)];
            ImportTrafficTemplate(template);
            agent.targetVelocity = template.targetVelocity;
        }
            

        // For each lane
        density = 0;
        foreach (TrafficParameters lane in trafficParameters)
        {
            lane.laneDensity = (int)Math.Ceiling((trafficFlow * lane.trafficFraction * amountOfKms) / lane.meanVehicleSpeed);
            density += lane.laneDensity;
            GenerateSpawnParameters(lane);
        }

        // Spawn traffic
        int vID = 0;
        for (int l = 0; l < numberOfLanes; l++)
        {
            if (trafficParameters[l].spawnParameters.Count == 0)
                continue;

            for (int p = 0; p < trafficParameters[l].spawnParameters.Count; p++)
            {
                if (l == startLane - 1 && p == startPos - 1)
                    continue;

                //Vector3 location = new Vector3(
                //    LaneCenters[l],
                //    0f,
                //    trafficParameters[l].spawnParameters[p].pos - trafficParameters[startLane - 1].spawnParameters[startPos - 1].pos);
                Vector3 location = new Vector3(LaneCenters[l], 0f, trafficParameters[l].spawnParameters[p].pos);
                Spawn(location, l + 1, trafficParameters[l].spawnParameters[p].speed);
                vID++;
            }
        }

        // Set agent position and speed
        Vector3 agentPos = new Vector3(LaneCenters[startLane - 1], 0f, trafficParameters[startLane - 1].spawnParameters[startPos - 1].pos);
        float agentInitVel = trafficParameters[startLane - 1].spawnParameters[startPos - 1].speed;
        agent.transform.localPosition = agentPos;
        agent.transform.localRotation = Quaternion.identity;
        agent.GetComponent<VehicleControl>().SetInitialVelocity(agentInitVel);
        Traffic.Add(agent.GetComponent<VehicleControl>());

        ranker.SetInitialRanks(Traffic);
    }


    private void GenerateSpawnParameters(TrafficParameters lane)
    {
        lane.spawnParameters = new List<TrafficParameters.SpawnParameters>();

        float meanHeadway = 3600f / (trafficFlow * lane.trafficFraction);

        float pos = randomNumber.Uniform(0f, 20f);
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


    public void Spawn(Vector3 pos, int targetLane, float velocity)
    {
        if (vehiclePool.Count < 1)
            CreateVehicleInstance();

        GameObject obj = vehiclePool.Dequeue(); 

        obj.transform.localPosition = pos;
        obj.transform.localRotation = Quaternion.identity;

        SimpleVehicle sv = obj.GetComponent<SimpleVehicle>();
        sv.TargetLane = targetLane;
        sv.TargetVelocity = velocity;

        VehicleControl vc = obj.GetComponent<VehicleControl>();
        vc.TargetVelocity = velocity;

        Traffic.Add(vc);
        obj.SetActive(true);
        vc.SetInitialVelocity(velocity);
    }

    public void Despawn(GameObject obj)
    {
        obj.SetActive(false);
        Traffic.Remove(obj.GetComponent<VehicleControl>());
        vehiclePool.Enqueue(obj);
    }

    private void DespawnAll()
    {
        foreach(VehicleControl vehicle in Traffic)
        {
            if (!vehicle.gameObject.CompareTag("Agent"))
            {
                vehicle.gameObject.SetActive(false);
                vehiclePool.Enqueue(vehicle.gameObject);
            }
        }

        Traffic.Clear();
    }

    private void CreateVehicleInstance()
    {
        GameObject obj = vehiclePrefabs[randomNumber.Next(vehiclePrefabs.Count - 1)];
        obj.SetActive(false);
        GameObject instance = Instantiate(obj, transform);
        obj.SetActive(true);
        vehiclePool.Enqueue(instance);
    }

    private void ImportTrafficTemplate(TrafficTemplate template)
    {
        trafficFlow = template.flow;
        minHeadway = template.minHeadway;
        targetVelocity = template.targetVelocity;
        trafficParameters = template.trafficParameters;
    }

    //private void OnDrawGizmosSelected()
    //{
    //    Vector3 position = transform.position;
    //    Vector3 direction = transform.TransformDirection(transform.forward);
        
    //    // Draw centerline, borders and lane centers
    //    List<float> centers = DetermineLaneCenters();

    //    Gizmos.color = Color.red;
    //    Gizmos.DrawRay(position, 100 * direction);

    //    Gizmos.color = Color.green;
    //    Gizmos.DrawRay(transform.TransformPoint(new Vector3(-0.5f * centers.Count * laneWidth, position.y, position.z)), 100 * direction);
    //    Gizmos.DrawRay(transform.TransformPoint(new Vector3(0.5f * centers.Count * laneWidth, position.y, position.z)), 100 * direction);

    //    Gizmos.color = Color.cyan;
    //    foreach(float center in centers)
    //    {
    //        Gizmos.DrawRay(transform.TransformPoint(new Vector3(center, position.y, position.z)), 100 * direction);
    //    }
    //} 
}
