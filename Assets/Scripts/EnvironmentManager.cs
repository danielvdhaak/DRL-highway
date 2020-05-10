using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using TMPro;

public class EnvironmentManager : MonoBehaviour
{
    [Serializable]
    public class LaneInfo
    {
        public float center;
        public int minVehicleNumber;
        public int maxVehicleNumber;
        public float minVehicleSpeed;
        public float maxVehicleSpeed;
        public int minVehicleSpread;
        public int maxVehicleSpread;
        public GameObject[] vehiclePrefabs;
    }

    public VehicleAgent vehicleAgent;
    public TextMeshPro cumulativeRewardText;
    [Range(2,6)] public int numberOfLanes;
    public float laneWidth = 3.5f;
    public LaneInfo[] laneInfo;

    private System.Random rnd = new System.Random();

    
    public void ResetEnvironment()
    {
        // Distribute cars per lane
        int laneNr = 1;
        foreach (LaneInfo lane in laneInfo)
        {
            int numberOfVehicles = rnd.Next(lane.minVehicleNumber, lane.maxVehicleNumber);
            float dz = transform.position.z;

            // Distribute cars on every lane
            for (int vehicle = 0; vehicle < numberOfVehicles; vehicle++)
            {
                // Determine position
                float x = lane.center;
                float y = transform.position.y;
                float z = dz + rnd.Next(lane.minVehicleSpread, lane.maxVehicleSpread);
                Vector3 position = transform.TransformPoint(new Vector3(x, y, z));

                // Determine vehicle prefab
                int prefabIndex = rnd.Next(lane.vehiclePrefabs.Length);
                GameObject vehiclePrefab = lane.vehiclePrefabs[prefabIndex];

                // Instantiate car prefab
                GameObject vehicleInstance = Instantiate(vehiclePrefab, new Vector3(x, y, z), transform.rotation) as GameObject;

                // Set car properties
                VehicleControl vehiclecontrol = vehicleInstance.GetComponent<VehicleControl>();
                vehiclecontrol.targetLane = laneNr;
                vehiclecontrol.desiredVelocity = 10;
                vehiclecontrol.velocity = 10f;

                dz = z;
            }
            laneNr++;
        }
    }

    private void Awake()
    {
        DetermineLaneCenters();
        ResetEnvironment();
    }

    private void DetermineLaneCenters()
    {
        for (int i = 0; i < numberOfLanes; i++)
        {
            laneInfo[i].center = -0.5f * numberOfLanes * laneWidth + ((float)i + 0.5f) * laneWidth;
        }

    }



    private void Update()
    {
        // Update cumulative reward text
        //cumulativeRewardText.text = carAgent.GetCumulativeReward().ToString("0.00");
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
        foreach(LaneInfo lane in laneInfo)
        {
            float center = lane.center;
            Gizmos.DrawRay(transform.TransformPoint(new Vector3(center, position.y, position.z)), 100 * direction);
        }
    } 
}
