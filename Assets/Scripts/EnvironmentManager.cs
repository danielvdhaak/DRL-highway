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
        public GameObject[] vehiclePrefabs;
        public VehicleParameters[] vehicleParameters;
    }

    public VehicleAgent vehicleAgent;
    public TextMeshPro cumulativeRewardText;

    [Range(2,6)] public int numberOfLanes;
    public float laneWidth = 3.5f;
    public int density;
    public LaneData[] laneData;

    private RandomNumber randomNumber = new RandomNumber();

    private void Awake()
    {
        DetermineLaneCenters();
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

    private void SetTrafficParameters()
    {
        foreach(LaneData laneData in laneData)
        {
            // Initialize vehicleparameters array according to density
            laneData.vehicleParameters = new VehicleParameters[density]; // Needs implementation of densityfactor!

            // Set prefab and initial speed
            foreach (VehicleParameters vehicleParameters in laneData.vehicleParameters)
            {
                vehicleParameters.prefab = laneData.vehiclePrefabs[0];
                vehicleParameters.speed = randomNumber.Gaussian(laneData.meanVehicleSpeed, laneData.stdVehicleSpeed);
            }
        }


    }

    /*
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
    */

    private void DetermineLaneCenters()
    {
        for (int i = 0; i < numberOfLanes; i++)
        {
            laneData[i].center = -0.5f * numberOfLanes * laneWidth + ((float)i + 0.5f) * laneWidth;
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
