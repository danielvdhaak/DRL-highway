using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Traffic : MonoBehaviour
{
    [Header("Traffic flow parameters")]
    [SerializeField] private int trafficDensity;
    [SerializeField] private float meanSpeed;
    [SerializeField] private float speedStd;
    [SerializeField] private GameObject[] vehiclePrefabs;

    private RandomNumber randomNumber = new RandomNumber();
    [SerializeField] private EnvironmentManager environment;

    // Start is called before the first frame update
    void Start()
    {
        // Distribute vehicles over lanes
        // For now, do this uniformly
        int vehiclesPerLane = trafficDensity / environment.numberOfLanes;


    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
