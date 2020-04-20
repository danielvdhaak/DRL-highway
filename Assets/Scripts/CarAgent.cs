using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using MLAgents;
using MLAgents.Sensors;
using System.Linq;

public class CarAgent : Agent
{
    private Rigidbody rBody;
    public EnvironmentManager environmentManager;
    private List<Ray> laneCenterList = new List<Ray>();
    private List<float> latErrorList = new List<float>();
    private int currentLaneNr = 99;

    public Transform Target;

    void Start()
    {
        // Check whether class EnvironmentManager is correctly referenced
        if (environmentManager == null)
        {
            Debug.LogError("Missing <GameObject> environmentManager reference!");
            Debug.Break();
        }

        rBody = GetComponent<Rigidbody>();
    }

    public override void OnEpisodeBegin()
    {
        Debug.Log("OnEpisodeBegin() executed!");

        // Debugging for now, should probably move to CollectObservations()
        laneCenterList = environmentManager.laneCenterList;
        latErrorList = CalculateLateralError(laneCenterList);
        currentLaneNr = DetermineCurrentLane(latErrorList);


        // If the agent falls
        if (transform.position.y < 0)
        {
            this.rBody.angularVelocity = Vector3.zero;
            this.rBody.velocity = Vector3.zero;
            this.transform.position = new Vector3(0, 0.5f, 0);
        }

        // Car collision
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        latErrorList = CalculateLateralError(laneCenterList);
        sensor.AddObservation(rBody.velocity.x);
    }

    public override void OnActionReceived(float[] vectorAction)
    {
        rBody.AddForce(Vector3.zero);
    }

    public override float[] Heuristic()
    {
        return base.Heuristic();
    }


    private List<float> CalculateLateralError(List<Ray> laneCenterList)
    {
        // Calculates lateral errors to an input list of lane center rays
        List<float> latErrorList = new List<float>();
        Vector3 pos = transform.position;

        if (laneCenterList.Count != 0)
        {
            foreach (Ray ray in laneCenterList)
            {
                latErrorList.Add(Vector3.Cross(ray.direction, pos - ray.origin).magnitude);
                //Debug.Log("lat error: " + Vector3.Cross(ray.direction, pos - ray.origin).magnitude);
            }
        } else
        {
            Debug.LogError("laneCenterList is empty!");
        }

        return latErrorList;
    }

    private int DetermineCurrentLane(List<float> latErrorList)
    {
        // Determines whether agent is on the highway and if so, determines current lane
        float laneWidth = environmentManager.m_LaneWidth;
        float minLatError = latErrorList.Min();
        int lane = latErrorList.IndexOf(minLatError) + 1;

        if (minLatError <= 0.5*laneWidth)
        {
            //Debug.Log("Car is on lane " + lane);
            return lane;
        } else
        {
            //Debug.Log("Car is off the road");
            return 99;
        }
    }
}
