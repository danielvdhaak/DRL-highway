using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using TMPro;

public class EnvironmentManager : MonoBehaviour
{
    [Header("Environment setup")]
    [Tooltip("The car agent in the environment")] public CarAgent carAgent;

    //[Tooltip("Car prefab for other cars")]
    // public carPrefab;
    [Tooltip("The TextMeshPro text that shows the cumulative reward of the agent")] public TextMeshPro cumulativeRewardText;

    private List<GameObject> carList = new List<GameObject>();
    [HideInInspector] public List<float> laneCenterList = new List<float>();



    [Header("Highway parameters")]
    [Range(2,6)] public int m_NumberOfLanes = 5;
    public float m_LaneWidth = 3.5f;
    [Range(0,359)] public int m_Direction = 0;

    public void ResetEnvironment()
    {
        // Initialize lane rays
        InitRays();
    }

    private void Awake()
    {
        ResetEnvironment();
    }

    private void InitRays()
    {
        // Initialise lane change center rays
        Vector3 centerPosition = transform.position;
        Quaternion rotation = transform.rotation;
        Vector3 direction = rotation * Vector3.forward;

        for (int i = 0; i < m_NumberOfLanes; i++)
        {
            float localPos = -0.5f * m_NumberOfLanes * m_LaneWidth + ((float)i + 0.5f) * m_LaneWidth;
            laneCenterList.Add(localPos);
        }

    }

    private void Update()
    {
        // Update cumulative reward text
        //cumulativeRewardText.text = carAgent.GetCumulativeReward().ToString("0.00");
    }

    private void OnDrawGizmosSelected()
    {
        Vector3 centerPosition = transform.position;
        Quaternion rotation = Quaternion.Euler(0, m_Direction, 0);
        Vector3 direction = rotation * Vector3.forward;

        // Draw highway borders
        Gizmos.color = Color.red;
        Gizmos.DrawRay(centerPosition, 100 * direction);
        Gizmos.color = Color.green;
        Gizmos.DrawRay(centerPosition + rotation * (0.5f * m_NumberOfLanes * m_LaneWidth * Vector3.right), 100 * direction);
        Gizmos.DrawRay(centerPosition + rotation * (0.5f * m_NumberOfLanes * m_LaneWidth * Vector3.left), 100 * direction);

        // Draw lane centers
        Gizmos.color = Color.cyan;
        InitRays();
        foreach(float LC in laneCenterList)
        {
            //Debug.Log("Pos: " + LC.origin + "Dir: " + LC.direction);
            Ray ray = new Ray(new Vector3(0, LC, 0), Vector3.forward);
            Gizmos.DrawRay(ray);
        } 
    }
}
