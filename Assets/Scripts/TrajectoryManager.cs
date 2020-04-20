using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TrajectoryManager : MonoBehaviour
{
    [SerializeField] private Vector3 startingPoint;
    [SerializeField] private float w = 3.5f;
    [SerializeField] private float tlc = 3f;
    [SerializeField] private float v = 10f;
    [SerializeField] private float llc = 100;

    private List<Vector3> trajectoryPoints = new List<Vector3>();

    private void Start()
    {
        CalcTrajectory();
    }

    private void CalcTrajectory()
    {
        if (startingPoint == null)
        {
            Debug.LogError("Missing <Vector3> startingPoint!");
            Debug.Break();
        } 

        float x;
        float z;

        for(float t = 0; t < tlc; t += 0.1f)
        {
            x = w * (10 * Mathf.Pow((t / tlc), 3) - 15 * Mathf.Pow((t / tlc), 4) + 6 * Mathf.Pow((t / tlc), 5));
            z = t * v + (llc - tlc * v) * (10 * Mathf.Pow((t / tlc), 3) - 15 * Mathf.Pow((t / tlc), 4) + 6 * Mathf.Pow((t / tlc), 5));

            Vector3 pos = new Vector3(x, 0, z);
            pos = pos + startingPoint;

            trajectoryPoints.Add(pos);
        }
    }

    private void OnDrawGizmos()
    {
        Gizmos.color = Color.green;
        
        foreach(Vector3 pos in trajectoryPoints)
        {
            Gizmos.DrawSphere(pos, 0.1f);
        }
    }
}
