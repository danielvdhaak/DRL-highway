/*
 * This code is part of DRL-Highway by Daniel (2020)
 * 
 */

using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

[Serializable]
public class LidarPoint
{
    public Vector3 position;
    public float distance;
    public float normDistance;

    public LidarPoint(Vector3 p, float d, float nd)
    {
        position = p;
        distance = d;
        normDistance = nd;
    }
}

[Serializable]
public class LidarData
{
    // General sensor information
    public Vector3 sensorPos;
    public Vector3 sensorRotation;

    // Data array
    public LidarPoint[] lidarPoints;

    // Initialize arrays
    public void Init(int nRays)
    {
        lidarPoints = new LidarPoint[nRays];
    }
}

public class LidarSensor : MonoBehaviour
{
    [Header("Sensor parameters")]
    [Range(10,100)] public int maxRange = 50;
    public int deltaAzimuthal = 3;
    public float polarOffset = 20f;
    public int deltaPolar = -1;
    public int polarSweep = 15;
    public float noise = 0.2f;

    [Header("Debugging")]
    public float gizmoSize = 0.05f;
    public bool displayInScene = false;
    public bool displayInGame = false;

    [Header("Sensor data")]
    public LidarData data;

    private void FixedUpdate()
    {
        data = GetOutput();
    }

    public LidarData GetOutput()
    {
        LidarData data = new LidarData();
        data.Init(360 / deltaAzimuthal * polarSweep);

        data.sensorPos = transform.position;
        data.sensorRotation = transform.rotation.eulerAngles;

        Ray lidarRay = new Ray();
        lidarRay.origin = transform.position;
        lidarRay.direction = transform.forward;

        // Determine starting point
        Quaternion offset = Quaternion.AngleAxis(polarOffset, transform.right);
        lidarRay.direction = offset * lidarRay.direction;

        // Rotation steps
        Quaternion rotAzimuth = Quaternion.AngleAxis(deltaAzimuthal, transform.up);
        Quaternion rotPolar = Quaternion.AngleAxis(deltaPolar, transform.right);

        RaycastHit hit;
        int p = 0;
        
        for (int iP = 0; iP < polarSweep; iP++)
        {
            for (int iA = 0; iA < 360/deltaAzimuthal; iA++)
            {
                if (p >= data.lidarPoints.Length)
                    break;

                if (Physics.Raycast(lidarRay, out hit, maxRange))
                {
                    float distance = Mathf.Clamp(hit.distance + noise * Mathf.PerlinNoise(UnityEngine.Random.Range(0f, 1f), UnityEngine.Random.Range(0f, 1f)),
                                                0f, maxRange);
                    float normDistance = distance / maxRange;
                    Vector3 position = lidarRay.GetPoint(distance);

                    data.lidarPoints[p] = new LidarPoint(position, distance, normDistance);

                    lidarRay.direction = rotAzimuth * lidarRay.direction;
                    p++;
                }
                else
                {
                    data.lidarPoints[p] = new LidarPoint(lidarRay.GetPoint(maxRange), maxRange, 1f);

                    lidarRay.direction = rotAzimuth * lidarRay.direction;
                    p++;
                }
            }

            lidarRay.direction = rotPolar * lidarRay.direction;
        }

        return data;
    }

    private void OnDrawGizmosSelected()
    {
        if (displayInScene == false)
            return;

        LidarData GizmoData = GetOutput();

        for (int i = 0; i < GizmoData.lidarPoints.Length; i++)
        {
            if (GizmoData.lidarPoints[i] == null)
                continue;

            Gizmos.color = Color.HSVToRGB(GizmoData.lidarPoints[i].normDistance, 1f, 1f);
            Gizmos.DrawSphere(GizmoData.lidarPoints[i].position, gizmoSize);
        }
    }

    private void OnRenderObject()
    {
        if (displayInGame == false)
            return;

        CreateLineMaterial();
        lineMaterial.SetPass(0);

        GL.PushMatrix();
        GL.MultMatrix(Matrix4x4.identity);
        GL.Begin(GL.LINES);

        if (data == null)
            return;

        for (int i = 0; i < data.lidarPoints.Length; i++)
        {
            if (data.lidarPoints[i] == null)
                continue;

            GL.Color(Color.HSVToRGB(data.lidarPoints[i].normDistance, 1f, 1f));
            GL.Vertex3(data.lidarPoints[i].position.x, data.lidarPoints[i].position.y, data.lidarPoints[i].position.z);
            GL.Vertex3(data.lidarPoints[i].position.x, data.lidarPoints[i].position.y + gizmoSize, data.lidarPoints[i].position.z);
        }

        GL.End();
        GL.PopMatrix();
    }

    private static Material lineMaterial;
    private static void CreateLineMaterial()
    {
        if (!lineMaterial)
        {
            // Use Unity built-in shader
            Shader shader = Shader.Find("Hidden/Internal-Colored");
            lineMaterial = new Material(shader);
            lineMaterial.hideFlags = HideFlags.HideAndDontSave;

            // Turn on alpha blending
            lineMaterial.SetInt("_SrcBlend", (int)UnityEngine.Rendering.BlendMode.SrcAlpha);
            lineMaterial.SetInt("_DstBlend", (int)UnityEngine.Rendering.BlendMode.OneMinusSrcAlpha);

            // Turn backface culling off
            lineMaterial.SetInt("_Cull", (int)UnityEngine.Rendering.CullMode.Off);

            // Turn off depth writes
            lineMaterial.SetInt("_ZWrite", 0);
        }
    }
}
