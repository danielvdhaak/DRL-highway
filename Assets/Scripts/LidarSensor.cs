/*
 * This code is part of DRL-Highway by Daniel (2020)
 * 
 */

using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

public class LidarSensor : MonoBehaviour
{
    [Serializable]
    public class LidarPoint
    {
        public Vector3 position;
        public Vector3 distance;
        public float normDistance;
    }

    [Serializable]
    public class Data
    {
        // General sensor information
        public Vector3 sensorPos;
        public Vector3 sensorRotation;

        // Data arrays
        public Vector3[] points;
        public float[] distances;
        public float[] normDistances;
        public string[] tags;

        // Initialize arrays
        public void Init(int nRays)
        {
            points = new Vector3[nRays];
            distances = new float[nRays];
            normDistances = new float[nRays];
            tags = new string[nRays];
        }
    }

    [Header("Sensor parameters")]
    [Range(10,100)] public int range = 50;
    public int deltaAzimuthal = 2;
    public float polarOffset = 25f;
    public int deltaPolar = -1;
    public int polarSweep = 25;
    public float noise = 0.2f;

    [Header("Debugging")]
    public float gizmoSize = 0.05f;
    public bool displayInScene = false;
    public bool displayInGame = false;

    [Header("Sensor data")]
    public Data data;

    public void Update()
    {
        GetOutput();
    }

    public void GetOutput()
    {
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
                if (p >= data.points.Length)
                    break;

                if (Physics.Raycast(lidarRay, out hit, range))
                {
                    // Determine hit info and apply noise
                    float distance = hit.distance + noise * Mathf.PerlinNoise(UnityEngine.Random.Range(0f, 1f), UnityEngine.Random.Range(0f, 1f));
                    distance = Mathf.Clamp(distance, 0f, range);

                    data.points[p] = lidarRay.GetPoint(distance);
                    data.distances[p] = distance;
                    data.normDistances[p] = distance / range;
                    data.tags[p] = hit.transform.tag;

                    lidarRay.direction = rotAzimuth * lidarRay.direction;
                    p++;
                }
                else
                {
                    // Set hit info to sensor range as default
                    data.points[p] = lidarRay.GetPoint(range);
                    data.distances[p] = range;
                    data.normDistances[p] = 1f;
                    data.tags[p] = "Untagged";

                    lidarRay.direction = rotAzimuth * lidarRay.direction;
                    p++;
                }
            }
            lidarRay.direction = rotPolar * lidarRay.direction;
        }
    }

    private void OnDrawGizmosSelected()
    {
        if (displayInScene == false)
            return;

        GetOutput();

        for (int i = 0; i < data.points.Length; i++)
        {
            if (data.points[i] == null ^ data.normDistances[i] == null)
                continue;

            Gizmos.color = Color.HSVToRGB(data.normDistances[i], 1f, 1f);
            Gizmos.DrawSphere(data.points[i], gizmoSize);
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

        for (int i = 0; i < data.points.Length; i++)
        {
            if (data.points[i] == null ^ data.normDistances[i] == null)
                continue;

            GL.Color(Color.HSVToRGB(data.normDistances[i], 1f, 1f));
            GL.Vertex3(data.points[i].x, data.points[i].y, data.points[i].z);
            GL.Vertex3(data.points[i].x, data.points[i].y + gizmoSize, data.points[i].z);
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
