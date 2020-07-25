using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

[Serializable]
public class RayPoint
{
    public Vector3 position;
    public float distance;
    public float normDistance;

    public RayPoint(Vector3 p, float d, float nd)
    {
        position = p;
        distance = d;
        normDistance = nd;
    }
}

[Serializable]
public class SensorData
{
    public Vector3 sensorPos;
    public Vector3 sensorRotation;
    public RayPoint[] sensorPoints;

    public void Init(int nRays)
    {
        sensorPoints = new RayPoint[nRays];
    }
}

public class RaySensor : MonoBehaviour
{
    [Header("Sensor parameters")]
    [SerializeField] [Range(1, 200)] private int maxRange = 20;
    private int totalRays;
    [SerializeField] private float noise = 0.2f;
    [SerializeField] private bool displayInGame = false;
    [SerializeField] private bool displayGizmos = false;
    [SerializeField] private float gizmoSize = 0.05f;
    //[SerializeField] private Color gizmoColor = new Color();

    [Header("Azimuthal sweep")]
    [SerializeField] private float startAzimuthal = 45f;
    [SerializeField] private float endAzimuthal = 135f;
    [SerializeField] private int aRays = 10;

    [Header("Polar sweep")]
    [SerializeField] private float startPolar = 0f;
    [SerializeField] private float endPolar = 0f;
    [SerializeField] private int pRays = 1;

    [Header("Sensor data")]
    [SerializeField] private SensorData data;

    public SensorData GetData()
    {
        GetOutput();
        return data;
    }

    private void GetOutput()
    {
        totalRays = aRays * pRays;

        // Initialize data
        data = new SensorData();
        data.Init(totalRays);
        data.sensorPos = transform.position;
        data.sensorRotation = transform.rotation.eulerAngles;

        // Set up ray + starting configuration
        Ray ray = new Ray();
        RaycastHit hit;
        ray.origin = data.sensorPos;
        
        // Rotation steps
        float deltaAzimuthal = 0f, deltaPolar = 0f;
        if (aRays > 1)
            deltaAzimuthal = (endAzimuthal - startAzimuthal) / (aRays - 1);
        if (pRays > 1)
            deltaPolar = (endPolar - startPolar) / (pRays - 1);

        // Execute the sweeps
        int p = 0;
        for (int iP = 0; iP < pRays; iP++)
        {
            for (int iA = 0; iA < aRays; iA++)
            {
                if (p >= data.sensorPoints.Length)
                    break;

                float rX = startPolar + iP * deltaPolar;
                float rY = startAzimuthal + iA * deltaAzimuthal;
                float rZ = 0f;

                ray.direction = Quaternion.Euler(rX, rY, rZ) * transform.forward;

                if (Physics.Raycast(ray, out hit, maxRange))
                {
                    float distance = Mathf.Clamp(hit.distance + noise * Mathf.PerlinNoise(UnityEngine.Random.Range(0f, 1f), UnityEngine.Random.Range(0f, 1f)),
                                                    0f, maxRange);
                    float normDistance = distance / maxRange;
                    Vector3 position = ray.GetPoint(distance);

                    data.sensorPoints[p] = new RayPoint(position, distance, normDistance);
                }
                else
                {
                    data.sensorPoints[p] = new RayPoint(ray.GetPoint(maxRange), maxRange, 1f);
                }
                p++;
            }
        }
    }

    private void OnDrawGizmosSelected()
    {
        if (!displayGizmos)
            return;

        if (!EditorApplication.isPlaying)
        {
            GetOutput();
        }

        for (int i = 0; i < data.sensorPoints.Length; i++)
        {
            if (data.sensorPoints[i] == null)
                continue;

            Gizmos.color = Color.HSVToRGB(data.sensorPoints[i].normDistance, 1f, 1f);
            Gizmos.DrawSphere(data.sensorPoints[i].position, gizmoSize);
        }
    }

    private void OnRenderObject()
    {
        if (displayInGame == false || data == null)
            return;

        CreateLineMaterial();
        lineMaterial.SetPass(0);

        GL.PushMatrix();
        GL.MultMatrix(Matrix4x4.identity);
        GL.Begin(GL.LINES);

        for (int i = 0; i < data.sensorPoints.Length; i++)
        {
            if (data.sensorPoints[i] == null)
                continue;

            GL.Color(Color.HSVToRGB(data.sensorPoints[i].normDistance, 1f, 1f));
            GL.Vertex3(data.sensorPoints[i].position.x, data.sensorPoints[i].position.y, data.sensorPoints[i].position.z);
            GL.Vertex3(data.sensorPoints[i].position.x, data.sensorPoints[i].position.y + gizmoSize, data.sensorPoints[i].position.z);
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
