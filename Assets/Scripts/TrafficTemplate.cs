using UnityEngine;

[CreateAssetMenu(fileName = "Traffic", menuName = "Traffic template")]
public class TrafficTemplate : ScriptableObject
{
    /// <summary>
    /// Highway total traffic flow in veh/h
    /// </summary>
    public int flow;

    /// <summary>
    /// Min time headway between vehicles in s
    /// </summary>
    public float minHeadway;

    /// <summary>
    /// Target velocity of the agent in km/h
    /// </summary>
    public int targetVelocity;

    /// <summary>
    /// Highway traffic parameters, adjust array size to number of lanes
    /// </summary>
    public TrafficParameters[] trafficParameters;
}
