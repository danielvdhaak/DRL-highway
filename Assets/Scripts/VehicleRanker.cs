using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

public class VehicleRanker : MonoBehaviour
{
    private class VehicleComparer : IComparer<VehicleControl>
    {
        int IComparer<VehicleControl>.Compare(VehicleControl a, VehicleControl b)
        {
            return a.Compare.CompareTo(b.Compare);
        }
    }
    private static IComparer<VehicleControl> comparer = new VehicleComparer();

    public void UpdateRanks(List<VehicleControl> traffic)
    {
        foreach (VehicleControl vehicle in traffic)
        {
            vehicle.Compare = (vehicle.transform.localPosition.z);
        }
        traffic.Sort(comparer);

        for (int i = 0; i < traffic.Count; i++)
        {
            // Check for overtake(s)
            if (traffic[i].Rank < i)
            {
                int nOvertakes = i - traffic[i].Rank;
                for (int j = 0; j < nOvertakes; j++)
                {
                    int dir = Math.Sign(traffic[i].Lane - traffic[i - j - 1].Lane);
                    Events.Instance.Overtake(traffic[i].GetInstanceID(), dir);
                }
            }
            traffic[i].Rank = i;
        }
    }

    public void SetInitialRanks(List<VehicleControl> traffic)
    {
        for (int i = 0; i < traffic.Count; i++)
        {
            traffic[i].Rank = i;
        }
    }
}
