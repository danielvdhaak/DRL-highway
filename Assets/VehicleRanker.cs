using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class VehicleRanker : MonoBehaviour
{
    private class VehicleComparer : IComparer<VehicleControl>
    {
        int IComparer<VehicleControl>.Compare(VehicleControl a, VehicleControl b)
        {
            return a.Compare.CompareTo(b.Compare);
        }
    }
}
