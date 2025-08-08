using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class HandTracking : MonoBehaviour 
{
    public UDPReceive udpReceive;
    public GameObject[] handPoints;

    void Start()
    {

    }

    void Update()
    {
        string data = udpReceive.data;

        if (string.IsNullOrEmpty(data)) return;

        // Remove the first and last character '[' and ']'
        data = data.Remove(0, 1);   
        data = data.Remove(data.Length - 1, 1);
        // Split numbers by ','
        string[] points = data.Split(',');

        for (int i = 0; i < 21; i++)
        {
            float x = 5 - float.Parse(points[i * 3]) / 50;
            float y = float.Parse(points[i * 3 + 1]) / 50;
            float z = 5 - float.Parse(points[i * 3 + 2]) / 50;
            
            handPoints[i].transform.localPosition = new Vector3(x, y, z);
        }
    }
}
