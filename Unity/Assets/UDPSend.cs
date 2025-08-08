using UnityEngine;
using System.Net.Sockets;
using System.Text;
using System.Net;

public class UDPSend : MonoBehaviour
{
    UdpClient client;
    public string serverIP = "127.0.0.1";
    public int port = 5022;

    void Start()
    {
        client = new UdpClient();
    }

    public void SendData(string message)
    {
        byte[] data = Encoding.UTF8.GetBytes(message);
        client.Send(data, data.Length, serverIP, port);
        Debug.Log("Sent UDP: " + message);
    }

    void OnApplicationQuit()
    {
        client.Close();
    }
}

