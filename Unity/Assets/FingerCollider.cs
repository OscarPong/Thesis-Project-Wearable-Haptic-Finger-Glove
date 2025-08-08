using UnityEngine;

public class FingerCollider : MonoBehaviour
{
    public UDPSend udpSend;
    private string currentObject = "";
    private float touchTimer = 0f;
    private float releaseTimer = 0f;
    private float debounceTime = 0.05f; // Adjust as needed

    private string pendingObject = "";

    void OnCollisionEnter(Collision collision)
    {
        pendingObject = collision.gameObject.name;
        touchTimer = 0f;
    }

    void OnCollisionExit(Collision collision)
    {
        // Only clear pendingObject if we're leaving the same one
        if (collision.gameObject.name == pendingObject)
        {
            pendingObject = "";
            releaseTimer = 0f;
        }
    }

    void Update()
    {
        // Handle pending touch
        if (!string.IsNullOrEmpty(pendingObject))
        {
            touchTimer += Time.deltaTime;
            if (touchTimer >= debounceTime && pendingObject != currentObject)
            {
                currentObject = pendingObject;
                SendHaptic(currentObject);
            }
        }

        // Handle release
        if (string.IsNullOrEmpty(pendingObject) && !string.IsNullOrEmpty(currentObject))
        {
            releaseTimer += Time.deltaTime;
            if (releaseTimer >= debounceTime)
            {
                currentObject = "";
                udpSend.SendData("0");
            }
        }
    }

    private void SendHaptic(string objName)
    {
        Debug.Log("Confirmed touch on: " + objName);
        int vibrationValue = GetVibrationValue(objName);
        udpSend.SendData(vibrationValue.ToString());
    }

    private int GetVibrationValue(string name)
    {
        if (name.StartsWith("Vibration"))
        {
            string number = name.Replace("Vibration", "");
            if (int.TryParse(number, out int val))
                return val * 10;
        }
        return 0;
    }
}
