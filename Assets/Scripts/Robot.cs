using UnityEngine;

public class Robot
{
    public int ID;
    public string Name;
    public string Status;
    public float Battery;
    public float Latitude;
    public float Longitude;
    public float Heading;
    public int index = 0;


    public Environment_Struct.Mission currentMission;


    public Robot(int id, string name, string status, float battery,
             float latitude, float longitude, float heading)
    {
        ID = id;
        Name = name;
        Status = status;
        Battery = battery;
        Latitude = latitude;
        Longitude = longitude;
        Heading = heading;
    }

    // Getter and Setter Methods for Properties

    public int GetID()
    {
        return ID;
    }

    public void SetID(int newID)
    {
        ID = newID;
    }

    public string GetName()
    {
        return Name;
    }

    public void SetName(string newName)
    {
        Name = newName;
    }

    public string GetStatus()
    {
        return Status;
    }

    public void SetStatus(string newStatus)
    {
        Status = newStatus;
    }

    public float GetBattery()
    {
        return Battery;
    }

    public void SetBattery(float newBattery)
    {
        Battery = newBattery;
    }

    public float GetLatitude()
    {
        return Latitude;
    }

    public void SetLatitude(float newLatitude)
    {
        Latitude = newLatitude;
    }

    public float GetLongitude()
    {
        return Longitude;
    }

    public void SetLongitude(float newLongitude)
    {
        Longitude = newLongitude;
    }

    public float GetHeading()
    {
        return Heading;
    }

    public void SetHeading(float newHeading)
    {
        Heading = newHeading;
    }

    public string GetTopicNamePub()
    {
        return "sarim-ae/rover/robot-status";
    }

    public string GetTopicNameSub()
    {
        return "sarim-ae/rover/" + ID;
    }

    public string ToJson()
    {
        return JsonUtility.ToJson(this);
    }

    public void UpdatePos(Vector2 lonLat, float heading)
    {
        this.SetLongitude(lonLat.y);
        this.SetLatitude(lonLat.x);
        this.SetHeading(heading);

    }



    public void SetMission(Environment_Struct.Mission missionData)
    {
        currentMission = missionData;

        index = 0;


        // check if current mission is not null 

        // if null assing the current missionData to current mission 

        // if not null add on waypoints to the current list of waypoints
    }

}
