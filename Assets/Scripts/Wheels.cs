using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Wheels : MonoBehaviour
{
    // Wheel colliders for left and right wheels
    public WheelCollider[] leftWheelColliders;
    public WheelCollider[] rightWheelColliders;

    // Visual wheels game objects
    public GameObject[] leftVisualWheels;
    public GameObject[] rightVisualWheels;

    Vector2 destination;

    // Vehicle parameters
    public float motorForce = 1000f;
    public float speedLimit = 10.0f;
    public float maxSteerAngle = 30f;
    public float differentialFactor = 0.5f; // Adjust this value to control differential steering
    public float rotatePower = 2f;

    // Flag to control user input
    public bool userInputEnabled = true;

    private float desiredAngle = 30f; // Desired angle in degrees


    float threshhold = 3f;

    public Robot robot;



    private void FixedUpdate()
    {
        // Read user input if enabled
        if (userInputEnabled)
        {
            float verticalInput = Input.GetAxis("Vertical"); // W/S or Up/Down arrow keys
            float horizontalInput = -Input.GetAxis("Horizontal"); // A/D or Left/Right arrow keys

            // Apply motor force only if user input is enabled
            if (userInputEnabled)
            {
                ApplyMotorForce(verticalInput, horizontalInput);
            }

        }
        else
        {
            Robot robot1 = GetComponent<UGVMQTT>().robot1;

            if (robot1.currentMission.id != null) {
               
                CoordsConverter converter = GetComponent<CoordsConverter>();

                if (robot1.index < robot1.currentMission.waypoints.Length)
                {
                    Vector2 destination = converter.ConvertLonLatToXZ(new Vector2(robot1.currentMission.waypoints[robot1.index].lat, (robot1.currentMission.waypoints[robot1.index].lng)));
                   
                    

                    if (Vector2.Distance(new Vector2(transform.position.z, transform.position.x), destination) > threshhold)

                    {

                       // Debug.Log(robot1.index);

                        float angleIWantToGo = CalculateYawAngle(transform.position, new Vector3(destination.y, 0, destination.x)); // Replace with your desired angle
                        float currentAngle = transform.eulerAngles.y;
                        float angleSign = Mathf.Sign(angleIWantToGo - currentAngle);
                        float angleDiffAbs = Mathf.Abs(angleIWantToGo - currentAngle);

                        if (angleSign > 0 && angleDiffAbs < 180 && angleDiffAbs > 1f)
                        {
                            ApplyMotorForce(1, -1);
                            Debug.Log("Right+");
                        }
                        else if (angleSign > 0 && angleDiffAbs > 180 && angleDiffAbs > 1f)
                        {
                            ApplyMotorForce(1, 1);
                            Debug.Log("Left+");
                        }
                        else if (angleSign < 0 && angleDiffAbs < 180 && angleDiffAbs > 1f)
                        {
                            ApplyMotorForce(1, 1);
                            Debug.Log("Left-");
                        }
                        else if (angleSign < 0 && angleDiffAbs > 180 && angleDiffAbs > 1f)
                        {
                            ApplyMotorForce(1, -1);
                            Debug.Log("Right-");
                        }
                        else
                            ApplyMotorForce(1, 0);

                    } else
                    {
                        robot1.index++;
                        //ApplyMotorForce(0, 0);
                    }


                }



                else
                    ApplyMotorForce(0, 0);





            }
        }




        // Update visual wheels
        UpdateVisualWheels();
    }

    // Calculate the yaw angle (y-rotation) needed to face point B from point A
    float CalculateYawAngle(Vector3 pointA, Vector3 pointB)
    {
        Vector3 directionToB = (pointB - pointA).normalized;
        float angle = Mathf.Atan2(directionToB.x, directionToB.z) * Mathf.Rad2Deg;
        if (angle < 0)
            return angle + 360;

        return angle;
    }

    // Apply motor force based on speed limit
    void ApplyMotorForce(float verticalInput, float horizontalInput)
    {
        float currentSpeed = leftWheelColliders[0].rpm * (leftWheelColliders[0].radius * 2 * Mathf.PI) * 60 / 1000;

        if (Mathf.Abs(currentSpeed) < speedLimit)
        {
            for (int i = 0; i < leftWheelColliders.Length; i++)
            {
                leftWheelColliders[i].motorTorque = verticalInput * motorForce * Time.deltaTime;
                rightWheelColliders[i].motorTorque = verticalInput * motorForce * Time.deltaTime;
            }
        }

        // Differential control for rotation
        float rotationFactor = horizontalInput * rotatePower;

        for (int i = 0; i < leftWheelColliders.Length; i++)
        {
            if (rotationFactor > 0)
            {
                leftWheelColliders[i].motorTorque -= rotationFactor * motorForce * Time.deltaTime;
                rightWheelColliders[i].motorTorque += rotationFactor * motorForce * Time.deltaTime;
            }
            else if (rotationFactor < 0)
            {
                leftWheelColliders[i].motorTorque += Mathf.Abs(rotationFactor) * motorForce * Time.deltaTime;
                rightWheelColliders[i].motorTorque -= Mathf.Abs(rotationFactor) * motorForce * Time.deltaTime;
            }
        }
    }

    // Update visual wheels' positions and rotations
    void UpdateVisualWheels()
    {
        for (int i = 0; i < leftVisualWheels.Length; i++)
        {
            WheelCollider wheelCollider = leftWheelColliders[i];
            GameObject visualWheel = leftVisualWheels[i];

            Vector3 wheelPosition;
            Quaternion wheelRotation;
            wheelCollider.GetWorldPose(out wheelPosition, out wheelRotation);
            visualWheel.transform.position = wheelPosition;
            visualWheel.transform.rotation = wheelRotation;
        }

        for (int i = 0; i < rightVisualWheels.Length; i++)
        {
            WheelCollider wheelCollider = rightWheelColliders[i];
            GameObject visualWheel = rightVisualWheels[i];

            Vector3 wheelPosition;
            Quaternion wheelRotation;
            wheelCollider.GetWorldPose(out wheelPosition, out wheelRotation);
            visualWheel.transform.position = wheelPosition;
            visualWheel.transform.rotation = wheelRotation;
        }
    }
}
